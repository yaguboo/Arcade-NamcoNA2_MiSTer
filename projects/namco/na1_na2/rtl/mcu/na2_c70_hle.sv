//============================================================================
//  Namco C70 -- Phase 1 functional stand-in  *** TEMPORARY_HLE ***
//
//  TEMPORARY_HLE.  This module exists so the 68000 can be brought up and the
//  video hardware exercised before the M37702 core is in place.  Everything
//  here must be deleted once rtl/mcu/na2_m37702.sv runs the real c70.bin
//  (project Level 9).
//
//  ---------------------------------------------------------------------
//  1. Boot vector deposit + 68000 release        ROM_DISASM_CONFIRMED
//  ---------------------------------------------------------------------
//  c70.bin at 0x00C62E, with the data bank register set to $20 (the shared
//  work RAM window):
//      LDM #$0000, $0000     LDM #$0004, $0002
//      LDM #$C000, $0004     LDM #$0000, $0006
//  then at 0x00C646 "LDM #$0C,$0A" sets port P4 bit 3, which MAME's port4_w
//  documents as the 68000 run signal.  Byte-swapping the MCU view gives
//  68000 SSP = 0x00000400, PC = 0x00C00000; the game's own entry code at
//  0xC0000C reloads A7 with exactly 0x400.  Only the *timing* is fake.
//
//  ---------------------------------------------------------------------
//  2. The mailbox command protocol               ROM_DISASM_CONFIRMED
//  ---------------------------------------------------------------------
//  The 68000 stages a parameter block in shared work RAM at 0x0F30, puts a
//  command code in the byte at 0x0F72, then writes mailbox slot 4 (0x3F8008)
//  which raises MCU IRQ0.  The parameter block is an array of 16-byte
//  descriptors; within a descriptor:
//      +0x04  source address
//      +0x06  length in bytes
//      +0x08  destination address
//
//  Two commands are used during boot, and they are the same DMA primitive in
//  opposite directions:
//
//    command 3  (0xC2D58C)  68000 work RAM -> C70 address space.
//        The game first copies 0x13E bytes from ROM 0xC2D694 to work RAM
//        0x0400, then asks for src=0x0400 len=0x013E dst=0x8A00.  0x8A00 is
//        inside the C70's external SRAM window (0x3000-0xAFFF, the LH5168 on
//        the PCB), so this is the game uploading sound/driver tables into the
//        MCU.  Issued three times with different blocks.
//
//    command 7  (0xC2DA42)  C70 address space -> 68000 work RAM.
//        src=0xC000 len=0x0010 dst=0x1000, i.e. copy the C70's own BIOS
//        header into work RAM.  The game then compares 16 bytes at 0x1000
//        against "SN-AIBSOv re.113" -- "NSA-BIOS ver1.31" with every 16-bit
//        word byte-swapped, which is exactly what a straight byte copy through
//        the MCU's little-endian view of the shared window produces.  On
//        mismatch it clears GFX_ENABLE and hangs forever at 0xC2DA82.
//
//  ---------------------------------------------------------------------
//  3. The status byte at 0x0F60                  ROM_DISASM_CONFIRMED
//  ---------------------------------------------------------------------
//  This is the piece MAME gets wrong.  MAME's simulate_mcu() writes
//  workram[0x0F60] = 0 once per frame and its own comment says
//  "FIXME: These two functions shouldn't be necessary?".  FBNeo does the same
//  at interleave 240.  Neither explains it.
//
//  The game does:
//      C2D58C  MOVE.B #$3,$0F72.W          ; command
//      C2D592  MOVE.W #$0,$3F8008.L        ; doorbell
//      C2D59A  CMPI.B #$3,$0F60.W          ; spin until the byte changes
//      C2D5A0  BEQ.B  $C2D59A
//  and the descriptor block it just copied in pre-loads 0x0F60 with 0x03
//  itself.  So 0x0F60 is a *command acknowledge* byte: the 68000 arms it and
//  the MCU changes it when the request has been serviced.
//
//  This module therefore clears it on command completion rather than on a
//  frame timer.  That removes MAME's per-frame hack entirely (a Level 10 item
//  done early) and is faster and more faithful: the real MCU answers in
//  microseconds, not on the next vblank.
//
//  INFERRED: clearing to 0x00 specifically.  The game only tests "not 3", so
//  any other value would also work; 0 is what both emulators write.
//
//  ---------------------------------------------------------------------
//  NOT modelled: input polling, the C219, EEPROM sequencing, the watchdog,
//  the C70 external SRAM contents, and every command other than 3 and 7.
//  Unrecognised commands are acknowledged but not acted on, and reported in
//  simulation so they cannot pass unnoticed.
//============================================================================
`default_nettype none

`include "na2_pkg.svh"

module na2_c70_hle (
    input  wire        clk,
    input  wire        rst,          // system reset, active high
    input  wire        ce_en,

    // --- 68000 reset control ----------------------------------------------
    output reg         m68k_rst,     // 1 = hold the 68000 in reset

    // --- mailbox, as seen by the 68000 at 0x3F8000-0x3FFFFF ---------------
    input  wire        mbx_sel,
    input  wire [2:0]  mbx_addr,     // word address, 8 slots (mirrored)
    input  wire [15:0] mbx_din,
    input  wire        mbx_wr,
    input  wire  [1:0] mbx_ds,       // {upper, lower} byte enables
    output wire [15:0] mbx_dout,

    // --- work RAM master port (arbitrated ahead of the 68000) -------------
    output reg         ram_req,
    output reg         ram_we,
    output reg  [1:0]  ram_ds,
    output reg  [18:1] ram_addr,     // word address inside 512 KiB
    output reg  [15:0] ram_dout,
    input  wire [15:0] ram_din,
    input  wire        ram_ack,
    output wire        ram_busy,     // 1 while this module owns the work RAM

    // --- C70 BIOS image (16 KiB, byte addressed 0x0000-0x3FFF) ------------
    output reg  [13:0] bios_addr,
    input  wire  [7:0] bios_data,

    // --- muxed inputs, active low as on the JAMMA edge --------------------
    input  wire  [7:0] in_p1,
    input  wire  [7:0] in_p2,
    // Player 3 reaches the real C70 through the M37702's A-D lines, and this
    // model does not emulate those.  The port exists so that na2_top's
    // g_c70_hle branch still elaborates: it connects .in_p3, and without this
    // the bisect path fails to compile.  Found by Verilator lint -- Quartus
    // never sees it, because USE_REAL_C70 defaults to 1 and the branch is not
    // elaborated in any shipping build.
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire  [7:0] in_p3,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire  [7:0] in_p4,
    input  wire  [7:0] in_dsw,
    input  wire        vblank_rise,

    // --- observability ----------------------------------------------------
    output wire [4:0]  dbg_state,
    output reg  [7:0]  dbg_last_cmd,
    output reg         dbg_cmd_done,
    // TEMPORARY: command 7 passes in simulation (sim/tb_na2_c70.sv) and fails
    // on hardware, so put the parameters it read and the last word it wrote
    // where a screenshot can see them.
    output wire [15:0] dbg_p_src,
    output wire [15:0] dbg_p_dst,
    output wire [15:0] dbg_copy_word
);

  // ---------------------------------------------------------------------
  // Mailbox: 8 x 16-bit, mirrored across 0x3F8000-0x3FFFFF.
  // MAME_VERIFIED: mcu_mailbox_r/w use (offset & 7).
  // ---------------------------------------------------------------------
  reg [15:0] mailbox[0:7];
  assign mbx_dout = mailbox[mbx_addr];

  // Doorbell: the 68000 writing slot 4 raises C70 IRQ0.
  // MAME_VERIFIED: mcu_mailbox_w_68k -> M37710_LINE_IRQ0.
  // ROM_DISASM_CONFIRMED: the game writes 0x3F8008 (= slot 4) at 0xC2D592,
  // 0xC2D5BA, 0xC2D5E2 (command 3) and 0xC2DA48 (command 7).
  wire doorbell = mbx_sel & mbx_wr & (mbx_addr == 3'd4);

  // ---------------------------------------------------------------------
  // Work RAM word addresses this module touches.
  // ---------------------------------------------------------------------
  localparam [18:1] WA_VEC0 = 18'h00000;          // 0x000000, +1..+3
  localparam [18:1] WA_ACK  = 18'h00F60 >> 1;     // 0x000F60 status byte
  localparam [18:1] WA_CMD  = 18'h00F72 >> 1;     // 0x000F72 command byte
  localparam [18:1] WA_SRC  = 18'h00F34 >> 1;
  localparam [18:1] WA_LEN  = 18'h00F36 >> 1;
  localparam [18:1] WA_DST  = 18'h00F38 >> 1;
  // Input deposit, ROM_DISASM_CONFIRMED both ends:
  //   c70.bin 0x00D26E polls the four muxed ports and stores them at MCU
  //   0x2FFC..0x2FFF, which is the shared work RAM mirror -> 68000 0x0FFC.
  //   The game reads them back with "MOVE.L $0FFC.W,D0" at 0xC2E080 and
  //   "MOVE.B $0FFD.W,D1" at 0xC009CC.
  //     68000 0x0FFC = P4, 0x0FFD = DSW, 0x0FFE = P1, 0x0FFF = P2
  localparam [18:1] WA_IN0  = 18'h00FFC >> 1;   // {P4, DSW}
  localparam [18:1] WA_IN1  = 18'h00FFE >> 1;   // {P1, P2}

  localparam S_VEC        = 5'd0,
             S_VEC_W      = 5'd1,
             S_RUN        = 5'd2,
             S_CMD_RD     = 5'd3,
             S_CMD_W      = 5'd4,
             S_P_SRC      = 5'd5,
             S_P_SRC_W    = 5'd6,
             S_P_LEN      = 5'd7,
             S_P_LEN_W    = 5'd8,
             S_P_DST      = 5'd9,
             S_P_DST_W    = 5'd10,
             S_COPY_FETCH = 5'd11,
             S_COPY_B0    = 5'd12,
             S_COPY_B1    = 5'd13,
             S_COPY_W0    = 5'd22,      // BIOS BRAM read latency
             S_COPY_W1    = 5'd23,
             S_COPY_WR    = 5'd14,
             S_COPY_W     = 5'd15,
             S_ACK        = 5'd16,
             S_ACK_W      = 5'd17,
             S_IN0        = 5'd18,
             S_IN0_W      = 5'd19,
             S_IN1        = 5'd20,
             S_IN1_W      = 5'd21;

  reg [4:0]  st;
  reg [1:0]  vec_i;
  reg [15:0] p_src, p_len, p_dst;
  reg [15:0] copy_word;
  assign dbg_p_src     = p_src;
  assign dbg_p_dst     = p_dst;
  assign dbg_copy_word = copy_word;
  reg [15:0] copy_cnt;
  reg [7:0]  cmd;
  reg        pending_cmd;
  reg        pending_poll;

  assign ram_busy  = (st != S_RUN);
  assign dbg_state = st;

  // The four words the C70 deposits, in 68000 (big-endian) form.
  localparam [31:0] BOOT_SSP = `NA2_BOOT_SSP;
  localparam [31:0] BOOT_PC  = `NA2_BOOT_PC;

  function [15:0] vec_word(input [1:0] i);
    case (i)
      2'd0: vec_word = BOOT_SSP[31:16];   // 0x0000
      2'd1: vec_word = BOOT_SSP[15:0];    // 0x0400
      2'd2: vec_word = BOOT_PC [31:16];   // 0x00C0
      2'd3: vec_word = BOOT_PC [15:0];    // 0x0000
    endcase
  endfunction

  // destination word address for the running block copy
  wire [18:1] copy_dst_wa = {3'd0, p_dst[15:1]} + {3'd0, copy_cnt[15:1]};

  integer i;
  always @(posedge clk) begin
    if (rst) begin
      st           <= S_VEC;
      vec_i        <= 2'd0;
      m68k_rst     <= 1'b1;      // HW_CONFIRMED: P4 bit 3 clear at C70 boot
      ram_req      <= 1'b0;
      ram_we       <= 1'b0;
      ram_ds       <= 2'b11;
      pending_cmd  <= 1'b0;
      pending_poll <= 1'b0;
      copy_cnt     <= 16'd0;
      cmd          <= 8'd0;
      dbg_last_cmd <= 8'd0;
      dbg_cmd_done <= 1'b0;
      for (i = 0; i < 8; i = i + 1) mailbox[i] <= 16'h0000;
    end else if (ce_en) begin
      dbg_cmd_done <= 1'b0;

      // ---- mailbox writes from the 68000 -------------------------------
      if (mbx_sel & mbx_wr) begin
        if (mbx_ds[1]) mailbox[mbx_addr][15:8] <= mbx_din[15:8];
        if (mbx_ds[0]) mailbox[mbx_addr][ 7:0] <= mbx_din[ 7:0];
      end
      if (doorbell)    pending_cmd  <= 1'b1;
      if (vblank_rise) pending_poll <= 1'b1;

      case (st)
        // ---------------- boot: deposit the reset vector ----------------
        S_VEC: begin
          ram_req  <= 1'b1;
          ram_we   <= 1'b1;
          ram_ds   <= 2'b11;
          ram_addr <= WA_VEC0 + {16'd0, vec_i};
          ram_dout <= vec_word(vec_i);
          st       <= S_VEC_W;
        end
        S_VEC_W: if (ram_ack) begin
          ram_req <= 1'b0;
          ram_we  <= 1'b0;
          if (vec_i == 2'd3) begin
            // ROM_DISASM_CONFIRMED: c70.bin 0x00C646, P4 bit 3 -> 1
            m68k_rst <= 1'b0;
            st       <= S_RUN;
          end else begin
            vec_i <= vec_i + 2'd1;
            st    <= S_VEC;
          end
        end

        // ---------------- idle ------------------------------------------
        S_RUN: begin
          if (pending_cmd) begin
            pending_cmd <= 1'b0;
            st          <= S_CMD_RD;
          end else if (pending_poll) begin
            pending_poll <= 1'b0;
            st           <= S_IN0;
          end
        end

        // ---------------- command dispatch ------------------------------
        S_CMD_RD: begin
          ram_req <= 1'b1; ram_we <= 1'b0; ram_addr <= WA_CMD;
          st      <= S_CMD_W;
        end
        S_CMD_W: if (ram_ack) begin
          ram_req      <= 1'b0;
          // the command code is the high byte of the word at 0x0F72
          // (the game writes it with MOVE.B #$n,$F72.W)
          cmd          <= ram_din[15:8];
          dbg_last_cmd <= ram_din[15:8];
          st           <= S_P_SRC;
        end

        S_P_SRC:   begin ram_req <= 1'b1; ram_addr <= WA_SRC; st <= S_P_SRC_W; end
        S_P_SRC_W: if (ram_ack) begin ram_req <= 1'b0; p_src <= ram_din; st <= S_P_LEN; end
        S_P_LEN:   begin ram_req <= 1'b1; ram_addr <= WA_LEN; st <= S_P_LEN_W; end
        S_P_LEN_W: if (ram_ack) begin ram_req <= 1'b0; p_len <= ram_din; st <= S_P_DST; end
        S_P_DST:   begin ram_req <= 1'b1; ram_addr <= WA_DST; st <= S_P_DST_W; end
        S_P_DST_W: if (ram_ack) begin
          ram_req  <= 1'b0;
          p_dst    <= ram_din;
          copy_cnt <= 16'd0;
          // Only command 7 (C70 internal ROM -> work RAM) moves data that
          // anything can observe yet.  Command 3 writes into the C70's
          // external SRAM, which nothing reads until the C219 exists, so it
          // is accepted and acknowledged without transferring.
          if (cmd == 8'h07 && p_len != 16'd0)
            st <= S_COPY_FETCH;
          else begin
            // synthesis translate_off
            if (cmd != 8'h03 && cmd != 8'h07)
              $display("[%0t] C70_HLE: unhandled command %02h (src=%04h len=%04h dst=%04h)",
                       $time, cmd, p_src, p_len, ram_din);
            // synthesis translate_on
            st <= S_ACK;
          end
        end

        // ---------------- block copy: C70 ROM -> shared work RAM --------
        // A straight byte copy.  The MCU is little-endian and the shared RAM
        // window byte-swaps, so byte N of the source ends up at 68000 byte
        // address dst+N with the two bytes of each word exchanged.  That is
        // what puts "SN-AIBSO..." (not "NSA-BIOS...") in front of the 68000.
        // The BIOS image lives in block RAM, which answers one clock after
        // the address, so each byte needs a wait state.  Simulation models it
        // the same way so the two cannot drift apart.
        S_COPY_FETCH: begin
          bios_addr <= p_src[13:0] + copy_cnt[13:0];
          st        <= S_COPY_W0;
        end
        S_COPY_W0: st <= S_COPY_B0;
        S_COPY_B0: begin
          copy_word[7:0] <= bios_data;                        // byte src+0
          bios_addr      <= p_src[13:0] + copy_cnt[13:0] + 14'd1;
          st             <= S_COPY_W1;
        end
        S_COPY_W1: st <= S_COPY_B1;
        S_COPY_B1: begin
          copy_word[15:8] <= bios_data;                       // byte src+1
          st              <= S_COPY_WR;
        end
        S_COPY_WR: begin
          ram_req  <= 1'b1; ram_we <= 1'b1; ram_ds <= 2'b11;
          ram_addr <= copy_dst_wa;
          // copy_word is {byte src+1, byte src+0}, and that is exactly the
          // 68000 word: byte src+1 lands at 68000 address dst+0.
          //
          // It used to write {copy_word[7:0], copy_word[15:8]}, i.e. a
          // straight unswapped copy, which is the opposite of what the comment
          // above this state has always said.  ROM_DISASM_CONFIRMED which way
          // round it goes: c70.bin starts "NSA-BIOS ver1.31" and the 16 bytes
          // the game compares against, at 0xC2DA88, are
          //     53 4E 2D 41 49 42 53 4F 76 20 72 65 2E 31 31 33
          // = "SN-AIBSOv re.113", the byte-swapped form, byte for byte.
          //
          // Nothing caught this because command 7 has never been reached in
          // simulation.  On hardware the version check failed and the game sat
          // in its hang loop at 0xC2DA82 -- which the debug overlay's sampled
          // fetch address showed directly.
          ram_dout <= copy_word;
          st       <= S_COPY_W;
        end
        S_COPY_W: if (ram_ack) begin
          ram_req <= 1'b0; ram_we <= 1'b0;
          if (copy_cnt + 16'd2 >= p_len) st <= S_ACK;
          else begin copy_cnt <= copy_cnt + 16'd2; st <= S_COPY_FETCH; end
        end

        // ---------------- acknowledge -----------------------------------
        // Clear the byte at 0x0F60.  It is the *high* byte of the word there,
        // because the 68000 arms it with MOVE.B to an even address.
        S_ACK: begin
          ram_req  <= 1'b1; ram_we <= 1'b1; ram_ds <= 2'b10;
          ram_addr <= WA_ACK; ram_dout <= 16'h0000;
          st       <= S_ACK_W;
        end
        S_ACK_W: if (ram_ack) begin
          ram_req      <= 1'b0; ram_we <= 1'b0; ram_ds <= 2'b11;
          dbg_cmd_done <= 1'b1;
          st           <= S_RUN;
        end

        // ---------------- input poll ------------------------------------
        // INFERRED: once per frame.  MAME raises MCU IRQ1 at vblank and the
        // BIOS polling routine at 0x00D26E is the obvious consumer, but the
        // call site has not been traced yet, so the *rate* is an assumption
        // even though the addresses are confirmed.
        S_IN0: begin
          ram_req  <= 1'b1; ram_we <= 1'b1; ram_ds <= 2'b11;
          ram_addr <= WA_IN0; ram_dout <= {in_p4, in_dsw};
          st       <= S_IN0_W;
        end
        S_IN0_W: if (ram_ack) begin ram_req <= 1'b0; ram_we <= 1'b0; st <= S_IN1; end
        S_IN1: begin
          ram_req  <= 1'b1; ram_we <= 1'b1; ram_ds <= 2'b11;
          ram_addr <= WA_IN1; ram_dout <= {in_p1, in_p2};
          st       <= S_IN1_W;
        end
        S_IN1_W: if (ram_ack) begin ram_req <= 1'b0; ram_we <= 1'b0; st <= S_RUN; end

        default: st <= S_RUN;
      endcase
    end
  end

endmodule

`default_nettype wire
