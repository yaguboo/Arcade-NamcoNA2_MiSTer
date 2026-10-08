//============================================================================
//  Namco C70 -- the real MCU: a Mitsubishi M37702M2 running c70.bin
//
//  This is the wrapper: the MCU's address decode, its two on-chip memories,
//  the mailbox the 68000 talks to, the byte-swapped window onto shared work
//  RAM, and the C219 register port.  The CPU is na2_m37702_cpu (VHDL,
//  vendored -- see LICENSE_NOTES.md and docs/m37702_survey.md) and the
//  on-chip peripherals are na2_m37702_periph.
//
//  It is a drop-in replacement for na2_c70_hle: same ports, plus the C219
//  register port, minus the command-7 debug outputs the stand-in needed.
//  na2_top keeps both and selects with a parameter, so a regression can be
//  bisected instead of argued about.
//
//  Address map, MAME_VERIFIED from namcona1_mcu_map (reference/mame/
//  namcona1.cpp) and the internal map in reference/mame/m37710.cpp:
//
//    0x000000-0x00007F  on-chip peripheral registers   na2_m37702_periph
//    0x000080-0x00027F  internal RAM, 512 B
//    0x000800-0x000FFF  mailbox, 8 x 16 bit, mirrored  (offset & 7)
//    0x001000-0x0011FF  C219 registers  (mirror 0x000E00)
//    0x002000-0x002FFF  first 4 KiB of shared work RAM
//    0x003000-0x00AFFF  external SRAM, 32 KiB
//    0x00C000-0x00FFFF  internal mask ROM, 16 KiB      (c70.bin, via na2_membus)
//    0x200000-0x27FFFF  shared work RAM, 512 KiB
//
//  INFERRED -- the external SRAM size.  RESEARCH.md section 2 records an
//  LH5168 (8K x 8) on the PCB, but MAME maps 32 KiB and the game's own mailbox
//  command 3 uploads a block to 0x8A00, which is past the end of 8 KiB.  The
//  game contradicts the part number, so 32 KiB is what is implemented and the
//  contradiction is recorded in KNOWN_ISSUES.md rather than quietly resolved.
//
//  MAME_VERIFIED -- the shared-RAM byte order.  na1mcu_shared_r/w swap both
//  data and mask, and the net effect is that MCU byte N is 68000 byte N.
//  RESEARCH.md section 13 works this through for the input deposit.  Here:
//  an even byte address is ram_din[15:8], an odd one is ram_din[7:0], which
//  is the same convention snd_data already uses in na2_top.sv.
//
//  MAME_VERIFIED -- the C219 register window is reached little-endian
//  (c219_le_r/le_w), so MCU byte offset X is chip register X^1.  This is the
//  same swap tools/c219_ref.py applies to the captured MAME writes that
//  rtl/sound/na2_c219.sv was verified against.
//
//  The bus handshake copies the one in the core's own upstream wrapper
//  (c76.vhd), which is the version that has actually been run: bus_ready is a
//  LEVEL, and an internal access is only "ready" once its address has been
//  stable for one MCU cycle.  A one-clock pulse would be wrong twice over --
//  the CPU only samples on its clock enable, and a ready that stays high
//  across two back-to-back accesses completes the second one on the first
//  one's data.  Upstream's comment records finding exactly that bug.
//============================================================================
`default_nettype none

module na2_c70 #(
    // clk_sys ticks per MCU clock.  master/4 = 12.5282 MHz from 50.113 MHz,
    // which is XIN, the pin -- not the execution clock.
    //
    // MAME_VERIFIED, and the units matter: namcona1.cpp instantiates the C70 at
    // MASTER_CLOCK/4, and m37710.h:209 then divides that by two before
    // executing anything, so instructions run at 6.2641 MHz.  That is FBNeo's
    // figure exactly; the "master/4 vs master/8" in MAME_FBNEO_DIFF.md was one
    // clock quoted at two points, and ISSUE-2 is closed on it.
    //
    // Timers are the part that has to be right, and MAME counts them off the
    // UNDIVIDED XIN (m37710.cpp:520,547 use clocks_to_attotime).  `mcu_ce` is
    // therefore XIN here and na2_m37702_periph counts timers on it, which puts
    // the sound driver's Timer A0 at 64 * 0x65B = 104128 clocks = 120.31 Hz,
    // MAME's own number.  sim/run_periph.sh measures it.
    parameter int CLK_DIV = 4
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        ce_en,          // global clock enable (pause / download)

    // --- the 68000's reset line, released by port P4 bit 3 -----------------
    output wire        m68k_rst,

    // --- mailbox, 68000 side: 0x3F8000-0x3FFFFF, (offset & 7) -------------
    input  wire        mbx_sel,
    input  wire  [3:1] mbx_addr,
    input  wire [15:0] mbx_din,
    input  wire        mbx_wr,
    input  wire  [1:0] mbx_ds,
    output wire [15:0] mbx_dout,

    // --- work RAM master port (arbitrated ahead of the 68000) -------------
    output reg         ram_req,
    output reg         ram_we,
    output reg  [1:0]  ram_ds,
    output reg  [18:1] ram_addr,
    output reg  [15:0] ram_dout,
    input  wire [15:0] ram_din,
    input  wire        ram_ack,
    output wire        ram_busy,

    // --- C70 BIOS image (16 KiB, byte addressed), 1 clock read latency ----
    output wire [13:0] bios_addr,
    input  wire  [7:0] bios_data,

    // --- muxed inputs, active low as on the JAMMA edge --------------------
    input  wire  [7:0] in_p1,
    input  wire  [7:0] in_p2,
    input  wire  [7:0] in_p4,
    input  wire  [7:0] in_p3,   // via the A-D inputs, not the digital mux
    input  wire  [7:0] in_dsw,
    input  wire        vblank_rise,    // raises MCU IRQ1

    // --- C219 register port -----------------------------------------------
    output wire        snd_reg_we,
    output wire  [8:0] snd_reg_addr,
    output wire  [7:0] snd_reg_din,
    input  wire [15:0] snd_voice_key,   // each voice's key bit, for read-back

    // --- observability -----------------------------------------------------
    output wire [23:0] dbg_pc,
    output wire  [7:0] dbg_opcode,
    output wire        dbg_valid,      // one pulse per retired instruction
    output wire [15:0] dbg_regA,
    output wire [15:0] dbg_regB,
    output wire [15:0] dbg_regX,
    // The sound driver's own channel block for the melody, read straight out of
    // external SRAM.  X = 0x4600 is the channel that drives C219 voice 0; the
    // fields come from the driver's code at 0x00E1AA-0x00E253:
    //
    //   $08,X  0x4608  the channel is alive          MAME 0x80
    //   $12,X  0x4612  envelope program pointer, 16  MAME 0x040A (into 68000 RAM)
    //   $14,X  0x4614  envelope fraction             MAME 0x50
    //   $15,X  0x4615  envelope LEVEL                MAME 0xFF, board 0x24
    //   $16,X  0x4616  ramp step, 16-bit signed
    //   $18,X  0x4618  envelope TARGET               MAME 0xFF
    //   $2D,X  0x462D  frames until the next advance MAME 0x00
    //   $30,X  0x4630  channel volume                MAME 0x44, board 0x4A
    //
    // and 0x00E24E/0x00E250 multiply level x volume into the C219's volume
    // register.  The board plays the melody at a sixth of MAME's level; these
    // taps say whether the level is being ramped to a different TARGET or is
    // failing to reach the same one.  docs/bgm_investigation.md section 15.
    output wire [15:0] dbg_chan,     // {$30,X, $15,X}  -- volume : level
    output wire [15:0] dbg_env_tl,   // {$18,X, $15,X}  -- target : level
    output wire [15:0] dbg_env_ptr,  //  $12,X          -- envelope pointer
    output wire [15:0] dbg_env_rate, //  $16,X          -- ramp step
    output wire [15:0] dbg_env_act,  // {$08,X, $2D,X}  -- alive : delay
    // 2026-08-23.  The ramp was cleared: our core executes all thirteen of its
    // instructions exactly as MAME does (54/54, tools/bgm_ramp_gen.py), yet the
    // board holds the level at 0x24 for nineteen seconds in a state where MAME
    // would move it to 0x1A.  So the question is no longer what the ramp does
    // with the channel, it is whether the driver runs the channel at all:
    //
    //   00DA26  LDX #$6420       ; the channel list, twelve entries then 0x0000
    //   00DA2C  LDA $00,X        ; 0x4600 is the FIRST of them
    //   00DA32  TAX
    //   00DA36  JMP ($010e) -> 00DF5A -> JMP ($0120) -> 00E1AA, the envelope
    //
    // Both dispatches go through vectors in internal RAM, which is the one part
    // of this path nothing has ever looked at.
    // docs/bgm_investigation.md section 16.5.
    output wire [15:0] dbg_env_fl,   // {$14,X, $15,X}  -- fraction : level
    output wire [15:0] dbg_chanlist, //  esram[0x6420]  -- list head, MAME 0x4600
    output wire [15:0] dbg_vec120,   //  iram[0x0120]   -- dispatch, MAME 0xE1AA
    // 2026-08-23, second half.  The volume is computed correctly and lost five
    // instructions later, in the fade:
    //     00DBC3  LDA $06,X    ; the fade control, esram[0x4006]   MAME 0x80
    //     00DBC5  AND #$03
    //     00DBC7  BEQ 00dbef   ;   zero -> clear the attenuation   MAME's path
    //     00DBCC  LDA $d7      ;   otherwise attenuation = $d7     board 0x50
    //     00E25A  SBCB $05,X   ; volume - attenuation
    //     00E260  LDB #$00     ;   borrowed -> silence
    // MAME never reaches 0x00DBCC in any of five captures.  $d7 is READ there
    // and written NOWHERE in the C70's ROM, so it arrives as a command
    // parameter -- which is why these two are worth a row.
    output wire [15:0] dbg_fade,     // {esram[0x4006], iram[0x00D7]}
    // A WORKAROUND, and it is on by default.  It makes esram[0x4005] -- the
    // attenuation 0x00E25A subtracts -- read as zero, so a volume can never be
    // turned into silence by the borrow at 0x00E260.
    //
    // Why it is defensible rather than a patch over ignorance: the cause is
    // known in full (section 21), and MAME reaches the instruction that
    // installs that attenuation, 0x00DBCC, in NONE of five captures while its
    // melody voice stays audible 99.3 % of attract.  The reference
    // implementation never takes this path, so removing it costs nothing that
    // can be observed -- and with it in place the board's melody plays.
    //
    // It is still a workaround.  It goes when the window model is fixed.
    input  wire        snd_fade_off,
    output wire        dbg_halted,     // the core met an opcode it cannot decode
    output wire        dbg_unmapped,   // an access outside the map
    output wire [23:0] dbg_unmapped_a,
    output wire [15:0] dbg_mbx4        // last doorbell word
);

  // ---------------------------------------------------------------------
  // MCU clock enable.  One system clock in CLK_DIV, gated by ce_en.
  // ---------------------------------------------------------------------
  reg [$clog2(CLK_DIV)-1:0] div;
  wire mcu_ce = ce_en && (div == 0);

  always @(posedge clk) begin
    if (rst)        div <= '0;
    else if (ce_en) div <= (div == CLK_DIV - 1) ? '0 : div + 1'b1;
  end

  // ---------------------------------------------------------------------
  // The CPU's byte bus.  docs/m37702_integration_spec.md section 3.
  // ---------------------------------------------------------------------
  wire [23:0] bus_addr;
  wire  [7:0] bus_dout;
  reg   [7:0] bus_din;
  wire        bus_rd, bus_wr;
  wire        bus_word;      // this access is one half of a 16-bit operand
  wire        bus_ready;
  wire        bus_active = bus_rd | bus_wr;

  reg  [23:0] addr_prev;     // the address one MCU cycle ago (data-valid gate)
  reg   [7:0] share_q;       // the byte a shared-RAM read returned
  reg  [23:0] ext_addr_l;    // the address that shared-RAM access was for
  reg         ext_we_l;      // ... and whether it was a write
  reg         ext_active, ext_done;
  reg         cmd_pending;   // mailbox slot 2 doorbell status, see below

  // ---------------------------------------------------------------------
  // Region decode
  // ---------------------------------------------------------------------
  wire [7:0]  a_hi   = bus_addr[23:16];
  wire [15:0] a_lo   = bus_addr[15:0];
  wire        bank0  = (a_hi == 8'h00);

  wire sel_sfr    = bank0 && (a_lo <  16'h0080);
  wire sel_iram   = bank0 && (a_lo >= 16'h0080) && (a_lo < 16'h0280);
  wire sel_mbox   = bank0 && (a_lo >= 16'h0800) && (a_lo < 16'h1000);
  // 0x1000-0x11FF, .mirror(0x000E00) -> the whole 0x1000-0x1FFF page decodes
  // to the same 512 registers.  MAME_VERIFIED (namcona1_mcu_map).
  wire sel_c219   = bank0 && (a_lo[15:12] == 4'h1);
  wire sel_shm4k  = bank0 && (a_lo >= 16'h2000) && (a_lo < 16'h3000);
  wire sel_esram  = bank0 && (a_lo >= 16'h3000) && (a_lo < 16'hB000);
  wire sel_rom    = bank0 && (a_lo >= 16'hC000);
  wire sel_shram  = (a_hi >= 8'h20) && (a_hi < 8'h28);
  // Bank 0x40: a second window onto the same shared work RAM, and the one the
  // BIOS probes for presence at 0x4007FE (0xC67D writes 0x12, 0xC6A0 reads it
  // back and increments an error counter at 0x01C9 if it does not match).
  // Command 7's destination bank field is 0x40 as well.
  // ROM_DISASM_CONFIRMED that it must answer; INFERRED that it is the whole
  // 512 KiB rather than a smaller aperture.  docs/c70_command7.md section 3.2.
  wire sel_shr40  = (a_hi >= 8'h40) && (a_hi < 8'h48);

  wire sel_ext    = sel_shm4k | sel_shram | sel_shr40;   // all reach work RAM
  wire sel_int    = sel_sfr | sel_iram | sel_mbox | sel_c219 | sel_esram | sel_rom;
  wire sel_any    = sel_int | sel_ext;

  // Shared work RAM byte address.  The 4 KiB mirror is the first page.
  wire [18:0] share_byte = sel_shm4k ? {7'd0, a_lo[11:0]} : bus_addr[18:0];

  // ---------------------------------------------------------------------
  // The byte order of the shared-RAM window, which is not one rule.
  //
  //   8-bit access, anywhere          MCU byte N is 68000 byte N
  //   16-bit access, bank 0x20        the 68000 word arrives byte-swapped
  //   16-bit access, 0x2000 page      byte-swapped too  (MEASURED 2026-08-23,
  //                                   was INFERRED verbatim and was wrong)
  //   16-bit access, bank 0x40        verbatim
  //
  // This looks arbitrary and is not: it is what a 16-bit port whose data path
  // runs straight through, but whose byte-lane steering follows the 68000's
  // convention, does.  docs/c70_command7.md section 3.3.
  //
  // Evidence: bank 0x20's swap is ROM_DISASM_CONFIRMED from the boot-vector
  // deposit (RESEARCH.md section 6) -- change it and the 68000 stops booting.
  // The verbatim rule is INFERRED from ten fields across the two mailbox
  // descriptors the game builds, all of which agree, plus the version string
  // the game compares against.  MAME implements one uniform swapped rule and
  // that is exactly why its C70 "never responds to the version string
  // command": with the words swapped, command 7 reads src=0x00C0 len=0x1000
  // dst=0x0010 instead of src=0xC000 len=0x0010 dst=0x1000.
  //
  // If this is wrong the failure is a hang at 0xC2DA82, not a glitch.
  //
  // --- 2026-08-23: the 0x2000 page is NOT the same window as bank 0x40 ------
  // Section 3.2 above put them together and tagged it INFERRED, from the BIOS
  // probing bank 0x40 in the routine that then writes descriptors into $2Fxx.
  // The page has now been OBSERVED, in MAME, both ways round in the same frame
  // (tools/bgm_lane.lua): a 16-bit MCU read at 0x2840 yields 0x2820 where the
  // 68000's own bytes at 0x0840/41 are 0x20 0x28.  That is the SAME rule as an
  // 8-bit access -- MCU byte N is 68000 byte N -- not the verbatim word.
  //
  // And the board says the same thing.  The sound driver reads that very
  // pointer and follows it:
  //
  //     00D9DB  LDA $20,X    ; X = 0x2820 -- 16-bit, through this page
  //     00D9DD  TAX
  //     00D9DF  LDB $01,X    ; the fade control byte
  //     00D9E8  STB $06,X    ; -> esram[0x4006]
  //
  // Flipping gave 0x2028 instead of 0x2820, so the driver read its fade byte
  // out of the wrong block: 0x85 where MAME reads 0x80, whose low bits put it
  // into a fade, whose attenuation of 0x50 turned every melody volume into
  // silence.  docs/bgm_investigation.md sections 18-19.
  //
  // Bank 0x40 keeps the flip: command 7's descriptor and the version string it
  // answers with are ROM_DISASM_CONFIRMED through that bank, and the boot
  // simulation checks them (sim/run_c70_boot.sh, CMD7=1).
  // The window's byte-lane rule lives below, after acc_done: ModelSim wants
  // every identifier declared before it is used, and the alignment latch
  // needs the access-complete strobe.


  // ---------------------------------------------------------------------
  // Ready.  Internal targets answer once the address has been stable for one
  // MCU cycle; shared RAM answers when the arbiter has answered.
  // ---------------------------------------------------------------------
  wire int_ready = sel_int && bus_active && (bus_addr == addr_prev);
  // The direction has to match as well as the address.  Without it, a write to
  // the byte just read -- which is what every read-modify-write on work RAM
  // does -- would be told "ready" by the read that preceded it and would never
  // reach the arbiter at all.  The data loss would be silent.
  wire ext_ready = sel_ext && bus_active && ext_done
                   && (bus_addr == ext_addr_l) && (bus_wr == ext_we_l);
  assign bus_ready = sel_ext ? ext_ready : int_ready;

  // The single MCU cycle on which an access completes.  Writes happen here,
  // exactly once, and this is what makes snd_reg_we a clean one-shot.
  wire acc_done = mcu_ce & bus_ready;
  wire wr_done  = acc_done & bus_wr;

  always @(posedge clk) begin
    if (rst)          addr_prev <= 24'hFFFFFF;   // != any first address
    else if (mcu_ce)  addr_prev <= bus_addr;
  end

  // --- 2026-08-23: the rule is ALIGNMENT, and that resolves it -------------
  // Section 21 recorded this window as needing two contradictory rules at once
  // -- the flip for command 7's descriptor at MCU 0x2F34, no flip for the
  // sound driver's fade byte at MCU 0x2821 -- and could not find what told the
  // two apart.  It is the alignment of the 16-bit access, and it is in the
  // M37702's datasheet: on a 16-bit external bus an EVEN-addressed 16-bit
  // operation accesses both bytes simultaneously, and when that condition is
  // not met the bus interface accesses them individually under A0 and BHE.
  //
  //   even-aligned 16-bit   one word transaction   -> the 68000's word verbatim
  //   odd-aligned  16-bit   two byte transactions  -> byte identity, like 8-bit
  //   8-bit                 one byte transaction   -> byte identity
  //
  // One rule, told apart by something this wrapper can see.
  //
  //   00E2xx  command 7's descriptor   MCU 0x2F34, EVEN -> flip, as before
  //   00DC0C  LDA $01,X                MCU 0x2821, ODD  -> no flip
  //
  // What the old rule cost: 0x00DC0C feeds `(A & 0x0F) | 0x80` into the sound
  // driver's fade control.  The 68000 writes the pair {song id, 0x40} and the
  // flag is the literal 0x40, so the answer is always 0x80.  Flipped, the odd
  // access took A's low byte from the song id instead -- 0x35 for one of the
  // tunes -- giving 0x85, whose low bits put the driver into a fade with
  // attenuation 0x50, which silenced every melody voice under it.
  // docs/bgm_investigation.md sections 18-23.
  //
  // The wrapper sees one byte cycle at a time and the core emits `ea` then
  // `ea+1`, so the start alignment has to be latched on the first cycle: a
  // second cycle is the one whose address is exactly one above the cycle that
  // just completed.
  reg        w_start_odd;
  reg [23:0] w_prev_addr;
  reg        w_prev_word;
  wire       w_second = bus_word & w_prev_word & (bus_addr == w_prev_addr + 24'd1);
  wire       w_odd    = w_second ? w_start_odd : bus_addr[0];

  always @(posedge clk) begin
    if (rst) begin
      w_start_odd <= 1'b0;
      w_prev_addr <= 24'hFFFFFF;
      w_prev_word <= 1'b0;
    end else if (acc_done) begin
      w_prev_addr <= bus_addr;
      w_prev_word <= bus_word;
      if (!w_second) w_start_odd <= bus_addr[0];
    end
  end

  wire lane_flip = bus_word & (sel_shm4k | sel_shr40) & ~w_odd;
  wire lane_odd  = share_byte[0] ^ lane_flip;

  // ---------------------------------------------------------------------
  // Internal RAM, 512 B
  // ---------------------------------------------------------------------
  reg [7:0] iram[0:511];
  reg [7:0] iram_q;
  wire [8:0] iram_a = a_lo[8:0] - 9'h80;
  // MCU 0x0120, the vector the channel dispatch jumps through.  iram covers
  // 0x0080-0x027F, so the index is the address less 0x80.  MAME 0xE1AA.
  assign dbg_vec120 = {iram[9'h0A1], iram[9'h0A0]};
  always @(posedge clk) begin
    if (sel_iram && wr_done) iram[iram_a] <= bus_dout;
    iram_q <= iram[iram_a];
  end

  // ---------------------------------------------------------------------
  // External SRAM, 32 KiB  (MCU 0x3000-0xAFFF)
  // ---------------------------------------------------------------------
  reg [7:0] esram[0:32767];
  reg [7:0] esram_q;
  // The melody channel's block, MCU 0x4600 + field, esram index = addr - 0x3000.
  // Quartus keeps the RAM in M10K and mirrors each constant-address byte into
  // eight registers (see esram[5653] in the fit report), so these cost about
  // sixty registers between them and no memory.
  assign dbg_chan     = {esram[15'h1630], esram[15'h1615]};
  assign dbg_env_tl   = {esram[15'h1618], esram[15'h1615]};
  assign dbg_env_ptr  = {esram[15'h1613], esram[15'h1612]};   // little-endian
  assign dbg_env_rate = {esram[15'h1617], esram[15'h1616]};   // little-endian
  assign dbg_env_act  = {esram[15'h1608], esram[15'h162D]};
  assign dbg_fade     = {esram[15'h1006], iram[9'h057]};   // 0x4006, 0x00D7
  assign dbg_env_fl   = {esram[15'h1614], esram[15'h1615]};
  // MCU 0x6420, the channel list's first entry.  Little-endian, MAME 0x4600.
  assign dbg_chanlist = {esram[15'h3421], esram[15'h3420]};
  wire [14:0] esram_a = a_lo[14:0] - 15'h3000;
  // The probe.  Only the one byte 0x00E25A subtracts, and only while the OSD
  // switch is on; everything else reads the memory as it stands.
  //
  // It has to substitute AFTER the read, not inside it.  Putting the condition
  // in the same expression as `esram[esram_a]` stops Quartus inferring the RAM
  // at all: the 32 KiB became 32,768 registers, 26,859 -> 288,834, which does
  // not fit on the device.  The map report caught it before the fitter spent
  // twenty minutes finding out.
  reg fade_hit;
  always @(posedge clk) begin
    if (sel_esram && wr_done) esram[esram_a] <= bus_dout;
    esram_q  <= esram[esram_a];
    fade_hit <= snd_fade_off && (esram_a == 15'h1005);
  end
  wire [7:0] esram_rd = fade_hit ? 8'h00 : esram_q;

  // ---------------------------------------------------------------------
  // C219 registers.  Write-through to the chip, with a shadow copy so the
  // BIOS can read back what it wrote -- na2_c219 has no read port, and a
  // silent zero on read would be a bug that only shows up as wrong music.
  // UNKNOWN whether the real chip reads its register file back; the shadow is
  // the safe assumption and is tagged as such.
  // ---------------------------------------------------------------------
  wire [8:0] c219_i = a_lo[8:0] ^ 9'd1;      // little-endian view, MAME_VERIFIED
  reg [7:0] c219sh[0:511];
  reg [7:0] c219sh_q;
  always @(posedge clk) begin
    if (sel_c219 && wr_done) c219sh[c219_i] <= bus_dout;
    c219sh_q <= c219sh[c219_i];
  end

  // Reading a voice's mode register (offset 5 of each 16-byte voice block)
  // does NOT return what was written: the chip reports the key state in bit 6
  // and keeps only the low six bits of the written value.
  // MAME_VERIFIED, c140.cpp: data = (v.key ? 0x40 : 0) | (REG & 0x3F).
  // The C70 BIOS's power-on register test depends on it -- with a plain
  // write-through shadow the RTL diverged from MAME at instruction 2,377,
  // taking a branch the real BIOS does not (docs/c70_rtl_boot.md section 5.3).
  wire       c219_is_mode = (c219_i[8] == 1'b0) && (c219_i[3:0] == 4'h5);
  wire [3:0] c219_voice   = c219_i[7:4];
  wire [7:0] c219_rd      = c219_is_mode
                          ? {1'b0, snd_voice_key[c219_voice], c219sh_q[5:0]}
                          : c219sh_q;

  assign snd_reg_we   = sel_c219 & wr_done;
  assign snd_reg_addr = c219_i;
  assign snd_reg_din  = bus_dout;

  // ---------------------------------------------------------------------
  // Mailbox: 8 x 16 bit, written from both sides.
  // MAME_VERIFIED: mcu_mailbox_r/w use (offset & 7); a write from the 68000
  // to slot 4 raises MCU IRQ0 (mcu_mailbox_w_68k -> M37710_LINE_IRQ0).
  // ---------------------------------------------------------------------
  reg [15:0] mailbox[0:7];
  assign mbx_dout = mailbox[mbx_addr];

  wire        doorbell  = mbx_sel & mbx_wr & (mbx_addr == 3'd4);
  wire [2:0]  mcu_mbx_i = a_lo[3:1];
  reg  [15:0] mbx_q;

  integer mi;
  always @(posedge clk) begin
    if (rst) begin
      for (mi = 0; mi < 8; mi = mi + 1) mailbox[mi] <= 16'h0000;
    end else begin
      // 68000 side, byte enables
      if (mbx_sel && mbx_wr) begin
        if (mbx_ds[1]) mailbox[mbx_addr][15:8] <= mbx_din[15:8];
        if (mbx_ds[0]) mailbox[mbx_addr][ 7:0] <= mbx_din[ 7:0];
      end
      // MCU side, one byte at a time.  The MCU's view is little-endian, so
      // its byte at an even address is the low half of the 16-bit slot.
      if (sel_mbox && wr_done) begin
        if (a_lo[0]) mailbox[mcu_mbx_i][15:8] <= bus_dout;
        else         mailbox[mcu_mbx_i][ 7:0] <= bus_dout;
      end
    end
    mbx_q <= mailbox[mcu_mbx_i];
  end

  // ---------------------------------------------------------------------
  // Mailbox slot 2 is not a shared register: it is doorbell status.
  //
  // The BIOS's INT0 handler at 0xC23A reads MCU word 0x0804, masks 0x00C0 and
  // dispatches four ways; with both bits clear it does nothing at all and
  // returns.  Nothing in either ROM ever writes that word, so it cannot be
  // plain shared RAM.  MAME models the mailbox as eight symmetric 16-bit
  // slots, reads 0x0000 here, and therefore never services a single command --
  // which is what its own FIXME in mcu_mailbox_w_68k is describing.
  //
  // MEASURED by injection (tools/c70_mbox_probe.lua, docs/c70_bios_map.md
  // section 3.5): forcing slot 2 to 0x40 as the doorbell is written makes the
  // MCU clear the command byte at 0x0F72 and the descriptor status; leaving it
  // 0x00 never does, over 400 frames.
  //
  // INFERRED: that the bit clears when the MCU reads the slot.  Clear-on-
  // service would fit the evidence equally -- the handler reads the word
  // exactly once per interrupt -- and clear-on-read is the one that cannot
  // re-enter.  Bit 7 selects a second service (0xCE53, a BCD coin/credit path)
  // that Numan Athletics never uses; it stays 0.
  // ---------------------------------------------------------------------
  wire mcu_rd_slot2 = sel_mbox & acc_done & ~bus_wr & (mcu_mbx_i == 3'd2);

  always @(posedge clk) begin
    if (rst)               cmd_pending <= 1'b0;
    else if (doorbell)     cmd_pending <= 1'b1;
    else if (mcu_rd_slot2) cmd_pending <= 1'b0;
  end

  assign dbg_mbx4 = mailbox[4];

  // ---------------------------------------------------------------------
  // On-chip peripherals.  One interrupt line per source plus that source's
  // priority, which is the shape the CPU core takes (it does its own IPL
  // comparison internally).
  // ---------------------------------------------------------------------
  wire [7:0] sfr_q;
  wire       irq0, irq1, irq2;
  wire       tick_ta0, tick_ta1, tick_ta2, tick_ta3, tick_ta4;
  wire       tick_tb0, tick_tb1, tick_tb2, tick_ad;
  wire [2:0] prio_int0, prio_int1, prio_int2;
  wire [2:0] prio_ta0, prio_ta1, prio_ta2, prio_ta3, prio_ta4;
  wire [2:0] prio_tb0, prio_tb1, prio_tb2, prio_ad;

  // ---------------------------------------------------------------------
  // The C70's software reset.
  //
  // na2_m37702_periph raises soft_rst when the BIOS writes bit 3 of the
  // processor mode register -- its dead-man switch on the 68000 firing.  See
  // the long note in that file; in short, a board that cannot do this needs a
  // canned EEPROM for every set that programs its own, which is exactly the
  // list in MAME's To-Do.
  //
  // Stretched to 1024 clocks so the CPU and the peripheral block both see a
  // clean asynchronous-length reset rather than a single-clock blip that the
  // MCU clock enable could step over.  Work RAM is deliberately NOT cleared:
  // a real reset does not clear DRAM, and the BIOS marches through all 512 KiB
  // itself on the way back up.  The EEPROM the game has just programmed is
  // outside this module and survives, which is the entire point.
  // ---------------------------------------------------------------------
  wire        soft_rst;
  reg  [9:0]  soft_rst_cnt;

  always @(posedge clk) begin
    if (rst)                  soft_rst_cnt <= 10'd0;
    else if (soft_rst)        soft_rst_cnt <= 10'd1023;
    else if (|soft_rst_cnt)   soft_rst_cnt <= soft_rst_cnt - 10'd1;
  end

  wire mcu_rst = rst | (|soft_rst_cnt);

  na2_m37702_periph u_periph (
      .clk        (clk),
      .rst        (mcu_rst),
      .ce         (mcu_ce),

      .sel        (sel_sfr),
      .we         (sel_sfr & wr_done),
      .addr       (a_lo[6:0]),
      .din        (bus_dout),
      .dout       (sfr_q),

      .in_p1      (in_p1),
      .in_p2      (in_p2),
      .in_p4      (in_p4),
      .in_p3      (in_p3),
      .in_dsw     (in_dsw),

      .irq0_set   (doorbell),
      .irq1_set   (vblank_rise),

      .m68k_rst   (m68k_rst),
      .soft_rst   (soft_rst),

      .irq0       (irq0),
      .irq1       (irq1),
      .irq2       (irq2),
      .tick_ta0   (tick_ta0),
      .tick_ta1   (tick_ta1),
      .tick_ta2   (tick_ta2),
      .tick_ta3   (tick_ta3),
      .tick_ta4   (tick_ta4),
      .tick_tb0   (tick_tb0),
      .tick_tb1   (tick_tb1),
      .tick_tb2   (tick_tb2),
      .tick_ad    (tick_ad),
      .prio_int0  (prio_int0),
      .prio_int1  (prio_int1),
      .prio_int2  (prio_int2),
      .prio_ta0   (prio_ta0),
      .prio_ta1   (prio_ta1),
      .prio_ta2   (prio_ta2),
      .prio_ta3   (prio_ta3),
      .prio_ta4   (prio_ta4),
      .prio_tb0   (prio_tb0),
      .prio_tb1   (prio_tb1),
      .prio_tb2   (prio_tb2),
      .prio_ad    (prio_ad)
  );

  // ---------------------------------------------------------------------
  // BIOS read port
  // ---------------------------------------------------------------------
  assign bios_addr = a_lo[13:0];

  // ---------------------------------------------------------------------
  // Shared work RAM, through the main-memory arbiter.
  // ---------------------------------------------------------------------
  assign ram_busy = ext_active;

  always @(posedge clk) begin
    if (rst) begin
      ram_req    <= 1'b0;
      ram_we     <= 1'b0;
      ext_active <= 1'b0;
      ext_done   <= 1'b0;
    end else begin
      if (!ext_active && !ext_done && sel_ext && bus_active && ce_en) begin
        ram_req    <= 1'b1;
        ram_we     <= bus_wr;
        ram_addr   <= share_byte[18:1];
        // Which half of the 68000 word this byte is.  lane_odd carries the
        // 8-bit/16-bit rule above.
        ram_ds     <= lane_odd ? 2'b01 : 2'b10;
        ram_dout   <= {bus_dout, bus_dout};
        ext_addr_l <= bus_addr;
        ext_we_l   <= bus_wr;
        ext_active <= 1'b1;
      end

      if (ext_active && ram_ack) begin
        share_q    <= lane_odd ? ram_din[7:0] : ram_din[15:8];
        ram_req    <= 1'b0;
        ram_we     <= 1'b0;
        ext_active <= 1'b0;
        ext_done   <= 1'b1;
      end

      // Release once the CPU has taken it and moved on.  Without this a
      // second read of the same address would complete instantly on the
      // first one's data.
      if (ext_done && (!bus_active || (bus_addr != ext_addr_l)
                                    || (bus_wr != ext_we_l))) ext_done <= 1'b0;
    end
  end

  // ---------------------------------------------------------------------
  // Read data mux
  // ---------------------------------------------------------------------
  always @(*) begin
    bus_din = 8'h00;
    if      (sel_sfr)   bus_din = sfr_q;
    else if (sel_iram)  bus_din = iram_q;
    else if (sel_mbox)  bus_din = (mcu_mbx_i == 3'd2)
                                    ? (a_lo[0] ? 8'h00 : {1'b0, cmd_pending, 6'h00})
                                    : (a_lo[0] ? mbx_q[15:8] : mbx_q[7:0]);
    else if (sel_c219)  bus_din = c219_rd;
    else if (sel_esram) bus_din = esram_rd;
    else if (sel_rom)   bus_din = bios_data;
    else if (sel_ext)   bus_din = share_q;
  end

  // ---------------------------------------------------------------------
  // Unmapped access reporting.  Project rule: never silently return 0.
  // ---------------------------------------------------------------------
  assign dbg_unmapped   = mcu_ce & bus_active & ~sel_any;
  assign dbg_unmapped_a = bus_addr;

`ifndef SYNTHESIS
  always @(posedge clk) if (mcu_ce && bus_active && !sel_any)
    $display("[na2_c70] unmapped MCU access %s %06h",
             bus_wr ? "write" : "read", bus_addr);
`endif

  // ---------------------------------------------------------------------
  // The CPU.  VHDL, vendored -- LICENSE_NOTES.md, docs/m37702_survey.md.
  // ---------------------------------------------------------------------
  na2_m37702_cpu u_cpu (
      .clk        (clk),
      .ce         (mcu_ce),
      .reset      (mcu_rst),

      .bus_addr   (bus_addr),
      .bus_dout   (bus_dout),
      .bus_din    (bus_din),
      .bus_rd     (bus_rd),
      .bus_wr     (bus_wr),
      .bus_ready  (bus_ready),
      .bus_word   (bus_word),

      .irq0       (irq0),
      .irq1       (irq1),
      .irq2       (irq2),
      .irq_ta0    (tick_ta0),
      .irq_ta1    (tick_ta1),
      .irq_ta2    (tick_ta2),
      .irq_ta3    (tick_ta3),
      .irq_ta4    (tick_ta4),
      .irq_tb0    (tick_tb0),
      .irq_tb1    (tick_tb1),
      .irq_tb2    (tick_tb2),
      .irq_ad     (tick_ad),

      .prio_int0  (prio_int0),
      .prio_int1  (prio_int1),
      .prio_int2  (prio_int2),
      .prio_ta0   (prio_ta0),
      .prio_ta1   (prio_ta1),
      .prio_ta2   (prio_ta2),
      .prio_ta3   (prio_ta3),
      .prio_ta4   (prio_ta4),
      .prio_tb0   (prio_tb0),
      .prio_tb1   (prio_tb1),
      .prio_tb2   (prio_tb2),
      .prio_ad    (prio_ad),

      .dbg_pc     (dbg_pc),
      .dbg_opcode (dbg_opcode),
      .dbg_valid  (dbg_valid),
      .dbg_regA   (dbg_regA),
      .dbg_regB   (dbg_regB),
      .dbg_regX   (dbg_regX),
      .dbg_halted (dbg_halted),
      .dbg_x      ()
  );

endmodule

`default_nettype wire
