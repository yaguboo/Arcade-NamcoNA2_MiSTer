//============================================================================
//  Namco NA-2 -- main-memory bus adapter
//
//  Sits between na2_top's single `mem_*` port and the SDRAM controller, and
//  owns the ROM download path.  Also holds the C70 BIOS in block RAM, because
//  the C70 model reads it as a byte at a time with no handshake.
//
//  SDRAM layout, in 16-bit words.  The download image produced by
//  tools/build_rom.py is copied in verbatim, so a download byte address maps
//  to SDRAM word (byte >> 1) and the region bases fall out of
//  games/numanath/romset.json "download_map" unchanged:
//
//     SDRAM word          bytes        contents            download bytes
//     0x000000-0x0FFFFF   2 MiB        68000 program ROM   0x000000-0x1FFFFF
//     0x100000-0x4FFFFF   8 MiB        mask ROM            0x200000-0x9FFFFF
//     0x500000-0x501FFF   16 KiB       C70 BIOS (also BRAM)0xA00000-0xA03FFF
//     0x520000-0x55FFFF   512 KiB      shared work RAM     (not downloaded)
//
//  68000 address -> SDRAM word:
//     work RAM 0x000000-0x07FFFF  ->  0x520000 + a[18:1]
//     mask ROM 0x400000-0xBFFFFF  ->  0x100000 + (a - 0x400000) >> 1
//     prg  ROM 0xC00000-0xDFFFFF  ->  0x000000 + (a - 0xC00000) >> 1
//
//  Byte order: the download image is a stream of 68000 big-endian words, so
//  the byte at an even address is the HIGH half of its word.  The assembler
//  below latches the even byte and writes the pair on the odd byte, which
//  keeps SDRAM contents identical to what sim/na2_mem_model.sv loads.  Get
//  this wrong and the reset vector check in tools/build_rom.py would still
//  pass while the core booted garbage -- hence the explicit note.
//============================================================================
`default_nettype none

module na2_membus #(
    // Where the self-test writes its pattern.  Only overridden by
    // sim/tb_na2_sdram.sv, whose SDRAM model has a small row window.
    parameter [24:0] ST_BASE = 25'h0520000
) (
    input  wire        clk,
    input  wire        rst,

    // --- core side (na2_top mem_* port) ------------------------------------
    input  wire [23:1] mem_addr,
    input  wire [15:0] mem_dout,
    output wire [15:0] mem_din,
    input  wire        mem_req,
    input  wire        mem_we,
    input  wire [1:0]  mem_ds,
    output wire        mem_ack,

    // --- C70 BIOS read port -------------------------------------------------
    input  wire [13:0] bios_addr,
    output reg  [7:0]  bios_data,

    // --- ROM download (hps_io ioctl, 8-bit) --------------------------------
    input  wire        ld_download,
    input  wire        ld_wr,
    input  wire [24:0] ld_addr,
    input  wire [7:0]  ld_data,
    // Backpressure to the HPS.  hps_io drives this onto HPS_BUS[37]; while it
    // is high the HPS stops sending.  Without it the HPS streams faster than
    // SDRAM can absorb -- and, worse, leaving the port unconnected made the
    // HPS wait forever and hung the whole load.  See KNOWN_ISSUES.md.
    output wire        ld_wait,

    // --- post-download SDRAM self-test  *** TEMPORARY *** ------------------
    output wire        st_busy,      // hold the core in reset while this is up
    output wire [15:0] st_word0,     // SDRAM word 0, must read 0x46FC
    output wire [15:0] st_errs,      // work-RAM read-back mismatches, word writes
    output wire [15:0] st_berrs,     // ... and with byte writes
    output reg  [15:0] st_dlwords,   // words this module actually wrote, saturating

    // --- SDRAM controller port ----------------------------------------------
    output wire [24:0] sd_addr,
    output wire [15:0] sd_din,
    input  wire [15:0] sd_dout,
    output wire        sd_req,
    output wire        sd_we,
    output wire [1:0]  sd_ds,
    input  wire        sd_ack
);

  // ---------------------------------------------------------------------
  // Region bases, in SDRAM words
  // ---------------------------------------------------------------------
  localparam [24:0] SD_PRG   = 25'h0000000;
  localparam [24:0] SD_MASK  = 25'h0100000;
  localparam [24:0] SD_WORK  = 25'h0520000;

  localparam [24:0] DL_C70_BASE = 25'h0A00000;
  localparam [24:0] DL_C70_END  = 25'h0A04000;

  // ---------------------------------------------------------------------
  // 68000 address -> SDRAM word
  // ---------------------------------------------------------------------
  wire [23:0] a24 = {mem_addr, 1'b0};
  wire [24:0] a_word = {2'd0, mem_addr};        // full 23-bit word address
  wire is_work = (a24 <  24'h080000);
  wire is_mask = (a24 >= 24'h400000) && (a24 < 24'hC00000);

  //  work RAM  word 0x000000-0x03FFFF -> 0x520000 + word
  //  mask ROM  word 0x200000-0x5FFFFF -> 0x100000 + (word - 0x200000)
  //  prg  ROM  word 0x600000-0x6FFFFF -> 0x000000 + (word - 0x600000)
  wire [24:0] core_word = is_work ? (SD_WORK + a_word)
                        : is_mask ? (SD_MASK + a_word - 25'h0200000)
                                  : (SD_PRG  + a_word - 25'h0600000);

  // ---------------------------------------------------------------------
  // Download: assemble big-endian byte pairs into words
  // ---------------------------------------------------------------------
  reg [7:0]  dl_hi;
  // Power-up value is not optional here.  ld_wait goes onto HPS_BUS[37],
  // which sys_top.v calls io_wait, and io_wait high stops sys_top from ever
  // raising io_ack.  MiSTer's main spins on io_ack for every single word it
  // sends the core, so a dl_req that comes out of configuration set wedges the
  // HPS before the core has done anything at all -- and the project sets
  // ALLOW_POWER_UP_DONT_CARE, so an uninitialised register is genuinely free to
  // power up either way.  Nothing else in this core has that reach.
  reg        dl_req = 1'b0;
  reg [24:0] dl_word;
  reg [15:0] dl_data;

  wire in_c70 = (ld_addr >= DL_C70_BASE) && (ld_addr < DL_C70_END);

  // C70 BIOS block RAM, 16 KiB
  reg [7:0] bios[0:16383];
  always @(posedge clk) begin
    if (ld_wr && in_c70) bios[ld_addr[13:0]] <= ld_data;
    bios_data <= bios[bios_addr];
  end

  always @(posedge clk) begin
    if (rst) begin
      dl_req <= 1'b0;
    end else begin
      if (ld_wr && ld_download) begin
        if (!ld_addr[0])
          dl_hi <= ld_data;                 // even byte = high half
        else begin
          dl_word <= {1'b0, ld_addr[24:1]};
          dl_data <= {dl_hi, ld_data};
          dl_req  <= 1'b1;
        end
      end
      if (dl_req && sd_ack) dl_req <= 1'b0;
    end
  end

  // How many words this module actually put into SDRAM.  If the download is
  // reaching the FPGA but this stays at zero, the gate in NamcoNA2.sv is
  // rejecting it -- which is a different fault from the memory being broken.
  always @(posedge clk) begin
    if (rst) st_dlwords <= 16'd0;
    else if (dl_req && sd_ack && !(&st_dlwords)) st_dlwords <= st_dlwords + 16'd1;
  end

  // A word is assembled and waiting for SDRAM: hold the HPS off.
  assign ld_wait = dl_req;

  // ---------------------------------------------------------------------
  // Post-download SDRAM self-test          *** TEMPORARY, see KNOWN_ISSUES ***
  //
  // sim/tb_na2_sdram.sv proves the controller's *protocol* against a
  // datasheet-faithful model.  It cannot prove the board: tAC plus board delay
  // against the FPGA's setup window, and how many column bits the fitted
  // module actually has.  na2_sdram maps {row[12:0], bank[1:0], col[9:0]},
  // i.e. it assumes 10 column bits; a 32 MB module has 9 and would ignore A9,
  // aliasing every 512-word block onto its neighbour.
  //
  // So check it on the hardware, once, before the core is let go:
  //   * word 0 of the program ROM must read 0x46FC -- the first instruction
  //     sim/tb_na2_boot.sv fetches at 0xC00000
  //   * then write a pattern over 256 work-RAM words and read it back
  //
  // The addresses step by 511 words, so consecutive accesses cross rows and
  // banks and address bit 9 changes on every step -- which is exactly the bit
  // a 9-column part would drop.
  // ---------------------------------------------------------------------
  localparam [3:0] T_WAIT = 4'd0, T_ROM = 4'd1, T_ROM_W = 4'd2,
                   T_WR   = 4'd3, T_WR_W = 4'd4,
                   T_RD   = 4'd5, T_RD_W = 4'd6, T_DONE = 4'd7,
                   T_BZ   = 4'd8, T_BZ_W = 4'd9,   // zero the word
                   T_BL   = 4'd10, T_BL_W = 4'd11, // write the low byte only
                   T_BH   = 4'd12, T_BH_W = 4'd13, // write the high byte only
                   T_BR   = 4'd14, T_BR_W = 4'd15; // read it back

  reg [3:0]  t_st = T_WAIT;
  reg [7:0]  t_i;
  reg [24:0] t_addr;
  reg [15:0] t_word0 = 16'h0000;
  reg [15:0] t_errs  = 16'h0000;
  reg [15:0] t_berrs = 16'h0000;
  reg [1:0]  t_ds;
  reg        t_req   = 1'b0;
  reg        t_we;
  reg [15:0] t_din;
  reg        dl_seen = 1'b0;

  function automatic [15:0] t_pat(input [7:0] i);
    t_pat = {i, ~i} ^ 16'h5A3C;
  endfunction

  wire [24:0] t_off  = { 8'd0, t_i, 9'd0} - {17'd0, t_i};    // i * 511
  wire [24:0] t_next = ST_BASE + t_off;

  wire tst_active = (t_st != T_WAIT) && (t_st != T_DONE);
  assign st_busy  = tst_active;
  assign st_word0 = t_word0;
  assign st_errs  = t_errs;
  assign st_berrs = t_berrs;

  always @(posedge clk) begin
    if (rst) begin
      t_st    <= T_WAIT;
      t_req   <= 1'b0;
      dl_seen <= 1'b0;
      t_errs  <= 16'h0000;
      t_berrs <= 16'h0000;
      t_ds    <= 2'b11;
    end else begin
      if (ld_download) dl_seen <= 1'b1;
      case (t_st)
        T_WAIT: if (dl_seen && !ld_download && !dl_req) begin
          t_addr <= SD_PRG;
          t_we   <= 1'b0;
          t_ds   <= 2'b11;
          t_req  <= 1'b1;
          t_st   <= T_ROM_W;
        end
        T_ROM: ;                       // unused, kept so the encoding is stable
        T_ROM_W: if (sd_ack) begin
          t_word0 <= sd_dout;
          t_req   <= 1'b0;
          t_i     <= 8'd0;
          t_st    <= T_WR;
        end
        T_WR: begin
          t_addr <= t_next;
          t_din  <= t_pat(t_i);
          t_we   <= 1'b1;
          t_req  <= 1'b1;
          t_st   <= T_WR_W;
        end
        T_WR_W: if (sd_ack) begin
          t_req <= 1'b0;
          if (t_i == 8'd255) begin t_i <= 8'd0; t_st <= T_RD; end
          else                begin t_i <= t_i + 8'd1; t_st <= T_WR; end
        end
        T_RD: begin
          t_addr <= t_next;
          t_we   <= 1'b0;
          t_req  <= 1'b1;
          t_st   <= T_RD_W;
        end
        T_RD_W: if (sd_ack) begin
          t_req <= 1'b0;
          if (sd_dout !== t_pat(t_i)) t_errs <= t_errs + 16'd1;
          if (t_i == 8'd255) begin t_i <= 8'd0; t_st <= T_BZ; end
          else begin t_i <= t_i + 8'd1; t_st <= T_RD; end
        end

        // ---- byte writes ------------------------------------------------
        // The word test above only ever drives ds = 2'b11.  Every parameter
        // the game hands the C70 is written with MOVE.B, so if the byte lanes
        // do not work on the board the C70 reads zeros and the version check
        // fails -- which is exactly what the overlay showed: cmd7 src 0x0000,
        // dst 0x0000.  Simulation says the byte enables are right
        // (sim/tb_na2_sdram.sv), so this is here to say whether the *board*
        // agrees.
        T_BZ: begin
          t_addr <= t_next; t_din <= 16'h0000; t_we <= 1'b1; t_ds <= 2'b11;
          t_req  <= 1'b1;   t_st  <= T_BZ_W;
        end
        T_BZ_W: if (sd_ack) begin t_req <= 1'b0; t_st <= T_BL; end
        T_BL: begin
          t_addr <= t_next; t_din <= 16'h00A5; t_we <= 1'b1; t_ds <= 2'b01;
          t_req  <= 1'b1;   t_st  <= T_BL_W;
        end
        T_BL_W: if (sd_ack) begin t_req <= 1'b0; t_st <= T_BH; end
        T_BH: begin
          t_addr <= t_next; t_din <= 16'h5A00; t_we <= 1'b1; t_ds <= 2'b10;
          t_req  <= 1'b1;   t_st  <= T_BH_W;
        end
        T_BH_W: if (sd_ack) begin t_req <= 1'b0; t_st <= T_BR; end
        T_BR: begin
          t_addr <= t_next; t_we <= 1'b0; t_ds <= 2'b11;
          t_req  <= 1'b1;   t_st  <= T_BR_W;
        end
        T_BR_W: if (sd_ack) begin
          t_req <= 1'b0;
          if (sd_dout !== 16'h5AA5) t_berrs <= t_berrs + 16'd1;
          if (t_i == 8'd255) t_st <= T_DONE;
          else begin t_i <= t_i + 8'd1; t_st <= T_BZ; end
        end

        default: ;                     // T_DONE: stay here
      endcase
    end
  end

  // ---------------------------------------------------------------------
  // Arbitration: the download owns the bus while it runs, then the self-test,
  // then the core (which is held in reset until both are finished).
  //
  // The download selector is `dl_active`, not `ld_download`, so that a word
  // already assembled keeps the bus until it is acknowledged.  Selecting on
  // ld_download alone drops the last word of every transfer, and worse,
  // leaves dl_req set with nothing left to acknowledge it -- which holds
  // ld_wait high forever and hangs the HPS.
  // ---------------------------------------------------------------------
  wire dl_active = ld_download | dl_req;

  assign sd_addr = dl_active ? dl_word : tst_active ? t_addr : core_word;
  assign sd_din  = dl_active ? dl_data : tst_active ? t_din  : mem_dout;
  assign sd_req  = dl_active ? dl_req  : tst_active ? t_req  : mem_req;
  assign sd_we   = dl_active ? 1'b1    : tst_active ? t_we   : mem_we;
  assign sd_ds   = dl_active ? 2'b11   : tst_active ? t_ds   : mem_ds;

  assign mem_din = sd_dout;
  assign mem_ack = (dl_active || tst_active) ? 1'b0 : sd_ack;

endmodule

`default_nettype wire
