//============================================================================
//  Namco NA-2 -- video-side memories
//
//  Architecture decision (KNOWN_ISSUES.md RISK-1, resolved 2026-08-19):
//  all seven video memories live in on-chip block RAM; only the 512 KiB shared
//  work RAM and the ROMs go to SDRAM.  Budget on a DE10-Nano Cyclone V
//  (553 M10K blocks, x16 mode = 512 words per block):
//
//      palette    0xF00000-0xF01FFF     8 KiB    4096 w    8 blocks
//      cgram      0xF40000-0xF7FFFF   256 KiB  131072 w  256 blocks
//      shaperam   0xF40000-0xF47FFF    32 KiB   16384 w   32 blocks
//      videoram   0xFF0000-0xFFBFFF    48 KiB   24576 w   48 blocks
//      scratch    0xFFC000-0xFFDFFF     8 KiB    4096 w    8 blocks
//      scroll     0xFFE000-0xFFEFFF     4 KiB    2048 w    4 blocks
//      spriteram  0xFFF000-0xFFFFFF     4 KiB    2048 w    4 blocks
//                                     -------            ---------
//                                     360 KiB            360 blocks
//
//  The alternative -- CG RAM in SDRAM -- would have needed a scanline fetch
//  scheduler for a memory that is read once per pixel.  Keeping it on chip
//  makes the renderer a straight single-cycle read, which is worth the space.
//
//  Every memory is true dual port: port A is the CPU/blitter side, port B is
//  the renderer side.  On a real board these are the same physical RAMs being
//  time-shared with the video scan; the renderer never writes, so a dual-port
//  BRAM is a faithful stand-in.
//
//  MAME_VERIFIED: the region boundaries come from namcona1_main_map, and the
//  cgram/shaperam overlay from gfxram_r/w switching on vreg $0C
//  (3 = shape RAM, first 0x8000 bytes only; 2 = CG RAM).
//  FBNEO_CROSSCHECK: d_namcona1.cpp bankswitch() maps DrvShapeRAM over
//  0xF40000-0xF47FFF for bank 3 and DrvGfxRAM over the full window for bank 2,
//  which independently confirms the 32 KiB shape-RAM extent.
//============================================================================
`default_nettype none

// ---------------------------------------------------------------------------
// Simple true dual-port RAM with byte enables on port A.
// ---------------------------------------------------------------------------
module na2_dpram #(
    parameter int AW = 12
) (
    input  wire            clk,
    // port A: read/write (CPU + blitter)
    input  wire [AW-1:0]   a_addr,
    input  wire [15:0]     a_din,
    output wire [15:0]     a_dout,
    input  wire            a_we,
    input  wire [1:0]      a_ds,
    // port B: read only (renderer)
    input  wire [AW-1:0]   b_addr,
    output wire [15:0]     b_dout
);
  // Split into two byte-wide arrays rather than writing bit slices of a
  // 16-bit array.  Both forms are correct, but this one is the shape Quartus
  // reliably infers as byte-enabled block RAM; the bit-slice form risks
  // falling back to logic, and for the 131072-word CG RAM that is fatal.
  // Each array has one write port and two read ports' worth of addressing --
  // port A (read/write) and port B (read) -- which is exactly an M10K in
  // true dual-port mode.
  reg [7:0] mem_hi[0:(1<<AW)-1];
  reg [7:0] mem_lo[0:(1<<AW)-1];

  // synthesis translate_off
  // Simulation only.  On hardware the M10K blocks are already cleared during
  // FPGA configuration, and Quartus refuses elaboration loops longer than
  // 5000 iterations.
  integer ii;
  initial for (ii = 0; ii < (1<<AW); ii = ii + 1) begin
    mem_hi[ii] = 8'h00;
    mem_lo[ii] = 8'h00;
  end
  // synthesis translate_on

  reg [7:0] qa_hi, qa_lo, qb_hi, qb_lo;

  // ONE ALWAYS BLOCK PER PORT.  This is not a style preference.  With the
  // write and both reads in a single block Quartus reads the array as
  // "one write, two reads" and infers *two copies* of it -- the fitter report
  // shows mem_hi_rtl_0 and mem_hi_rtl_1 -- which doubled the block memory to
  // 5.71 Mbit against the 5.66 Mbit the device has, and pushed the small RAMs
  // out into flip-flops entirely (2048 x 16 as 32768 registers, twice).  The
  // design went from 29,827 to 186,429 logic cells and stopped fitting.
  // Split by port, it is Quartus's own true-dual-port template: port A reads
  // and writes, port B reads, one M10K.
  // Port A is write-first (`q <= data` on a write) because that is the shape
  // Quartus's own true-dual-port template has, and it only recognises that
  // one.  With a read-old-data port A it infers "one write, two reads" and
  // duplicates the array instead.  The difference is invisible here: the
  // arbiter never issues a read and a write in the same cycle, so a_dout is
  // only ever consumed for a read.
  always @(posedge clk) begin                       // port A, read/write
    if (a_we && a_ds[1]) begin
      mem_hi[a_addr] <= a_din[15:8];
      qa_hi          <= a_din[15:8];
    end else
      qa_hi <= mem_hi[a_addr];
  end

  always @(posedge clk) begin
    if (a_we && a_ds[0]) begin
      mem_lo[a_addr] <= a_din[7:0];
      qa_lo          <= a_din[7:0];
    end else
      qa_lo <= mem_lo[a_addr];
  end

  always @(posedge clk) qb_hi <= mem_hi[b_addr];    // port B, read only
  always @(posedge clk) qb_lo <= mem_lo[b_addr];

  assign a_dout = {qa_hi, qa_lo};
  assign b_dout = {qb_hi, qb_lo};
endmodule


// ---------------------------------------------------------------------------
// The video memory block
// ---------------------------------------------------------------------------
module na2_vram #(
    // 0 = CG in on-chip block RAM (what this board has always done).
    // 1 = CG in an external store reached through cg_xm_*.  See na2_cgmem.sv.
    parameter bit CG_EXTERNAL = 1'b0,
    // Row buffers in the external CG cache.  Only meaningful when
    // CG_EXTERNAL; see na2_cgmem.sv.
    parameter int CG_ROWS     = 2,
    // Lines in the block-RAM CG row cache.  0 = none.  See na2_cgmem.sv.
    parameter int CG_CLINES   = 0,
    parameter bit CG_PREFETCH = 1'b1,
    parameter int CG_LINE_LOG = 2,
    // A second CG store for ROZ alone, with its own line width.  See
    // na2_video.sv's SPLIT_ROZ_CG for why one width cannot serve both.
    parameter bit CG_SPLIT_ROZ    = 1'b0,
    parameter int CG_ROZ_LINE_LOG = 4,
    parameter int CG_ROZ_CLINES   = 256,
    parameter int CG_ROZ_ROWS     = 16,
    // Set-index fold for the ROZ store's cache.  It had none -- the shared
    // store is given tuned values and this one was left at a plain low-bits
    // index, which put 1,024 of emrot/f720's 1,536 rows in pairs on 512 sets
    // while half the cache went unused.  See na2_cgmem's cfold().
    // Posted port-A writes on the ROZ store.  0 = blocking, as shipped.
    // See na2_cgmem's WRITE_QUEUE: this store's a_stall goes nowhere, so a
    // write it cannot take is lost, and WRITE_FIRST pays to shorten that
    // window out of the renderer's line budget.  A queue answers both.
    parameter int CG_ROZ_WRITE_QUEUE = 0,
    parameter int CG_ROZ_WRITE_QUEUE_URGENT = 0,
    parameter bit CG_ROZ_CACHE_WRITE_THROUGH = 1'b0,
    parameter bit CG_ROZ_CACHE_LOOKUP_AIM = 1'b0,
    parameter int CG_ROZ_INDEX_XOR_HI = 0,
    parameter int CG_ROZ_INDEX_XOR_LO = 0,
    parameter int CG_INDEX_XOR_HI = 0,
    parameter int CG_INDEX_XOR_LO = 0
) (
    input  wire        clk,
    input  wire        rst,

    // --- CPU / blitter side ------------------------------------------------
    input  wire [23:1] a,
    input  wire [15:0] din,
    output reg  [15:0] dout,
    input  wire        we,
    input  wire [1:0]  ds,
    input  wire        sel_pal,
    input  wire        sel_gfx,
    input  wire        sel_vram,
    input  wire        sel_scratch,
    input  wire        sel_scroll,
    input  wire        sel_spr,
    // vreg $0C.  MAME_VERIFIED: 2 = CG RAM, 3 = shape RAM, anything else
    // reads back 0 and swallows writes.
    input  wire [15:0] gfx_bank,
    output wire        dbg_dead_wr,

    // --- renderer side (read only) ----------------------------------------
    input  wire [11:0] r_pal_addr,
    output wire [15:0] r_pal_data,
    // ROZ's port, used only when CG_SPLIT_ROZ is set.
    input  wire [16:0] rz_cg_addr,
    output wire [15:0] rz_cg_data,
    output wire        rz_cg_stall,
    output wire        rz_xm_req,
    output wire        rz_xm_we,
    output wire [16:0] rz_xm_addr,
    output wire [15:0] rz_xm_wdata,
    output wire  [1:0] rz_xm_wstrb,
    input  wire        rz_xm_ack,
    input  wire [(16<<CG_ROZ_LINE_LOG)-1:0] rz_xm_rdata,
    input  wire [16:0] r_cg_addr,
    output wire [15:0] r_cg_data,
    // Raised by the CG backend when the word for r_cg_addr is not ready.  The
    // on-chip backend ties it low; see na2_cgmem.sv.
    output wire        r_cg_stall,
    // Prefetch hint from the renderer; see na2_cgmem.sv.
    input  wire [16:0] r_cg_pf_addr,
    input  wire        r_cg_pf_req,
    // Held high while a 68000/blitter access to CG has not completed.  Always
    // low with the on-chip backend.
    // BOTH STORES, NOT ONE.
    //
    // With CG_SPLIT_ROZ the same port-A write has to land in two places, and
    // this reported only the shared store's progress.  na2_top releases the
    // bus grant on this signal, so once cram0 had taken the word the grant
    // went away -- while the ROZ store could still be mid-fetch and had not
    // seen it.  A testbench with both stores, both controllers and both part
    // models measured 38 of 64 writes never reaching cram1 when ROZ was
    // fetching; with ROZ idle all 64 arrived.  cram1 goes stale exactly when
    // the game rewrites CG with ROZ on, which is when it matters.
    //
    // This costs the CPU at most one ROZ fetch of extra wait per write.
    output wire        cg_a_stall,

    // --- external CG store, only driven when CG_EXTERNAL -------------------
    output wire        cg_xm_req,
    output wire        cg_xm_we,
    output wire [16:0] cg_xm_addr,
    output wire [15:0] cg_xm_wdata,
    output wire  [1:0] cg_xm_wstrb,
    input  wire        cg_xm_ack,
    input  wire [(16<<CG_LINE_LOG)-1:0] cg_xm_rdata,
    output wire [15:0] cg_dbg_row_req,
    output wire [15:0] cg_dbg_row_miss,
    output wire [15:0] cg_dbg_max_wait,
    input  wire [13:0] r_shape_addr,
    output wire [15:0] r_shape_data,
    input  wire [14:0] r_vram_addr,
    output wire [15:0] r_vram_data,
    input  wire [10:0] r_scroll_addr,
    output wire [15:0] r_scroll_data,
    input  wire [10:0] r_spr_addr,
    output wire [15:0] r_spr_data
);

  // gfx window decode: shape RAM only covers the first 0x8000 bytes
  wire [23:0] a24 = {a, 1'b0};
  wire gfx_is_shape = sel_gfx && (gfx_bank == 16'd3) && (a24 < 24'hF48000);
  wire gfx_is_cg    = sel_gfx && (gfx_bank == 16'd2);
  wire gfx_dead     = sel_gfx && !gfx_is_shape && !gfx_is_cg;
  // A write into the CG/shape window that the bank register routes to
  // neither array.  MAME's gfxram_w does the same thing, so this is not a
  // bug by itself -- but a blitter write that lands here is a graphic that
  // never arrives, and that is worth counting rather than assuming.
  assign dbg_dead_wr = we & gfx_dead;

  wire [15:0] q_pal, q_cg, q_shape, q_vram, q_scratch, q_scroll, q_spr;

  na2_dpram #(.AW(12)) u_pal (       // 4096 words
      .clk(clk), .a_addr(a[12:1]), .a_din(din), .a_dout(q_pal),
      .a_we(we & sel_pal), .a_ds(ds), .b_addr(r_pal_addr), .b_dout(r_pal_data));

  // CG goes through na2_cgmem rather than na2_dpram directly: it is the one
  // array that has to be able to live off-chip, and that seam is worth naming.
  wire cg_a_stall_main;
  // THE CPU WAITS FOR THE SHARED STORE ONLY.
  //
  // This ORed in the ROZ store's stall so a write could not be acknowledged
  // until both copies had it.  Correct, and unaffordable: na2_cgmem serves the
  // renderer first and the ROZ store's line is now sixteen words, so a write
  // arriving mid-burst waits for the whole of it.  Measured on em3/f880 at the
  // shipped parameters, one frame of 713,216 clocks:
  //
  //     shared store only          39,429 clocks of port-A stall
  //     both stores               192,641
  //     both stores, WRITE_FIRST  187,724
  //
  // Twenty-six per cent of the frame with the 68000 held.  On hardware that is
  // the watchdog resetting the board in the boot ROM check -- build 35 did
  // exactly that, over and over.  Write priority barely moved it because the
  // wait is not arbitration, it is the sixteen-word burst already in flight.
  //
  // So the two stores can go out of step again, as they have since the split
  // was introduced.  That is a real defect and it is NOT fixed here; the fix
  // is a write holding register on the ROZ store that stalls ROZ rather than
  // the CPU, which is a design change and not a one-line one.  WRITE_FIRST
  // stays on because it shortens the window at no cost to anyone.
  assign cg_a_stall = cg_a_stall_main;

  na2_cgmem #(.AW(17), .EXTERNAL(CG_EXTERNAL), .ROWS(CG_ROWS),
              .CLINES(CG_CLINES), .PREFETCH(CG_PREFETCH),
              .INDEX_XOR_HI(CG_INDEX_XOR_HI),
              .INDEX_XOR_LO(CG_INDEX_XOR_LO),
              .LINE_LOG(CG_LINE_LOG)) u_cg ( // 131072 words
      .clk(clk), .rst(rst),
      .a_addr(a[17:1]), .a_din(din), .a_dout(q_cg),
      .a_we(we & gfx_is_cg), .a_ds(ds),
      .a_sel(gfx_is_cg), .a_stall(cg_a_stall_main),
      .r_addr(r_cg_addr), .r_data(r_cg_data), .r_stall(r_cg_stall),
      .pf_addr(r_cg_pf_addr), .pf_req(r_cg_pf_req),
      .xm_req(cg_xm_req), .xm_we(cg_xm_we), .xm_addr(cg_xm_addr),
      .xm_wdata(cg_xm_wdata), .xm_wstrb(cg_xm_wstrb),
      .xm_ack(cg_xm_ack), .xm_rdata(cg_xm_rdata),
      .dbg_row_req(cg_dbg_row_req), .dbg_row_miss(cg_dbg_row_miss),
      .dbg_max_wait(cg_dbg_max_wait), .dbg_q_drop());

  // ROZ's own CG store.  Same words -- port A writes reach both -- and a
  // different line width, which is the whole point.
  generate
  if (CG_SPLIT_ROZ) begin : g_roz_cg
    na2_cgmem #(.AW(17), .EXTERNAL(CG_EXTERNAL), .ROWS(CG_ROZ_ROWS),
                .CLINES(CG_ROZ_CLINES), .PREFETCH(1'b0), .WRITE_FIRST(1'b1),
                .WRITE_QUEUE(CG_ROZ_WRITE_QUEUE),
                .WRITE_QUEUE_URGENT(CG_ROZ_WRITE_QUEUE_URGENT),
                .CACHE_WRITE_THROUGH(CG_ROZ_CACHE_WRITE_THROUGH),
                .CACHE_LOOKUP_AIM(CG_ROZ_CACHE_LOOKUP_AIM),
                .LINE_LOG(CG_ROZ_LINE_LOG),
                .INDEX_XOR_HI(CG_ROZ_INDEX_XOR_HI),
                .INDEX_XOR_LO(CG_ROZ_INDEX_XOR_LO)) u_rzcg (
        .clk(clk), .rst(rst),
        .a_addr(a[17:1]), .a_din(din), .a_dout(),
        .a_we(we & gfx_is_cg), .a_ds(ds),
        .a_sel(gfx_is_cg), .a_stall(),   // deliberately unused -- see above
        .r_addr(rz_cg_addr), .r_data(rz_cg_data), .r_stall(rz_cg_stall),
        .pf_addr(17'd0), .pf_req(1'b0),
        .xm_req(rz_xm_req), .xm_we(rz_xm_we), .xm_addr(rz_xm_addr),
        .xm_wdata(rz_xm_wdata), .xm_wstrb(rz_xm_wstrb),
        .xm_ack(rz_xm_ack), .xm_rdata(rz_xm_rdata),
        // Left open on purpose.  The gate that chooses the depth reads it
        // by hierarchy in simulation (sim/pocket/run_store_gate.sh); it is
        // worth a wire up to the Pocket diagnostic grid only once a depth
        // has been chosen and is going on a card.
        .dbg_row_req(), .dbg_row_miss(), .dbg_max_wait(), .dbg_q_drop());
  end else begin : g_no_roz_cg
    assign rz_cg_data = 16'd0;   assign rz_cg_stall = 1'b0;
    assign rz_xm_req  = 1'b0;    assign rz_xm_we    = 1'b0;
    assign rz_xm_addr = 17'd0;   assign rz_xm_wdata = 16'd0;
    assign rz_xm_wstrb = 2'd0;
  end
  endgenerate

  na2_dpram #(.AW(14)) u_shape (     // 16384 words
      .clk(clk), .a_addr(a[14:1]), .a_din(din), .a_dout(q_shape),
      .a_we(we & gfx_is_shape), .a_ds(ds), .b_addr(r_shape_addr), .b_dout(r_shape_data));

  na2_dpram #(.AW(15)) u_vram (      // 32768 words allocated, 24576 used
      .clk(clk), .a_addr(a[15:1]), .a_din(din), .a_dout(q_vram),
      .a_we(we & sel_vram), .a_ds(ds), .b_addr(r_vram_addr), .b_dout(r_vram_data));

  na2_dpram #(.AW(12)) u_scratch (   // 4096 words
      .clk(clk), .a_addr(a[12:1]), .a_din(din), .a_dout(q_scratch),
      .a_we(we & sel_scratch), .a_ds(ds), .b_addr(12'd0), .b_dout());

  na2_dpram #(.AW(11)) u_scroll (    // 2048 words
      .clk(clk), .a_addr(a[11:1]), .a_din(din), .a_dout(q_scroll),
      .a_we(we & sel_scroll), .a_ds(ds), .b_addr(r_scroll_addr), .b_dout(r_scroll_data));

  na2_dpram #(.AW(11)) u_spr (       // 2048 words
      .clk(clk), .a_addr(a[11:1]), .a_din(din), .a_dout(q_spr),
      .a_we(we & sel_spr), .a_ds(ds), .b_addr(r_spr_addr), .b_dout(r_spr_data));

  // The BRAMs answer one clock after the address, so the select has to be
  // delayed with them.
  reg s_pal, s_cg, s_shape, s_vram, s_scratch, s_scroll, s_spr, s_dead;
  always @(posedge clk) begin
    s_pal     <= sel_pal;
    s_cg      <= gfx_is_cg;
    s_shape   <= gfx_is_shape;
    s_dead    <= gfx_dead;
    s_vram    <= sel_vram;
    s_scratch <= sel_scratch;
    s_scroll  <= sel_scroll;
    s_spr     <= sel_spr;
  end

  always @(*) begin
    if      (s_pal)     dout = q_pal;
    else if (s_cg)      dout = q_cg;
    else if (s_shape)   dout = q_shape;
    else if (s_dead)    dout = 16'h0000;   // MAME_VERIFIED: gfxram_r returns 0
    else if (s_vram)    dout = q_vram;
    else if (s_scratch) dout = q_scratch;
    else if (s_scroll)  dout = q_scroll;
    else if (s_spr)     dout = q_spr;
    else                dout = 16'h0000;
  end

endmodule

`default_nettype wire
