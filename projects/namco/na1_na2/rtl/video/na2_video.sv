//============================================================================
//  Namco NA-2 -- scanline sequencer
//
//  Turns the per-line pieces (na2_tilemap, na2_linebuf, the scroll RAM and the
//  video registers) into a picture.  One line is rendered into the back half of
//  the line buffer while the front half is scanned out, and the halves swap.
//
//  Written from reference/mame/namcona1_v.cpp screen_update()/draw_background()
//  and reference/fbneo/d_namcona1.cpp DrvDrawTo()/draw_layer(), read directly
//  rather than from a paraphrase.  Where the two disagree it is called out.
//
//  Per frame (MAME_VERIFIED, FBNEO_CROSSCHECK -- identical in both):
//      scrollx[layer] = 0x3a - 2*layer      "xadjust"
//      scrolly[layer] = 0
//
//  Per raw line 0..255, for each layer, *including lines that are never shown*
//  because the accumulator has to keep running:
//      xdata = scroll[layer*0x200 + line]
//      ydata = scroll[layer*0x200 + line + 0x100]
//      scrollx = (xdata & 0x4000) ? xadjust + xdata : scrollx + (xdata & 0x1ff)
//      if (ydata & 0x4000) scrolly = (ydata - line) & 0x1ff
//
//  MAME wraps the scrollx update in `if (xdata)`; FBNeo does not.  The two are
//  the same function: when xdata is 0 the 0x4000 branch cannot be taken and
//  `xdata & 0x1ff` is 0, so MAME's guard changes nothing.  Not implemented.
//
//  A layer whose per-line scroll word is exactly 0xC001 does not draw its
//  tilemap at all: that scanline comes straight out of video RAM as packed
//  8-bit pixels (`na2_pixline`, MAME's draw_pixel_line()).  It replaces that
//  layer's turn in the priority loop and nothing else, except that it declares
//  every pixel of the line off-limits to sprites -- MAME writes 0xff into the
//  priority buffer across the whole width, which here is "set every occupancy
//  bit".  MAME calls the trigger condition itself unproven; keep the UNKNOWN.
//
//  Then, when the line is inside the display window:
//      clear the line buffer to (vreg[0xBA] & 0xF) << 8, priority 0
//      for priority = 0..7:
//          for which = 4..0:                     ROZ is which == 4
//              pri = (which == 4) ? vreg[0xAA] & 7 : vreg[0xA0 + 2*which] & 7
//              if (pri == priority) draw that layer
//      draw sprites
//
//  MAME_VERIFIED: the ROZ priority comes from `m_vreg[0xa0/2 + 5]`, i.e. vreg
//  $AA, not from $A8.  Both emulators agree.
//
//  The ROZ sampling point is accumulated per line the same way the tilemap
//  scroll is, and for the same reason -- MAME evaluates
//  `startx + x*incxx + y*incyx` for absolute screen coordinates, so a
//  per-line adder reproduces it exactly and costs no multiplier.  The base is
//  reloaded at the frame wrap, which is where MAME's screen_update() reads the
//  registers.
//
//  Sprite compositing lives here rather than in na2_sprite, because neither of
//  its two rules is local to one sprite (MAME_VERIFIED, pdraw_tile):
//
//      if (pri[x] <= priority)  dest[x] = pen;    // versus the background
//      pri[x] = 0xff;                             // whether it drew or not
//
//  `occ_ev`/`occ_od` are that 0xff: one bit per pixel, cleared at the top of
//  every line, set by any non-transparent sprite pixel whether or not it won.
//  So sprite 0 beats sprite 1 regardless of their priorities, and the priority
//  field only ever settles sprite-versus-background.
//
//  The background priority the comparison needs is the `pri` field already in
//  the line buffer, but reading it back would want a third port on a memory
//  that has two.  `pri_sh_ev`/`pri_sh_od` are a register-file shadow of that
//  field, written by the same clear and the same tilemap writes, so the sprite
//  pass can read it combinationally.  It costs about 1500 flip-flops in a
//  design at 28 % logic and leaves the M10K count -- the resource that is
//  actually tight, at 86 % -- alone.
//
//  The display window (MAME `display_rect`, FBNeo `clip`) is
//      x from vreg[$80]-0x48 to vreg[$82]-0x48-1
//      y from max(vreg[$84], 32) to min(vreg[$86]-1, 255)
//  and nothing is drawn at all if it is empty or starts negative
//  (MAME screen_enabled(), FBNeo enable_screen_clipped()).  Numan Athletics
//  drives it for the window effects at event start and on the title-to-demo
//  transition, so it is not decoration.  Here the y half is applied by skipping
//  the draw pass for lines outside it, and the x half at scan-out by showing
//  the background pen outside it; the visible result is the same, because
//  everything outside the window is background fill either way.
//
//  Scan-out pipeline, and why the fetch runs one pixel ahead:
//      t+0  hpos becomes H, ce_pix pulses, fetch address H+1 goes out
//      t+1  line buffer returns the pen for H+1
//      t+2  palette RAM returns the word
//      t+3  na2_palette returns RGB, and holds it until the next pixel
//      t+8  ce_pix pulses with hpos = H+1, and RGB is already H+1's colour
//  The line buffer swaps at hpos == H_TOTAL-2 so that the H_TOTAL-1 fetch of
//  pixel 0 already reads the half that is about to be displayed.
//============================================================================
`default_nettype none

module na2_video #(
    parameter int H_TOTAL  = 398,
    // Give ROZ a CG port of its own; see roz_cg_* below.
    parameter bit SPLIT_ROZ_CG = 1'b0,
    parameter bit SPR_EARLY_EVAL = 1'b0,
    parameter bit SPR_SKIP_EMPTY = 1'b0,
    // Per-line sprite hit list length; na2_sprite's header says how 128 was
    // chosen.  Pocket pins the old 64 until it has been measured there.
    parameter int SPR_MAXHIT = 128,
    // hpos before the end of the line at which an unfinished sprite pass is
    // abandoned.  The line buffer swaps at H_TOTAL-2, so this must stay ahead
    // of the engine's write pipeline -- which is one or two cycles, not the
    // sixteen hpos (128 core clocks) the default reserves.
    //
    // MiSTer keeps 16 and never notices: with CG on chip the sprite pass never
    // reaches the deadline at all (measured: zero aborted lines).  Pocket does
    // reach it, because a CG row costs ten clocks there instead of one, and on
    // the heaviest frame in the regression set 128 of 224 lines were dropping
    // sprites at it.
    //
    // Measured over the 16-frame Pocket regression with CPU write traffic:
    //
    //     SPR_MARGIN   frames pixel exact
    //         16          15 / 16     numanath f1200 loses 184 pixels
    //          8          16 / 16
    //          4          16 / 16
    //          2          16 / 16
    //
    // Pocket takes 8 -- the largest value that is exact, leaving 6 hpos
    // (48 core clocks) before the buffer swaps.
    parameter int SPR_MARGIN = 16,
    parameter int H_ACTIVE = 304,
    parameter int V_TOTAL  = 262,
    parameter int V_START  = 32,     // first visible raw line
    parameter int V_END    = 256     // one past the last visible raw line
) (
    input  wire        clk,
    input  wire        rst,

    // --- CRTC --------------------------------------------------------------
    input  wire [8:0]  hpos,
    input  wire [8:0]  vpos,
    input  wire        ce_pix,

    // --- video registers ---------------------------------------------------
    input  wire [15:0] vreg_xmin,    // $80
    input  wire [15:0] vreg_xmax,    // $82
    input  wire [15:0] vreg_ymin,    // $84
    input  wire [15:0] vreg_ymax,    // $86
    input  wire        gfx_enable,   // $8E
    input  wire [15:0] vreg_pri0,    // $A0
    input  wire [15:0] vreg_pri1,    // $A2
    input  wire [15:0] vreg_pri2,    // $A4
    input  wire [15:0] vreg_pri3,    // $A6
    input  wire [15:0] vreg_clr0,    // $B0
    input  wire [15:0] vreg_clr1,    // $B2
    input  wire [15:0] vreg_clr2,    // $B4
    input  wire [15:0] vreg_clr3,    // $B6
    input  wire [15:0] vreg_bg,      // $BA, and the backdrop colour with it
    input  wire [15:0] vreg_bpp,     // $BC, one bit per layer, 1 = 4bpp
    input  wire [15:0] vreg_sprctl,  // $22, bit 0 = alternate sprite RAM bank
    input  wire        spr_bank_sel, // 0 = follow $22 live, 1 = latch per frame
    input  wire [15:0] vreg_pri_roz,  // $AA
    input  wire [15:0] vreg_rz_xx,    // $C0
    input  wire [15:0] vreg_rz_xy,    // $C2
    input  wire [15:0] vreg_rz_yx,    // $C4
    input  wire [15:0] vreg_rz_yy,    // $C6
    input  wire [15:0] vreg_rz_x0,    // $C8
    input  wire [15:0] vreg_rz_y0,    // $CA

    // --- renderer read ports on na2_vram -----------------------------------
    output wire [10:0] scroll_addr,
    input  wire [15:0] scroll_data,
    output wire [14:0] vram_addr,
    input  wire [15:0] vram_data,
    // ROZ gets its own CG port when SPLIT_ROZ_CG is set.  The three engines
    // never overlap in time, so one port was always enough for correctness --
    // but they do not want the same MEMORY.  ROZ walks CG with a rotation
    // stride and improves monotonically with cache line width; the tilemaps
    // walk it sequentially and want the round trip short.  Measured: sixteen
    // words a line clears em3/f880's overrun, the only setting that ever has,
    // and collapses FA to 56 overrun lines.  Two ports let each have what it
    // wants.  With SPLIT_ROZ_CG=0 the roz_* port is unused and the mux below
    // is exactly what it always was.
    output wire [16:0] roz_cg_addr,
    input  wire [15:0] roz_cg_data,
    input  wire        roz_cg_stall,
    output wire [16:0] cg_addr,
    // Prefetch hint for the CG backend.  Only the sprite engine raises one:
    // it is the engine whose reads are all misses, one row per cell.
    output wire [16:0] cg_pf_addr,
    output wire        cg_pf_req,
    input  wire [15:0] cg_data,
    // The CG backend is not ready with cg_data for cg_addr.  Routed only to
    // whichever engine currently owns the shared CG port, so an engine that
    // is not reading CG is never frozen and cannot miss its start pulse.
    input  wire        cg_stall,
    output wire [13:0] shape_addr,
    input  wire [15:0] shape_data,
    output wire [10:0] spr_addr,
    input  wire [15:0] spr_data,

    // --- scan-out ----------------------------------------------------------
    output wire [11:0] pal_addr,
    output wire        pal_alt,
    output wire        pal_shadow,

    // --- diagnostics -------------------------------------------------------
    output wire        dbg_line_overrun,  // the line did not finish in time
    output wire [7:0]  dbg_spr_hits,      // sprites drawn on the last line
    output wire        dbg_spr_overflow,  // a line needed more than MAXHIT
    // The sprite pass was still running when the deadline arrived, so this
    // line keeps whatever it had drawn and loses the rest.  A line that does
    // this still FINISHES -- dbg_line_overrun stays low -- which is why the
    // board could report a healthy frame while sprites were visibly missing
    // from it.  Emeraldia's attract bubbles, 2026-09-06.
    output wire        dbg_spr_abort,
    // ROZ waiting on CG, GATED BY THE ROZ PASS.
    //
    // na2_cgmem drives r_stall as plain !r_hit, so the ROZ store raises it
    // whenever the address standing on its port is not in a row buffer --
    // including the whole of the tilemap and sprite passes, when that address
    // is last line's leftover and nobody is waiting on it.  A counter fed the
    // raw signal therefore reports the CPU's own writes through that store as
    // ROZ stalls, and reported "over half the line" on hardware for a frame
    // the simulator renders with none.  This is the signal that means what the
    // name says: ROZ is running AND it is waiting.
    output wire        dbg_roz_stall
);

  // =====================================================================
  // Which raw line are we rendering, and when
  // =====================================================================
  localparam int SPR_DEADLINE = H_TOTAL - SPR_MARGIN;

  // Line L is rendered while line L-1 is on screen, so at the top of CRTC line
  // v the target is v+1.  The frame's last line wraps round to raw line 0 and
  // resets the scroll accumulators, exactly where MAME's `for (line = 0; ...)`
  // starts over.
  wire line_start  = ce_pix && (hpos == 9'd0);
  // The back buffer becomes the front buffer here.  No producer may remain
  // alive past this edge, or its next registered write lands in the line being
  // displayed.  Normal lines finish well before it; this is containment for a
  // line that already exhausted its budget.
  wire render_cutoff = ce_pix && (hpos == H_TOTAL - 2);
  wire render_rst = rst || render_cutoff;

  // ---------------------------------------------------------------------
  // Per-line snapshot of the layer priorities and colour banks.
  //
  // This renderer reads the video registers live, one scanline at a time.
  // MAME draws a whole frame (or a posirq half) from one snapshot, so a
  // register the game changes part way down the picture is invisible to any
  // frame-by-frame comparison -- by construction, no matter how far apart the
  // two pictures really are.  MEASURED over 2,100 attract frames: 347 writes
  // to $80-$CF land inside the visible area, on 3.3 % of frames, and frame 746
  // of the "We are the Numan Athletes" demo rewrites layer 0's priority and
  // three colour banks at raw line 127 -- halfway down the screen.
  //
  // Live reads make that a fault rather than a curiosity: the layer walk in
  // S_PICK compares each layer's priority as it goes, so a priority that
  // changes mid-walk can make one layer match twice (costing another ~238
  // clocks, which is what the line-overrun flag saw on hardware) or match
  // ZERO times -- the layer simply is not drawn, and whatever it should have
  // covered shows through instead.
  //
  // INFERRED: real hardware must sample these somewhere, and the start of the
  // line is the only choice that makes a scanline renderer self-consistent.
  // Nothing here proves the hardware does it exactly at hpos 0.
  // docs/glitch_investigation.md sections 17-23.
  // ---------------------------------------------------------------------
  // Declared at the width actually used, not the register's 16.  With them at
  // [15:0] Quartus reported every one as "assigned a value but never read" --
  // which in this project is the shape of a diagnostic that has quietly become
  // dead code, and it cost a look to establish that it was not.  Narrow
  // declarations make the next such warning mean something.
  reg [2:0] l_pri0, l_pri1, l_pri2, l_pri3, l_pri_roz;
  reg [3:0] l_clr0, l_clr1, l_clr2, l_clr3;
  always @(posedge clk) begin
    if (rst) begin
      l_pri0    <= 3'd0;
      l_pri1    <= 3'd0;
      l_pri2    <= 3'd0;
      l_pri3    <= 3'd0;
      l_pri_roz <= 3'd0;
      l_clr0    <= 4'd0;
      l_clr1    <= 4'd0;
      l_clr2    <= 4'd0;
      l_clr3    <= 4'd0;
    end else if (line_start) begin
      l_pri0    <= vreg_pri0[2:0];
      l_pri1    <= vreg_pri1[2:0];
      l_pri2    <= vreg_pri2[2:0];
      l_pri3    <= vreg_pri3[2:0];
      l_pri_roz <= vreg_pri_roz[2:0];
      // The colour banks, which section 21 prescribed alongside the priorities
      // and which the first build left out.  Frame 746 of the demo rewrites
      // layer 0's priority AND colour banks 1, 2 and 3 at raw line 127, in one
      // go, so latching half of that set leaves the walk consistent and the
      // palette torn -- a layer selected by its start-of-line priority and then
      // drawn with a bank that changed underneath it.  Nine registers was
      // always the right number.  Still INFERRED: nothing proves the hardware
      // samples at hpos 0.
      l_clr0    <= vreg_clr0[3:0];
      l_clr1    <= vreg_clr1[3:0];
      l_clr2    <= vreg_clr2[3:0];
      l_clr3    <= vreg_clr3[3:0];
    end
  end
  wire frame_wrap  = (vpos == V_TOTAL - 1);

  // ---------------------------------------------------------------------
  // The sprite RAM bank, and when a change to it takes effect.
  //
  // MAME_VERIFIED: draw_sprites() starts at m_spriteram and adds 0x400 words
  // when vreg $22 bit 0 is set, so the game double-buffers its object list and
  // flips banks.  MAME reads that bit once, when it draws the frame.  This
  // renderer follows it live, per line, and that is a real difference: if the
  // game flips the bank while the picture is still being scanned, the top of
  // the frame comes from one list and the bottom from the other -- objects
  // that moved tear, and objects written late into the new list are missing
  // entirely.  Both were reported from the board.
  //
  // UNKNOWN which the hardware does.  A sprite engine that reads object RAM
  // line by line, as this one does, would have to sample the control register
  // somewhere, and once per frame is the only choice that makes double
  // buffering work at all.  But that is reasoning, not evidence, so it is
  // selectable and the answer comes from looking at a real screen.
  //   0 = live (what this core has always done, and what MAME's source reads
  //             literally)
  //   1 = latched at the top of the frame
  // ---------------------------------------------------------------------
  reg spr_bank_l;
  always @(posedge clk) begin
    if (rst)                       spr_bank_l <= 1'b0;
    else if (ce_pix && frame_wrap) spr_bank_l <= vreg_sprctl[0];
  end

  wire spr_bank = spr_bank_sel ? spr_bank_l : vreg_sprctl[0];
  wire [8:0] next_line = frame_wrap ? 9'd0 : (vpos + 9'd1);
  wire line_valid  = frame_wrap || (vpos < 9'd255);

  reg [8:0] line;            // the raw line being rendered

  // =====================================================================
  // Display window
  // =====================================================================
  wire signed [17:0] win_x0 = $signed({2'b00, vreg_xmin}) - 18'sd72;
  wire signed [17:0] win_x1 = $signed({2'b00, vreg_xmax}) - 18'sd73;
  wire signed [17:0] ymin_r = $signed({2'b00, vreg_ymin});
  wire signed [17:0] ymax_r = $signed({2'b00, vreg_ymax}) - 18'sd1;
  wire signed [17:0] win_y0 = (ymin_r > 18'sd32)  ? ymin_r : 18'sd32;
  wire signed [17:0] win_y1 = (ymax_r < 18'sd255) ? ymax_r : 18'sd255;

  wire win_ok = (win_x0 >= 0) && (win_x1 >= 0) &&
                (win_x0 <= win_x1) && (win_y0 <= win_y1);

  wire line_in_win = win_ok &&
                     ($signed({9'd0, line}) >= win_y0) &&
                     ($signed({9'd0, line}) <= win_y1);
  wire next_line_in_win = win_ok &&
                     ($signed({9'd0, next_line}) >= win_y0) &&
                     ($signed({9'd0, next_line}) <= win_y1);
  wire sp_prepare = SPR_EARLY_EVAL && line_start && line_valid && gfx_enable &&
                    next_line_in_win;

  // =====================================================================
  // Scroll accumulators
  // =====================================================================
  reg [8:0] scrollx [0:3];
  reg [8:0] scrolly [0:3];

  function automatic [8:0] xadjust(input [1:0] l);
    xadjust = 9'h03A - {6'd0, l, 1'b0};
  endfunction

  // =====================================================================
  // Per-layer register decode
  // =====================================================================
  // Plain combinational muxes rather than `function automatic`.
  //
  // As functions, Quartus reported every one of l_pri0-3 and l_clr0-3 as
  // "assigned a value but never read" (Warning 10036) -- it does not appear to
  // count a read inside a function body.  l_pri_roz, which is also read at a
  // port connection outside any function, drew no such warning.  That warning
  // is the only thing standing between "the per-line snapshot is in the
  // bitstream" and "it was optimised away and the renderer is still reading the
  // registers live", and it is not worth carrying an unresolvable warning in
  // the module that draws the picture.  As muxes there is nothing to argue
  // about: if the warning ever comes back on these, it means what it says.
  // The two muxes that read `which` live further down, after `which` itself
  // is declared.  They used to sit here, which Quartus accepts and ModelSim
  // 10.5b does not -- `Undefined variable: 'which'` -- and that stopped
  // sim/run_mame_frame.sh compiling at all.  KNOWN_ISSUES.md lesson 41.


  wire [12:0] bg_pen = {1'b0, vreg_bg[3:0], 8'd0};

  // =====================================================================
  // ROZ sampling matrix.  MAME_VERIFIED / FBNEO_CROSSCHECK, both identical:
  //   inc* = (s16)reg << 8,  start = (offset << 12) + incxx*46 - incyx*8
  // 46 and -8 are MAME's `dx` and `dy` screen adjustments.
  // =====================================================================
  wire signed [31:0] rz_incxx = {{8{vreg_rz_xx[15]}}, vreg_rz_xx, 8'd0};
  wire signed [31:0] rz_incxy = {{8{vreg_rz_xy[15]}}, vreg_rz_xy, 8'd0};
  wire signed [31:0] rz_incyx = {{8{vreg_rz_yx[15]}}, vreg_rz_yx, 8'd0};
  wire signed [31:0] rz_incyy = {{8{vreg_rz_yy[15]}}, vreg_rz_yy, 8'd0};
  wire signed [31:0] rz_xoff  = {{4{vreg_rz_x0[15]}}, vreg_rz_x0, 12'd0};
  wire signed [31:0] rz_yoff  = {{4{vreg_rz_y0[15]}}, vreg_rz_y0, 12'd0};

  // 46 = 32 + 8 + 4 + 2, so the shifts stand in for a multiplier
  wire signed [31:0] rz_startx = rz_xoff
                               + (rz_incxx <<< 5) + (rz_incxx <<< 3)
                               + (rz_incxx <<< 2) + (rz_incxx <<< 1)
                               - (rz_incyx <<< 3);
  wire signed [31:0] rz_starty = rz_yoff
                               + (rz_incxy <<< 5) + (rz_incxy <<< 3)
                               + (rz_incxy <<< 2) + (rz_incxy <<< 1)
                               - (rz_incyy <<< 3);

  reg signed [31:0] rz_cx, rz_cy;      // the sampling point at screen x = 0

  // =====================================================================
  // Sequencer
  // =====================================================================
  localparam S_IDLE   = 4'd0,
             S_SCR_A  = 4'd1,   // issue xdata address
             S_SCR_B  = 4'd2,   // issue ydata address
             S_SCR_C  = 4'd3,   // xdata arrives
             S_SCR_D  = 4'd4,   // ydata arrives, accumulate, next layer
             S_CLEAR  = 4'd5,
             S_PICK   = 4'd6,
             S_GO     = 4'd7,
             S_WAIT   = 4'd8,
             S_SPR_GO = 4'd9,
             S_SPR_W  = 4'd10,
             S_DONE   = 4'd11;

  reg [3:0]  st;
  reg [1:0]  sl;               // layer being scrolled
  reg [7:0]  clr_i;            // clear index, 0..151
  reg [2:0]  prio;
  reg [2:0]  which;                // 4 = ROZ, 3..0 = the tilemaps
  // vreg_bpp is a 16-bit VECTOR ($BC, one bit per layer), not a 4-entry array,
  // so selecting a bit out of it needs four index bits.  Only layers 0..3 are
  // ever chosen here, hence the zero-extend.  Named wire so the width is
  // unambiguous rather than an inline concat.
  wire [3:0] bpp_idx = {2'd0, which[1:0]};
  reg [8:0]  xdata_lo;         // (xdata & 0x1ff) latched
  reg        xdata_reset;      // xdata & 0x4000
  reg [15:0] xdata_full;

  reg [2:0] pri_sel;
  always @(*) begin
    case (which)
      3'd0: pri_sel = l_pri0;
      3'd1: pri_sel = l_pri1;
`ifdef TM2_RASTER
      // DIAGNOSTIC, never defined in a build.  Emeraldia's HOW TO PLAY screen
      // takes a raster interrupt at the line in vreg $8A -- 240 in the dumped
      // frames -- and draws its dolphin by giving tilemap 2 priority 0 above
      // that line and its register value below it, so the water covers only
      // the dolphin's lower half.  A frame dump carries ONE value for the
      // whole picture, the one from below the line, so forcing the split here
      // is the only way to put that to a dumped frame.
      //
      // MEASURED 2026-08-26: with this defined the four .mame/dump-emeralda-sea
      // frames render pixel-exact against MAME -- 1289, 1476, 1367 and 1231
      // differing pixels all go to zero -- with no change to any rule in this
      // renderer.  The fault was in the comparison, not the renderer; see
      // docs/emeraldia_dolphin.md.
      3'd2: pri_sel = (line <= 9'd240) ? 3'd0 : l_pri2;
`else
      3'd2: pri_sel = l_pri2;
`endif
      3'd3: pri_sel = l_pri3;
      default: pri_sel = l_pri_roz;   // which == 4
    endcase
  end

  reg [3:0] clr_sel;
  always @(*) begin
    case (which[1:0])
      2'd0: clr_sel = l_clr0;
      2'd1: clr_sel = l_clr1;
      2'd2: clr_sel = l_clr2;
      default: clr_sel = l_clr3;
    endcase
  end

  reg        tm_start;
  wire       tm_busy;
  reg        sp_start;
  wire       sp_busy;
  reg        rz_start;
  wire       rz_busy;
  reg        pl_start;
  wire       pl_busy;
  wire       pl_wr_a, pl_wr_b;
  wire [8:0] pl_addr_a, pl_addr_b;
  wire [12:0] pl_pen_a, pl_pen_b;
  wire [2:0]  pl_pri;
  reg         pl_on [0:3];        // this layer's scroll word was 0xC001
  reg [15:0]  pl_yd [0:3];        // and the line-select word that goes with it
  wire       rz_wr;
  wire [8:0] rz_addr;
  wire [12:0] rz_pen;
  wire [2:0]  rz_pri;
  reg [1:0]  tm_layer;
  reg [8:0]  tm_scrollx, tm_scrolly;
  reg        tm_depth4;
  reg [3:0]  tm_color;
  reg [2:0]  tm_prio;

  reg [10:0] scr_addr_r;
  assign scroll_addr = scr_addr_r;

  wire [16:0] tm_cg_addr;
  wire [13:0] tm_sh_addr;
  wire [14:0] tm_vram_addr;

  reg overrun;
  assign dbg_line_overrun = overrun;

  // line buffer, and the mux between the clear engine and the tilemap
  reg        lb_bank;
  wire       tm_wr_a, tm_wr_b;
  wire [8:0] tm_addr_a, tm_addr_b;
  wire [12:0] tm_pen_a, tm_pen_b;
  wire [2:0]  tm_pri_o;

  wire clearing = (st == S_CLEAR);
  wire in_spr   = (st == S_SPR_GO) || (st == S_SPR_W);

  // Per-engine CG stalls.  Declared here and driven next to the cg_addr mux
  // further down, where in_roz is in scope: the freeze has to track exactly
  // the same selector that decides whose address is on the port.
  wire tm_cg_stall, rz_cg_stall, sp_cg_stall;

  // --- sprite candidates, and the two rules that decide their fate --------
  wire        sp_px_a, sp_px_b;
  wire [8:0]  sp_xa, sp_xb;         // always adjacent, sp_xb == sp_xa + 1
  wire [12:0] sp_pen_a, sp_pen_b;
  wire [2:0]  sp_pri;
  wire        sp_hwsh;              // darken what is there instead of writing a pen

  // occupancy and the background-priority shadow, both routed by parity so
  // each array takes one indexed write per clock
  reg [255:0] occ_ev, occ_od;
  reg [2:0]   pri_sh_ev [0:255];
  reg [2:0]   pri_sh_od [0:255];

  wire       sp_a_even = ~sp_xa[0];
  wire [7:0] sp_ev_i   = sp_a_even ? sp_xa[8:1] : sp_xb[8:1];
  wire [7:0] sp_od_i   = sp_a_even ? sp_xb[8:1] : sp_xa[8:1];

  wire       sp_occ_a  = sp_a_even ? occ_ev[sp_ev_i]    : occ_od[sp_od_i];
  wire       sp_occ_b  = sp_a_even ? occ_od[sp_od_i]    : occ_ev[sp_ev_i];
  wire [2:0] sp_bg_a   = sp_a_even ? pri_sh_ev[sp_ev_i] : pri_sh_od[sp_od_i];
  wire [2:0] sp_bg_b   = sp_a_even ? pri_sh_od[sp_od_i] : pri_sh_ev[sp_ev_i];

  // SPR_NOPRI is a DIAGNOSTIC, never defined in a build: it drops the
  // background-priority half of the test so a missing sprite can be told from
  // a sprite that is drawn and then loses.  Added 2026-08-26 chasing
  // Emeraldia's dolphin, which is the only priority-0 sprite in its frame.
`ifdef SPR_NOPRI
  wire sp_draw_a = sp_px_a && !sp_occ_a;
  wire sp_draw_b = sp_px_b && !sp_occ_b;
`else
  wire sp_draw_a = sp_px_a && !sp_occ_a && (sp_bg_a <= sp_pri);
  wire sp_draw_b = sp_px_b && !sp_occ_b && (sp_bg_b <= sp_pri);
`endif
  wire sp_any    = sp_px_a || sp_px_b;

  // SPR_WHYNOT is a DIAGNOSTIC, never defined in a build.  A sprite pixel that
  // is present, unblocked by an earlier sprite, and still not drawn can only
  // have lost the background-priority test -- this prints the two numbers that
  // decided it, which is the only way to find out WHICH background layer put
  // that priority there.  Added 2026-08-26.
`ifdef SPR_WHYNOT
  // synthesis translate_off
  integer whynot_n = 0;
  always @(posedge clk) begin
    if (sp_px_a && !sp_occ_a && !(sp_bg_a <= sp_pri) && whynot_n < 8
        && line >= 9'd200 && sp_xa < 9'd110) begin
      $display("[whynot] line %0d x %0d  bg_pri %0d  spr_pri %0d",
               line, sp_xa, sp_bg_a, sp_pri);
      whynot_n = whynot_n + 1;
    end
  end
  // synthesis translate_on
`endif

  // Only one of the clear, the ROZ engine, the tilemap engine and the sprite
  // engine can be writing at any moment; the mux is explicit anyway so that
  // the order is readable rather than implied.
  // A hardware-shadow sprite pixel wins the priority test and then leaves the
  // pen alone (MAME_VERIFIED: pdraw_tile runs the pixel already on screen
  // through the palette's shadow table), so it takes the flag port and not the
  // pen port.
  // TM_KILL is a DIAGNOSTIC, never defined in a build: it silences one tilemap
  // layer so "which layer put that pixel/priority there" can be answered by
  // subtraction.  Added 2026-08-26.
`ifdef TM_KILL
  wire tm_killed = (which == `TM_KILL);
`else
  wire tm_killed = 1'b0;
`endif
  wire        lb_wr_a   = clearing ? 1'b1          : sp_any ? (sp_draw_a && !sp_hwsh) : (rz_wr | pl_wr_a | (tm_wr_a & ~tm_killed));
  wire [8:0]  lb_addr_a = clearing ? {clr_i, 1'b0} : sp_any ? sp_xa     : (rz_wr ? rz_addr : pl_wr_a ? pl_addr_a : tm_addr_a);
  wire [12:0] lb_pen_a  = clearing ? bg_pen        : sp_any ? sp_pen_a  : (rz_wr ? rz_pen  : pl_wr_a ? pl_pen_a  : tm_pen_a);
  wire [2:0]  lb_pri_a  = clearing ? 3'd0          : sp_any ? sp_pri    : (rz_wr ? rz_pri  : pl_wr_a ? pl_pri    : tm_pri_o);
  wire        lb_wr_b   = clearing ? 1'b1          : sp_any ? (sp_draw_b && !sp_hwsh) : (rz_wr ? 1'b0 : (pl_wr_b | (tm_wr_b & ~tm_killed)));
  wire [8:0]  lb_addr_b = clearing ? {clr_i, 1'b1} : sp_any ? sp_xb     : (rz_wr ? (rz_addr + 9'd1) : pl_wr_b ? pl_addr_b : tm_addr_b);
  wire [12:0] lb_pen_b  = clearing ? bg_pen        : sp_any ? sp_pen_b  : (pl_wr_b ? pl_pen_b : tm_pen_b);
  wire [2:0]  lb_pri_b  = clearing ? 3'd0          : sp_any ? sp_pri    : (rz_wr ? rz_pri : pl_wr_b ? pl_pri : tm_pri_o);

  // The shadow flag: cleared with the line, cleared again by anything that
  // writes a pen, and set by a hardware-shadow sprite pixel that wins.
  wire lb_hwsh_we_a = clearing ? 1'b1 : sp_any ? sp_draw_a
                                            : (rz_wr | pl_wr_a | tm_wr_a);
  wire lb_hwsh_we_b = clearing ? 1'b1 : sp_any ? sp_draw_b
                                            : (rz_wr ? 1'b0 : (pl_wr_b | tm_wr_b));
  wire lb_hwsh_a    = sp_any && sp_hwsh && !clearing;
  wire lb_hwsh_b    = lb_hwsh_a;

  // The priority mirror follows only the background writers.  Sprites
  // deliberately do not update it: occupancy already blocks every later
  // sprite, and MAME's 0xff is not a priority value anything compares against
  // afterwards.
  wire        sh_wr_a   = clearing ? 1'b1          : (in_spr ? 1'b0 : (rz_wr | pl_wr_a | (tm_wr_a & ~tm_killed)));
  wire [8:0]  sh_x_a    = clearing ? {clr_i, 1'b0} : (rz_wr ? rz_addr : pl_wr_a ? pl_addr_a : tm_addr_a);
  wire [2:0]  sh_p_a    = clearing ? 3'd0          : (rz_wr ? rz_pri  : pl_wr_a ? pl_pri    : tm_pri_o);
  wire        sh_wr_b   = clearing ? 1'b1          : (in_spr ? 1'b0 : (rz_wr ? 1'b0 : (pl_wr_b | (tm_wr_b & ~tm_killed))));
  wire [8:0]  sh_x_b    = clearing ? {clr_i, 1'b1} : (rz_wr ? (rz_addr + 9'd1) : pl_wr_b ? pl_addr_b : tm_addr_b);
  wire [2:0]  sh_p_b    = clearing ? 3'd0          : (pl_wr_b ? pl_pri : tm_pri_o);

  wire       sh_a_even = ~sh_x_a[0];
  wire       sh_ev_we  = sh_a_even ? sh_wr_a : sh_wr_b;
  wire [7:0] sh_ev_i   = sh_a_even ? sh_x_a[8:1] : sh_x_b[8:1];
  wire [2:0] sh_ev_p   = sh_a_even ? sh_p_a : sh_p_b;
  wire       sh_od_we  = sh_a_even ? sh_wr_b : sh_wr_a;
  wire [7:0] sh_od_i   = sh_a_even ? sh_x_b[8:1] : sh_x_a[8:1];
  wire [2:0] sh_od_p   = sh_a_even ? sh_p_b : sh_p_a;

`ifdef SPR_WHYNOT
  // synthesis translate_off
  integer scr_n = 0;
  always @(posedge clk) begin
    if (st == S_SCR_D && scroll_data[14] && scr_n < 20) begin
      $display("[scroll] LINE-SELECT line %0d layer %0d: ydata %04X -> scrolly %0d",
               line, sl, scroll_data, scroll_data[8:0] - line);
      scr_n = scr_n + 1;
    end
    if (st == S_SCR_D && sl == 2'd2 && line == 9'd208 && scr_n < 40) begin
      $display("[scroll] at line 208 tm2: scrolly is %0d", scrolly[2]);
      scr_n = scr_n + 1;
    end
  end
  integer who_n = 0;
  always @(posedge clk) begin
    if (sh_wr_a && line == 9'd208 && sh_x_a == 9'd66 && who_n < 30) begin
      $display("[who] line %0d x %0d  prio-pass %0d  which %0d  <- pri %0d  by %s",
               line, sh_x_a, prio, which, sh_p_a,
               rz_wr ? "ROZ" : pl_wr_a ? "pixel-line" : "tilemap");
      who_n = who_n + 1;
    end
  end
  // synthesis translate_on
`endif

  wire sp_ev_set = sp_a_even ? sp_px_a : sp_px_b;
  wire sp_od_set = sp_a_even ? sp_px_b : sp_px_a;

  always @(posedge clk) begin
    if (sh_ev_we) pri_sh_ev[sh_ev_i] <= sh_ev_p;
    if (sh_od_we) pri_sh_od[sh_od_i] <= sh_od_p;
    if (line_start) begin
      occ_ev <= 256'd0;
      occ_od <= 256'd0;
    end else if (pl_start) begin
      // MAME writes 0xff across the whole width of a pixel line, which is the
      // value sprites refuse to draw over
      occ_ev <= {256{1'b1}};
      occ_od <= {256{1'b1}};
    end else begin
      if (sp_ev_set) occ_ev[sp_ev_i] <= 1'b1;
      if (sp_od_set) occ_od[sp_od_i] <= 1'b1;
    end
  end

  always @(posedge clk) begin
    integer k;
    if (rst) begin
      st       <= S_IDLE;
      tm_start <= 1'b0;
      sp_start <= 1'b0;
      rz_start <= 1'b0;
      pl_start <= 1'b0;
      lb_bank  <= 1'b0;
      rz_cx    <= 32'd0;
      rz_cy    <= 32'd0;
      overrun  <= 1'b0;
      line     <= 9'd0;
      for (k = 0; k < 4; k = k + 1) begin
        scrollx[k] <= xadjust(k[1:0]);
        scrolly[k] <= 9'd0;
      end
    end else begin
      tm_start <= 1'b0;
      sp_start <= 1'b0;
      rz_start <= 1'b0;
      pl_start <= 1'b0;

      // the halves swap two pixels before the line ends, so that the
      // one-pixel-ahead fetch of pixel 0 already sees the new front half
      if (ce_pix && (hpos == H_TOTAL - 2)) lb_bank <= ~lb_bank;

      if (render_cutoff) begin
        // Report the late line before terminating it.  na2_top makes this
        // pulse sticky for the Pocket diagnostic grid.
        if (st != S_IDLE) overrun <= 1'b1;
        st <= S_IDLE;
      end else if (line_start) begin
        overrun <= (st != S_IDLE);          // last line did not finish in time
        line    <= next_line;
        if (frame_wrap) begin
          for (k = 0; k < 4; k = k + 1) begin
            scrollx[k] <= xadjust(k[1:0]);
            scrolly[k] <= 9'd0;
          end
          rz_cx <= rz_startx;
          rz_cy <= rz_starty;
        end else begin
          rz_cx <= rz_cx + rz_incyx;
          rz_cy <= rz_cy + rz_incyy;
        end
        st <= line_valid ? S_SCR_A : S_IDLE;
        sl <= 2'd0;
      end else begin
        case (st)
          S_IDLE: ;

          // ---- scroll accumulator, one layer at a time ------------------
          S_SCR_A: begin
            scr_addr_r <= {sl, 9'd0} + {2'd0, line};              // xdata
            st         <= S_SCR_B;
          end
          S_SCR_B: begin
            scr_addr_r <= {sl, 9'd0} + {1'b1, 8'd0} + {2'd0, line}; // ydata
            st         <= S_SCR_C;
          end
          S_SCR_C: begin
            xdata_full  <= scroll_data;
            xdata_lo    <= scroll_data[8:0];
            xdata_reset <= scroll_data[14];
            st          <= S_SCR_D;
          end
          S_SCR_D: begin
            // MAME checks the raw scroll word for 0xC001 *after* letting it
            // through the scrollx accumulator, so the accumulation below is
            // unchanged; only the draw for this layer is replaced.
            pl_on[sl] <= (xdata_full == 16'hC001);
            pl_yd[sl] <= scroll_data;
            // scrollx
            if (xdata_reset) scrollx[sl] <= xadjust(sl) + xdata_full[8:0];
            else             scrollx[sl] <= scrollx[sl] + xdata_lo;
            // scrolly, line select
            if (scroll_data[14]) scrolly[sl] <= scroll_data[8:0] - line;
            if (sl == 2'd3) begin
              // The clear runs for every line, in or out of the display
              // window.  MAME fills the backdrop over the whole cliprect and
              // only then restricts drawing to display_rect, so a line outside
              // the window shows the backdrop -- not, as this used to, whatever
              // the line buffer happened to hold from two lines ago.
              st    <= gfx_enable ? S_CLEAR : S_IDLE;
              clr_i <= 8'd0;
            end else begin
              sl <= sl + 2'd1;
              st <= S_SCR_A;
            end
          end

          // ---- clear to the backdrop -----------------------------------
          S_CLEAR: begin
            if (clr_i == (H_ACTIVE / 2) - 1) begin
              st    <= line_in_win ? S_PICK : S_IDLE;
              prio  <= 3'd0;
              which <= 3'd4;
            end else
              clr_i <= clr_i + 8'd1;
          end

          // ---- layers, in MAME's order ---------------------------------
          S_PICK: begin
            if (pri_sel == prio) begin
              if (which == 3'd4) begin
                rz_start <= 1'b1;
              end else if (pl_on[which[1:0]]) begin
                pl_start <= 1'b1;
              end else begin
                tm_layer   <= which[1:0];
                tm_scrollx <= scrollx[which[1:0]];
                tm_scrolly <= scrolly[which[1:0]];
                 tm_depth4  <= vreg_bpp[bpp_idx];
                tm_color   <= clr_sel;
                tm_prio    <= prio;
                tm_start   <= 1'b1;
              end
              st <= S_GO;
            end else begin
              if (which == 3'd0) begin
                if (prio == 3'd7) st <= S_DONE;
                else begin
                  prio  <= prio + 3'd1;
                  which <= 3'd4;
                end
              end else
                which <= which - 3'd1;
            end
          end
          S_GO:   st <= S_WAIT;             // let busy rise
          S_WAIT: if (!tm_busy && !rz_busy && !pl_busy) begin
            if (which == 3'd0) begin
              if (prio == 3'd7) st <= S_DONE;
              else begin
                prio  <= prio + 3'd1;
                which <= 3'd4;
                st    <= S_PICK;
              end
            end else begin
              which <= which - 3'd1;
              st    <= S_PICK;
            end
          end

          // ---- sprites, after every tilemap, as MAME does ---------------
          S_DONE: begin
            // If the background/ROZ pass consumed the sprite deadline, do not
            // launch a one-cycle start pulse only to abort it a cycle later.
            if (hpos < SPR_DEADLINE[8:0]) begin
              sp_start <= 1'b1;
              st       <= S_SPR_GO;
            end else begin
              st <= S_IDLE;
            end
          end
          S_SPR_GO: st <= S_SPR_W;          // let busy rise
          S_SPR_W:  if (!sp_busy) st <= S_IDLE;

          default: st <= S_IDLE;
        endcase
      end
    end
  end

  // =====================================================================
  // One tilemap layer engine, reused for all four
  // =====================================================================
  na2_tilemap u_tilemap (
      .clk        (clk),
      .rst        (render_rst),
      .start      (tm_start),
      .busy       (tm_busy),
      .layer      (tm_layer),
      .line       (line),
      .prio       (tm_prio),
      .scrollx    (tm_scrollx),
      .scrolly    (tm_scrolly),
      .depth4     (tm_depth4),
      .color_bank (tm_color),
      .vram_addr  (tm_vram_addr),
      .vram_data  (vram_data),
      .cg_addr    (tm_cg_addr),
      .cg_data    (cg_data),
      .cg_stall   (tm_cg_stall),
      .shape_addr (tm_sh_addr),
      .shape_data (shape_data),
      .lb_wr_a    (tm_wr_a),
      .lb_addr_a  (tm_addr_a),
      .lb_pen_a   (tm_pen_a),
      .lb_wr_b    (tm_wr_b),
      .lb_addr_b  (tm_addr_b),
      .lb_pen_b   (tm_pen_b),
      .lb_pri     (tm_pri_o)
  );

  // =====================================================================
  // ROZ
  // =====================================================================
  wire [14:0] rz_vram_addr;
  wire [16:0] rz_cg_addr;
  wire [13:0] rz_sh_addr;
  wire        in_roz = rz_busy | rz_start;

  na2_roz #(.H_VISIBLE(H_ACTIVE)) u_roz (
      .clk        (clk),
      .rst        (render_rst),
      .start      (rz_start),
      .busy       (rz_busy),
      .prio       (l_pri_roz),         // the same per-line snapshot
      .depth4     (vreg_bpp[4]),
      .color_bank (vreg_bg[3:0]),
      .cx0        (rz_cx),
      .cy0        (rz_cy),
      .incxx      (rz_incxx),
      .incxy      (rz_incxy),
      .vram_addr  (rz_vram_addr),
      .vram_data  (vram_data),
      .cg_addr    (rz_cg_addr),
      // Its own port's data when ROZ has one, the shared port's otherwise.
      .cg_data    (SPLIT_ROZ_CG ? roz_cg_data : cg_data),
      .cg_stall   (rz_cg_stall),
      .shape_addr (rz_sh_addr),
      .shape_data (shape_data),
      .lb_wr      (rz_wr),
      .lb_addr    (rz_addr),
      .lb_pen     (rz_pen),
      .lb_pri     (rz_pri)
  );

  // =====================================================================
  // Direct-pixel line mode
  // =====================================================================
  wire [14:0] pl_vram_addr;
  wire        in_pl = pl_busy | pl_start;

  na2_pixline #(.H_VISIBLE(H_ACTIVE)) u_pixline (
      .clk       (clk),
      .rst       (render_rst),
      .start     (pl_start),
      .busy      (pl_busy),
      .ydata     (pl_yd[which[1:0]]),
      .prio      (prio),
      .vram_addr (pl_vram_addr),
      .vram_data (vram_data),
      .lb_wr_a   (pl_wr_a), .lb_addr_a(pl_addr_a), .lb_pen_a(pl_pen_a),
      .lb_wr_b   (pl_wr_b), .lb_addr_b(pl_addr_b), .lb_pen_b(pl_pen_b),
      .lb_pri    (pl_pri)
  );

  // =====================================================================
  // Sprites
  // =====================================================================
  wire [16:0] sp_cg_addr;
  wire [16:0] sp_cg_pf_addr;
  wire        sp_cg_pf_req;
  wire [13:0] sp_sh_addr;

  // The line buffer swaps at H_TOTAL-2, so anything still writing after that
  // scribbles on the half being displayed.  A heavy sprite line can get there:
  // the scroll accumulators, the clear, four tilemap layers and the ROZ layer
  // cost about 1420 clocks of the 3184 in a line, sprite evaluation another
  // 258, and 64 four-cell sprites would want 1856 more (5 + 6 x 4 each).  Stop the sprite pass
  // while there is still room to wind down, and say so.
  // The margin is a parameter so a testbench can ask what the sprite pass would
  // have needed if it had not been cut off.  Shipping value is 16.
  // Early evaluation can leave the sprite engine busy in S_HOLD before the
  // video FSM reaches its sprite window.  Its budget still expires at the same
  // deadline; keying abort only from in_spr allowed that prepared list to leak
  // into the next line when a background pass ran late.
  wire sp_abort = sp_busy && (hpos >= SPR_DEADLINE[8:0]);
  assign dbg_spr_abort = sp_abort;

  na2_sprite #(.SKIP_EMPTY(SPR_SKIP_EMPTY), .MAXHIT(SPR_MAXHIT)) u_sprite (
      .clk         (clk),
      .rst         (render_rst),
      .prepare     (sp_prepare),
      .start       (sp_start),
      .busy        (sp_busy),
      .abort       (sp_abort),
      .line        (line),
      .bank        (spr_bank),
      .spr_addr    (spr_addr),
      .spr_data    (spr_data),
      .cg_addr     (sp_cg_addr),
      .cg_pf_addr  (sp_cg_pf_addr),
      .cg_pf_req   (sp_cg_pf_req),
      .cg_data     (cg_data),
      .cg_stall    (sp_cg_stall),
      .shape_addr  (sp_sh_addr),
      .shape_data  (shape_data),
      .px_hwshadow (sp_hwsh),
      .px_a        (sp_px_a),
      .px_xa       (sp_xa),
      .px_pena     (sp_pen_a),
      .px_b        (sp_px_b),
      .px_xb       (sp_xb),
      .px_penb     (sp_pen_b),
      .px_pri      (sp_pri),
      .dbg_hits    (dbg_spr_hits),
      .dbg_overflow(dbg_spr_overflow)
  );

  // The three engines never run at the same time, so they share the read
  // ports.  The ROZ engine is the only other user of video RAM.
  assign roz_cg_addr = rz_cg_addr;
  assign cg_addr    = SPLIT_ROZ_CG
                      ? (in_spr ? sp_cg_addr : tm_cg_addr)
                      : (in_spr ? sp_cg_addr : in_roz ? rz_cg_addr : tm_cg_addr);
  // Same selector as the address mux above, so the engine that is stalled is
  // always the one whose address the backend is answering.
  assign sp_cg_stall = cg_stall &&  in_spr;
  assign rz_cg_stall = SPLIT_ROZ_CG ? (roz_cg_stall && in_roz)
                                    : (cg_stall && !in_spr && in_roz);

  assign dbg_roz_stall = rz_cg_stall;
  assign tm_cg_stall = cg_stall && !in_spr && !in_roz;
  // The hint is only meaningful while the sprite engine owns the port.
  assign cg_pf_addr = sp_cg_pf_addr;
  assign cg_pf_req  = sp_cg_pf_req && in_spr;

  assign shape_addr = in_spr ? sp_sh_addr : in_roz ? rz_sh_addr : tm_sh_addr;
  assign vram_addr  = in_roz ? rz_vram_addr
                    : in_pl  ? pl_vram_addr : tm_vram_addr;

  // =====================================================================
  // Line buffer and scan-out
  // =====================================================================
  wire [8:0]  fetch_x = (hpos == H_TOTAL - 1) ? 9'd0 : (hpos + 9'd1);
  wire [12:0] rd_pen;
  wire [2:0]  rd_pri;
  wire        rd_hwsh;

  na2_linebuf u_linebuf (
      .clk     (clk),
      .bank    (lb_bank),
      .wr_a    (lb_wr_a), .addr_a(lb_addr_a), .pen_a(lb_pen_a), .pri_a(lb_pri_a),
      .hwsh_we_a(lb_hwsh_we_a), .hwsh_a(lb_hwsh_a),
      .wr_b    (lb_wr_b), .addr_b(lb_addr_b), .pen_b(lb_pen_b), .pri_b(lb_pri_b),
      .hwsh_we_b(lb_hwsh_we_b), .hwsh_b(lb_hwsh_b),
      .rd_addr (fetch_x), .rd_pen(rd_pen), .rd_pri(rd_pri), .rd_hwsh(rd_hwsh)
  );

  // the window test has to be about the pixel the buffer is returning, which
  // is one clock behind the address
  reg [8:0] fetch_x_d;
  always @(posedge clk) fetch_x_d <= fetch_x;

  wire in_win = win_ok &&
                ($signed({9'd0, fetch_x_d}) >= win_x0) &&
                ($signed({9'd0, fetch_x_d}) <= win_x1);

  wire show = gfx_enable && in_win;

  assign pal_addr   = show ? rd_pen[11:0] : bg_pen[11:0];
  assign pal_alt    = show ? rd_pen[12]   : 1'b0;
  assign pal_shadow = show ? rd_hwsh      : 1'b0;

endmodule

`default_nettype wire
