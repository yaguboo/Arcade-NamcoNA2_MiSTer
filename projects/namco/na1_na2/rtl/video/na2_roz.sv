//============================================================================
//  Namco NA-2 -- the ROZ layer, one scanline
//
//  The fifth background layer, `which == 4` in MAME's priority loop.  It is a
//  512x512 tilemap sampled through a 2x2 matrix, so unlike the four ordinary
//  tilemaps every pixel can land anywhere in the source and nothing about the
//  previous pixel helps.
//
//  MAME_VERIFIED (namcona1_v.cpp roz_get_info() and draw_background()'s
//  which == 4 branch, plus tilemap_t::draw_roz), FBNEO_CROSSCHECK
//  (d_namcona1.cpp predraw_roz()/draw_roz(), which agrees on every field --
//  see the one place it does not, below).
//
//      incxx = (s16)vreg[$C0] << 8      xoffset = (s16)vreg[$C8]
//      incxy = (s16)vreg[$C2] << 8      yoffset = (s16)vreg[$CA]
//      incyx = (s16)vreg[$C4] << 8      dx = 46, dy = -8
//      incyy = (s16)vreg[$C6] << 8
//      startx = (xoffset << 12) + incxx*dx + incyx*dy
//      starty = (yoffset << 12) + incxy*dx + incyy*dy
//
//      cx(x,y) = startx + x*incxx + y*incyx        16.16 fixed point
//      cy(x,y) = starty + x*incxy + y*incyy
//      drawn only where (u32)cx < 512<<16 and (u32)cy < 512<<16   (no wrap)
//
//  The source tile under (tx, ty) = (cx >> 16, cy >> 16):
//
//      data  = videoram[0x4000 + ((ty >> 5) << 6) + (tx >> 5)]
//      tile  = ((data & 0xFBF) + ((tx >> 3) & 3) + (((ty >> 3) & 3) << 6)) & 0xFFF
//      colour= depth4 ? ((data & 0x7000) >> 8) : 0,  plus (vreg[$BA] & 0xF) << 8
//      opaque= data & 0x8000, else masked through shaperam[tile*8 + (ty & 7)]
//
//  A logical ROZ tile is 4x4 ordinary tiles, which is where the >> 5 and the
//  `+ (c & 3) + ((r & 3) << 6)` come from.  The `& 0xFBF` drops bit 6 of the
//  tile number; MAME's comment calls it a patch for Emeraldia Japan and does
//  not explain it, so it is copied and tagged rather than justified.
//
//  Timing.  Three lookups per pixel -- video RAM, then shape and CG -- and
//  each depends on the one before, so the naive form is three clocks a pixel
//  and 912 in a line, which does not fit.  But they are three *separate*
//  memories, so the chain pipelines: pixel n's video RAM read, pixel n-2's
//  shape and CG reads and pixel n-4's line-buffer write all happen in the same
//  clock.  One pixel per clock, 308 clocks a line, which is about what one
//  ordinary tilemap layer costs.
//
//  MAME/FBNeo difference, recorded rather than split: MAME's draw_roz advances
//  the start point by cliprect.min_x * incxx before the first pixel, so
//  (startx, starty) belongs to screen pixel (0,0).  FBNeo starts its x loop at
//  clip.nMinx without that pre-advance, and separately adds 32*incyy for y.
//  For Numan Athletics that is an eight-pixel-worth horizontal difference.
//  MAME's convention is the one every other ROZ driver uses, and `dx = 46` was
//  tuned against it, so this follows MAME.  See MAME_FBNEO_DIFF.md.
//============================================================================
`default_nettype none

module na2_roz #(
    parameter int H_VISIBLE = 304
) (
    input  wire        clk,
    input  wire        rst,

    // --- job ---------------------------------------------------------------
    input  wire        start,           // one-cycle pulse
    output reg         busy,
    input  wire [2:0]  prio,            // priority value to stamp
    input  wire        depth4,          // vreg $BC bit 4
    input  wire [3:0]  color_bank,      // vreg $BA bits 3:0
    input  wire signed [31:0] cx0,      // cx at screen x = 0 for this line
    input  wire signed [31:0] cy0,
    input  wire signed [31:0] incxx,
    input  wire signed [31:0] incxy,

    // --- video memory read ports ------------------------------------------
    output reg  [14:0] vram_addr,
    input  wire [15:0] vram_data,
    output reg  [16:0] cg_addr,
    input  wire [15:0] cg_data,
    // The CG backend is not ready with cg_data.  The whole four-stage pipeline
    // holds -- n, cx, cy and every stage register -- so that a stall costs
    // time and nothing else.  na2_cgmem.sv.
    input  wire        cg_stall,
    output reg  [13:0] shape_addr,
    input  wire [15:0] shape_data,

    // --- line buffer, one pixel per clock ---------------------------------
    output reg         lb_wr,
    output reg  [8:0]  lb_addr,
    output reg  [12:0] lb_pen,
    output wire [2:0]  lb_pri
);

  localparam [14:0] ROZ_BASE = 15'h4000;      // videoram byte 0x8000

  assign lb_pri = prio;

  // ---- the sampling point ------------------------------------------------
  reg signed [31:0] cx, cy;
  reg [8:0] n;                                 // pixel being issued, 0..H_VISIBLE

  wire [8:0] tx = cx[24:16];
  wire [8:0] ty = cy[24:16];
  // wraparound is off, so a sample outside the 512x512 source is simply not
  // drawn.  The C does it with an unsigned compare against 512<<16, which is
  // the same as "no bits above 24 set".
  wire in_src = (cx[31:25] == 7'd0) && (cy[31:25] == 7'd0);

  // ---- pipeline ----------------------------------------------------------
  // a: issued this clock.  b: its video RAM word is on the bus.
  // c: its shape and CG addresses went out.  d: their data is on the bus.
  reg [8:0] a_x, b_x, c_x, d_x;
  reg [8:0] a_tx, b_tx, c_tx, d_tx;
  reg [8:0] a_ty, b_ty, c_ty, d_ty;
  reg       a_v, b_v, c_v, d_v;
  reg       stall_drained;
  reg [15:0] stall_vram_data;
  reg [12:0] c_color, d_color;
  reg        c_opaque, d_opaque;

  // The VRAM port itself is not stalled.  Once a CG miss freezes this
  // pipeline it advances from b's word to a's word, while b_* stays put.  The
  // word sampled on the first stall clock is therefore the one stage B must
  // use on the first resume clock.
  wire [15:0] b_vram_data = stall_drained ? stall_vram_data : vram_data;
  wire [11:0] b_tile = (b_vram_data[11:0] & 12'hFBF)
                      + {10'd0, b_tx[4:3]}
                      + {4'd0, b_ty[4:3], 6'd0};

  wire [7:0] d_shape_byte = d_ty[0] ? shape_data[7:0] : shape_data[15:8];
  wire [7:0] d_cg_byte    = d_tx[0] ? cg_data[7:0]    : cg_data[15:8];
  wire       d_mask       = d_opaque | d_shape_byte[3'd7 - d_tx[2:0]];
  wire [12:0] d_pen       = d_color | {5'd0, depth4 ? {4'd0, d_cg_byte[3:0]}
                                                    : d_cg_byte};

  always @(posedge clk) begin
    if (rst) begin
      busy  <= 1'b0;
      lb_wr <= 1'b0;
      n     <= 9'd0;
      a_v   <= 1'b0; b_v <= 1'b0; c_v <= 1'b0; d_v <= 1'b0;
      stall_drained <= 1'b0;
      stall_vram_data <= 16'd0;
    // A STALL MUST NOT SWALLOW `start`.
    //
    // `start` is a one-clock pulse.  Freezing the whole state machine on
    // cg_stall freezes S_IDLE too, so the pulse passes while the engine is
    // held and the pass is silently skipped -- no busy, no overflow flag, no
    // line-overrun flag, nothing.  And the stall that does it is not even this
    // engine's: na2_video muxes cg_addr by whose turn it is, so the instant
    // this engine's window opens the port carries ITS idle address, which is
    // not in na2_cgmem's row cache, and cg_stall is high before the engine has
    // asked for anything.
    //
    // Measured 2026-08-30 on a real MAME frame through the Pocket CG path:
    // sprite starts 448, busy 0 cycles, sp_cg_stall high for all 896 cycles of
    // the window, peak sprites 0.  On hardware: no characters, no objects,
    // correct text and backgrounds.
    //
    // MiSTer ties cg_stall to 0 (na2_cgmem's on-chip branch), so this line has
    // never done anything there.
    end else if (cg_stall && busy) begin
      // cg_stall describes the NEW address issued by stage B.  On its first
      // high clock, stage D still holds the previous address's completed
      // pixel and cg_data/shape_data still belong to that pixel.  Freezing D
      // here used to discard it; after the miss completed, the old D metadata
      // was combined with the new row's data.  Drain that one completed item,
      // then invalidate D while the rest of the pipeline remains frozen.
      lb_wr <= 1'b0;
      if (!stall_drained) begin
        stall_vram_data <= vram_data;
        if (d_v && d_mask) begin
          lb_wr   <= 1'b1;
          lb_addr <= d_x;
          lb_pen  <= d_pen;
        end
        d_v          <= 1'b0;
        stall_drained <= 1'b1;
      end
    end else begin
      lb_wr <= 1'b0;
      stall_drained <= 1'b0;

      if (!busy) begin
        if (start) begin
          busy <= 1'b1;
          cx   <= cx0;
          cy   <= cy0;
          n    <= 9'd0;
          a_v  <= 1'b0; b_v <= 1'b0; c_v <= 1'b0; d_v <= 1'b0;
        end
      end else begin
        // ---- stage A: issue the video RAM word for pixel n ---------------
        vram_addr <= ROZ_BASE + {5'd0, ty[8:5], 6'd0} + {11'd0, tx[8:5]};
        a_x  <= n;
        a_tx <= tx;
        a_ty <= ty;
        a_v  <= (n < H_VISIBLE[8:0]) && in_src;
        if (n < H_VISIBLE[8:0]) begin
          n  <= n + 9'd1;
          cx <= cx + incxx;
          cy <= cy + incxy;
        end

        // ---- one clock of slack so the word lines up with b -------------
        b_x <= a_x; b_tx <= a_tx; b_ty <= a_ty; b_v <= a_v;

        // ---- stage B: vram_data belongs to b; issue shape and CG --------
        shape_addr <= {b_tile, 2'd0} + {12'd0, b_ty[2:1]};
        cg_addr    <= {b_tile, 5'd0} + {12'd0, b_ty[2:0], 2'd0}
                    + {14'd0, b_tx[2:1]};
        c_x      <= b_x;
        c_tx     <= b_tx;
        c_ty     <= b_ty;
        c_v      <= b_v;
        c_opaque <= b_vram_data[15];
        c_color  <= {1'b0, color_bank,
                     depth4 ? {1'b0, b_vram_data[14:12], 4'd0} : 8'd0};

        d_x <= c_x; d_tx <= c_tx; d_ty <= c_ty; d_v <= c_v;
        d_color <= c_color; d_opaque <= c_opaque;

        // ---- stage C: shape_data and cg_data belong to d ---------------
        if (d_v && d_mask) begin
          lb_wr   <= 1'b1;
          lb_addr <= d_x;
          lb_pen  <= d_pen;
        end

        // four clocks after the last issue, the last pixel has been written
        if ((n >= H_VISIBLE[8:0]) && !a_v && !b_v && !c_v && !d_v)
          busy <= 1'b0;
      end
    end
  end

endmodule

`default_nettype wire
