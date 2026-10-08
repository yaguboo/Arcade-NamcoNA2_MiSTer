//============================================================================
//  Namco NA-2 -- one tilemap layer, one scanline
//
//  Renders layer `layer` for raw line `line` into the line buffer.  Driven by
//  na2_video, which runs the layers in priority order.
//
//  Reference (MAME_VERIFIED + FBNEO_CROSSCHECK -- the two agree pixel for
//  pixel; MAME namcona1_v.cpp draw_background()/tilemap_get_info(), FBNeo
//  d_namcona1.cpp draw_layer()):
//
//      depth      = vreg[0xBC] bit `layer`          1 = 4bpp, 0 = 8bpp
//      color_bank = (vreg[0xB0 + 2*layer] & 0xF) << 8
//      sy = (scrolly + line) & 0x1FF
//      for x = 0, 8, 16 ... :
//          sx    = (scrollx + x) & 0x1FF
//          tword = videoram[layer*0x1000 + (sx>>3) + ((sy>>3)<<6)]
//          tile  = tword & 0xFFF
//          color = ((tword & (depth ? 0x7000 : 0)) >> 8) + color_bank
//          mask  = (tword & 0x8000) ? 0xFF : shaperam[tile*8 + (sy&7)]
//          for xx = 0..7:
//              dx = x + xx - (sx & 7)
//              if mask bit (0x80 >> xx):
//                  pen = color | (cgram[tile*64 + (sy&7)*8 + xx] & (depth?0xF:0xFF))
//
//  Bit order, cross-checked because it is easy to get wrong:
//    * an 8bpp pixel value IS the byte, not a bit reversal.  MAME's gfx_layout
//      has planeoffset {0..7} and xoffset {0,8,16,...}, and MAME reads bits
//      MSB-first, so plane p lands on byte bit (7-p) and is placed at pen bit
//      (7-p) -- an identity mapping.  FBNeo confirms with a plain
//      `pgfx[xx] & pixel_mask`.
//    * 4bpp uses the LOW nibble (planeoffset {4,5,6,7} -> pen bits 3..0);
//      FBNeo confirms with `pixel_mask = 0xf`.
//    * the shape mask bit for pixel xx is (0x80 >> xx): bit 7 is leftmost.
//    * FBNeo's `pgfx[xx ^ 1]` and `sgfx[(sy&7) ^ 1]` are artefacts of FBNeo
//      storing these RAMs in host byte order.  We store them as 68000
//      big-endian words, so byte n of a tile is the high half of word n>>1
//      when n is even.  No XOR here.
//
//  Timing: six clocks per 8-pixel group, 39 groups, so ~239 clocks per layer
//  and ~956 for four of the 3184 system clocks in a line.
//
//  It used to be ten, and the four saved clocks are the next group's fetches
//  moved underneath this group's pixel writes.  Every address here is a
//  register, so a word asked for in cycle T arrives in T+2, and the group
//  reads three different memories in a chain: video RAM says which tile, the
//  tile says where to look in shape RAM and CG RAM.  Serialising that chain
//  costs six clocks of pure latency per group and only four do any work.  But
//  the video RAM and shape ports sit idle during the four write clocks, so the
//  next group's video RAM read is issued there instead:
//
//      L0   tile word arrives; issue shape and CG word 0
//      L1   issue CG word 1
//      L2   issue CG word 2, latch the mask, write pixels 0,1
//      L3   issue CG word 3,                  write pixels 2,3
//      L4                                     write pixels 4,5,
//           and issue the *next* group's video RAM read
//      L5                                     write pixels 6,7
//
//  so the next group's L0 lands exactly when its tile word is ready.  The
//  scroll-dependent registers (gx, sx) move at L1, where no pixel is being
//  written, which is what keeps one group's writes from seeing the next
//  group's x.
//
//  NOT implemented yet: the `xdata == 0xC001` direct-pixel line mode, which
//  Numan Athletics is the only game to use.  A Level 4 follow-up -- see
//  ROADMAP.md.  (The ROZ layer is `rtl/video/na2_roz.sv`.)
//============================================================================
`default_nettype none

module na2_tilemap (
    input  wire        clk,
    input  wire        rst,

    // --- job ---------------------------------------------------------------
    input  wire        start,          // one-cycle pulse
    output reg         busy,
    input  wire [1:0]  layer,          // 0..3
    input  wire [8:0]  line,           // raw scanline
    input  wire [2:0]  prio,           // priority value to stamp
    input  wire [8:0]  scrollx,
    input  wire [8:0]  scrolly,
    input  wire        depth4,         // 1 = 4bpp
    input  wire [3:0]  color_bank,

    // --- video memory read ports ------------------------------------------
    output reg  [14:0] vram_addr,
    input  wire [15:0] vram_data,
    output reg  [16:0] cg_addr,
    input  wire [15:0] cg_data,
    // The CG backend is not ready with cg_data.  Everything in this engine
    // holds; only the write strobes drop, because holding those high would
    // write the same pen again on every stalled clock.  na2_cgmem.sv.
    input  wire        cg_stall,
    output reg  [13:0] shape_addr,
    input  wire [15:0] shape_data,

    // --- line buffer, two pixels per clock ---------------------------------
    output reg         lb_wr_a,
    output reg  [8:0]  lb_addr_a,
    output reg  [12:0] lb_pen_a,
    output reg         lb_wr_b,
    output reg  [8:0]  lb_addr_b,
    output reg  [12:0] lb_pen_b,
    output wire [2:0]  lb_pri
);

  localparam int H_VISIBLE = 304;
  localparam int GROUPS    = (H_VISIBLE / 8) + 1;   // 39, one extra for fine scroll

  localparam S_IDLE = 4'd0,
             S_PRE  = 4'd1,    // issue the first group's video RAM read
             S_PRE1 = 4'd2,
             L0     = 4'd3,
             L1     = 4'd4,
             L2     = 4'd5,
             L3     = 4'd6,
             L4     = 4'd7,
             L5     = 4'd8;

  reg [3:0]  st;
  reg [5:0]  grp;              // 0 .. GROUPS-1
  reg [9:0]  gx;               // group base x, of the group being written
  reg [9:0]  nx_gx;            // ... and of the one being fetched
  reg [8:0]  sx, nx_sx, sy;
  reg [11:0] tile;
  reg [12:0] color;
  reg [7:0]  mask;
  reg        opaque;
  reg [2:0]  pn;               // index of the even pixel being written

  assign lb_pri = prio;

  // the tile word is used in the same clock it arrives, to get CG word 0 out
  // one cycle earlier than the registered copy would allow
  wire [11:0] tile_now    = vram_data[11:0];
  wire [16:0] cg_base_now = {tile_now, 5'd0} + {12'd0, sy[2:0], 2'd0};
  wire [16:0] cg_base     = {tile,     5'd0} + {12'd0, sy[2:0], 2'd0};

  wire [8:0]  sx_of_next = scrollx + gx[8:0] + 9'd8;
  wire [8:0]  sx_first   = scrollx;

  // same trick for the mask: shape RAM answers exactly as the first pair is
  // written, so that pair reads the bus and the other three read the register
  wire [7:0] shape_byte = opaque ? 8'hFF
                                 : (sy[0] ? shape_data[7:0] : shape_data[15:8]);
  wire [7:0] mask_now   = (st == L2) ? shape_byte : mask;

  // destination x for the pair currently being written
  wire signed [10:0] dxa = $signed({1'b0, gx}) + $signed({8'd0, pn})
                         - $signed({8'd0, sx[2:0]});
  wire signed [10:0] dxb = dxa + 11'sd1;
  wire vis_a = (dxa >= 0) && (dxa < H_VISIBLE) && mask_now[3'd7 - pn];
  wire vis_b = (dxb >= 0) && (dxb < H_VISIBLE) && mask_now[3'd6 - pn];
  wire [12:0] pen_a = color | {5'd0, depth4 ? {4'd0, cg_data[11:8]} : cg_data[15:8]};
  wire [12:0] pen_b = color | {5'd0, depth4 ? {4'd0, cg_data[3:0]}  : cg_data[7:0]};

  wire consuming = (st == L2) || (st == L3) || (st == L4) || (st == L5);

  function automatic [14:0] vram_of(input [8:0] sxv);
    vram_of = {1'b0, layer, 12'd0}          // layer * 0x1000
            + {9'd0, sxv[8:3]}              // sx >> 3
            + {3'd0, sy[8:3], 6'd0};        // (sy >> 3) << 6
  endfunction

  always @(posedge clk) begin
    if (rst) begin
      st      <= S_IDLE;
      busy    <= 1'b0;
      lb_wr_a <= 1'b0;
      lb_wr_b <= 1'b0;
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
      lb_wr_a <= 1'b0;
      lb_wr_b <= 1'b0;
    end else begin
      lb_wr_a <= 1'b0;
      lb_wr_b <= 1'b0;

      case (st)
        S_IDLE: if (start) begin
          busy  <= 1'b1;
          grp   <= 6'd0;
          gx    <= 10'd0;
          nx_gx <= 10'd0;
          sy    <= scrolly + line;         // 9-bit wrap == & 0x1FF
          st    <= S_PRE;
        end

        // ---- prologue: the first group's tile word ---------------------
        S_PRE: begin
          sx        <= sx_first;
          nx_sx     <= sx_first;
          vram_addr <= vram_of(sx_first);
          st        <= S_PRE1;
        end
        S_PRE1: st <= L0;

        // ---- L0: the tile word is on the bus ---------------------------
        L0: begin
          tile   <= tile_now;
          opaque <= vram_data[15];
          color  <= {1'b0, color_bank,
                     depth4 ? {1'b0, vram_data[14:12], 4'd0} : 8'd0};
          // shaperam byte = tile*8 + row  ->  word = tile*4 + (row >> 1)
          shape_addr <= {tile_now, 2'd0} + {12'd0, sy[2:1]};
          cg_addr    <= cg_base_now;
          pn         <= 3'd0;
          st         <= L1;
        end

        // gx and sx catch up here, where no pixel is being written
        L1: begin
          gx      <= nx_gx;
          sx      <= nx_sx;
          cg_addr <= cg_base + 17'd1;
          st      <= L2;
        end

        // MAME: tile-word bit 15 forces an opaque tile (TILE_FORCE_LAYER0),
        // which `shape_byte` already folds in
        L2: begin mask <= shape_byte; cg_addr <= cg_base + 17'd2; st <= L3; end
        L3: begin                     cg_addr <= cg_base + 17'd3; st <= L4; end

        // the next group's video RAM read goes out under this group's writes
        L4: begin
          nx_gx     <= gx + 10'd8;
          nx_sx     <= sx_of_next;
          vram_addr <= vram_of(sx_of_next);
          st        <= L5;
        end

        L5: begin
          if (grp + 6'd1 == GROUPS[5:0]) begin
            busy <= 1'b0;
            st   <= S_IDLE;
          end else begin
            grp <= grp + 6'd1;
            st  <= L0;
          end
        end

        default: st <= S_IDLE;
      endcase

      if (consuming) begin
        lb_wr_a   <= vis_a;
        lb_addr_a <= dxa[8:0];
        lb_pen_a  <= pen_a;
        lb_wr_b   <= vis_b;
        lb_addr_b <= dxb[8:0];
        lb_pen_b  <= pen_b;
        pn        <= pn + 3'd2;
      end
    end
  end

endmodule

`default_nettype wire
