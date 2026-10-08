//============================================================================
//  Namco NA-2 -- the direct-pixel line mode
//
//  When a layer's per-line scroll word is exactly 0xC001, that scanline is not
//  drawn from the tilemap at all: it comes straight out of video RAM as packed
//  8-bit pixels.  Numan Athletics is the only game that uses it, which is why
//  it cannot be deferred to "later games".
//
//  MAME_VERIFIED (draw_pixel_line() in namcona1_v.cpp), FBNEO_CROSSCHECK
//  (d_namcona1.cpp has the same special case, and MAME's own comment says this
//  is "a simplification, but produces the correct behavior for the only game
//  that uses this feature"):
//
//      for (x = 0; x < 38*8; x += 2) {
//          data = *pSource++;              // pSource = videoram + ydata + 25
//          pPri[x] = pPri[x+1] = 0xff;
//          pDest[x]   = paldata[data >> 8];
//          pDest[x+1] = paldata[data & 0xff];
//      }
//
//  Three details, all of them easy to lose:
//
//  * `paldata` is `&m_palette->pen(0)`, so the pen IS the byte -- no colour
//    bank, unlike every other layer.  MAME's comment next to it says this
//    breaks colours in the VS Express event and that the palette bank may live
//    somewhere else in this mode, so the arrangement is `UNKNOWN`, not settled.
//  * the source is a *word* offset: videoram + ydata + 25, where ydata is the
//    line-select word for this line.  The 25 is unexplained in both emulators.
//  * `pPri[x] = 0xff` covers all 304 pixels whether or not they were clipped,
//    and 0xff is the value sprites refuse to draw over.  na2_video turns that
//    into "set every occupancy bit", which is the same statement.
//
//  A later tilemap layer still draws over this one: MAME's tilemap draw writes
//  its pixels and its priority unconditionally, and only sprites test what is
//  already there.  So this engine is a drop-in replacement for one layer's
//  turn in the priority loop, nothing more.
//
//  Cost: 152 words at one per clock, so ~156 clocks -- cheaper than the 236 a
//  tilemap layer takes, and it replaces one.
//============================================================================
`default_nettype none

module na2_pixline #(
    parameter int H_VISIBLE = 304
) (
    input  wire        clk,
    input  wire        rst,

    input  wire        start,          // one-cycle pulse
    output reg         busy,
    input  wire [15:0] ydata,          // this line's line-select word
    input  wire [2:0]  prio,

    output reg  [14:0] vram_addr,
    input  wire [15:0] vram_data,

    output reg         lb_wr_a,
    output reg  [8:0]  lb_addr_a,
    output reg  [12:0] lb_pen_a,
    output reg         lb_wr_b,
    output reg  [8:0]  lb_addr_b,
    output reg  [12:0] lb_pen_b,
    output wire [2:0]  lb_pri
);

  localparam int NWORDS = H_VISIBLE / 2;      // 152

  assign lb_pri = prio;

  reg [14:0] base;
  reg [7:0]  i;                                // word being issued
  reg [7:0]  x1, x2;                           // and its x, two clocks behind
  reg        v1, v2;

  always @(posedge clk) begin
    if (rst) begin
      busy    <= 1'b0;
      lb_wr_a <= 1'b0;
      lb_wr_b <= 1'b0;
      v1      <= 1'b0;
      v2      <= 1'b0;
    end else begin
      lb_wr_a <= 1'b0;
      lb_wr_b <= 1'b0;

      if (!busy) begin
        if (start) begin
          busy <= 1'b1;
          base <= ydata[14:0] + 15'd25;
          i    <= 8'd0;
          v1   <= 1'b0;
          v2   <= 1'b0;
        end
      end else begin
        // issue
        vram_addr <= base + {7'd0, i};
        v1 <= (i < NWORDS[7:0]);
        x1 <= i;
        if (i < NWORDS[7:0]) i <= i + 8'd1;

        // one clock of slack, then the word is on the bus
        v2 <= v1;
        x2 <= x1;

        if (v2) begin
          lb_wr_a   <= 1'b1;
          lb_addr_a <= {x2, 1'b0};
          lb_pen_a  <= {5'd0, vram_data[15:8]};
          lb_wr_b   <= 1'b1;
          lb_addr_b <= {x2, 1'b1};
          lb_pen_b  <= {5'd0, vram_data[7:0]};
        end

        if ((i >= NWORDS[7:0]) && !v1 && !v2) busy <= 1'b0;
      end
    end
  end

endmodule

`default_nettype wire
