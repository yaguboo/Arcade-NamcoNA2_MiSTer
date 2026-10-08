//============================================================================
//  Namco NA-2 -- palette decode
//
//  Palette RAM is 4096 x 16 bits at 0xF00000.  Each entry is read two ways
//  (MAME_VERIFIED: namcona1_state::update_palette in namcona1_v.cpp,
//   FBNEO_CROSSCHECK: palette_update_entry in d_namcona1.cpp -- identical
//   arithmetic):
//
//    normal          -RRRRRGG GGGBBBBB      straight 5:5:5
//    alternate       RRRGGGBB RRRGGGBB      two 3:3:2 halves, weighted 1:2
//
//  MAME builds both interpretations into one 8192-entry pen table: pens
//  0x0000-0x0FFF are the normal reading and 0x1000-0x1FFF the alternate one.
//  A sprite flagged for shadow selects the alternate bank by adding 0x1000 to
//  its pen (pdraw_tile), so the "alternate" reading is really the shadow /
//  blended palette.  MAME says the alternate form is used in-game by
//  Emeraldia; it costs almost nothing to support, so it is here.
//
//  The alternate channel value is (a + 2*b) scaled to 8 bits:
//      r: a = bits 7:5,  b = bits 15:13   -> 0..21, scale 255/21
//      g: a = bits 4:2,  b = bits 12:10   -> 0..21, scale 255/21
//      b: a = bits 1:0,  b = bits  9:8    -> 0..9,  scale 255/9
//  Small enough to be a lookup rather than a divider.
//
//  A third case sits on top of both: the hardware shadow table.
//
//  MAME_VERIFIED: shadows are enabled on the palette device
//  (PALETTE(...).enable_shadows()), and pdraw_tile picks that table only when
//  the sprite's colour is exactly 0x0F (8bpp) or 0xFF (4bpp) -- every other
//  shadow-flagged sprite takes the alternate palette bank above.  Where it
//  applies, it darkens the pixel *already on screen*; the sprite contributes
//  no pen at all, which is why na2_video carries it as a flag in the line
//  buffer rather than as a pen.
//
//  MAME_VERIFIED, and against MAME's *output* rather than its source: the rule
//  is floor(channel * 0.6), on the 8-bit channel after decode.  Attract
//  frame 4600, tools/compare_frame.py: 4f4999 is floor(0.6 * 847bff) and
//  49458f is floor(0.6 * 7b73ef).  615/1024 gives the same answer for every
//  8-bit input and costs shifts instead of a divider.
//
//  UNKNOWN: whether the real board darkens by 0.6.  What is established is
//  that MAME does, and that this game leans on the path -- the attract mode
//  keeps sprites with colour word 0x7FFE on screen for seconds at a time.
//  MAME's own TODO says "non-shadow pixels for sprites flagged to enable
//  shadows have bad colors", so the corner is not settled upstream either.
//============================================================================
`default_nettype none

module na2_palette (
    input  wire        clk,

    // palette RAM word, already fetched by the caller
    input  wire [15:0] pal_word,
    // 0 = normal 5:5:5, 1 = alternate (shadow / blended) reading
    input  wire        alt,
    // hardware shadow table: darken the decoded colour by 0.6
    input  wire        shadow,

    output reg  [7:0]  r,
    output reg  [7:0]  g,
    output reg  [7:0]  b
);

  // ---- normal: 5 bits per channel, replicated to 8 ----------------------
  // {v[4:0], v[4:2]} == round(v * 255 / 31) exactly for all 5-bit v.
  wire [4:0] n_r = pal_word[14:10];
  wire [4:0] n_g = pal_word[9:5];
  wire [4:0] n_b = pal_word[4:0];
  wire [7:0] r_norm = {n_r, n_r[4:2]};
  wire [7:0] g_norm = {n_g, n_g[4:2]};
  wire [7:0] b_norm = {n_b, n_b[4:2]};

  // ---- alternate: (a + 2b) scaled ---------------------------------------
  wire [4:0] a_r = {2'd0, pal_word[7:5]}  + {1'd0, pal_word[15:13], 1'b0};  // 0..21
  wire [4:0] a_g = {2'd0, pal_word[4:2]}  + {1'd0, pal_word[12:10], 1'b0};  // 0..21
  wire [3:0] a_b = {2'd0, pal_word[1:0]}  + {1'd0, pal_word[9:8],   1'b0};  // 0..9

  // round(n * 255 / 21) for n = 0..21
  function [7:0] scale21(input [4:0] n);
    case (n)
      5'd0:  scale21 = 8'd0;   5'd1:  scale21 = 8'd12;  5'd2:  scale21 = 8'd24;
      5'd3:  scale21 = 8'd36;  5'd4:  scale21 = 8'd48;  5'd5:  scale21 = 8'd60;
      5'd6:  scale21 = 8'd72;  5'd7:  scale21 = 8'd85;  5'd8:  scale21 = 8'd97;
      5'd9:  scale21 = 8'd109; 5'd10: scale21 = 8'd121; 5'd11: scale21 = 8'd133;
      5'd12: scale21 = 8'd145; 5'd13: scale21 = 8'd157; 5'd14: scale21 = 8'd170;
      5'd15: scale21 = 8'd182; 5'd16: scale21 = 8'd194; 5'd17: scale21 = 8'd206;
      5'd18: scale21 = 8'd218; 5'd19: scale21 = 8'd230; 5'd20: scale21 = 8'd242;
      default: scale21 = 8'd255;
    endcase
  endfunction

  // round(n * 255 / 9) for n = 0..9
  function [7:0] scale9(input [3:0] n);
    case (n)
      // MAME truncates: n * 0xff / (0x3*3), integer division.  These three
      // entries were rounded instead, which is one LSB high on 2, 5 and 8.
      // Measured on Emeraldia frames 2400 and 4594: 27 of 27 and 28 of 28
      // differing pixels, all of them alternate-palette sprite pixels, and
      // both frames go to 0 with these values.  docs/level11_second_game.md
      4'd0: scale9 = 8'd0;   4'd1: scale9 = 8'd28;  4'd2: scale9 = 8'd56;
      4'd3: scale9 = 8'd85;  4'd4: scale9 = 8'd113; 4'd5: scale9 = 8'd141;
      4'd6: scale9 = 8'd170; 4'd7: scale9 = 8'd198; 4'd8: scale9 = 8'd226;
      default: scale9 = 8'd255;
    endcase
  endfunction

  reg [7:0] r_d, g_d, b_d;

  always @(*) begin
    if (alt) begin
      r_d = scale21(a_r);
      g_d = scale21(a_g);
      b_d = scale9 (a_b);
    end else begin
      r_d = r_norm;
      g_d = g_norm;
      b_d = b_norm;
    end
  end

  // floor(v * 0.6) for every 8-bit v; 615/1024 is exact over that range and
  // synthesises to adds and shifts rather than a multiplier.
  function [7:0] darken(input [7:0] v);
    reg [17:0] t;
    begin
      t = v * 18'd615;
      darken = t[17:10];
    end
  endfunction

  always @(*) begin
    r = shadow ? darken(r_d) : r_d;
    g = shadow ? darken(g_d) : g_d;
    b = shadow ? darken(b_d) : b_d;
  end

endmodule

`default_nettype wire
