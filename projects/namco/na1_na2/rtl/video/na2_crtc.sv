//============================================================================
//  Namco NA-2 -- CRTC / video timing
//
//  HW_CONFIRMED (NA-2 GAME PCB notes): VSync 60 Hz, HSync 15.73 kHz, and one
//  50.113 MHz oscillator on the board.
//
//  INFERRED, from those three numbers: a pixel clock of master/8 with
//  H_total = 398 and V_total = 262 gives
//        50.113e6 / 8 / 398       = 15739.0 Hz   (PCB says 15.73 kHz)
//        50.113e6 / 8 / 398 / 262 =    60.07 Hz  (PCB says 60 Hz)
//  which is the best fit over divisors 6..9 and H_total 380..420.  master/8
//  is also exactly 8 system clocks per pixel, which gives the renderer a
//  comfortable time-multiplexing budget.  Not proven -- flagged as INFERRED
//  until someone measures a real board.
//
//  Line numbering follows MAME's `m_screen->vpos()` so that the posirq
//  comparison against vreg $8A means the same thing here as there.
//
//  Visible area: 304 x 224 at raw lines **32..255**.
//  MAME_VERIFIED: set_visarea(0, 38*8-1, 4*8, 32*8-1) over a 320x256 bitmap.
//  FBNEO_CROSSCHECK: d_namcona1.cpp draw_layer() iterates raw y 0..255 and
//  writes `pTransDraw + (y - 32) * nScreenWidth`, i.e. raw line 32 is the first
//  visible line.  This is not cosmetic: the per-line scroll table is indexed by
//  the *raw* line number (`scroll[y]`, `scroll[y + 0x100]`), so getting the
//  offset wrong would shift every layer's scroll by 32 lines.
//
//  SETTLED ON A BOARD 2026-08-26: the vblank line is the END OF THE PICTURE,
//  raw 256, not MAME's 224 or FBNeo's 240 -- both of which fall INSIDE the
//  active region at raw 32..255 and so cannot be literally right.  Emeraldia's
//  HOW TO PLAY screen decides it.  The game raises tilemap 2 over its leaping
//  dolphin by writing that layer's priority 0 in the vblank handler and 1 in
//  the raster (posirq) handler at raw 240.  With the interrupt at 224 the
//  board drew the dolphin ONLY in raw lines 227..240 -- the band between the
//  two handlers, the 227 being interrupt latency -- and with it at 256 the
//  picture matches MAME.  MAME gets away with 224 because it draws everything
//  above the posirq line in one pass AT scanline 240, after the handler has
//  run; a scanline renderer cannot.  docs/emeraldia_dolphin.md, KNOWN_ISSUES
//  ISSUE-1.
//============================================================================
`default_nettype none

module na2_crtc #(
    parameter int CLK_DIV  = 8,     // system clocks per pixel
    parameter int H_TOTAL  = 398,
    parameter int H_ACTIVE = 304,
    parameter int H_SYNC_S = 320,   // INFERRED: porch split not established
    parameter int H_SYNC_E = 356,
    parameter int V_TOTAL  = 262,
    parameter int V_START  = 32,    // first visible raw line
    parameter int V_END    = 256,   // one past the last visible raw line
    parameter int V_SYNC_S = 258,   // INFERRED: porch split not established
    parameter int V_SYNC_E = 261,
    parameter int V_BLANK_IRQ = 256 // default; overridden by irq_line_sel
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       ce_en,

    output reg        ce_pix,       // one system clock wide, once per pixel
    output reg  [8:0] hpos,
    output reg  [8:0] vpos,
    output wire       hblank,
    output wire       vblank,
    output reg        hsync,
    output reg        vsync,
    output reg        vblank_rise,  // one system clock, at the start of V_BLANK_IRQ

    // Which raw scanline raises the 68000's level-4 interrupt.  This is an
    // open question, not a setting: MAME uses 224 and FBNeo 240, both of which
    // land inside the visible region (raw 32..255), and neither has any
    // hardware backing.  It does not matter to an emulator that draws a whole
    // frame at once, and it matters a great deal to a scanline renderer -- an
    // interrupt 32 lines before the end of the picture lets the game rewrite
    // sprite RAM while the bottom of the frame is still being scanned, which
    // tears every object that moved.  Made selectable so a person in front of
    // the board can decide it by looking.  KNOWN_ISSUES.md ISSUE-1.
    //   0 = 256 (end of active display, the measured answer)
    //   1 = 224 (MAME)   2 = 240 (FBNeo)
    input  wire [1:0] irq_line_sel
);

  reg [3:0] divcnt;

  wire [8:0] irq_line = (irq_line_sel == 2'd1) ? 9'd224
                      : (irq_line_sel == 2'd2) ? 9'd240
                                               : V_BLANK_IRQ[8:0];

  always @(posedge clk) begin
    if (rst) begin
      divcnt      <= 4'd0;
      ce_pix      <= 1'b0;
      hpos        <= 9'd0;
      vpos        <= 9'd0;
      hsync       <= 1'b0;
      vsync       <= 1'b0;
      vblank_rise <= 1'b0;
    end else begin
      ce_pix      <= 1'b0;
      vblank_rise <= 1'b0;

      if (ce_en) begin
        if (divcnt == CLK_DIV - 1) begin
          divcnt <= 4'd0;
          ce_pix <= 1'b1;

          if (hpos == H_TOTAL - 1) begin
            hpos <= 9'd0;
            if (vpos == V_TOTAL - 1) vpos <= 9'd0;
            else begin
              vpos <= vpos + 9'd1;
              if (vpos + 9'd1 == irq_line) vblank_rise <= 1'b1;
            end
          end else
            hpos <= hpos + 9'd1;

          hsync <= (hpos >= H_SYNC_S) && (hpos < H_SYNC_E);
          vsync <= (vpos >= V_SYNC_S) && (vpos < V_SYNC_E);
        end else
          divcnt <= divcnt + 4'd1;
      end
    end
  end

  assign hblank = (hpos >= H_ACTIVE);
  assign vblank = (vpos < V_START) || (vpos >= V_END);

endmodule

`default_nettype wire
