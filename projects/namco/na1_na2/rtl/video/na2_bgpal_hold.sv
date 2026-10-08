//============================================================================
//  na2_bgpal_hold -- the backdrop colour while graphics are disabled
//
//  EMULATION_DERIVED
//  Matches MAME namcona1_v.cpp:131-143 and 564-572: a palette write while
//  vreg[0x8E] (GFX enable) is zero is stored in palette RAM but not shown; the
//  palette device is rebuilt from RAM at the next draw with graphics enabled.
//  While graphics are off MAME draws nothing but the backdrop pen,
//  (vreg[0xBA] & 0xF) << 8 (namcona1_v.cpp:559-561), so the only colours that
//  can be seen held are those sixteen entries.  This module keeps just those,
//  instead of a second 4096-entry palette.
//
//  FBNeo updates the colour immediately (d_namcona1.cpp:445-456), which is
//  what this core did before.  The two emulators diverge; the PCB behaviour is
//  NOT verified.
//  TODO(HARDWAREIZE): does the board transfer palette RAM to the DAC only
//  while the display is enabled, or is this MAME's own economy?
//
//  Why it is here: Numan Athletics, during a scene load with graphics off,
//  writes 0xE7E7 and 0.3 ms later 0x0000 to entry 0xF00 -- the backdrop
//  (C11C22 / C11C34, MEASURED(MAME) frames 4200, 4204, 7548, 7551).  Shown
//  live that is a lime band a few lines tall on the black screen, which is
//  what a player saw (docs/NUMAN_SCREEN_COLLAPSE_AUDIT.md 5.4).
//
//  Not reproduced: MAME rebuilds only when graphics are enabled AND the
//  display window is non-empty (namcona1_v.cpp:561).  Here the rebuild is at
//  the rising edge of GFX enable alone.  While graphics are enabled palette
//  writes are shown immediately in both, so the difference is confined to a
//  frame whose window is empty -- where the backdrop is all there is and MAME
//  would show the held colour a frame longer.
//============================================================================
`default_nettype none

module na2_bgpal_hold (
    input  wire        clk,
    input  wire        rst,
    input  wire        we,          // one clock per completed palette write
    input  wire [11:0] idx,         // palette entry, 0x000-0xFFF
    input  wire [1:0]  ds,          // {upper, lower} byte strobes
    input  wire [15:0] din,
    input  wire        gfx_enable,  // vreg[0x8E] != 0
    input  wire [3:0]  bg_sel,      // vreg[0xBA] & 0xF
    output wire [15:0] word         // the backdrop's colour word while gfx is off
);

  reg [15:0] ram  [0:15];   // mirror of palette RAM entries k<<8
  reg [15:0] disp [0:15];   // what MAME's palette device holds for them
  reg        gfx_d;

  wire        hit    = we && (idx[7:0] == 8'h00);
  wire [3:0]  k      = idx[11:8];
  wire [15:0] merged = { ds[1] ? din[15:8] : ram[k][15:8],
                         ds[0] ? din[7:0]  : ram[k][7:0] };

  integer i;
  always @(posedge clk) begin
    if (rst) begin
      gfx_d <= 1'b0;
      for (i = 0; i < 16; i = i + 1) begin
        ram[i]  <= 16'd0;
        disp[i] <= 16'd0;
      end
    end else begin
      gfx_d <= gfx_enable;
      if (hit) begin
        ram[k] <= merged;
        if (gfx_enable) disp[k] <= merged;
      end
      // graphics just came on: MAME's rebuild of the whole palette.  A write
      // landing on this same clock is part of what gets rebuilt.
      if (gfx_enable && !gfx_d)
        for (i = 0; i < 16; i = i + 1)
          disp[i] <= (hit && (k == i[3:0])) ? merged : ram[i];
    end
  end

  assign word = disp[bg_sel];

endmodule

`default_nettype wire
