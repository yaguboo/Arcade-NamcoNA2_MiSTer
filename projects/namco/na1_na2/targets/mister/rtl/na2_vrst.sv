//============================================================================
//  NA-1 / NA-2 -- let the video raster outlive the board's reset
//
//  MiSTer holds the core in reset for the whole ROM download (RESET, plus
//  ioctl_download and the SDRAM self-test in this wrapper), and that held
//  na2_crtc, so the core put out no hsync, no vsync and no pixel enable for the
//  whole load.  The loading bar is drawn by the framework's OSD, which counts
//  lines and pixels of the video it is handed (sys/osd.v works from de_in /
//  hs_in / vs_in); with no raster it has nothing to draw on and an analog
//  monitor gets no sync at all.  HDMI is saved by the scaler, which makes its
//  own timing.  Reported on a CRT, 2026-10-14; the same fix worked on Super
//  Slams the same day (projects/vsystem/super_slams/rtl/video/ss_vrst.sv).
//
//  Two outputs:
//
//    rstc   the reset the CORE sees: `rst` stretched by one clock
//    vrst   the reset the RASTER sees: ~pll_locked, plus ONE clock on the clock
//           the board's reset is released
//
//  Why the stretch.  At the clock `rst` falls the raster is still on whatever
//  phase it free-ran to, and a core released on that same clock would see a
//  hsync / vblank that is about to be forced to counter 0.  Holding the core
//  one clock longer than `rst`, and restarting the raster on that clock, makes
//  both come out of reset on the SAME clock with the raster at counter 0 --
//  exactly the state they came out of reset in when the raster was simply held.
//  sim/tb_na2_vidrst.sv compares the two cycle for cycle on the real board top.
//
//  The raster is NOT restarted when `rst` rises, so a user Reset or the start of
//  a load does not interrupt the picture.
//
//  Local to this project.  The sibling lane has the same module under another
//  name; that is the second independent use, so it is a common/ CANDIDATE
//  (root CLAUDE.md 1.4), not promoted.
//============================================================================
`default_nettype none

module na2_vrst (
    input  wire clk,
    input  wire rst,
    input  wire pll_locked,
    output wire rstc,
    output wire vrst
);

  reg rst_q = 1'b1;
  always @(posedge clk) rst_q <= rst;

  assign rstc = rst | rst_q;
  assign vrst = ~pll_locked | (rst_q & ~rst);

endmodule

`default_nettype wire
