derive_pll_clocks
derive_clock_uncertainty

# ---------------------------------------------------------------------------
# SDRAM
#
# There are deliberately NO set_input_delay / set_output_delay constraints on
# the SDRAM pins.  An earlier version of this file had some, invented from
# memory, and the fitter duly failed them by 3.4 ns -- a made-up requirement
# missed by a made-up margin, which tells you nothing about the board.
#
# The established MiSTer practice, checked against MiSTer-devel/NES_MiSTer
# (NES.sdc) and MiSTer-devel/Genesis_MiSTer (Genesis.sdc), is to leave the
# SDRAM pins unconstrained and set the interface timing physically, with the
# phase of the clock driven out on SDRAM_CLK.
#
# Ours is 180 degrees (rtl/pll/pll_0002.v, phase_shift1 = -9977 ps, half of the
# 19954.9 ps period).  At 50.113 MHz that puts the SDRAM's clock edge 9.98 ns
# after the controller's, which leaves:
#   output path  9.98 ns for FPGA clock-to-out plus board delay plus SDRAM
#                setup -- comfortably more than the ~5 ns needed
#   input path   19.95 - 9.98 = 9.98 ns for SDRAM tAC plus board delay plus
#                FPGA setup -- comfortably more than the ~7 ns needed
#
# The half-period shift is also what sim/tb_na2_sdram.sv models, and it is what
# makes the read latch land three states after S_CMD rather than two.  See
# rtl/memory/na2_sdram.sv.
# Both margins are roughly 2x, which is why 180 degrees is a reasonable choice
# at this clock rate.  UNVERIFIED ON HARDWARE: if reads come back corrupted on
# a real DE10-Nano, this phase is the first thing to change.
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Framework video paths.  The scaler and the HQ2x filter are deeply pipelined
# and do not need to settle in a single core clock; the same relaxations appear
# in the reference cores above.
# ---------------------------------------------------------------------------
set_multicycle_path -to {*Hq2x*} -setup 4
set_multicycle_path -to {*Hq2x*} -hold 3

set_multicycle_path -from [get_clocks {*|pll|pll_inst|altera_pll_i|general[0].*|divclk}] -to {ascal|*} -setup 4
set_multicycle_path -from [get_clocks {*|pll|pll_inst|altera_pll_i|general[0].*|divclk}] -to {ascal|*} -hold 3
