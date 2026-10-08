//============================================================================
//  na2_m37702_periph -- the Namco C70's on-chip peripherals, MCU 0x00-0x7F
//
//  The half of the M37702 that no existing 7700 FPGA core implements: the
//  I/O ports (and with them the input multiplexer and the 68000's reset line),
//  the interrupt controller, the eight timers, and the A-D converter.  The CPU
//  core is na2_m37702_cpu; the wrapper that decodes 0x00-0x7F to this block is
//  rtl/mcu/na2_c70.sv.  Contract: docs/m37702_integration_spec.md sections 3
//  and 5.
//
//  Register addresses are MAME_VERIFIED from the M37702M2 internal map in
//  reference/mame/m37710.cpp (BSD-3-Clause), and the board-level behaviour of
//  ports P4-P8 is MAME_VERIFIED from reference/mame/namcona1.cpp.
//
//  ------------------------------------------------------------------------
//  What the C70 BIOS actually uses -- MEASURED, not assumed
//  ------------------------------------------------------------------------
//  ROM_DISASM_CONFIRMED, by a recursive-descent walk of c70.bin from its own
//  vector table (`python tools/c70_analyze.py sfr`, non-indexed operands).
//  Every claim below is a program counter in the BIOS.  This is what decides
//  which parts of this file are complete and which are registers that count
//  but are never read:
//
//    FULLY IMPLEMENTED AND EXERCISED BY THE BIOS
//      P4  0x0A/0x0C   dir=0x0C at C076; P4=0x04 at C067 (68000 held),
//                      P4=0x0C at C646 (68000 released).  dir goes 0xFC at
//                      CD3B and back to 0x0C at CCDB.
//      P5  0x0B/0x0D   dir=0x0A at C073.  The bit0-mirrors-bit1 quirk is not
//                      cosmetic: the BIOS busy-waits on it at CCEA, CCFA,
//                      CD01, CD19, CD54, CD5B, CD74, CD7B, CD98, CD9F, CDAE,
//                      CDB5 (BBS/BBC #$01,$0b around CLB/SEB #$02,$0b).
//                      Without the mirror the BIOS hangs in those loops.
//      P6  0x0E/0x10   dir=0xE3 at C070.  Mux select, see below.
//      P7  0x0F/0x11   dir=0x00 at C06D/D26F (mux read), 0xFF at D330/D39A
//                      (used as an output port), back to 0x00 at D0BC.
//      P8  0x12/0x14   dir=0xBB at C06A; bit 5 toggled at C13F/C143, C16B,
//                      C18E, C277, C534, C5BC, C5F7 (MAME: "watchdog?").
//      Timer A0        mode 0x80 (f64, timer mode) at C0ED, reload 0x065A at
//                      C0FA and C3F6, started at C0FE/C1F5/C401, stopped at
//                      C284.  Interrupt priority 3 at C0F0.  Period is
//                      64 * (0x065A + 1) = 104128 MCU clocks = 120.31 Hz at
//                      12.5282 MHz.
//
//                      THIS IS THE ONE THAT HAS TO BE EXACTLY RIGHT.  It is
//                      the sound driver's tick, and boot does not proceed
//                      without it: C109 CLI, C10B LDM #$00,$85, and then
//                      C10E LDA $85 / C110 BEQ C10E spins on internal RAM
//                      0x85 until an interrupt sets it.  The Timer A0 vector
//                      0xFFEE reaches C397, which is JMP ($01EA) through a
//                      dispatch table the BIOS builds in internal RAM.  A
//                      timer that never fires is a machine that never boots,
//                      and a timer with the wrong period is music at the
//                      wrong tempo -- sim/tb_na2_periph.sv measures this
//                      exact configuration in MCU clocks.
//      Timer A1        mode 0x00 (f2, timer mode) at CF57, reload written
//                      from RAM at D552, started at D555, stopped at D54B /
//                      D5AA / D7F5.  Its request bit is cleared at D558 but
//                      its priority is never set non-zero, so it never
//                      interrupts -- it counts and is never read.  (It is the
//                      UART1 receive path's timeout.)
//      Timer A4        mode 0x80 (f64) at C0F3, reload written at C41B,
//                      started at C41E, stopped at C36D/C4CD.  Priority 2 at
//                      C0F6.
//      A-D converter   sweep select 0x03 at CF38, control 0xD0 at D1F9
//                      (f2/2 clock, sweep, start).  ADC interrupt priority 5
//                      at D1F3/D1F6.  The results are read at D202-D23A and
//                      reassembled into player 3's byte -- see below.
//      Interrupt ctl   0x70 (ADC), 0x71/0x72 (UART0), 0x73 (UART1), 0x75
//                      (TA0), 0x76 (TA1), 0x79 (TA4), 0x7D (IRQ0, priority 3
//                      at C0E1), 0x7E (IRQ1/vblank, priority 6 at CCA5 and
//                      0 = disabled at CCAA), 0x7F (IRQ2, priority 4 at C0E7).
//                      All sixteen are cleared to 0x00 at CF8C-CFB9.
//
//    IMPLEMENTED AS REGISTERS THAT COUNT BUT ARE NEVER READ
//      Timer A2/A3     mode 0x11 at CF53/CF56 = event counter mode, up/down
//                      from the timer-out pin.  Reload 0x8000 at CF70/CF74,
//                      rewritten at D3AF/D3C1, started at D3B2/D3C4.  Nothing
//                      on the NA-2 drives the TA2IN/TA3IN pins, so on the real
//                      board they never count either (UNKNOWN -- the pins are
//                      not traced).  The registers read back; no interrupt is
//                      ever generated, which is what MAME does too.
//      Timer B0/B1/B2  mode 0x00 and reload 0x0000 at CF59-CF62 and
//                      CF7C-CF84, but count-start bits 5-7 are never set
//                      (the only writes to 0x40 are bits 0-4), so they never
//                      run.  Implemented anyway.
//      UART0/UART1     initialised at CF3B-CF4D (mode 0x05, baud 0x29, ctrl0
//                      0x04, ctrl1 0x00) and used at D40C/D47F (transmit) and
//                      D49E/D55B (receive).  Modelled exactly as MAME does:
//                      the registers hold their values, the receive buffer
//                      reads 0, and no UART interrupt is ever raised.  MAME
//                      runs the game this way, so the BIOS does not depend on
//                      a transmit-empty flag -- but that is MAME_VERIFIED, not
//                      HW_CONFIRMED, and it is the most likely place for this
//                      module to hang if it ever does.  See KNOWN RISK below.
//      Watchdog        0x60 kicked at C147, C173, C196, C27D, C53A, C5C4,
//                      C5FF; frequency register 0x61 never written.  No
//                      watchdog reset is generated (MAME does not either) --
//                      a watchdog that never fires cannot hang the BIOS, one
//                      that fires early can.
//      DMA 0x6C-0x6F   not present on the M37702M2 at all (they are the
//                      M37720S1's).  Decoded as interrupt control registers so
//                      an access is visible rather than lost.  INFERRED.
//
//  ------------------------------------------------------------------------
//  How the CPU is told about an interrupt
//  ------------------------------------------------------------------------
//  The CPU is the vendored m37702.vhd (XelaNotPu/SYSTEM11_MiSTer), which does
//  its own IPL comparison internally.  It therefore takes one request line per
//  source plus that source's priority, rather than an arbitrated
//  {request, source, priority, acknowledge} bus.  This block presents:
//
//    irq0 / irq1 / irq2   the three external INT pins, one ce-wide pulse per
//                         event (the core edge-detects and latches them)
//    tick_ta0..tick_tb2   one ce-wide pulse when that timer underflows
//    tick_ad              one ce-wide pulse when a conversion sequence ends
//    prio_*               bits [2:0] of that source's interrupt control
//                         register, 0 meaning disabled
//
//  Bit 3 of each control register, the request flag, is still implemented
//  because software can read and clear it -- it is simply no longer how the
//  CPU finds out.  See the interrupt controller section below for the two
//  places this differs from the real part, both tagged.
//
//  ------------------------------------------------------------------------
//  The input multiplexer
//  ------------------------------------------------------------------------
//  MAME_VERIFIED (namcona1.cpp port6_w / port7_r) and independently
//  ROM_DISASM_CONFIRMED from the BIOS's own polling routine at 0x00D26E
//  (RESEARCH.md section 13 quotes it in full):
//
//      P6 bit 7 = 0 enables the mux; P6 bits 6:5 select
//         0 = P4, 1 = DSW, 2 = P1, 3 = P2
//      P7 reads the selected port, with P7's direction register set to
//         "all inputs" first (LDM #$00,$11 at D26F).
//
//      D272  CLB #$e0,$0e   mux on, select 0
//      D275  SEB #$40,$0e   select 2 = P1   -> D278 LDA $0f -> 0x2FFE
//      D296  SEB #$60,$0e   select 3 = P2   -> D299 LDA $0f -> 0x2FFF
//      D2BA  SEB #$20,$0e   select 1 = DSW  -> D2BD LDA $0f -> 0x2FFD
//      D2EA  CLB #$60,$0e   select 0 = P4   -> D2ED LDA $0f -> 0x2FFC
//
//  Two independent sources agreeing on the same four select values is
//  unusually good evidence, so this is the one part of this file that is not
//  "MAME says so".
//
//  ------------------------------------------------------------------------
//  Player 3 and the A-D converter -- ROM_DISASM_CONFIRMED
//  ------------------------------------------------------------------------
//  RESEARCH.md section 13 records "UNKNOWN -- where the BIOS reads the A/D
//  converter and where it deposits the result has not been located."  It is
//  located: the routine at 0x00D1FD reads the eight results in this order and
//  rotates one bit out of each into external SRAM 0x8086, then stores the
//  active-high copy at 0x8006:
//
//      D202 LDA $26 (AN3)   D20A LDA $28 (AN4)   D212 LDA $2a (AN5)
//      D21A LDA $2c (AN6)   D222 LDA $24 (AN2)   D22A LDA $22 (AN1)
//      D232 LDA $20 (AN0)   D23A LDA $2e (AN7)
//      D260 ASL A / ROR $8086 ...        (one bit per result, LSB first)
//      D242 LDB $8086 / EORB #$ff / STB $8006
//
//  Against the scramble in RESEARCH.md section 13 --
//      AN0<-bit6 AN1<-bit5 AN2<-bit4 AN3<-bit0 AN4<-bit1 AN5<-bit2
//      AN6<-bit3 AN7<-bit7
//  -- that read order is exactly player 3's bits 0,1,2,3,4,5,6,7 in order.
//  The scramble and the read order are inverses of each other, which is what
//  a reassembly routine has to look like, so both are now confirmed from the
//  ROM as well as from MAME.
//
//  The conversion is started at 0xD1F2 (control = 0xD0) and the results are
//  read with NO status poll at all, so the BIOS cannot stall on a busy flag.
//  It relies on the ADC interrupt (priority 5, vector 0xFFD6 -> 0xC373).
//  This block always completes a conversion in the MAME_VERIFIED number of
//  clocks and always raises the interrupt at the end of the sweep.
//
//  Player 3 is REAL as of 2026-08-26: `in_p3` is a port, wired up through
//  na2_c70 and na2_top to joystick_2.  It used to be tied to the P3_IDLE
//  parameter -- the scramble and the conversion were right, and nothing was
//  ever fed to them, so player 3 pressed nothing.
//
//  The old note here said "UNKNOWN -- where the 68000 reads player 3".  That
//  was never really unknown: MAME runs the same M37702 on the same c70.bin and
//  player 3 works there, so the BIOS does the reassembly (it is
//  ROM_DISASM_CONFIRMED at 0xD202-0xD23A below) and puts the result somewhere
//  the game finds.  What was missing was on this side of the wire.
//
//  P3_IDLE remains as the value the port is compared against in
//  sim/tb_na2_periph.sv, which drives the port directly now.
//
//  ------------------------------------------------------------------------
//  KNOWN RISK, recorded rather than hidden
//  ------------------------------------------------------------------------
//  UNKNOWN -- the UART transmit-empty flag.  MAME never sets bit 3 of the
//  UART control register 0 and never raises a UART interrupt, and the game
//  runs, so this follows MAME.  If the real BIOS ever polls that flag on a
//  path MAME does not take, this block will spin there.  It is the first
//  thing to check if the MCU hangs with the PC inside 0xD400-0xD560.
//============================================================================
`default_nettype none

module na2_m37702_periph #(
    // Kept only so an old instantiation still elaborates; in_p3 is a port now.
    parameter [7:0] P3_IDLE = 8'hFF
) (
    input  wire        clk, rst, ce,          // ce = one pulse per MCU clock (master/4)
    input  wire        sel,                   // address 0x000000-0x00007F selected
    input  wire        we,                    // one-clock write strobe
    input  wire  [6:0] addr,                  // register byte address
    input  wire  [7:0] din,
    output wire  [7:0] dout,                  // registered, valid one clock after the address
    input  wire  [7:0] in_p1, in_p2, in_p4, in_dsw,   // JAMMA order, active low
    // Player 3, active low.  Not part of the digital mux: its eight bits
    // arrive one per ANALOG input, scrambled -- see ana_bit() below.
    input  wire  [7:0] in_p3,
    input  wire        irq0_set,              // mailbox doorbell from the 68000
    input  wire        irq1_set,              // vblank
    output wire        m68k_rst,              // 68000 held in reset; released by P4 bit 3
    output reg         soft_rst,              // processor mode register bit 3: software reset

    // --- interrupts, in the shape the vendored m37702.vhd core wants ------
    // external INT pins.  The core edge-detects these under its own `ce` and
    // holds the request in irqN_pend until it takes it, so what it needs from
    // here is one clean rising edge per event -- see the note at the
    // interrupt controller below.
    output wire        irq0,                  // mailbox doorbell   vector 0xFFF4
    output wire        irq1,                  // vblank             vector 0xFFF2
    output wire        irq2,                  // unused on the NA-2 vector 0xFFF0

    // one-clock ticks from the on-chip timers and the A-D, each aligned with
    // `ce` so the core samples them
    output wire        tick_ta0, tick_ta1, tick_ta2, tick_ta3, tick_ta4,
    output wire        tick_tb0, tick_tb1, tick_tb2, tick_ad,

    // each source's priority = its interrupt-control register bits [2:0]
    output wire  [2:0] prio_int0, prio_int1, prio_int2,
    output wire  [2:0] prio_ta0, prio_ta1, prio_ta2, prio_ta3, prio_ta4,
    output wire  [2:0] prio_tb0, prio_tb1, prio_tb2, prio_ad
);

  // ======================================================================
  // Declarations.  All of them, before any use -- HANDOFF.md section 4
  // item 8: ModelSim rejects the file otherwise and this project lost two
  // sessions to exactly that.
  // ======================================================================

  // --- the write strobe, qualified by the region select -------------------
  wire wr = sel & we;

  // --- I/O ports P0-P8 ----------------------------------------------------
  // MAME_VERIFIED (m37710.cpp m37702m2_device::map): the port registers live
  // at 0x02-0x15 on even bytes for P0/P2/P4/P6/P8 and 0x03-0x11 on odd bytes
  // for P1/P3/P5/P7, data register first and direction register two bytes
  // later.  In m37710.cpp's own terms: p = (offset & ~1) + Base, and
  // BIT(offset,0) selects the direction register.
  //
  //   0x02 P0   0x04 P0dir   0x03 P1   0x05 P1dir
  //   0x06 P2   0x08 P2dir   0x07 P3   0x09 P3dir
  //   0x0A P4   0x0C P4dir   0x0B P5   0x0D P5dir
  //   0x0E P6   0x10 P6dir   0x0F P7   0x11 P7dir
  //   0x12 P8   0x14 P8dir
  reg  [7:0] p_reg [0:8];      // last byte written, raw (m37710.cpp m_port_regs)
  reg  [7:0] p_dir [0:8];      // direction, 1 = output      (m_port_dir)

  // The board-side latches namcona1.cpp keeps for the pins it drives.  MAME
  // only calls the out callback when the direction register is non-zero and
  // masks the data with it (set_port_reg), so a write while the port is still
  // all-inputs never reaches the pin.  The C70 BIOS depends on that: it writes
  // P4/P5/P6 at C061-C067 *before* setting their directions at C070-C076.
  reg  [7:0] pin4, pin5, pin6, pin8;
  reg        m68k_rst_r;

  // Port address decode
  wire       p_even  = (addr >= 7'h02) && (addr <= 7'h14) && !addr[0];
  wire       p_odd   = (addr >= 7'h03) && (addr <= 7'h11) &&  addr[0];
  wire       p_hit   = p_even | p_odd;
  // p_off is only meaningful when p_hit, where the subtraction is in range;
  // the truncation to 4 bits is deliberate.
  wire [6:0] p_off7  = p_even ? ((addr - 7'h02) >> 1) : ((addr - 7'h03) >> 1);
  wire [3:0] p_off   = p_off7[3:0];
  wire [3:0] p_idx   = {p_off[3:1], 1'b0} + (p_even ? 4'd0 : 4'd1);
  wire       p_isdir = p_off[0];

  reg  [7:0] mux_val;          // what port 7 reads
  reg  [7:0] p_pin;            // the "pin" side of the addressed port
  reg  [7:0] p_masked;         // din & p_dir[p_idx], what MAME hands the callback

  // --- A-D converter ------------------------------------------------------
  // MAME_VERIFIED (m37710.cpp ad_register_map / ad_control_w / ad_timer_cb):
  //   0x1E ad_control  b7 clock (0 = f2/4, 1 = f2/2), b6 start/busy,
  //                    b4 sweep, b3 repeat, b2:0 channel
  //   0x1F ad_sweep    b1:0 selects how far a sweep runs: last channel is
  //                    (sweep & 3) * 2 + 1
  //   0x20-0x2F        eight 16-bit results, little endian
  reg  [7:0]  ad_control, ad_sweep;
  reg  [15:0] ad_result [0:7];
  reg  [9:0]  ad_cnt;
  reg         ad_run;
  wire [9:0]  ad_period = ad_control[7] ? 10'd228 : 10'd456;  // 57 * 2 * (2 or 4)
  wire        ad_tick   = ce && ad_run && (ad_cnt == ad_period - 10'd1);
  wire [2:0]  ad_line   = ad_control[2:0];
  wire        ad_more   = ad_control[3]
                        || (ad_control[4] && (ad_line != {ad_sweep[1:0], 1'b1}));
  wire        ad_irq    = ad_tick && !ad_more;
  reg  [15:0] ad_sample;       // what the addressed channel converts to

  // --- UARTs --------------------------------------------------------------
  reg  [7:0]  u_mode [0:1];
  reg  [7:0]  u_baud [0:1];
  reg  [15:0] u_tbuf [0:1];
  reg  [7:0]  u_ctl0 [0:1];
  reg  [7:0]  u_ctl1 [0:1];
  wire        u_i  = addr[3];          // 0x30-0x37 = UART0, 0x38-0x3F = UART1
  wire [2:0]  u_rg = addr[2:0];

  // --- timers -------------------------------------------------------------
  // MAME_VERIFIED (m37710.cpp timer_register_map, m37710_recalc_timer,
  // m37710_timer_cb):
  //   0x40 count start   bit i enables timer i
  //   0x42 one-shot start (write only)
  //   0x44 up/down       reads back masked to 0x1F
  //   0x46-0x55          eight 16-bit reload/count registers, A0..A4,B0..B2
  //   0x56-0x5D          eight mode registers
  // Mode bits 1:0 pick the mode (00 = timer), bits 7:6 pick the count source
  // f2 / f16 / f64 / f512.  m37710_recalc_timer makes the period
  // tscale * (register + 1) MCU clocks, with tscales = {2,16,64,512}.
  reg  [7:0]  count_start, one_shot, up_down;
  reg  [15:0] tmr_reload [0:7];
  reg  [15:0] tmr_cnt    [0:7];
  reg  [8:0]  tmr_pre    [0:7];
  reg  [7:0]  tmr_mode   [0:7];
  reg  [7:0]  tmr_run, tmr_tick, tmr_fire;
  // t_idx / m_idx are likewise only meaningful inside their own address
  // ranges, where the subtraction gives 0..7.
  wire [6:0]  t_idx7 = (addr - 7'h46) >> 1;
  wire [6:0]  m_idx7 = (addr - 7'h56);
  wire [2:0]  t_idx  = t_idx7[2:0];
  wire [2:0]  m_idx  = m_idx7[2:0];

  // --- processor mode and watchdog ---------------------------------------
  reg  [7:0]  proc_mode, wdog_freq;

  // --- interrupt controller ----------------------------------------------
  // MAME_VERIFIED (m37710.cpp int_control_r/w and m37710i_update_irqs):
  // one byte per source, bits 2:0 priority, bit 3 request, and for the three
  // external INT lines bit 4 polarity and bit 5 level-sense.  Indexed here by
  // the M37710_LINE_* number (reference/mame/m37710.h), because the CPU core
  // derives its vector from that number: vector = 0xFFCE + 2 * line.
  reg  [2:0]  ic_pri [0:19];
  reg  [19:0] ic_ir;
  reg  [1:0]  ic_pol [0:3];    // IRQ0/IRQ1/IRQ2 bits 5:4, indexed addr[1:0]
  wire        ic_hit = (addr >= 7'h6C);

  // External INT pin shapers.  intN_q remembers an event, intN_st issues it
  // as exactly one ce-wide high pulse.
  reg  [1:0]  int0_st, int1_st;
  reg         int0_q,  int1_q;
  wire [4:0]  ic_wl;           // the line a write to `addr` selects

  // --- read path ----------------------------------------------------------
  reg  [7:0]  rd;
  reg  [7:0]  dout_r;

  // --- region hits -------------------------------------------------------
  // These five are read ONLY by the diagnostics in the `ifndef SYNTHESIS
  // block below (the read mux uses p_hit / ic_hit, which stay here), so
  // under synthesis they were assigned and never read.  Declared beside
  // their use instead.  2026-08-27.
  // hit_uart stays here: the read mux reads it (lines ~609, ~945).
  wire hit_uart  = (addr >= 7'h30) && (addr <= 7'h3F);
  // hit_any / hit_stub moved into the `ifndef SYNTHESIS block below:
  // they are read only there, so under synthesis they were assigned and
  // never read.  Declared beside their use instead of deleted.

  // Registers that exist but whose behaviour is not modelled -- every access
  // to one of these is reported in simulation.  Project rule: never silently
  // return zero, and never let an unexpected dependency hide.

  integer i, t;

  // ======================================================================
  // Helper functions
  // ======================================================================

  // MAME_VERIFIED: static const int tscales[4] = { 2, 16, 64, 512 };
  function automatic [8:0] tscale_m1(input [1:0] s);
    case (s)
      2'd0: tscale_m1 = 9'd1;
      2'd1: tscale_m1 = 9'd15;
      2'd2: tscale_m1 = 9'd63;
      default: tscale_m1 = 9'd511;
    endcase
  endfunction

  // Interrupt control register address -> M37710_LINE_* number.
  // MAME_VERIFIED, from ad_register_map / uart0_register_map /
  // uart1_register_map / timer_register_map / irq_register_map, plus the
  // M37720S1's DMA controls at 0x6C-0x6F (INFERRED here: the M37702M2 leaves
  // 0x6C-0x6F unmapped, but decoding them costs nothing and makes an
  // unexpected access visible).
  function automatic [4:0] ic_line(input [6:0] a);
    case (a)
      7'h6C:   ic_line = 5'd3;    // DMA0
      7'h6D:   ic_line = 5'd2;    // DMA1
      7'h6E:   ic_line = 5'd1;    // DMA2
      7'h6F:   ic_line = 5'd0;    // DMA3
      7'h70:   ic_line = 5'd4;    // A-D
      7'h71:   ic_line = 5'd7;    // UART0 transmit
      7'h72:   ic_line = 5'd8;    // UART0 receive
      7'h73:   ic_line = 5'd5;    // UART1 transmit
      7'h74:   ic_line = 5'd6;    // UART1 receive
      7'h75:   ic_line = 5'd16;   // Timer A0
      7'h76:   ic_line = 5'd15;   // Timer A1
      7'h77:   ic_line = 5'd14;   // Timer A2
      7'h78:   ic_line = 5'd13;   // Timer A3
      7'h79:   ic_line = 5'd12;   // Timer A4
      7'h7A:   ic_line = 5'd11;   // Timer B0
      7'h7B:   ic_line = 5'd10;   // Timer B1
      7'h7C:   ic_line = 5'd9;    // Timer B2
      7'h7D:   ic_line = 5'd19;   // INT0  -- mailbox doorbell
      7'h7E:   ic_line = 5'd18;   // INT1  -- vblank
      default: ic_line = 5'd17;   // 0x7F, INT2
    endcase
  endfunction

  assign ic_wl = ic_line(addr);

  // Player 3's bit for analog input `ch`.
  // MAME_VERIFIED (namcona1.cpp c70() an0_cb..an7_cb -> portana_r<Bit>) and
  // ROM_DISASM_CONFIRMED by the reassembly order at 0xD202-0xD23A:
  //   AN0<-bit6 AN1<-bit5 AN2<-bit4 AN3<-bit0 AN4<-bit1 AN5<-bit2
  //   AN6<-bit3 AN7<-bit7
  // portana_r returns 0xFFFF when the (active low) bit is 1, else 0x0000.
  function automatic [2:0] ana_bit(input [2:0] ch);
    case (ch)
      3'd0: ana_bit = 3'd6;
      3'd1: ana_bit = 3'd5;
      3'd2: ana_bit = 3'd4;
      3'd3: ana_bit = 3'd0;
      3'd4: ana_bit = 3'd1;
      3'd5: ana_bit = 3'd2;
      3'd6: ana_bit = 3'd3;
      default: ana_bit = 3'd7;
    endcase
  endfunction

  // ======================================================================
  // The input multiplexer
  //
  // MAME_VERIFIED (namcona1.cpp port7_r):
  //     if ((m_mcu_port6 & 0x80) == 0) return m_muxed_inputs[m_mcu_port6>>5];
  //     else return 0xff;
  // with m_muxed_inputs = { P4, DSW, P1, P2 }.
  // ROM_DISASM_CONFIRMED as well -- see the header, routine at 0x00D26E.
  // ======================================================================
  always @(*) begin
    case (pin6[6:5])
      2'd0:    mux_val = in_p4;
      2'd1:    mux_val = in_dsw;
      2'd2:    mux_val = in_p1;
      default: mux_val = in_p2;
    endcase
    if (pin6[7]) mux_val = 8'hFF;
  end

  // The "pin" side of each port, i.e. what MAME's in callback returns.
  // MAME_VERIFIED: port4_r/port5_r/port8_r return the board latch, port6_r
  // returns 0, port7_r returns the mux, and P0-P3 have no callback bound so
  // m_port_in_cb's default of 0xFF applies.
  always @(*) begin
    case (p_idx)
      4'd4:    p_pin = pin4;
      4'd5:    p_pin = pin5;
      4'd6:    p_pin = 8'h00;
      4'd7:    p_pin = mux_val;
      4'd8:    p_pin = pin8;
      default: p_pin = 8'hFF;
    endcase
    p_masked = din & p_dir[p_idx];
  end

  // ======================================================================
  // I/O port registers
  // ======================================================================
  always @(posedge clk) begin
    if (rst) begin
      for (i = 0; i < 9; i = i + 1) begin
        p_reg[i] <= 8'h00;
        p_dir[i] <= 8'h00;
      end
      // MAME_VERIFIED (namcona1.cpp machine_reset): m_mcu_port4 = 0 and
      // m_mcu_port5 = 1.  Port 5's bit 0 therefore already mirrors bit 1
      // before the BIOS touches it.
      pin4       <= 8'h00;
      pin5       <= 8'h01;
      pin6       <= 8'h00;
      pin8       <= 8'h00;
      m68k_rst_r <= 1'b1;      // HW_CONFIRMED: P4 bit 3 is clear at C70 boot
                               // (c70.bin C067 writes P4 = 0x04)
    end else if (wr && p_hit) begin
      if (p_isdir) begin
        p_dir[p_idx] <= din;
      end else begin
        p_reg[p_idx] <= din;
        // MAME_VERIFIED (m37710.cpp set_port_reg): the out callback fires
        // only when the direction register is non-zero, and it is handed
        // (data & dir), not data.
        if (p_dir[p_idx] != 8'h00) begin
          case (p_idx)
            // MAME_VERIFIED (namcona1.cpp port4_w).  The 68000 is released on
            // the RISING EDGE of bit 3, not on its level:
            //   if ((data & 0x08) && !(m_mcu_port4 & 0x08)) ... CLEAR_LINE
            4'd4: begin
              if (p_masked[3] && !pin4[3]) m68k_rst_r <= 1'b0;
              pin4 <= p_masked;
            end
            // MAME_VERIFIED (namcona1.cpp port5_w):
            //   m_mcu_port5 = data;
            //   m_mcu_port5 &= 0xfe;
            //   m_mcu_port5 |= ((m_mcu_port5 & 0x2) >> 1);
            // MAME's own comment is "not sure yet, but MCU code requires this
            // interaction at least", so this is MAME_VERIFIED and not
            // HW_CONFIRMED -- but ROM_DISASM_CONFIRMED that it is required:
            // the C70 BIOS busy-waits on bit 0 at CCEA/CCFA/CD01/CD19/CD54/
            // CD5B/CD74/CD7B/CD98/CD9F/CDAE/CDB5 while toggling bit 1.
            4'd5: pin5 <= {p_masked[7:1], p_masked[1]};
            4'd6: pin6 <= p_masked;
            // MAME_VERIFIED: port7_w does nothing at all.
            4'd7: ;
            // MAME_VERIFIED (namcona1.cpp port8_w).  MAME's comment says bit 5
            // toggles and guesses "watchdog?"; nothing acts on it.
            4'd8: pin8 <= p_masked;
            default: ;
          endcase
        end
      end
    end
  end

  assign m68k_rst = m68k_rst_r;

  // ======================================================================
  // A-D converter
  //
  // MAME_VERIFIED (m37710.cpp ad_control_w and ad_timer_cb).  The conversion
  // always completes: the BIOS reads the results with no status poll (see the
  // header), but a busy bit that never clears is exactly the failure this
  // block must not have, so bit 6 is cleared when the sweep ends.
  // ======================================================================
  always @(*) begin
    ad_sample = in_p3[ana_bit(ad_line)] ? 16'hFFFF : 16'h0000;
  end

  always @(posedge clk) begin
    if (rst) begin
      ad_control <= 8'h00;
      ad_sweep   <= 8'h00;
      ad_cnt     <= 10'd0;
      ad_run     <= 1'b0;
      for (i = 0; i < 8; i = i + 1) ad_result[i] <= 16'h0000;
    end else begin
      if (ce && ad_run) begin
        if (ad_cnt == ad_period - 10'd1) begin
          ad_cnt              <= 10'd0;
          ad_result[ad_line]  <= ad_sample;
          // sweep mode advances the channel
          if (ad_control[4]) ad_control[2:0] <= ad_line + 3'd1;
          // repeat, or sweep that has not reached its last channel
          if (!ad_more) begin
            ad_control[6] <= 1'b0;
            ad_run        <= 1'b0;
          end
        end else begin
          ad_cnt <= ad_cnt + 10'd1;
        end
      end

      // The CPU write wins over the tick above, exactly as a write during a
      // conversion does on the real part.
      if (wr && (addr == 7'h1E)) begin
        // MAME_VERIFIED: a 0->1 on bit 6 starts a conversion, and in sweep
        // mode the channel field is forced to 0 (data &= 0xf8).
        if (din[6] && !ad_control[6]) begin
          ad_cnt     <= 10'd0;
          ad_run     <= 1'b1;
          ad_control <= din[4] ? (din & 8'hF8) : din;
        end else begin
          if (!din[6]) ad_run <= 1'b0;
          ad_control <= din;
        end
      end
      if (wr && (addr == 7'h1F)) ad_sweep <= din;
    end
  end

  // ======================================================================
  // UARTs -- MAME_VERIFIED, and modelled exactly as far as MAME models them
  // (which is: they hold their registers and never do anything).  See the
  // KNOWN RISK note in the header.
  // ======================================================================
  always @(posedge clk) begin
    if (rst) begin
      for (i = 0; i < 2; i = i + 1) begin
        u_mode[i] <= 8'h00;
        u_baud[i] <= 8'h00;
        u_tbuf[i] <= 16'h0000;
        u_ctl0[i] <= 8'h00;
        u_ctl1[i] <= 8'h00;
      end
    end else if (wr && hit_uart) begin
      case (u_rg)
        3'd0: u_mode[u_i] <= din;
        3'd1: u_baud[u_i] <= din;
        3'd2: u_tbuf[u_i][7:0]  <= din;
        3'd3: u_tbuf[u_i][15:8] <= din;
        // MAME_VERIFIED: "Tx empty flag is read-only" -- bit 3 is preserved.
        3'd4: u_ctl0[u_i] <= (din & ~8'h08) | (u_ctl0[u_i] & 8'h08);
        // MAME_VERIFIED (uart0_ctrl_reg1_w):
        //   reg = (reg & (BIT(data,2) ? 0xfa : 0x0a)) | (data & 0x05);
        3'd5: u_ctl1[u_i] <= (u_ctl1[u_i] & (din[2] ? 8'hFA : 8'h0A)) | (din & 8'h05);
        // 0x36/0x37 and 0x3E/0x3F are the receive buffer, read only.
        default: ;
      endcase
    end
  end

  // ======================================================================
  // Timers A0-A4 and B0-B2
  // ======================================================================
  always @(*) begin
    for (t = 0; t < 8; t = t + 1) begin
      // MAME_VERIFIED (m37710_recalc_timer): only mode 00, timer mode, counts
      // from an internal clock source and schedules an interrupt.  Event
      // counter / one-shot / PWM mode are driven from the external timer
      // pins, and nothing on the NA-2 drives them.  This is the single place
      // that decides whether a timer is running, so that the counting below
      // and the tick above cannot disagree.
      tmr_run[t]  = count_start[t] && (tmr_mode[t][1:0] == 2'b00);
      tmr_tick[t] = ce && tmr_run[t]
                    && (tmr_pre[t] == tscale_m1(tmr_mode[t][7:6]));
      tmr_fire[t] = tmr_tick[t] && (tmr_cnt[t] == 16'd0);
    end
  end

  always @(posedge clk) begin
    if (rst) begin
      count_start <= 8'h00;
      one_shot    <= 8'h00;
      up_down     <= 8'h00;
      for (i = 0; i < 8; i = i + 1) begin
        tmr_reload[i] <= 16'h0000;
        tmr_cnt[i]    <= 16'h0000;
        tmr_pre[i]    <= 9'd0;
        tmr_mode[i]   <= 8'h00;
      end
    end else begin
      // --- counting, one prescaler step per `tscale` MCU clocks ----------
      // Period = tscale * (reload + 1), because the counter is loaded with
      // `reload` and fires on the step after it reaches 0.  That is exactly
      // m37710_recalc_timer's  clocks_to_attotime(tscale) * (tval + 1).
      for (t = 0; t < 8; t = t + 1) begin
        if (ce && tmr_run[t]) begin
          if (tmr_pre[t] == tscale_m1(tmr_mode[t][7:6])) begin
            tmr_pre[t] <= 9'd0;
            // INFERRED -- the reload source.  MAME latches the whole period
            // when the timer is started (m_reload[timer]) and never looks at
            // the register again; the real part reloads the counter from the
            // register at every underflow.  They differ only if software
            // rewrites the register without stopping the timer, which the C70
            // BIOS never does (it always does CLB #$xx,$40 / STA / SEB, e.g.
            // D54B / D552 / D555).  The hardware behaviour is implemented.
            if (tmr_cnt[t] == 16'd0) tmr_cnt[t] <= tmr_reload[t];
            else                     tmr_cnt[t] <= tmr_cnt[t] - 16'd1;
          end else begin
            tmr_pre[t] <= tmr_pre[t] + 9'd1;
          end
        end
      end

      // --- register writes ------------------------------------------------
      if (wr) begin
        // MAME_VERIFIED (count_start_w): every 0->1 restarts that timer.
        if (addr == 7'h40) begin
          for (t = 0; t < 8; t = t + 1) begin
            if (din[t] && !count_start[t]) begin
              tmr_cnt[t] <= tmr_reload[t];
              tmr_pre[t] <= 9'd0;
            end
          end
          count_start <= din;
        end
        if (addr == 7'h42) one_shot <= din;
        if (addr == 7'h44) up_down  <= din;
        if ((addr >= 7'h46) && (addr <= 7'h55)) begin
          if (addr[0]) tmr_reload[t_idx][15:8] <= din;
          else         tmr_reload[t_idx][7:0]  <= din;
        end
        if ((addr >= 7'h56) && (addr <= 7'h5D)) tmr_mode[m_idx] <= din;
      end
    end
  end

  // ======================================================================
  // Processor mode and watchdog
  //
  // MAME_VERIFIED: proc_mode_r returns the register masked with 0xF7;
  // watchdog_timer_w (0x60) is a kick with no timer behind it in MAME either,
  // and watchdog_freq (0x61) is a plain register.  INFERRED that not
  // implementing the watchdog reset is safe: a watchdog that never fires
  // cannot hang the BIOS, one that fires too early can.
  //
  // --- 2026-08-24: bit 3 of the processor mode register IS a reset ---------
  // The sentence above is still true of the 0x60 watchdog.  It was NOT true of
  // this register, and that omission is what makes seven sets in this family
  // need a canned EEPROM in MAME.
  //
  // The C70 BIOS runs a dead-man switch on the 68000 at 0xD1C1: it watches
  // shared work RAM 0x0FBC-0x0FBF, the 68000 keeps it fed with `clr.b $0FBF.w`,
  // and if that stops, the counter at 0x802A runs down from 60 and the BIOS
  // writes 0x1D here.  docs/c70_bios_map.md:1035, 1040-1046 -- disassembled by
  // this project before anyone knew what it was for.
  //
  // MEASURED in MAME, quiztou, 1800 frames, blank 28C16 against factory image
  // (tools/procmode.lua, tools/deadman.lua):
  //
  //            SFR 0x5E writes        dead-man counter 0x802A
  //   factory  1  (0x15 at boot)      pinned at 0x3C all run
  //   blank    6  (0x15, then 0x1D    FC -> 98 -> 34, running down
  //               five times)
  //
  // and the 68000's feeds of 0x0FBF go 5594 -> 0 per 100 frames at exactly the
  // point the game stops advancing.  So the game finishes programming a blank
  // EEPROM -- byte-for-byte the image MAME ships, cmp reports no differences --
  // and then deliberately stops feeding the switch to ask for a reset.
  //
  // 0x15 is the normal mode and 0x1D is 0x15 with bit 3 set, and MAME's own
  // proc_mode_r masks that bit off on read (`& 0xf7`, m37710.cpp:879), which is
  // what a write-only self-clearing reset bit looks like.  MAME latches the
  // byte and does nothing (m37710.cpp:882-886), so its board never resets and
  // the game sits on the NOTICE screen forever -- which is the whole reason for
  // "Canned EEPROM data ... oughtn't be necessary" in its own To-Do list.
  //
  // `INFERRED`, and the one thing that would settle it is the M37702 datasheet's
  // processor-mode register bit map.  Three things agree meanwhile: the BIOS
  // writes exactly this value at exactly this moment, MAME masks exactly this
  // bit on read, and a machine reset applied at that moment makes the game boot
  // (tools/watchdog_test.lua -- attract mode, INSERT COIN).
  //
  // Two candidates were killed by measurement before this one and are recorded
  // so nobody re-runs them: keycus offset 3 (quiztou writes 0x9A69 half a
  // million times, but numanath, knckhead and emeralda never write it at all,
  // so nothing board-wide can hang off it), and the C70's own heartbeat -- P8
  // bit 5 and SFR 0x60 -- which is identical in both runs, 200 kicks per 100
  // frames from start to finish.
  // ======================================================================
  always @(posedge clk) begin
    if (rst) begin
      proc_mode <= 8'h00;
      wdog_freq <= 8'h00;
      soft_rst  <= 1'b0;
    end else begin
      // one clean pulse per write, whatever `we` does
      soft_rst <= wr && (addr == 7'h5E) && din[3];
      if (wr) begin
        // bit 3 self-clears: it is the act, not a mode.  Storing it would make
        // the next read report a reset that has already happened.
        if (addr == 7'h5E) proc_mode <= din & 8'hF7;
        if (addr == 7'h61) wdog_freq <= din;
      end
    end
  end

  // ======================================================================
  // Interrupt controller
  // ======================================================================

  // The CPU core does its own IPL comparison and takes one line per source,
  // so there is no arbiter here any more: this block presents every source's
  // request and every source's priority, and the core picks.
  //
  // MAME_VERIFIED, the M37710_LINE_* numbering (reference/mame/m37710.h) and
  // the vectors (m37710_irq_vectors, m37710.cpp):
  //   line 19 INT0  0xFFF4      line 16 Timer A0 0xFFEE   line 11 Timer B0 0xFFE4
  //   line 18 INT1  0xFFF2      line 15 Timer A1 0xFFEC   line 10 Timer B1 0xFFE2
  //   line 17 INT2  0xFFF0      line 14 Timer A2 0xFFEA   line  9 Timer B2 0xFFE0
  //   line  4 A-D   0xFFD6      line 13 Timer A3 0xFFE8
  //                             line 12 Timer A4 0xFFE6
  //
  // INFERRED -- the external INT pins are driven as one ce-wide pulse, not as
  // a level held until the CPU takes it.  The vendored core samples them
  // under its own clock enable and latches the RISING edge into an internal
  // pending flag that it clears when it takes the interrupt
  // (.vendor/SYSTEM11_MiSTer/source/rtl/m37702.vhd, irq0_l / irq0_pend).  A
  // held level would therefore deliver exactly one interrupt ever, because
  // nothing outside this block can clear the request bit -- this interface
  // has no acknowledge path back from the core.  One edge per event is what
  // the core actually consumes, and it cannot be dropped by IPL masking
  // because the core holds it internally.  If the core is ever changed to
  // follow the level this still works: the pulse is a full ce wide.
  //
  // The doorbell arrives on the 68000's clock, not the MCU's, so it is
  // latched in int0_q first and only then issued -- a raw one-system-clock
  // pulse would fall between two ce pulses and be lost entirely.
  always @(posedge clk) begin
    if (rst) begin
      int0_st <= 2'd0; int0_q <= 1'b0;
      int1_st <= 2'd0; int1_q <= 1'b0;
    end else begin
      if (ce) begin
        case (int0_st)
          2'd0:    if (int0_q) begin int0_st <= 2'd1; int0_q <= 1'b0; end
          2'd1:    int0_st <= 2'd2;   // high for exactly one ce, then release
          default: int0_st <= 2'd0;   // one ce low, so the next edge is clean
        endcase
        case (int1_st)
          2'd0:    if (int1_q) begin int1_st <= 2'd1; int1_q <= 1'b0; end
          2'd1:    int1_st <= 2'd2;
          default: int1_st <= 2'd0;
        endcase
      end
      // an event landing on the same clock as the consume above is kept
      if (irq0_set) int0_q <= 1'b1;
      if (irq1_set) int1_q <= 1'b1;
    end
  end

  assign irq0 = (int0_st == 2'd1);
  assign irq1 = (int1_st == 2'd1);
  // UNKNOWN -- nothing on the NA-2 drives INT2.  namcona1.cpp never calls
  // set_input_line(M37710_LINE_IRQ2), although c70.bin does give 0x7F a
  // priority of 4 at 0xC0E7 and clears its request bit at 0xC0EA, so the pin
  // may well be wired to something not yet identified.  Tied off, and its
  // priority is still presented so that the day it is found only this line
  // has to move.
  assign irq2 = 1'b0;

  assign tick_ta0 = tmr_fire[0];
  assign tick_ta1 = tmr_fire[1];
  assign tick_ta2 = tmr_fire[2];
  assign tick_ta3 = tmr_fire[3];
  assign tick_ta4 = tmr_fire[4];
  assign tick_tb0 = tmr_fire[5];
  assign tick_tb1 = tmr_fire[6];
  assign tick_tb2 = tmr_fire[7];
  assign tick_ad  = ad_irq;

  assign prio_int0 = ic_pri[19];
  assign prio_int1 = ic_pri[18];
  assign prio_int2 = ic_pri[17];
  assign prio_ta0  = ic_pri[16];
  assign prio_ta1  = ic_pri[15];
  assign prio_ta2  = ic_pri[14];
  assign prio_ta3  = ic_pri[13];
  assign prio_ta4  = ic_pri[12];
  assign prio_tb0  = ic_pri[11];
  assign prio_tb1  = ic_pri[10];
  assign prio_tb2  = ic_pri[9];
  assign prio_ad   = ic_pri[4];

  always @(posedge clk) begin
    if (rst) begin
      for (i = 0; i < 20; i = i + 1) ic_pri[i] <= 3'd0;
      for (i = 0; i < 4;  i = i + 1) ic_pol[i] <= 2'd0;
    end else if (wr && ic_hit) begin
      ic_pri[ic_wl] <= din[2:0];
      // bits 5:4 exist only on the three external INT control registers
      if (addr >= 7'h7D) ic_pol[addr[1:0]] <= din[5:4];
    end
  end

  // The request bits.  Kept in one flat vector rather than an array so that
  // the many setters and the one clearer stay obviously single-ported
  // (HANDOFF.md section 4 item 3).
  //
  // These are no longer how the CPU is told about an interrupt -- that is the
  // irqN / tick_* lines above.  They exist because they are software visible:
  // the 7700's interrupt control registers hold the request in bit 3 and
  // software can poll and clear it.
  //
  // UNKNOWN -- on the real part the request bit is cleared automatically when
  // the CPU accepts the interrupt.  This interface has no acknowledge coming
  // back from the core, so a request bit set by hardware stays set until
  // software clears it.  c70.bin never reads any of these registers except
  // through SEB/CLB (which preserve bit 3 or clear it deliberately), so
  // nothing in this game can see the difference; a game that polled bit 3 to
  // detect a *new* request would.
  //
  // Order below matters: the software clear comes FIRST so that a hardware
  // event landing on the same clock is not lost.
  //
  // INFERRED -- a software write of 0 to bit 3 clears the request, and a write
  // of 1 does not create one.  m37710.cpp keeps the pending set in LINE_IRQ
  // and its int_control_w touches neither direction, so in MAME a write can
  // neither set nor clear a request; the readable bit 3 just goes stale.  The
  // C70 BIOS writes both ways: CLB #$08,$71 (D403), CLB #$08,$76 (D558) and
  // CLB #$08,$7f (C0EA) are plainly "clear a stale request before enabling",
  // while SEB #$0b,$75 (C0F0) and SEB #$0a,$79 (C0F6) set bit 3 as a side
  // effect of setting the priority and clearly do not want an immediate
  // interrupt.  Honouring the clear and ignoring the set satisfies both, and
  // matches what MAME effectively does for the set.
  always @(posedge clk) begin
    if (rst) begin
      ic_ir <= 20'd0;
    end else begin
      if (wr && ic_hit && !din[3]) ic_ir[ic_wl] <= 1'b0;

      // MAME_VERIFIED (namcona1.cpp): a 68000 write to mailbox slot 4 raises
      // M37710_LINE_IRQ0 (= line 19) and vblank raises M37710_LINE_IRQ1 (= 18).
      if (irq0_set) ic_ir[19] <= 1'b1;
      if (irq1_set) ic_ir[18] <= 1'b1;

      // MAME_VERIFIED (m37710_timer_cb): curirq = M37710_LINE_TIMERA0 - which,
      // i.e. timer 0..7 -> line 16..9.
      for (t = 0; t < 8; t = t + 1)
        if (tmr_fire[t]) ic_ir[16 - t] <= 1'b1;

      // MAME_VERIFIED (ad_timer_cb): the interrupt happens only when the
      // conversion stops, not on every channel of a sweep.
      if (ad_irq) ic_ir[4] <= 1'b1;
    end
  end

  // ======================================================================
  // Read mux.  Registered, valid one clock after the address is presented --
  // the wrapper's fast path samples it two clocks after it drives the
  // address, and the register is deliberately NOT gated by `ce` because the
  // wrapper's bus sequencer runs on clk.
  // ======================================================================
  always @(*) begin
    rd = 8'h00;

    if (p_hit) begin
      // MAME_VERIFIED (get_port_reg):
      //   d != 0xff ? (in_cb(~d) & ~d) | (regs & d) : regs
      // which is the same expression when d == 0xff.
      rd = p_isdir ? p_dir[p_idx]
                   : ((p_pin & ~p_dir[p_idx]) | (p_reg[p_idx] & p_dir[p_idx]));

    end else if (addr == 7'h1E) begin
      rd = ad_control;
    end else if (addr == 7'h1F) begin
      rd = ad_sweep;
    end else if ((addr >= 7'h20) && (addr <= 7'h2F)) begin
      rd = addr[0] ? ad_result[addr[3:1]][15:8] : ad_result[addr[3:1]][7:0];

    end else if (hit_uart) begin
      case (u_rg)
        3'd0:    rd = u_mode[u_i];
        3'd1:    rd = u_baud[u_i];          // write-only in MAME; we read back
        3'd2:    rd = u_tbuf[u_i][7:0];     // ditto
        3'd3:    rd = u_tbuf[u_i][15:8];
        3'd4:    rd = u_ctl0[u_i];
        3'd5:    rd = u_ctl1[u_i];
        // MAME_VERIFIED: uart0_rbuf_r / uart1_rbuf_r return a literal 0.
        default: rd = 8'h00;
      endcase

    end else if (addr == 7'h40) begin
      rd = count_start;
    end else if (addr == 7'h42) begin
      rd = one_shot;                        // write-only in MAME
    end else if (addr == 7'h44) begin
      rd = up_down & 8'h1F;                 // MAME_VERIFIED: bits 7-5 read 0
    end else if ((addr >= 7'h46) && (addr <= 7'h55)) begin
      rd = addr[0] ? tmr_reload[t_idx][15:8] : tmr_reload[t_idx][7:0];
    end else if ((addr >= 7'h56) && (addr <= 7'h5D)) begin
      rd = tmr_mode[m_idx];

    end else if (addr == 7'h5E) begin
      rd = proc_mode & 8'hF7;               // MAME_VERIFIED (proc_mode_r)
    end else if (addr == 7'h60) begin
      rd = 8'h00;                           // watchdog kick, write only
    end else if (addr == 7'h61) begin
      rd = wdog_freq;

    end else if (ic_hit) begin
      rd = {2'b00,
            (addr >= 7'h7D) ? ic_pol[addr[1:0]] : 2'b00,
            ic_ir[ic_wl],
            ic_pri[ic_wl]};
    end
  end

  always @(posedge clk) dout_r <= rd;
  assign dout = dout_r;

  // ======================================================================
  // Diagnostics.  Project rule: never silently return zero, and make an
  // unexpected dependency show up instead of hiding.  Each address reports
  // once, so a watchdog kick in a loop does not bury the one message that
  // matters.
  // ======================================================================
`ifndef SYNTHESIS
  // synthesis translate_off
  wire hit_ad    = (addr == 7'h1E) || (addr == 7'h1F)
                || ((addr >= 7'h20) && (addr <= 7'h2F));
  wire hit_tmr   = (addr == 7'h40) || (addr == 7'h42) || (addr == 7'h44)
                || ((addr >= 7'h46) && (addr <= 7'h5D));
  wire hit_pmode = (addr == 7'h5E);
  wire hit_wdog  = (addr == 7'h60) || (addr == 7'h61);
  wire hit_any   = p_hit | hit_ad | hit_uart | hit_tmr | hit_pmode | hit_wdog | ic_hit;
  wire hit_stub  = hit_uart | hit_wdog | hit_pmode | (addr == 7'h42)
                | ((addr >= 7'h6C) && (addr <= 7'h6F));
  reg [127:0] seen_stub;
  reg [127:0] seen_unmapped;

  initial begin
    seen_stub     = 128'd0;
    seen_unmapped = 128'd0;
  end

  always @(posedge clk) begin
    if (!rst && sel) begin
      if (!hit_any && !seen_unmapped[addr]) begin
        seen_unmapped[addr] <= 1'b1;
        $display("[periph] %0t UNMAPPED SFR %s at 0x%02h (nothing is there on an M37702M2)",
                 $time, we ? "write " : "access", addr);
      end
      if (hit_stub && !seen_stub[addr]) begin
        seen_stub[addr] <= 1'b1;
        $display("[periph] %0t stub SFR %s at 0x%02h -- registers only, no behaviour modelled",
                 $time, we ? "write " : "access", addr);
      end
    end
  end
  // synthesis translate_on
`endif

endmodule

`default_nettype wire
