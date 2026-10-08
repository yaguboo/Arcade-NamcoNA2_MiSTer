//============================================================================
//  Namco NA-2 -- 68000 wrapper around fx68k
//
//  Turns fx68k's asynchronous 68000 bus into a simple synchronous request /
//  acknowledge port that the rest of the core can drive, and generates the
//  two-phase clock enables fx68k needs.
//
//  fx68k (rtl/cpu/fx68k/) is (c) 2018,2021 Jorge Cwik, GPLv3.  See
//  LICENSE_NOTES.md.  It is used unmodified.
//============================================================================
`default_nettype none

module na2_m68k #(
    // clk_sys ticks per 68000 clock.  master/4 = 12.5282 MHz from 50.113 MHz.
    parameter int CLK_DIV = 4,
    // Phases a bus cycle runs after /AS before a DTACK that has not arrived
    // freezes the CPU.  See "wait states" below; 2 is the measured value.
    parameter int KPH = 2
) (
    input  wire        clk,          // 50.113 MHz master clock
    input  wire        rst,          // synchronous, active high (system reset)
    input  wire        cpu_rst,      // active high: hold the 68000 in reset
                                     // (driven by the C70 -- see na2_c70_hle)
    input  wire        ce_en,        // global clock enable (pause / download)
    // 1 = this core's memory latency costs the 68000 no time (see below).
    // 0 = the wrapper as it always was: late acknowledges become wait states.
    input  wire        recover_en,

    // --- simple synchronous bus -------------------------------------------
    output wire [23:1] addr,
    output wire [15:0] dout,         // 68000 -> system
    input  wire [15:0] din,          // system -> 68000
    output wire        rd,           // read strobe, held until ack
    output wire        wr,           // write strobe, held until ack
    output wire        uds_n,
    output wire        lds_n,
    input  wire        ack,          // one-cycle pulse: din valid / write taken

    // --- interrupts (autovectored, like the real board) --------------------
    input  wire [2:0]  ipl_n,

    // --- observability for simulation / trace ------------------------------
    output wire [2:0]  fc,
    // LOCAL: RISK-10 debug -- D7 out of fx68k, see rtl/na2_top.sv
    output wire [31:0] dbg_d7,
    output wire        as_n,
    output wire        halted_n,
    output wire [15:0] dbg_debt      // clock phases owed to the 68000
);

  // ---------------------------------------------------------------------
  // Wait states, and why this core should not take them.
  //
  // fx68k samples DTACK in S4 and inserts a wait state -- one whole CPU clock
  // -- if it is not there yet (fx68k.sv:206).  This wrapper can only raise
  // DTACK the clock after `ack`, and `ack` comes from wherever the access went:
  // one clock for vreg/mailbox, two for video RAM, about seven for an SDRAM
  // read.  A write is worse: its data strobe only rises at S4, the very phase
  // DTACK is sampled in, so even a one-clock acknowledge is late.
  //
  // So nearly every 68000 bus cycle took a wait state.  MEASURED on the board
  // (NA12 probe, docs/NA12_TIMING_AUDIT.md section 5): 0.93-1.02 per cycle.
  // MEASURED in simulation (sim/tb_na12_dtack.sv): a RAM read/write loop runs
  // 78 % of a zero-wait 68000's iterations at a 7-clock acknowledge and 66 %
  // at 9-12.  MAME models no wait states, and nothing on the NA-2 PCB is known
  // to insert them (UNKNOWN -- PCB_VERIFICATION_NEEDED), so the game lost that
  // time only here: Numan Athletics' main loop overran frames MAME finishes,
  // and on those frames its vblank handler skips the screen-window step --
  // which is how the title-to-demo fade-in went missing.
  //
  // With recover_en the CPU does not see the latency at all:
  //
  //   * Phases are scheduled every HALF clocks, as before.
  //   * Once a cycle with its data strobe asserted has run KPH phases past
  //     /AS without DTACK, the CPU is FROZEN -- no phases at all -- until the
  //     acknowledge turns into DTACK.  The phases the schedule wanted in the
  //     meantime are counted as debt.  The CPU therefore never samples a
  //     missing DTACK and never takes a wait state.
  //   * While not frozen and in debt, a phase is issued on every clock rather
  //     than every HALF, and each extra one pays one back.
  //
  // KPH = 2 is measured, not derived: freezing one phase later (3 or 4) is
  // already too late -- fx68k has sampled DTACK and the wait state is taken --
  // and a zero-wait RAM loop loses 16 % again.  At 2 the loop runs the
  // zero-wait count exactly for every acknowledge latency from 1 to 9 clocks,
  // with the debt back at zero (sim/run_na12_dtack.sh).  Beyond that the
  // loop is bus-bound with no idle phases to repay into and throughput falls
  // off gradually: 94 % at 10, 85 % at 12.
  //
  // The debt saturates rather than wraps: a saturated debt means the CPU is
  // running as fast as the memory allows, never faster than its schedule.
  // A paused core (ce_en = 0) accrues nothing.
  // ---------------------------------------------------------------------
  localparam int HALF = CLK_DIV / 2;
  // typed copies, so the comparisons below are the width of what they compare
  localparam int HALF_M1 = HALF - 1;
  localparam [$clog2(CLK_DIV)-1:0] SCHED_LAST = HALF_M1[$clog2(CLK_DIV)-1:0];
  localparam [3:0]                 KPH4       = KPH[3:0];

  wire        cpu_as_n, cpu_lds_n, cpu_uds_n, cpu_rw_n;
  wire [15:0] cpu_dout;
  wire [23:1] cpu_addr;
  wire        fc0, fc1, fc2;
  wire        vma_n, e_clk;
  reg         dtack_n;
  reg         done;

  // Autovector every interrupt: the NA-1/NA-2 68000 has no vector generator,
  // MAME raises levels 3 and 4 with HOLD_LINE which is autovectored.
  // MAME_VERIFIED: namcona1_state::scanline_interrupt.
  wire iack   = (fc2 & fc1 & fc0);
  wire vpa_n  = ~(iack & ~cpu_as_n);

  // A 68000 cycle is in progress while ASn is low and at least one data
  // strobe is asserted.
  wire cyc = ~cpu_as_n & ~(cpu_lds_n & cpu_uds_n) & ~iack;

  reg  [$clog2(CLK_DIV)-1:0] sched;
  reg         ph;                    // 0: the next phase is phi1
  reg  [15:0] debt;
  reg  [3:0]  nph;                   // phases issued since /AS fell

  wire credit  = ce_en && (sched == SCHED_LAST);
  wire freeze  = recover_en && cyc && dtack_n && (nph >= KPH4);
  wire extra   = recover_en && (debt != 16'd0);
  wire emit    = ce_en && !freeze && (credit || extra);
  wire en_phi1 = emit & ~ph;
  wire en_phi2 = emit &  ph;

  always @(posedge clk) begin
    if (rst) begin
      sched <= '0;
      ph    <= 1'b0;
      debt  <= 16'd0;
    end else begin
      if (ce_en) sched <= (sched == SCHED_LAST) ? '0 : sched + 1'b1;
      if (emit)  ph <= ~ph;
      if (!recover_en)
        debt <= 16'd0;
      else if (credit && freeze) begin
        if (debt != 16'hFFFF) debt <= debt + 16'd1;
      end else if (emit && !credit && (debt != 16'd0)) begin
        debt <= debt - 16'd1;
      end
    end
  end

  always @(posedge clk) begin
    if (rst || cpu_as_n) nph <= 4'd0;
    else if (emit && (nph != 4'hF)) nph <= nph + 4'd1;
  end

  assign dbg_debt = debt;

  // ---------------------------------------------------------------------
  // fx68k
  // ---------------------------------------------------------------------
  fx68k u_fx68k (
      .clk       (clk),
      .HALTn     (1'b1),
      .extReset  (rst | cpu_rst),
      .pwrUp     (rst),
      .enPhi1    (en_phi1),
      .enPhi2    (en_phi2),

      .dbg_d7    (dbg_d7),
      .eRWn      (cpu_rw_n),
      .ASn       (cpu_as_n),
      .LDSn      (cpu_lds_n),
      .UDSn      (cpu_uds_n),
      .E         (e_clk),
      .VMAn      (vma_n),

      .FC0       (fc0),
      .FC1       (fc1),
      .FC2       (fc2),
      .BGn       (),
      .oRESETn   (),
      .oHALTEDn  (halted_n),

      .DTACKn    (dtack_n),
      .VPAn      (vpa_n),
      .BERRn     (1'b1),
      .BRn       (1'b1),
      .BGACKn    (1'b1),

      .IPL0n     (ipl_n[0]),
      .IPL1n     (ipl_n[1]),
      .IPL2n     (ipl_n[2]),

      .iEdb      (din),
      .oEdb      (cpu_dout),
      .eab       (cpu_addr)
  );

  // ---------------------------------------------------------------------
  // Bus cycle -> request/ack handshake.
  //
  // We raise rd/wr for the system, wait for ack, then assert DTACKn to let
  // fx68k finish the cycle.  DTACKn is released when ASn goes high so the next
  // cycle starts clean.
  // ---------------------------------------------------------------------
  always @(posedge clk) begin
    if (rst | cpu_rst) begin
      done    <= 1'b0;
      dtack_n <= 1'b1;
    end else if (!cyc) begin
      done    <= 1'b0;
      dtack_n <= 1'b1;
    end else if (ack) begin
      done    <= 1'b1;
      dtack_n <= 1'b0;
    end
  end

  assign rd    = cyc & ~done &  cpu_rw_n;
  assign wr    = cyc & ~done & ~cpu_rw_n;
  assign addr  = cpu_addr;
  assign dout  = cpu_dout;
  assign uds_n = cpu_uds_n;
  assign lds_n = cpu_lds_n;
  assign fc    = {fc2, fc1, fc0};
  assign as_n  = cpu_as_n;

endmodule

`default_nettype wire
