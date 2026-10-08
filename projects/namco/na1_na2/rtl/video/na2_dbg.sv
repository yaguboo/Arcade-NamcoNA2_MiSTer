//============================================================================
//  Namco NA-2 -- hardware debug overlay   *** TEMPORARY, REMOVE BEFORE RELEASE ***
//
//  On a DE10-Nano there is no debugger, no serial console and no way to see
//  what the 68000 is doing.  Without this, a black screen means "something is
//  wrong" and nothing more -- which would leave us guessing, and guessing is
//  exactly what this project's rule 2 forbids.
//
//  So the core draws its own state as coloured blocks across the top of the
//  screen, where a single screenshot answers "how far did it get".
//
//  NOTE ON THE PORT NAMES BELOW.  Several are historical and no longer say what
//  they carry: na2_top wires `st_errs` <- ack_cnt, `st_berrs` <- stall_snap,
//  `st_word0` <- c70_chan, `c70_p_src/dst` <- c70_volA/volB, and
//  `c70_copy_word` <- fetch_cnt.  The row comments here describe what is
//  actually on screen; the ports are just the wires it arrives on.  Read this
//  block together with the instantiation in na2_top.sv, never the port names
//  alone -- doing that on 2026-08-23 produced a confident claim that
//  tools/read_overlay.py's table was wrong in seven places, and it was right.
//
//  Row 0  (4 px)   16 landmark flags, latched.  green = reached, dark red = not.
//                  The addresses are the same ones sim/tb_na2_boot.sv checks,
//                  so hardware and simulation report progress identically.
//  Row 1  (4 px)   unmapped-access count, 16 bits, MSB left.  Should stay black.
//  Row 2  (4 px)   C70 commands serviced, 16 bits.  Expect 3 (cmd 3 x3) then 4.
//  Row 3  (4 px)   blits started, 16 bits.  Zero means the blitter never ran.
//  Row 4  (4 px)   SDRAM word 0 read back after the download.  Must be 0x46FC,
//                  the first instruction sim/tb_na2_boot.sv fetches at 0xC00000.
//  Row 5  (4 px)   work-RAM read-back mismatches out of 256.  Must be 0.
//  Row 6  (4 px)   vreg $0C as it stood at the FIRST CG/shape write the bank
//                  register routed nowhere.  0x0002 = CG RAM, 0x0003 = shape
//                  RAM, and those two are the only values that route anywhere:
//                  anything else here is the value that threw the write away.
//  Row 7  (4 px)   bit 15 = the game moved $0C while a blit was running at
//                  least once; low byte = bits 23:16 of the dead write's
//                  address.
//  Row 17 (4 px)   how many blits that happened to, wrapping.  EXPECTED to be
//                  large: it is the hazard, not the damage.  Row 15 is the
//                  damage and is expected to be ZERO.  The pair is the point --
//                  one flag that can only ever read one value proves nothing.
//  Row 16 (4 px)   the same address, bits 15:0.
//  Row 15 (4 px)   how many such writes, wrapping.  One at a scene change and
//                  eight hundred a frame are different faults; the count is the
//                  only thing that tells them apart.
//  Row 18 (4 px)   bit 15 = the first unmapped 68000 access was a write;
//                  low byte = bits 23:16 of its address.
//  Row 19 (4 px)   the same address, bits 15:0.  Row 1 counts them.
//  Row 20 (4 px)   the melody channel's envelope {target, level}.  MAME
//                  0xFFFF in the scene where the board plays it too quietly.
//                  Equal halves = settled; unequal = ramping.
//  Row 21 (4 px)   its envelope program pointer ($12,X).  MAME 0x040A, into
//                  the 68000's work RAM at 0x270000.  Zero = channel stopped.
//  Row 22 (4 px)   {its envelope fraction ($14,X), its level ($15,X)}.  The
//                  low half must equal row 20's low half -- two readings of
//                  the same byte, so they cross-check each other.  A fraction
//                  that moves while the level does not is a ramp whose carry
//                  into the high byte is being lost.
//  Row 23 (4 px)   how many times the driver has run this channel's envelope
//                  routine (0x00E1AA with X = 0x4600), wrapping.  Frozen while
//                  the game runs means the channel is being SKIPPED, which is
//                  a different fault from anything the envelope itself does.
//  Row 29 (4 px)   RISK-10: the last word the 68000 WROTE to work RAM
//                  0x07FFF8 -- the stack slot Quiztou's D7 is popped from.
//                  FIRST write, which is the JSR's push at 0xC00028.
//                  Expect 0x00C0, the high half of 0x00C0002E.
//  Row 30 (4 px)   the FIRST word read back from there, which is the
//                  `move.l (sp)+,d7` at 0xC22636.  Expect the same.
//  Row 31 (4 px)   {write seen, writes[6:0], read seen, reads[6:0]}.
//
//  Row 24 (4 px)   RISK-10: bit 15 = the 68000 has been caught fetching
//                  outside program ROM and outside page 0; bits 7:0 = that
//                  address, bits 23:16.
//  Row 25 (4 px)   the same address, bits 15:0.
//  Row 26 (4 px)   the fetch address immediately BEFORE it, bits 23:16.
//  Row 27 (4 px)   the same, bits 15:0.  Rows 24-27 together say where it
//                  jumped from and where it landed.
//  Row 28 (4 px)   how many such escapes, saturating at 0xFFFF.  One is a bad
//                  jump; thousands is a loop.
//  (rows 24-28 previously carried the sound diagnostics below, which answered
//   their question in session 10; the signals are still wired at the top level)
//  Row 24 (4 px)   voices keyed right now, to tie the shot to what was
//                  sounding when it was taken.
//  Row 25 (4 px)   writes that have landed on voice 0's two volume registers.
//                  Frozen while row 24 keeps changing means the driver stopped
//                  servicing the melody channel, not that it went quiet.
//  Row 26 (4 px)   voice 0's sample address at its last key-on, bits 15:0.
//  Row 27 (4 px)   its volume pair as last written.
//  Row 28 (4 px)   the last pair that was not both zero.
//  Row 29 (4 px)   the melody channel's {alive flag ($08,X, MAME 0x80), frames
//                  until its envelope advances again ($2D,X)}.
//  Row 30 (4 px)   the first entry of the driver's channel list, esram[0x6420].
//                  MAME 0x4600 -- the melody channel is the FIRST one, so a
//                  truncated list cannot skip it and anything else here is the
//                  list itself being wrong.
//  Row 31 (4 px)   {the fade control esram[0x4006], the fade level $d7}.
//                  MAME's high byte is 0x80 and its low two bits are what
//                  0x00DBC5 tests: zero means no fade and the attenuation is
//                  cleared, which is the only path MAME ever takes.  Ours
//                  attenuates by 0x50, which is more than the melody's whole
//                  volume.  The dispatch vector this row used to carry read
//                  0xE1AA on all 152 samples of the 2026-08-23 log.
//  Row 12 (4 px)   68000 program fetches, wrapping -- the CPU's liveness.
//  Row 13 (4 px)   the stall snapshot (see rtl/na2_top.sv).
//  Row 10 (4 px)   C70 command 7: the source parameter it read.  0xC000.
//  Row 11 (4 px)   C70 command 7: the destination it read.  0x1000.
//  Row 8  (4 px)   68000 fetch address bits 23:8, sampled once a frame.
//  Row 9  (4 px)   the same sample, bits 15:0.  Two frames with the same value
//                  and no unmapped accesses means a loop, and says where.
//  Row 14 (4 px)   the renderer.  bit 15 = sprites were dropped on some line
//                  (hit list full, or the line ran out of time), bit 14 = a
//                  line did not finish rendering before the buffers swapped,
//                  bits 7:0 = the most sprites drawn on any one line.
//
//  Blue tint on rows 1-3 marks a set bit so they cannot be confused with the
//  green/red of row 0.
//
//  Enable is under an OSD switch so the overlay can be turned off to look at
//  the real picture.
//============================================================================
`default_nettype none

module na2_dbg (
    input  wire        clk,
    input  wire        rst,

    // --- what to watch ------------------------------------------------------
    input  wire [23:1] cpu_addr,
    input  wire        cpu_fetch,      // program-space read, acknowledged
    input  wire        m68k_rst,
    input  wire        unmapped,       // acknowledged access outside the map
    input  wire        blit_start,
    input  wire        c70_done,
    input  wire        gfx_enable,
    input  wire [15:0] st_word0,     // row 4: now the melody channel's two
                                     // volume inputs, {esram[0x4630], esram[0x4615]}
    input  wire [15:0] st_errs,      // SDRAM self-test: read-back mismatches
    input  wire [15:0] st_dlwords,   // words the ROM download wrote
    input  wire [15:0] st_dlindex,   // asset load index the host used
    input  wire [15:0] c70_p_src,    // command 7 parameters, as the C70 read them
    input  wire [15:0] c70_p_dst,
    input  wire [15:0] c70_copy_word,
    input  wire [15:0] st_berrs,
    input  wire [15:0] st_video,     // renderer: sprite drops, overrun, peak hits

    // --- the two faults the board actually reports (2026-08-22) -------------
    // Rows 6, 7, 12 and 13 used to carry the ROM-download counters and the
    // command-7 copy diagnostics.  All four have read the same constant on
    // every screenshot for several sessions -- the download is long finished
    // and command 7 has been answered correctly since session 6 -- so they are
    // reused here rather than pushing the overlay further down the picture.
    // Their old sources are still wired at the top level; only the display
    // moved.  docs/glitch_investigation.md section 27.
    input  wire [15:0] dead_n,       // CG/shape writes routed nowhere (wraps)
    input  wire [15:0] dead_bank,    // vreg $0C at the first one
    input  wire [23:0] dead_a24,     // the address it was aimed at
    input  wire        slip_seen,    // the game moved $0C under a running blit
    input  wire [15:0] slip_n,       // and how many blits that happened to
    input  wire [23:0] unmap_a24,    // first unmapped 68000 address
    input  wire        unmap_wr,     // 1 = it was a write

    // --- the melody, 2026-08-22.  rtl/sound/na2_c219.sv has the reasoning ---
    input  wire [15:0] snd_key_ever,   // voices ever keyed on
    input  wire [15:0] snd_mute_ever,  // voices keyed that read only zeroes
    input  wire [15:0] snd_bank_seen,  // 4 bits per voice group, one per bank
    input  wire [15:0] snd_loud_ever,  // voices ever keyed at an audible volume
    // RISK-10, 2026-08-23: the stack word Quiztou's D7 is popped from.
    // na2_top.sv carries the measurement these three answer.
    input  wire [15:0] d7_wr,          // FIRST word written to work RAM 0x07FFF8
    input  wire [15:0] d7_rd,          // FIRST word read back from there
    input  wire [15:0] d7_seen,        // {wr_seen, wr_n[6:0], rd_seen, rd_n[6:0]}
    input  wire [15:0] d7_pc,          // program fetch before that read, 23:8
    input  wire [15:0] d7_pcl,         // ... and 7:0
    input  wire [15:0] vec,            // {seen, iack_n[6:0], vector byte addr}
    input  wire [15:0] vec_pc,         // the program fetch before it, 23:8
    input  wire [15:0] vec_pl,         // ... and 7:0
    // The last four words the 68000 wrote before the vector fetch: the
    // exception stack frame, which carries the address that faulted.
    input  wire [15:0] fr0,            // newest
    input  wire [15:0] fr1,
    input  wire [15:0] fr2,
    input  wire [15:0] fr3,
    input  wire [15:0] fr4,            // two more of the ring
    input  wire [15:0] fr5,
    input  wire [15:0] fr_a_hi,        // and the newest write's address
    input  wire [15:0] fr_a_lo,
    // RISK-10: the most recent value written to D7, and where
    input  wire [15:0] d7v_hi,
    input  wire [15:0] d7v_lo,
    input  wire [15:0] d7p_hi,     // {change count, PC 23:16}
    input  wire [15:0] d7p_lo,
    input  wire [15:0] n_enter,    // fetches of 0xC20728, the wrapper entry
    input  wire [15:0] n_restore,  // fetches of 0xC2074A, its movem restore

    // Which instruments rows 24-31 show.  0 = the boot/D7 set that RISK-10
    // needed, 1 = the C219 sound set that session 10 needed.  Both are wired
    // in and always have been; what was missing was a way to choose without a
    // fitter run, and swapping them by editing this file is how an afternoon
    // gets spent building a bitstream to read one counter.  2026-08-26.
    input  wire        page_snd,
    // DIAGNOSTIC, branch fix/na12-accuracy: when set, rows 8-31 show these 24
    // words instead.  na2_top's "NA12 probe" block says what each one is, and
    // tools/na12_probe_read.py decodes them.
    input  wire        page_na12,
    input  wire [15:0] na12_w [0:23],

    input  wire [15:0] snd_key_now,    // voices keyed right now
    input  wire [15:0] snd_oor_ever,   // voices whose address left work RAM
    input  wire [15:0] snd_pcm0,       // work-RAM fingerprint, per 128 KiB bank
    input  wire [15:0] snd_pcm1,
    input  wire [15:0] snd_pcm2,
    input  wire [15:0] snd_pcm3,
    // Rows 21, 26, 30 and 31 carried `voices mute`, the bank-0 PCM sum and the
    // C70 acknowledge counters.  All four answered their question and have read
    // the same value for entire sessions -- 0x0000, 0x0000, 30, 30 -- so they
    // carry voice 0's key-on snapshot instead rather than making the overlay
    // taller.  The signals behind them are still wired at the top level.
    input  wire [23:0] snap_addr,    // find_sample() for voice 0, at key-on
    input  wire [15:0] snap_freq,
    input  wire [15:0] snap_mv,      // {mode, loudest of the two volumes}
    // Rows 25, 27 and 28 carried `voices out of range` and two PCM
    // fingerprints.  All three answered their question -- no voice leaves work
    // RAM, and the PCM matches MAME's sets -- so they carry the volume
    // registers instead.  Row 29 keeps one fingerprint, which is enough to say
    // which sample set is loaded.
    input  wire [15:0] vol_n,        // writes landing on voice 0's volumes
    input  wire [15:0] vol_last,     // the pair as last written
    input  wire [15:0] vol_nz,       // the last pair that was not both zero
    input  wire [15:0] vol2_nz,      // the same for the beat voice

    // --- the envelope, 2026-08-22.  rtl/mcu/na2_c70.sv has the field map ---
    // Rows 20, 21, 22, 23, 29, 30 and 31 carried `voices keyed`, the high half
    // of voice 0's sample address, `banks seen`, `voices loud`, the beat
    // voice's volume, voice 0's key-on frequency and its mode.  All seven have
    // answered: every voice is keyed, at the address, pitch and mode MAME
    // uses, on banks MAME uses, and the beat is loud.  What is NOT known is why
    // the melody channel's envelope level sits at 0x24 where MAME's sits at
    // 0xFF, so those rows carry the envelope instead.
    input  wire [15:0] env_tl,       // {target, level} of the melody channel
    input  wire [15:0] env_ptr,      // its envelope program pointer
    input  wire [15:0] env_fl,       // {fraction, level} -- the ramp's own pair
    input  wire [15:0] env_act,      // {alive, frames to the next advance}
    input  wire [15:0] env_visits,   // times the driver ran its envelope (wraps)
    input  wire [15:0] env_fade,     // {esram[0x4006] fade control, $d7 level}
    input  wire [15:0] env_list,     // esram[0x6420], the channel list head

    // --- where to draw ------------------------------------------------------
    input  wire [8:0]  hpos,
    input  wire [8:0]  vpos,
    input  wire        enable,

    output wire        active,         // 1 = overlay owns this pixel
    output reg  [7:0]  r,
    output reg  [7:0]  g,
    output reg  [7:0]  b
);

  localparam int H_VISIBLE = 304;
  localparam int V_START   = 32;       // first visible raw line, see na2_crtc
  localparam int CELL_W    = H_VISIBLE / 16;   // 19 px
  localparam int ROW_H     = 4;
  localparam int N_ROWS    = 32;  // 32 rows = 128 of 224 lines.  Keep ROW_H and
                                  // N_ROWS in step with tools/read_overlay.py.

  // ---------------------------------------------------------------------
  // Landmarks.  Identical list to sim/tb_na2_boot.sv -- keep them in step.
  // ---------------------------------------------------------------------
  wire [23:0] a24 = {cpu_addr, 1'b0};

  // A fetch is not proof the instruction ran.  The 68000 keeps one word of
  // prefetch past the instruction it is executing, so the word after a taken
  // branch is fetched too -- and session 4's first good run came back with
  // VERSION PASSED *and* VERSION FAILED *and* the RAM-test error path all
  // latched, because 0xC2DA70 sits right after the `bra` at 0xC2DA6E and
  // 0xC2DAA4 sits right after the `jmp (a0)` at 0xC2DAA2.
  //
  // What does prove it: the *next* fetch is two bytes on.  Prefetch runs one
  // word ahead and no further, so a landmark word followed immediately by
  // landmark+2 means the instruction there really was decoded, and a landmark
  // followed by a branch target means it was only ever prefetched.  Every
  // landmark below is an instruction of at least four bytes, or a two-byte one
  // whose successor is fetched next, so the rule holds for all of them.
  //                                                     ROM_DISASM_CONFIRMED
  reg [23:0] fpre;
  reg        fpre_v;
  always @(posedge clk) begin
    if (rst) begin
      fpre   <= 24'd0;
      fpre_v <= 1'b0;
    end else if (cpu_fetch) begin
      fpre   <= a24;
      fpre_v <= 1'b1;
    end
  end
  // the address the 68000 has just been shown to execute, and a strobe for it
  wire        ran   = cpu_fetch && fpre_v && (a24 == fpre + 24'd2);
  wire [23:0] ran_a = fpre;

  reg [15:0] flags;
  always @(posedge clk) begin
    if (rst) flags <= 16'd0;
    else begin
      if (!m68k_rst)                        flags[0]  <= 1'b1;  // 68000 released
      if (ran) begin
        if (ran_a == 24'hC00000)            flags[1]  <= 1'b1;  // entry point
        if (ran_a == 24'hC2D55C)            flags[2]  <= 1'b1;  // boot subroutine
        if (ran_a == 24'hC2D592)            flags[3]  <= 1'b1;  // cmd3 doorbell 1
        if (ran_a == 24'hC2D5BA)            flags[4]  <= 1'b1;  // cmd3 doorbell 2
        if (ran_a == 24'hC2D5E2)            flags[5]  <= 1'b1;  // cmd3 doorbell 3
        if (ran_a == 24'hC2D7D2)            flags[6]  <= 1'b1;  // past cmd3 blocks
        if (ran_a == 24'hC2DAC0)            flags[7]  <= 1'b1;  // RAM test
        if (ran_a == 24'hC2DA48)            flags[8]  <= 1'b1;  // cmd7 doorbell
        if (ran_a == 24'hC2DA66)            flags[9]  <= 1'b1;  // version compare
        if (ran_a == 24'hC2DA6E)            flags[10] <= 1'b1;  // VERSION PASSED
        if (ran_a == 24'hC2DA70)            flags[11] <= 1'b1;  // VERSION FAILED
        if (ran_a == 24'hC2DA82)            flags[12] <= 1'b1;  // fail hang loop
        if (ran_a == 24'hC2DA9A)            flags[13] <= 1'b1;  // continue past
        if (ran_a == 24'hC2DAA4)            flags[14] <= 1'b1;  // RAM-test error
      end
      flags[15] <= gfx_enable;                                  // live, not latched
    end
  end

  // ---------------------------------------------------------------------
  // Counters.
  //
  // n_unmapped saturates: it is a fault count, it should be zero, and a fault
  // count that wraps to a healthy-looking value is a diagnostic that lies.
  //
  // n_cmd and n_blit WRAP.  They used to saturate for the same reason, and the
  // reason does not apply to them: they are rate and liveness counters, not
  // fault counters, and both of them read 0xFFFF on every screenshot this
  // project has ever taken.  KNOWN_ISSUES.md lesson 21 says liveness has to be
  // measured with a cumulative counter because the C70's dispatcher is a 60 Hz
  // loop and its PC never changes -- and then the only cumulative counter for
  // it was pinned at full scale.  Two shots of a wrapping counter always answer
  // "is it moving"; a saturated one never does again.
  // ---------------------------------------------------------------------
  reg [15:0] n_unmapped, n_cmd, n_blit;
  always @(posedge clk) begin
    if (rst) begin
      n_unmapped <= 16'd0; n_cmd <= 16'd0; n_blit <= 16'd0;
    end else begin
      if (unmapped   && !(&n_unmapped)) n_unmapped <= n_unmapped + 16'd1;
      if (c70_done)                     n_cmd      <= n_cmd      + 16'd1;
      if (blit_start)                   n_blit     <= n_blit     + 16'd1;
    end
  end

  // ---------------------------------------------------------------------
  // Where the 68000 is.  Latched on every fetch, then sampled once a frame so
  // a screenshot shows one instruction rather than a blur.  Two frames the same
  // with the unmapped counter at zero is a loop, and this says which one.
  // ---------------------------------------------------------------------
  reg [23:0] pc_last, pc_shown;
  always @(posedge clk) begin
    if (rst) begin
      pc_last  <= 24'd0;
      pc_shown <= 24'd0;
    end else begin
      if (cpu_fetch) pc_last <= a24;
      if (vpos == 9'd0 && hpos == 9'd0) pc_shown <= pc_last;
    end
  end

  wire [15:0] pc_hi = pc_shown[23:8];
  wire [15:0] pc_lo = pc_shown[15:0];

  // ---------------------------------------------------------------------
  // Where the 68000 LEFT the program ROM, and where it was standing when it
  // did.  RISK-10: Quiztou draws its notice screen and then executes in upper
  // work RAM, while MAME's 68000 spends 1718 of 1800 frames in program ROM and
  // every one of its work-RAM frames in page 0x000000.  Sampling the fetch
  // address once a frame says it happened; it cannot say where it jumped FROM,
  // because by the next sample the answer is long gone.
  //
  // So this latches the FIRST fetch that is neither in program ROM nor in that
  // low page, together with the fetch immediately before it.  pc_last still
  // holds the previous fetch at this moment -- it is written by the block
  // above, non-blocking, so reading it here gives the one before this.
  //
  // was_in_prg gates the whole thing so the C70's boot-time deposit and the
  // 68000's first moments cannot latch a false answer before the game is
  // running.  esc_n counts every escape, not just the first: once means one
  // bad jump, thousands means it is going round.
  // ---------------------------------------------------------------------
  localparam [23:0] PRG_LO = 24'hC00000;
  localparam [23:0] PRG_HI = 24'hDFFFFF;

  // Page 0 was excluded here at first, on the reasoning that the vector table
  // and any low-RAM routine would be normal traffic.  It is not: MAME's 68000
  // reads PC=0 only while the C70 still holds it in reset, and from the frame
  // it is released it never fetches outside program ROM again.  Excluding page
  // 0 cost a build -- the latch fired on 0x000006 -> 0x040000, which is the
  // SECOND hop, and said nothing about how the CPU left the ROM.  Any fetch
  // outside program ROM is the fault, so latch all of them.
  wire in_prg  = (a24 >= PRG_LO) && (a24 <= PRG_HI);
  wire escaped = cpu_fetch && !in_prg;

  // esc_from removed 2026-08-27: written but never read once its two
  // 16-bit split wires went (the overlay row was never wired).
  reg [23:0] esc_to;
  reg        esc_seen, was_in_prg;
  reg [15:0] esc_n;
  always @(posedge clk) begin
    if (rst) begin
      esc_to <= 24'd0;
      esc_seen <= 1'b0; was_in_prg <= 1'b0; esc_n <= 16'd0;
    end else begin
      if (cpu_fetch && in_prg) was_in_prg <= 1'b1;
      if (escaped && was_in_prg) begin
        if (!esc_seen) begin
          esc_to   <= a24;
          esc_seen <= 1'b1;
        end
        if (esc_n != 16'hFFFF) esc_n <= esc_n + 16'd1;
      end
    end
  end

  wire [15:0] esc_to_hi   = {esc_seen, 7'd0, esc_to[23:16]};
  wire [15:0] esc_to_lo   = esc_to[15:0];

  // ---------------------------------------------------------------------
  // Draw
  // ---------------------------------------------------------------------
  wire [8:0] y   = vpos - V_START[8:0];
  wire       in_v = enable && (vpos >= V_START) && (y < N_ROWS * ROW_H);
  wire       in_h = (hpos < H_VISIBLE);
  assign active = in_v && in_h;

  // 17 rows needs 5 bits of row index; y[5:2] tops out at 15 and would fold
  // rows 16 and 17 back onto rows 0 and 1.
  wire [4:0] row = y[6:2];                       // 0..31
  // column 0 is the leftmost cell, and bit 15 is drawn leftmost
  // hpos < H_VISIBLE and CELL_W = H_VISIBLE/16, so the quotient is 0..26 and
  // fits five bits; the division itself is 9-bit, hence the explicit select.
  wire [8:0] col9 = hpos / CELL_W[8:0];
  wire [4:0] col  = col9[4:0];
  wire [3:0] cell_i = (col > 5'd15) ? 4'd15 : col[3:0];
  wire [3:0] bit_i = 4'd15 - cell_i;

  // 1-pixel dark gutter between cells keeps them countable in a screenshot
  wire gutter = ((hpos % CELL_W[8:0]) == 0) || (y[1:0] == 2'd3);

  // Removed 2026-08-27: eight `wire [15:0]` helpers here were declared and
  // never read -- the overlay rows for the snapshot / dead-man / unmapped /
  // escape-from ADDRESSES were never wired up.  The values themselves
  // (snap_addr, dead_a24, unmap_a24, esc_from) are still live above; only
  // the unused 16-bit split wires are gone.  Restore from git if those rows
  // are ever added.  docs/DEBUG_LOG.md.
  // Named wires rather than indexing a part-select or a concatenation
  // directly: ModelSim rejects `sig[15:0][bit_i]` and `{a,b}[bit_i]` outright
  // where Quartus accepts both, and a file that will not compile takes the
  // boot testbench down with it -- KNOWN_ISSUES.md lesson 8, same shape.

  reg cur;
  wire [4:0] n12_i = row - 5'd8;
  always @(*) begin
    if (page_na12 && (row >= 5'd8)) cur = na12_w[n12_i][bit_i];
    else
    case (row)
      5'd0: cur = flags[bit_i];
      5'd1: cur = n_unmapped[bit_i];
      5'd2: cur = n_cmd[bit_i];
      5'd3: cur = n_blit[bit_i];
      5'd4: cur = st_word0[bit_i];
      5'd5: cur = st_errs[bit_i];
      5'd6: cur = d7p_hi[bit_i];
      5'd7: cur = d7p_lo[bit_i];
      5'd8: cur = pc_hi[bit_i];
      5'd9: cur = pc_lo[bit_i];
      // Rows 10-11 carried C70 volume debug that answered its question in
      // session 10; rows 6-7 the blitter dead-write bank.  RISK-10 needs four.
      5'd10: cur = d7v_hi[bit_i];
      5'd11: cur = d7v_lo[bit_i];
      5'd12: cur = c70_copy_word[bit_i];
      5'd13: cur = st_berrs[bit_i];
      5'd14: cur = st_video[bit_i];
      // Rows 15-16 carried blitter dead-write diagnostics that read zero on
      // every run since the blitter fix; RISK-10 needs them.
      // Rows 15-16 held the PC of the stack-slot read, which answered its
      // question (0xC22638, the pop).  RISK-10 needs them for the frame.
      5'd15: cur = fr_a_hi[bit_i];
      5'd16: cur = fr_a_lo[bit_i];
      // Rows 17-19 were blitter-slip and unmapped-address diagnostics, both
      // reading zero on every run for sessions; RISK-10 needs them.
      5'd17: cur = vec[bit_i];
      5'd18: cur = vec_pc[bit_i];
      5'd19: cur = vec_pl[bit_i];
      // Rows 20-23 carried envelope diagnostics that answered their question
      // in session 10.  RISK-10 needs them: the exception stack frame.
      5'd20: cur = fr0[bit_i];
      5'd21: cur = fr1[bit_i];
      5'd22: cur = fr2[bit_i];
      5'd23: cur = fr3[bit_i];
      // Rows 24-28 carried sound diagnostics that answered their question in
      // session 10 (the melody fault).  RISK-10 needs them more: see the
      // escape latch above.  The sound signals are still wired in at the top
      // level, so putting them back is a one-line change.
      // Rows 24-31 are the switchable page -- `page_snd` chooses.  Both sets
      // are here; neither displaces the other any more.
      5'd24: cur = page_snd ? snd_key_ever [bit_i] : esc_to_hi [bit_i];
      5'd25: cur = page_snd ? snd_loud_ever[bit_i] : esc_to_lo [bit_i];
      5'd26: cur = page_snd ? snd_mute_ever[bit_i] : n_enter   [bit_i];
      5'd27: cur = page_snd ? snd_key_now  [bit_i] : n_restore [bit_i];
      5'd28: cur = page_snd ? snd_bank_seen[bit_i] : esc_n     [bit_i];
      5'd29: cur = page_snd ? snd_oor_ever [bit_i] : d7_wr     [bit_i];
      5'd30: cur = page_snd ? snd_pcm0     [bit_i] : d7_rd     [bit_i];
      default: cur = page_snd ? snd_pcm1   [bit_i] : d7_seen   [bit_i];  // 31
    endcase
  end

  always @(*) begin
    if (gutter)          {r, g, b} = 24'h000000;
    else if (row == 0)   {r, g, b} = cur ? 24'h00E000 : 24'h300000;
    else                 {r, g, b} = cur ? 24'h00A0FF : 24'h101010;
  end

endmodule

`default_nettype wire
