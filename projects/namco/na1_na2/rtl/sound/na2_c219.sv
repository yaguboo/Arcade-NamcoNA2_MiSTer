//============================================================================
//  Namco C219 -- the NA-2's sound: 16 channels of PCM out of work RAM
//
//  The C219 is the 219 ASIC's C140 derivative.  MAME implements it as
//  c219_device in src/devices/sound/c140.cpp; this is written from that plus,
//  more importantly, from a reference checked against MAME's actual audio
//  output rather than against a reading of its source (tools/c219_ref.py --
//  219,619 samples of the attract mode, none differing).
//
//  Where the samples come from (MAME_VERIFIED, namcona1_c219_map): not a
//  sample ROM.  The chip's address space is the 68000's work RAM, so the C219
//  is a third master on main memory alongside the 68000 and the ROM loader.
//  One byte per sounding voice per output sample at most: 16 x 44100 =
//  705,600 reads a second against a 50.113 MHz memory clock.
//
//  Who writes the registers, and why this is silent for now: the C219's
//  register file lives in the *C70's* address space at 0x1000-0x11FF, and the
//  sound driver that fills it is inside c70.bin.  Measured by tapping MAME's
//  MCU bus: 79,291 register writes over 2400 frames, every one from the C70,
//  which never executes outside its internal ROM.  Until na2_c70_hle is
//  replaced by a real M37702 (ROADMAP Level 9) nothing drives this module.
//  It is built now because it is on the critical path either way and because
//  it can be checked exactly today.
//
//  Per output sample, for every voice whose key bit is set (MAME_VERIFIED,
//  c219_device::sound_stream_update):
//
//      delta   = frequency * 2                 pbase is exactly 2 at 44100 Hz
//      offset += delta                         16.16 phase
//      cnt     = offset >> 16                  0..2 source samples advanced
//      pos    += cnt
//      if (pos >= end - start)                 ran off the end
//          pos = loop - start                      if looping or noise
//          else key = 0                            otherwise
//      if (cnt)                                only re-read when it moved
//          prevdt = lastdt
//          lastdt = noise ? lfsr : decode(byte at base + pos)
//          if (inv_sign) lastdt = -lastdt
//          dltdt  = lastdt - prevdt
//      dt = ((dltdt * offset) >> 16) + prevdt  linear interpolation
//      lmix += (inv_lout ? -(dt*lvol) : dt*lvol) >> (5 + shift)
//      rmix += (dt*rvol) >> (5 + shift)        shift is 8 for noise, else 3
//
//  and the mixers are *16 bits wide and wrap*, which is MAME's s16 mixer
//  buffer.  The output is then clamped to +/-4096 and scaled by 8, which is
//  what put_int_clamp(..., 32768/8) does.
//
//  That mix chain is spread over five clocks (TIMING, not behaviour).  Written
//  as one combinational lump -- mu-law decode, an 18-bit subtract, an 18x16
//  multiply, a shift, a 19-bit add, a 19x9 multiply, a shift and a 16-bit add,
//  all between two flip-flops -- it misses the 19.955 ns core clock by 2.87 ns,
//  and it was the whole of the design's worst-path list.  So the per-voice
//  sequencer now spends a state on each arithmetic step: S_DEC decodes the
//  byte and takes lastdt - prevdt, S_MUL does the interpolation multiply, S_DT
//  the shift and the add, S_VOL the volume multiplies, S_MIX the output shift
//  and the accumulate.  Every expression is the one that was here before; only
//  the flip-flop positions moved.  The chip has 1136 system clocks per output
//  sample and the worst case is now about 13 per voice for 16 voices, so the
//  extra states are free.  What says the arithmetic really is unchanged is
//  that sim/run_c219.sh still reports 0 of 20,000 samples differing from
//  MAME's own audio.
//
//  Volumes are read from the register file every sample, so an envelope takes
//  effect immediately; only bank/mode/start/end/loop are latched, at key-on.
//  lvol = (volume * 32) / 24 -- the 32/MAX_VOICE scaling c140.cpp applies even
//  though the 219 has 16 voices and not 24.  MAME_VERIFIED, and it is exactly
//  (volume * 4) / 3, which (v*4*2731) >> 13 reproduces for every 8-bit v.
//
//  The mulaw table needs no ROM.  c219_device::device_start builds 128 entries
//  as a piecewise cumulative sum shifted left by 5, then mirrors them; every
//  use is `table[b] >> 5`, so the shift cancels and what is left is the running
//  sum itself, with the mirror at -(j+1).  Checked against all 256 entries.
//
//  MAME's arithmetic is signed C ints throughout, including places where a
//  value can legitimately go negative -- `sz` when end < start, and `pos` when
//  loop < start.  The widths here are signed for the same reason: an unsigned
//  wrap there would sound like a voice reading from the wrong place, which is
//  a hard thing to notice by ear.
//
//  UNKNOWN: the real chip's sample rate.  MAME's driver instantiates the C219
//  at 44100 Hz while c140.cpp's own notes say the 219's base pitch is the
//  C352's 42667 Hz.  44100 is used here because it is what the reference this
//  is checked against does; nothing on the board has been measured.
//
//  UNKNOWN, and untested: the mulaw path.  Numan Athletics keys on three mode
//  values (0x82, 0x92, 0xFF); the only one with the mulaw bit also sets the
//  noise bit, which wins.  decode_mulaw has no ground truth behind it.
//
//  UNKNOWN: what the output does at exactly +/-4096.  4096 * 8 does not fit a
//  signed 16-bit sample, MAME's float path rounds it to full scale, and the
//  attract mode never gets there (its peak is 20776 of 32767).  Saturating is
//  what a DAC does and wrapping would be an audible click, so it saturates.
//============================================================================
`default_nettype none

module na2_c219 #(
    parameter int NVOICE = 16
) (
    input  wire        clk,
    input  wire        rst,

    // one pulse per output sample; 44.1 kHz
    input  wire        sample_tick,

    // --- register file, as the chip sees it after the little-endian swap ---
    input  wire        reg_we,
    input  wire [8:0]  reg_addr,
    input  wire [7:0]  reg_din,

    // --- sample memory: work RAM, byte addressed ---------------------------
    output reg  [18:0] snd_addr,
    output reg         snd_req,
    input  wire        snd_ack,
    input  wire [7:0]  snd_data,

    // --- audio, the chip's own left and right ------------------------------
    // The board crosses these on the way to the speakers; that belongs to the
    // board, so na2_top does it and not this module.
    output reg  signed [15:0] audio_l,
    output reg  signed [15:0] audio_r,

    // --- diagnostics -------------------------------------------------------
    output wire        busy,            // a sample pass is in progress
    output reg         dbg_overrun,     // a sample tick arrived mid-pass
    output wire [4:0]  dbg_voices,      // voices with their key bit set

    // Each voice's key bit, for the register read-back the C70 depends on.
    // MAME_VERIFIED (c140.cpp): reading a voice's mode register returns
    // (key ? 0x40 : 0) | (what was written & 0x3F) -- the chip reports the key
    // state rather than the bit the driver wrote.  The C70 BIOS's power-on
    // register test reads it back and compares, so a plain write-through
    // shadow diverges there.  Found by diffing the RTL against MAME at
    // instruction 2,377 of the BIOS (docs/c70_rtl_boot.md).
    output wire [NVOICE-1:0] voice_key,

    // --- BGM diagnostics, 2026-08-22 -- TEMPORARY, REMOVE WITH THE FIX -----
    // Everything from here to dbg_vol2_nz, plus the work-RAM sweeper below,
    // exists to find one fault and comes out when it is found.  HANDOFF.md's
    // session 8 block has the removal list; docs/bgm_investigation.md section
    // 14 has what these measured.  The sweeper especially: it borrows this
    // module's memory port and adds 6 % to main-memory traffic, inside a module
    // that is otherwise verified sample-for-sample against MAME.
    // The melody is missing on every presentation screen while the drums keep
    // playing.  This chip is already exonerated GIVEN THE SAME REGISTER WRITES
    // (section 10.2: 380,000 samples across the failing scenes, 0 differing,
    // and its banking, mu-law and mode bits all match c140.cpp line for line),
    // so what is left is (a) the registers our C70 writes, or (b) the PCM in
    // our work RAM.  These three split (a) from (b) in one screenshot and
    // change nothing in the audio path.
    //
    //   key_ever   a voice was keyed on at least once.  A melody voice that
    //              never keys is (a): the driver never asked for it.
    //   mute_ever  a voice was keyed AND read 256 consecutive zero bytes.  A
    //              melody voice that keys and reads zeros is (b): it is
    //              pointed at memory that has nothing in it.
    //   bank_seen  four bits per voice group, one per bank value written to
    //              that group's bank register.  MAME's capture of this game
    //              uses banks 0-3 on every group; a board that has only ever
    //              seen bank 0 is (a), and a loud one.
    output reg  [NVOICE-1:0] dbg_key_ever,
    output reg  [NVOICE-1:0] dbg_mute_ever,
    output reg  [15:0]       dbg_bank_seen,
    //   loud_ever  a voice was keyed on with a volume that can be heard.  The
    //              first pair of rows found the melody voices ARE keyed and DO
    //              read data, which leaves "keyed, reading, and silent" -- and
    //              a voice keyed at volume zero is exactly that, however good
    //              its samples are.  MEASURED in MAME's own register stream
    //              for this game: only 19 of 2,363 key-ons (0.8 %) have both
    //              volumes at zero, and every voice reaches at least 0x43 at
    //              some point, so a voice that never gets a healthy volume on
    //              the board is a divergence and not the driver being quiet.
    //              The threshold is 0x20 of 0xFF -- audible, not merely non-zero.
    //   key_now    the live key mask, so a screenshot can be tied to what was
    //              actually sounding when it was taken.
    output reg  [NVOICE-1:0] dbg_loud_ever,
    output wire [NVOICE-1:0] dbg_key_now,
    //   oor_ever   a voice was keyed and its computed address fell outside the
    //              512 KiB the chip can see, so the fetch was refused and the
    //              sample forced to zero.  This has to be its own signal: the
    //              first version folded it into mute_ever and did not, because
    //              the zero-run counter only advanced on ACKNOWLEDGED fetches
    //              and the refused branch never reaches one.  That made
    //              mute_ever = 0 mean "no voice read 2048 in-range zeroes"
    //              while its comment claimed "nothing is pointed at empty
    //              memory" -- a stronger statement the code could not make.
    output reg  [NVOICE-1:0] dbg_oor_ever,

    // --- the PCM itself ----------------------------------------------------
    // Every check this chip has ever passed was fed MAME's work RAM.  Whether
    // OUR work RAM holds the same PCM has never been compared, and it is the
    // last unexamined link: MAME's own dumps show banks 1-3 wholly rewritten
    // between scenes, so a voice reading the PREVIOUS scene's samples reads
    // non-zero, in-range data and passes every other diagnostic here while
    // sounding like nothing.
    //
    // So checksum the memory rather than the reads.  A sum over what the
    // voices happen to fetch depends on the session; a sum over the bytes
    // themselves does not, which is what makes it comparable to a dump.
    // One 16-bit sum per 128 KiB bank, refreshed every complete sweep.
    // --- one voice's key-on, as the chip computed it -----------------------
    // Judgement step B of the plan: the melody in the scenes that fail is
    // voices 0-2 and 4-5, and MAME's own register stream says voice 0 keys on
    // there with group bank 2, voice bank 0 and start 0x00B0F0, which
    // find_sample turns into sample address 0x04B0F0.  If the board computes a
    // different address for the same note, the fault is in the bank selection
    // or the word-address shift, and nothing downstream needs looking at.  If
    // it computes the same one, the registers are right and the search moves
    // to the sample data or the renderer.
    //
    // Latched at every key-on of SNAP_VOICE, so a screenshot taken in the
    // failing scene carries the note that should be sounding.
    input  wire              sweep_en,
    output reg  [15:0]       dbg_pcm_sum0,
    output reg  [15:0]       dbg_pcm_sum1,
    output reg  [15:0]       dbg_pcm_sum2,
    output reg  [15:0]       dbg_pcm_sum3,
    output reg  [23:0]       dbg_snap_addr,   // find_sample() for SNAP_VOICE
    output reg  [15:0]       dbg_snap_freq,
    output reg  [15:0]       dbg_snap_mv,     // {mode, max(volL,volR)}

    // --- the volume registers of SNAP_VOICE --------------------------------
    // Measured 2026-08-22: in the scene where the melody is missing, voice 0 is
    // keyed with the sample address MAME computes (0x04B0F0), the frequency
    // MAME writes (0x3148 then 0x6290) and the mode MAME writes (0x92) -- and a
    // volume of ZERO, where MAME's own register stream has 0x4D written to both
    // channels immediately before the key-on.  The melody is being played
    // correctly at nothing.
    //
    // These three split the only question left, and they split it cleanly:
    //
    //   vol_n     how many byte writes have landed on this voice's two volume
    //             registers.  Zero means the write never arrives and the fault
    //             is in the C70 wrapper or the bus; non-zero means it does.
    //   vol_last  the pair as last written.
    //   vol_nz    the pair as last written with either byte non-zero.  If the
    //             driver sets 0x4D4D and then zeroes it, vol_nz holds the 0x4D4D
    //             and vol_last holds the 0000 -- which is a different fault
    //             again, and one no single sample of the registers could show.
    output reg  [15:0]       dbg_vol_n,
    output reg  [15:0]       dbg_vol_last,
    output reg  [15:0]       dbg_vol_nz,
    // The same, for the voice that carries the beat in that scene.  MAME keys
    // voice 14 there with mode 0x82 at address 0x022084, and the beat is
    // audible on the board, so its volume should be healthy.  Melody at ~0 and
    // beat at a normal level says the fault is per-channel driver state, not a
    // master volume -- and those are different repairs.
    output reg  [15:0]       dbg_vol2_nz
);

  // =====================================================================
  // Register file
  // =====================================================================
  // Flip-flops rather than an M10K: a key-on write latches seven other bytes
  // of the same voice in the same cycle, and M10K is the resource that is
  // actually tight in this design (87 % of the blocks) while registers are not.
  reg [7:0] rf [0:511];

  // the write path, from c219_device::c219_w
  wire [8:0] rf_addr  = (reg_addr >= 9'h1F8 && reg_addr[0]) ? (reg_addr - 9'd8)
                                                            : reg_addr;
  wire       is_mode  = (rf_addr < 9'h100) && (rf_addr[3:0] == 4'h5);
  wire [3:0] wr_voice = rf_addr[7:4];

  // NOTE on warning 10027 for every rf[{1'b0, ...}] below.
  // rf is [0:511], so nine index bits are exactly right and these are nine.
  // Quartus 17.0 still reports "index expression is not wide enough": it
  // accepts a 9-bit WIRE (rf_addr, right above) or a 9-bit literal, but not a
  // 9-bit concatenation or function result in the index position.
  //
  // Routing these through named 9-bit wires DOES silence it -- and cost 186
  // ALMs in this module, because fifteen separate index wires read as fifteen
  // read ports on a 512x8 register array.  Measured 2026-08-27.  Trading 186
  // ALMs for a cosmetic warning is the wrong way round on a core whose Pocket
  // fit is already resource-bound, so the concatenation stays and the warning
  // is allowlisted with this note.  docs/WARNING_BASELINE.md section 1a.



  wire       key_on   = reg_we && is_mode && reg_din[7];
  wire       key_off  = reg_we && is_mode && !reg_din[7];

  // =====================================================================
  // Voice state
  // =====================================================================
  reg               key    [0:NVOICE-1];
  reg        [15:0] ptoff  [0:NVOICE-1];
  reg signed [17:0] pos    [0:NVOICE-1];
  reg signed [16:0] lastdt [0:NVOICE-1];
  reg signed [16:0] prevdt [0:NVOICE-1];
  reg signed [17:0] dltdt  [0:NVOICE-1];
  reg        [7:0]  vbank  [0:NVOICE-1];
  reg        [7:0]  vmode  [0:NVOICE-1];
  reg signed [17:0] vstart [0:NVOICE-1];
  reg signed [17:0] vend   [0:NVOICE-1];
  reg signed [17:0] vloop  [0:NVOICE-1];

  // One LFSR for the whole chip, not one per voice: c219_device keeps it as a
  // device member, so noise voices share it and their sequences interleave.
  reg [15:0] lfsr;

  integer k;
  assign dbg_voices = key[0]  + key[1]  + key[2]  + key[3]  +
                      key[4]  + key[5]  + key[6]  + key[7]  +
                      key[8]  + key[9]  + key[10] + key[11] +
                      key[12] + key[13] + key[14] + key[15];

  // =====================================================================
  // Sequencer
  // =====================================================================
  // One state per arithmetic step of the mix chain; see the header.
  localparam [3:0] S_IDLE = 4'd0,
                   S_EVAL = 4'd1,
                   S_MEM  = 4'd2,
                   S_DEC  = 4'd3,   // decode the byte, and lastdt - prevdt
                   S_MUL  = 4'd4,   // dltdt * offset
                   S_DT   = 4'd5,   // (that >> 16) + prevdt
                   S_VOL  = 4'd6,   // dt * volume
                   S_MIX  = 4'd7,   // shift, accumulate, retire the voice
                   S_DONE = 4'd8,
                   // the sweeper.  It borrows the memory port while the chip
                   // is idle -- between passes there are about 900 clocks of
                   // nothing -- so it needs no new master on the arbiter and
                   // cannot collide with a voice fetch, which only happens
                   // between S_EVAL and S_DONE.
                   S_SWP  = 4'd9,
                   S_SWP_W = 4'd10;

  reg [3:0]  st;
  reg [3:0]  v;

  reg [18:0] swp_addr;
  reg [15:0] swp_acc0, swp_acc1, swp_acc2, swp_acc3;
  // One fetch per sample, striding 64 bytes.  That is 6 % more main-memory
  // traffic than the voices already ask for -- it cannot be what starves
  // anything -- and it still completes a pass over all 512 KiB in 8192
  // samples, 0.19 s, which matters: the PCM banks are rewritten between
  // scenes, so a sweep slower than a scene would checksum a mixture of two.
  // 8192 bytes sampled out of each 128 KiB bank is a fingerprint, not a
  // complete sum, and that is enough to tell two different sample sets apart.
  assign busy = (st != S_IDLE);
  reg [15:0] lacc, racc;             // the mixers: 16 bits, and they wrap

  // ---- the current voice's registers, read live every sample ------------
  // `vb` removed 2026-08-27: declared and never read. The register file is
  // indexed directly as rf[{v, 4'dN}] below, so the helper was left over.
  wire [7:0]  r_rvol = rf[{1'b0, v,        4'd0}];
  wire [7:0]  r_lvol = rf[{1'b0, v,        4'd1}];
  wire [15:0] r_freq = {rf[{1'b0, v,        4'd2}], rf[{1'b0, v,        4'd3}]};

  // (volume * 32) / 24, which is (volume * 4) / 3
  function automatic [8:0] volscale(input [7:0] vol);
    reg [21:0] t;
    begin
      t        = {12'd0, vol, 2'd0} * 22'd2731;
      volscale = t[21:13];
    end
  endfunction

  wire [8:0] lvol = volscale(r_lvol);
  wire [8:0] rvol = volscale(r_rvol);

  wire [7:0] mode     = vmode[v];
  wire       m_mulaw  = mode[0];
  wire       m_noise  = mode[2];
  wire       m_invl   = mode[3];
  wire       m_loop   = mode[4];
  wire       m_invsgn = mode[6];
  wire [3:0] shamt    = m_noise ? 4'd13 : 4'd8;    // 5 + (noise ? 8 : 3)

  // ---- phase advance -----------------------------------------------------
  wire [16:0] delta   = {r_freq, 1'b0};
  wire [17:0] off_sum = {2'b00, ptoff[v]} + {1'b0, delta};
  wire [1:0]  cnt     = off_sum[17:16];
  wire [15:0] off_new = off_sum[15:0];

  wire signed [17:0] sz      = vend[v] - vstart[v];
  wire signed [18:0] pos_sum = {pos[v][17], pos[v]} + {17'd0, cnt};
  wire               ran_off = (pos_sum >= {sz[17], sz});
  wire signed [17:0] pos_new = ran_off ? (vloop[v] - vstart[v]) : pos_sum[17:0];
  wire               ends    = ran_off && !(m_loop || m_noise);
  wire               active  = key[v] && (r_freq != 16'd0);

  // ---- where the byte comes from ----------------------------------------
  // find_sample(): ((bank register & 3) * 0x20000) + (voice bank << 16) + start
  wire [1:0] bankreg = (v[3:2] == 2'd0) ? rf[9'h1F7][1:0] :
                       (v[3:2] == 2'd1) ? rf[9'h1F1][1:0] :
                       (v[3:2] == 2'd2) ? rf[9'h1F3][1:0] :
                                          rf[9'h1F5][1:0];
  wire signed [25:0] byte_addr = $signed({7'd0, bankreg, 17'd0})
                               + $signed({2'd0, vbank[v], 16'd0})
                               + {{8{vstart[v][17]}}, vstart[v]}
                               + {{8{pos_new[17]}}, pos_new};
  // the chip's space holds only work RAM; anything outside reads as zero
  wire in_range = (byte_addr >= 0) && (byte_addr < 26'sd524288);

  // =====================================================================
  // Values latched between S_EVAL and the mix pipeline.  They have to be
  // latched: by the time S_DEC runs, ptoff[v] and lfsr have already moved on.
  // =====================================================================
  reg        [15:0] mix_off;
  reg signed [16:0] mix_prevdt;      // prevdt as the sample will use it
  reg signed [16:0] mix_lastdt;      // lastdt likewise
  reg signed [17:0] mix_dltdt;
  reg               mix_fresh;       // cnt != 0: a new source sample was read
  reg signed [16:0] mix_noise;
  reg        [7:0]  fetched;

  // ---- decode ------------------------------------------------------------
  // The mulaw table with its >>5 already cancelled; see the header.
  function automatic signed [11:0] decode_mulaw(input [7:0] b);
    reg [6:0]  i;
    reg [10:0] j;
    begin
      i = b[6:0];
      if      (i < 7'd16)  j = {4'd0, i};
      else if (i < 7'd24)  j = 11'd16  + {3'd0, (i - 7'd16),  1'b0};
      else if (i < 7'd48)  j = 11'd32  + {2'd0, (i - 7'd24),  2'b0};
      else if (i < 7'd100) j = 11'd128 + {1'd0, (i - 7'd48),  3'b0};
      else                 j = 11'd544 + {(i - 7'd100), 4'b0};
      decode_mulaw = b[7] ? -($signed({1'b0, j}) + 12'sd1) : $signed({1'b0, j});
    end
  endfunction

  genvar gk;
  generate
    for (gk = 0; gk < NVOICE; gk = gk + 1) begin : g_voice_key
      assign voice_key[gk]   = key[gk];
      assign dbg_key_now[gk] = key[gk];
    end
  endgenerate

  wire [15:0] lfsr_next = {1'b0, lfsr[15:1]} ^ (lfsr[0] ? 16'hFFF6 : 16'h0000);

  wire signed [11:0] mulaw_v = decode_mulaw(fetched);
  wire signed [16:0] raw_dt  = m_noise ? mix_noise
                             : m_mulaw ? {{5{mulaw_v[11]}}, mulaw_v}
                                       : {{6{fetched[7]}}, fetched, 3'b000};
  wire signed [16:0] new_lastdt = m_invsgn ? -raw_dt : raw_dt;

  // ---- interpolate and mix, one arithmetic step per clock ---------------
  // S_DEC's combinational input: the decode above, then the delta.
  wire signed [16:0] eff_prevdt = mix_prevdt;
  wire signed [16:0] eff_lastdt = mix_fresh ? new_lastdt : mix_lastdt;
  wire signed [17:0] eff_dltdt  = mix_fresh
                                ? ({new_lastdt[16], new_lastdt} - {eff_prevdt[16], eff_prevdt})
                                : mix_dltdt;

  // What S_DEC keeps.  The voice-state write-back still happens in the voice's
  // last state, as it did when that state did everything, so the decoded
  // values have to survive that far.
  reg signed [16:0] p_lastdt;
  reg signed [16:0] p_prevdt;
  reg signed [17:0] p_dltdt;

  // The volumes are read live from the register file every sample
  // (MAME_VERIFIED: an envelope write takes effect on the next sample), and
  // they are taken here rather than at the multiply so that the register-file
  // read and the (v*4)/3 scale are not sitting in front of the multiplier.
  // Both states are inside the same voice's pass, so this is the value the
  // accumulate used to see.
  reg        [8:0]  p_lvol, p_rvol;

  // The interpolated sample and the two products.  Deliberately outside the
  // reset: they are written from S_MUL onwards before anything reads them, and
  // keeping them clear-free lets Quartus put them in the DSP blocks' own
  // registers, which is the point of the exercise.
  reg signed [33:0] interp;
  reg signed [18:0] dt;
  reg signed [27:0] prod_l, prod_r;

  // interp is 34 bits; >>> 16 leaves the top 18, which is what this wants.
  // Written as the part-select so the width is the expression's own rather
  // than a 34-bit shift silently truncated to 18.  Same bits either way.
  wire signed [17:0] interp_hi = interp[33:16];

  wire signed [27:0] add_l  = m_invl ? ((-prod_l) >>> shamt) : (prod_l >>> shamt);
  wire signed [27:0] add_r  = prod_r >>> shamt;

  // clamp to +/-4096 and scale by 8 -- put_int_clamp(..., 32768/8)
  function automatic signed [15:0] clamp_out(input signed [15:0] mix);
    begin
      if (mix >= 16'sd4096)       clamp_out =  16'sd32767;
      // 16'sh8000 == -32768.  Written as a bit pattern because -16'sd32768
      // overflows a 16-bit signed literal (it lands on -32768 regardless,
      // but by accident rather than by construction).
      else if (mix <= -16'sd4096) clamp_out =  16'sh8000;
      else                        clamp_out = mix <<< 3;
    end
  endfunction

  // =====================================================================
  always @(posedge clk) begin
    if (rst) begin
      st          <= S_IDLE;
      swp_addr     <= 19'd0;
      swp_acc0     <= 16'd0;
      swp_acc1     <= 16'd0;
      swp_acc2     <= 16'd0;
      swp_acc3     <= 16'd0;
      dbg_pcm_sum0 <= 16'd0;
      dbg_pcm_sum1 <= 16'd0;
      dbg_pcm_sum2 <= 16'd0;
      dbg_pcm_sum3 <= 16'd0;
      snd_req     <= 1'b0;
      snd_addr    <= 19'd0;
      audio_l     <= 16'sd0;
      audio_r     <= 16'sd0;
      dbg_overrun <= 1'b0;
      lfsr        <= 16'h1234;           // c219_device::device_start
      lacc        <= 16'd0;
      racc        <= 16'd0;
      v           <= 4'd0;
      mix_fresh   <= 1'b0;
      for (k = 0; k < NVOICE; k = k + 1) begin
        key[k]    <= 1'b0;
        ptoff[k]  <= 16'd0;
        pos[k]    <= 18'sd0;
        lastdt[k] <= 17'sd0;
        prevdt[k] <= 17'sd0;
        dltdt[k]  <= 18'sd0;
        vbank[k]  <= 8'd0;
        vmode[k]  <= 8'd0;
        vstart[k] <= 18'sd0;
        vend[k]   <= 18'sd0;
        vloop[k]  <= 18'sd0;
      end
      for (k = 0; k < 512; k = k + 1) rf[k] <= 8'd0;
    end else begin
      // ---- the per-sample pass ------------------------------------------
      if (sample_tick && st != S_IDLE) dbg_overrun <= 1'b1;

      case (st)
        S_IDLE: if (sample_tick) begin
          v    <= 4'd0;
          lacc <= 16'd0;
          racc <= 16'd0;
          st   <= S_EVAL;
          // one sweep read per sample, taken at the tick so it can never be in
          // flight when the voices want the port
          if (sweep_en) st <= S_SWP;
        end

        // ---- the sweeper ------------------------------------------------
        S_SWP: begin
          snd_addr <= swp_addr;
          snd_req  <= 1'b1;
          st       <= S_SWP_W;
        end
        S_SWP_W: if (snd_ack) begin
          snd_req <= 1'b0;
          case (swp_addr[18:17])
            2'd0: swp_acc0 <= swp_acc0 + {8'd0, snd_data};
            2'd1: swp_acc1 <= swp_acc1 + {8'd0, snd_data};
            2'd2: swp_acc2 <= swp_acc2 + {8'd0, snd_data};
            default: swp_acc3 <= swp_acc3 + {8'd0, snd_data};
          endcase
          swp_addr <= swp_addr + 19'd64;
          // wrapped: publish this pass and start the next from zero
          if (swp_addr >= 19'h7FFC0) begin
            dbg_pcm_sum0 <= swp_acc0 + ((swp_addr[18:17] == 2'd0) ? {8'd0, snd_data} : 16'd0);
            dbg_pcm_sum1 <= swp_acc1 + ((swp_addr[18:17] == 2'd1) ? {8'd0, snd_data} : 16'd0);
            dbg_pcm_sum2 <= swp_acc2 + ((swp_addr[18:17] == 2'd2) ? {8'd0, snd_data} : 16'd0);
            dbg_pcm_sum3 <= swp_acc3 + ((swp_addr[18:17] == 2'd3) ? {8'd0, snd_data} : 16'd0);
            swp_acc0 <= 16'd0; swp_acc1 <= 16'd0;
            swp_acc2 <= 16'd0; swp_acc3 <= 16'd0;
            swp_addr <= 19'd0;
          end
          st <= S_EVAL;
        end

        S_EVAL: begin
          mix_off    <= off_new;
          mix_fresh  <= (cnt != 2'd0);
          mix_lastdt <= lastdt[v];
          mix_dltdt  <= dltdt[v];
          // when a new source sample is read, prevdt becomes the old lastdt;
          // otherwise everything keeps the value it had
          mix_prevdt <= (cnt != 2'd0) ? lastdt[v] : prevdt[v];

          if (!active) begin
            // no key or no frequency: MAME skips the voice and saves nothing
            if (v == NVOICE - 1) st <= S_DONE; else v <= v + 4'd1;
          end else if (ends) begin
            key[v]   <= 1'b0;
            ptoff[v] <= off_new;
            if (v == NVOICE - 1) st <= S_DONE; else v <= v + 4'd1;
          end else begin
            ptoff[v] <= off_new;
            pos[v]   <= pos_new;
            if (cnt != 2'd0) begin
              if (m_noise) begin
                mix_noise <= $signed({lfsr_next[15], lfsr_next});
                lfsr      <= lfsr_next;
                st        <= S_DEC;
              end else if (in_range) begin
                snd_addr <= byte_addr[18:0];
                snd_req  <= 1'b1;
                st       <= S_MEM;
              end else begin
                fetched <= 8'd0;
                st      <= S_DEC;
              end
            end else begin
              st <= S_DEC;
            end
          end
        end

        S_MEM: if (snd_ack) begin
          snd_req <= 1'b0;
          fetched <= snd_data;
          st      <= S_DEC;
        end

        // stage 1: the byte becomes a sample, and lastdt - prevdt
        S_DEC: begin
          p_lastdt <= eff_lastdt;
          p_prevdt <= eff_prevdt;
          p_dltdt  <= eff_dltdt;
          p_lvol   <= lvol;
          p_rvol   <= rvol;
          st       <= S_MUL;
        end

        // stage 2: the interpolation multiply
        S_MUL: begin
          interp <= p_dltdt * $signed({1'b0, mix_off});
          st     <= S_DT;
        end

        // stage 3: ((dltdt * offset) >> 16) + prevdt
        S_DT: begin
          dt <= {interp_hi[17], interp_hi} + {{2{p_prevdt[16]}}, p_prevdt};
          st <= S_VOL;
        end

        // stage 4: the volume multiplies
        S_VOL: begin
          prod_l <= dt * $signed({1'b0, p_lvol});
          prod_r <= dt * $signed({1'b0, p_rvol});
          st     <= S_MIX;
        end

        // stage 5: the output shift, the accumulate, and the write-back
        S_MIX: begin
          lacc      <= lacc + add_l[15:0];
          racc      <= racc + add_r[15:0];
          lastdt[v] <= p_lastdt;
          prevdt[v] <= p_prevdt;
          dltdt[v]  <= p_dltdt;
          if (v == NVOICE - 1) st <= S_DONE;
          else begin
            v  <= v + 4'd1;
            st <= S_EVAL;
          end
        end

        S_DONE: begin
          audio_l <= clamp_out(lacc);
          audio_r <= clamp_out(racc);
          st      <= S_IDLE;
        end

        default: st <= S_IDLE;
      endcase

      // ---- register writes, and the key-on latch ------------------------
      // Last in the block on purpose: a write landing in the same cycle as
      // the sequencer has to win, so that a key-on is never eaten by the
      // sequencer retiring the same voice.  That is MAME's order too -- a
      // register write flushes the stream first and then changes the state.
      if (reg_we) rf[rf_addr] <= reg_din;
      if (key_on) begin
        key[wr_voice]    <= 1'b1;
        ptoff[wr_voice]  <= 16'd0;
        pos[wr_voice]    <= 18'sd0;
        lastdt[wr_voice] <= 17'sd0;
        prevdt[wr_voice] <= 17'sd0;
        dltdt[wr_voice]  <= 18'sd0;
        vbank[wr_voice]  <= rf[{1'b0, wr_voice, 4'd4}];
        vmode[wr_voice]  <= reg_din;
        // on the 219 the sample addresses are word addresses
        vstart[wr_voice] <= $signed({1'd0, rf[{1'b0, wr_voice, 4'd6}], rf[{1'b0, wr_voice, 4'd7}], 1'b0});
        vend[wr_voice]   <= $signed({1'd0, rf[{1'b0, wr_voice, 4'd8}], rf[{1'b0, wr_voice, 4'd9}], 1'b0});
        vloop[wr_voice]  <= $signed({1'd0, rf[{1'b0, wr_voice, 4'd10}], rf[{1'b0, wr_voice, 4'd11}], 1'b0});
      end else if (key_off) begin
        key[wr_voice] <= 1'b0;
      end
    end
  end

  // =====================================================================
  // BGM diagnostics.  Observation only -- nothing below drives the audio
  // path, and every signal it reads already existed.
  // =====================================================================
  // A run of zero bytes long enough that the voice cannot be making a sound.
  //
  // 2048 source bytes is about 46 ms of continuous digital silence under a
  // KEYED voice.  The first draft of this used 256 (about 6 ms) and that was
  // wrong: real PCM has quiet passages, 6 ms of zeroes inside a genuine sample
  // is entirely possible, and one false positive would light a sticky bit and
  // send the next session after a fault that is not there.  46 ms of unbroken
  // zeroes while a voice is keyed is not a quiet passage, it is nothing being
  // there -- which is the whole question, since MAME's own dumps show banks
  // 1-3 rewritten 93-99 % between scenes and bank 0 sitting at 92 % zeroes.
  //
  // MEASURED, rather than argued: the longest unbroken run of zero bytes
  // anywhere inside MAME's own loaded PCM banks is 330 bytes (7.5 ms) in bank
  // 1, 126 in bank 2, 274 in bank 3.  So 256 would have fired on real sample
  // data and 2048 is 6.2x clear of the worst real run.  Bank 0 has a 61,333
  // byte run of zeroes, but bank 0 is the 68000's work area and holds no
  // samples -- which is itself worth knowing, because a voice pointed there
  // plays silence on any emulator.
  // The refused-fetch condition, spelled out where every term is still valid.
  // It has to be evaluated in the same cycle as the branch it mirrors, because
  // `v` advances afterwards: a strobe latched here and read next cycle would
  // charge the refusal to the wrong voice.
  wire oor_now = (st == S_EVAL) && active && !ends && (cnt != 2'd0)
                 && !m_noise && !in_range;

  // Voice 0.  In the between-stage music MAME keys it with mode 0x92, group
  // bank 2, start 0x00B0F0 -> address 0x04B0F0; in attract it is a different
  // note but the same voice, so one readout covers both and they can be told
  // apart by the frequency beside it.
  localparam [3:0] SNAP_VOICE = 4'd0;
  localparam [3:0] BEAT_VOICE = 4'd14;

  localparam int MUTE_RUN = 2048;
  reg [11:0] zrun [0:NVOICE-1];

  integer dz;
  always @(posedge clk) begin
    if (rst) begin
      dbg_key_ever  <= {NVOICE{1'b0}};
      dbg_mute_ever <= {NVOICE{1'b0}};
      dbg_loud_ever <= {NVOICE{1'b0}};
      dbg_oor_ever  <= {NVOICE{1'b0}};
      dbg_snap_addr <= 24'd0;
      dbg_snap_freq <= 16'd0;
      dbg_snap_mv   <= 16'd0;
      dbg_vol_n     <= 16'd0;
      dbg_vol_last  <= 16'd0;
      dbg_vol_nz    <= 16'd0;
      dbg_vol2_nz   <= 16'd0;
      dbg_bank_seen <= 16'd0;
      for (dz = 0; dz < NVOICE; dz = dz + 1) zrun[dz] <= 12'd0;
    end else begin
      // the two volume registers of SNAP_VOICE: rf_addr = {0, voice, 4'd0}
      // and {0, voice, 4'd1}.  rf_addr is the post-mirror, post-byte-swap index
      // the register file actually uses, so this counts exactly what the chip
      // sees rather than what the bus carried.
      if (reg_we && !rf_addr[8] && rf_addr[7:4] == SNAP_VOICE
          && rf_addr[3:1] == 3'd0) begin
        dbg_vol_n <= dbg_vol_n + 16'd1;          // wraps
        // the pair as it stands once this write has landed
        if (rf_addr[0]) begin
          dbg_vol_last <= {rf[{SNAP_VOICE, 4'd0}], reg_din};
          if (|{rf[{SNAP_VOICE, 4'd0}], reg_din})
            dbg_vol_nz <= {rf[{SNAP_VOICE, 4'd0}], reg_din};
        end else begin
          dbg_vol_last <= {reg_din, rf[{SNAP_VOICE, 4'd1}]};
          if (|{reg_din, rf[{SNAP_VOICE, 4'd1}]})
            dbg_vol_nz <= {reg_din, rf[{SNAP_VOICE, 4'd1}]};
        end
      end

      if (reg_we && !rf_addr[8] && rf_addr[7:4] == BEAT_VOICE
          && rf_addr[3:1] == 3'd0) begin
        if (rf_addr[0]) begin
          if (|{rf[{BEAT_VOICE, 4'd0}], reg_din})
            dbg_vol2_nz <= {rf[{BEAT_VOICE, 4'd0}], reg_din};
        end else begin
          if (|{reg_din, rf[{BEAT_VOICE, 4'd1}]})
            dbg_vol2_nz <= {reg_din, rf[{BEAT_VOICE, 4'd1}]};
        end
      end

      if (key_on && wr_voice == SNAP_VOICE) begin
        // the same arithmetic as byte_addr below, spelled out here because the
        // voice's own registers are not latched until this same edge
        dbg_snap_addr <= {5'd0, (wr_voice[3:2] == 2'd0) ? rf[9'h1F7][1:0] :
                                (wr_voice[3:2] == 2'd1) ? rf[9'h1F1][1:0] :
                                (wr_voice[3:2] == 2'd2) ? rf[9'h1F3][1:0] :
                                                          rf[9'h1F5][1:0], 17'd0}
                       + {rf[{1'b0, wr_voice, 4'd4}], 16'd0}
                       + {6'd0, rf[{1'b0, wr_voice, 4'd6}], rf[{1'b0, wr_voice, 4'd7}], 1'b0};
        dbg_snap_freq <= {rf[{1'b0, wr_voice, 4'd2}], rf[{1'b0, wr_voice, 4'd3}]};
        dbg_snap_mv   <= {reg_din,
                          (rf[{1'b0, wr_voice, 4'd0}] > rf[{1'b0, wr_voice, 4'd1}])
                            ? rf[{1'b0, wr_voice, 4'd0}] : rf[{1'b0, wr_voice, 4'd1}]};
      end
      if (key_on) begin
        dbg_key_ever[wr_voice] <= 1'b1;
        zrun[wr_voice]         <= 12'd0;    // a fresh note starts a fresh run
        // The volume registers are not latched at key-on -- the chip reads them
        // live -- so this is the value in force as the note starts, which is
        // what the driver chose for it.
        if (rf[{1'b0, wr_voice, 4'd0}] >= 8'h20 || rf[{1'b0, wr_voice, 4'd1}] >= 8'h20)
          dbg_loud_ever[wr_voice] <= 1'b1;
      end

      // Which bank values each group's bank register has ever been given.
      // rf_addr is the post-mirror address, so a write through 0x1F9/B/D/F
      // counts for the register it really lands in -- which is the whole point
      // of the mirror and is where a divergence would hide.
      if (reg_we) begin
        case (rf_addr)
          9'h1F7: dbg_bank_seen[{2'd0, reg_din[1:0]}] <= 1'b1;   // voices 0-3
          9'h1F1: dbg_bank_seen[{2'd1, reg_din[1:0]}] <= 1'b1;   // voices 4-7
          9'h1F3: dbg_bank_seen[{2'd2, reg_din[1:0]}] <= 1'b1;   // voices 8-11
          9'h1F5: dbg_bank_seen[{2'd3, reg_din[1:0]}] <= 1'b1;   // voices 12-15
          default: ;
        endcase
      end

      // A fetch that completed, or one the address check refused: both mean
      // "this voice got nothing this sample".  Counting the refused ones too
      // is deliberate -- an out-of-range address and a region of zeroes are
      // the same fault from the player's seat, and distinguishing them is what
      // the address readout is for.
      if (st == S_MEM && snd_ack) begin
        if (snd_data == 8'd0) begin
          if (zrun[v] != MUTE_RUN[11:0]) zrun[v] <= zrun[v] + 12'd1;
          else                          dbg_mute_ever[v] <= 1'b1;
        end else begin
          zrun[v] <= 12'd0;
        end
      end
      // The refused fetch, which the first version of this block missed.  The
      // address check forces the sample to zero without ever asking memory, so
      // no acknowledge arrives and the branch above never runs -- a voice
      // pointed clean outside work RAM was invisible to it.  Same accounting
      // as a zero byte, plus its own flag, because "read zeroes" and "was not
      // allowed to read at all" want different fixes.
      if (oor_now) begin
        dbg_oor_ever[v] <= 1'b1;
        if (zrun[v] != MUTE_RUN[11:0]) zrun[v] <= zrun[v] + 12'd1;
        else                           dbg_mute_ever[v] <= 1'b1;
      end
    end
  end

endmodule

`default_nettype wire
