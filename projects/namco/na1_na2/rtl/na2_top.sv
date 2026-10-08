//============================================================================
//  Namco NA-2 system top -- board independent
//
//  Scope now: 68000 + full address decode + shared work RAM + the seven video
//  memories + blitter + CRTC + C70 boot/mailbox stand-in + C359 keycus.
//  The pixel pipeline (tilemaps, ROZ, sprites, priority) and the C219 are not
//  here yet -- see ROADMAP.md Level 4 and Level 8.
//
//  Memory split (KNOWN_ISSUES.md RISK-1, resolved):
//    off chip (mem_* port -> SDRAM on hardware, behavioural model in sim)
//        work RAM 512 KiB, program ROM 2 MiB, mask ROM 8 MiB
//    on chip (na2_vram, 360 KiB of M10K)
//        palette, CG RAM, shape RAM, video RAM, scratch, scroll, sprite RAM
//
//  Bus masters:
//    main memory  C70 > blitter > 68000
//    video memory blitter > 68000
//============================================================================
`default_nettype none

`include "na2_pkg.svh"

module na2_top #(
    // 1 = the real Mitsubishi M37702 running c70.bin (ROADMAP Level 9).
    // 0 = the old functional stand-in, kept so a regression can be bisected
    //     against a known-good path instead of argued about.
    parameter bit USE_REAL_C70 = 1'b1,
    // The 28C16's power-on contents.  Defaults reproduce what the shipping
    // core has always done (rtl/memory/na2_eeprom.sv), so synthesis is
    // unchanged; a testbench overrides them to model a virgin part or another
    // set's factory image without editing RTL.  Added 2026-08-24 so the
    // blank-EEPROM boot can be reproduced in simulation.
    parameter bit EEPROM_DEFAULT_IMAGE = 1'b0,   // erased 28C16; see na2_eeprom.sv
    parameter     EEPROM_DEFAULT_FILE  = "rtl/memory/na2_eeprom_default.hex",
    // 0 = CG RAM in on-chip block RAM.  1 = in an external store the target
    // provides through cg_xm_*.  rtl/video/na2_cgmem.sv.
    parameter bit CG_EXTERNAL = 1'b0,
    // Row buffers in the external CG cache.  Only meaningful when
    // CG_EXTERNAL; see na2_cgmem.sv.
    parameter int CG_ROWS     = 2,
    // Lines in the block-RAM CG row cache.  0 = none.  See na2_cgmem.sv.
    parameter int CG_CLINES   = 0,
    // Speculative next-cell CG prefetch in na2_cgmem.  It was measured to help
    // while the sprite pass asked for every cell; with SPR_SKIP_EMPTY removing
    // most of that demand it perturbs the stream instead, so the target that
    // turns skipping on turns this off.  Default keeps the old behaviour.
    parameter bit CG_PREFETCH = 1'b1,
    // Words per CG line, log2.  See na2_cgmem.sv.
    parameter int CG_LINE_LOG = 2,
    // ROZ's own CG store on a second external port.  See na2_video.sv.
    parameter bit CG_SPLIT_ROZ    = 1'b0,
    parameter int CG_ROZ_INDEX_XOR_HI = 0,
    parameter int CG_ROZ_INDEX_XOR_LO = 0,
    parameter int CG_ROZ_LINE_LOG = 4,
    parameter int CG_ROZ_CLINES   = 256,
    parameter int CG_ROZ_ROWS     = 16,
    // ROZ store write policy.  All three default to what shipped.
    // na2_cgmem's WRITE_QUEUE / CACHE_WRITE_THROUGH say what they do and
    // DEBUG_LOG O-ROZ12 has the five-frame measurement that chose them.
    parameter int CG_ROZ_WRITE_QUEUE = 0,
    parameter int CG_ROZ_WRITE_QUEUE_URGENT = 0,
    parameter bit CG_ROZ_CACHE_WRITE_THROUGH = 1'b0,
    parameter bit CG_ROZ_CACHE_LOOKUP_AIM = 1'b0,
    parameter int CG_INDEX_XOR_HI = 0,
    parameter int CG_INDEX_XOR_LO = 0,
    // Skip sprite cells whose shape mask is zero before reading CG.  Pocket
    // enables this to avoid external transactions; MiSTer leaves it off.
    parameter bit SPR_SKIP_EMPTY = 1'b0,
    // Per-line sprite hit list length.  rtl/video/na2_sprite.sv header.
    parameter int SPR_MAXHIT = 128,
    // Overlap the fixed 258-cycle sprite Y-list evaluation with the background
    // passes.  Drawing remains in the normal post-ROZ slot.
    parameter bit SPR_EARLY_EVAL = 1'b0,
    // hpos before the end of a line at which an unfinished sprite pass is
    // abandoned.  Passed through to na2_video; see there.  MiSTer keeps the
    // default because its CG never stalls and the deadline is never reached.
    parameter int SPR_MARGIN  = 16
) (
    input  wire        clk,            // 50.113 MHz master clock
    input  wire        rst,            // synchronous, active high
    input  wire        ce_en,          // global clock enable (pause/download)

    // --- main memory: work RAM + program ROM + mask ROM --------------------
    output wire [23:1] mem_addr,
    output wire [15:0] mem_dout,
    input  wire [15:0] mem_din,
    output wire        mem_req,
    output wire        mem_we,
    output wire [1:0]  mem_ds,         // {upper byte, lower byte}
    input  wire        mem_ack,

    // --- C70 BIOS image, 16 KiB ------------------------------------------
    output wire [13:0] bios_addr,
    input  wire  [7:0] bios_data,

    // --- controls, active low as on the JAMMA edge ------------------------
    // Bit order per port: 0=right 1=left 2=down 3=up 4=B1 5=B2 6=B3 7=start.
    // DSW: 0=freeze 1=test 2..5=coin4..coin1 6=service 7=service1.
    // MAME_VERIFIED: INPUT_PORTS_START(namcona1_joy).
    // Player 3 is not wired: on real hardware its eight bits are spread one
    // per C70 analog input in a scrambled order, and the C70 stand-in does not
    // model the analog path yet.  See RESEARCH.md section 13.
    input  wire  [7:0] in_p1,
    input  wire  [7:0] in_p2,
    input  wire  [7:0] in_p4,
    input  wire  [7:0] in_p3,
    input  wire  [7:0] in_dsw,

    // --- 28C16 nvram port (the MiSTer save path, ioctl index 2) -----------
    // The array lives here, so the save port has to be brought out to the
    // board wrapper.  docs/nvram_save.md section 4 records why the two
    // alternatives were rejected: an upward hierarchical reference and
    // unconnected ports both let Quartus optimise the whole EEPROM away, and
    // a build that passes with the EEPROM missing is the failure mode this
    // project already lost three sessions to with the PLL.
    input  wire        nv_req,
    input  wire        nv_we,
    input  wire [10:0] nv_addr,
    input  wire  [7:0] nv_din,
    output wire  [7:0] nv_dout,
    output wire        nv_gnt,
    output wire        nv_game_we,

    // Which raw scanline raises the 68000's vblank interrupt: 0 = 224 (MAME),
    // 1 = 240 (FBNeo), 2 = 256 (end of active display).  KNOWN_ISSUES.md
    // ISSUE-1 -- an open question the board can settle.
    input  wire  [1:0] irq_line_sel,
    // 0 = the sprite bank follows vreg $22 live, 1 = latched per frame.
    input  wire        spr_bank_sel,

    // Which KEYCUS is socketed on this ROM board -- a property of the board,
    // so it comes in with the ROMs rather than being compiled in.  See
    // rtl/keycus/na2_keycus.sv.
    input  wire [15:0] keycus_id,
    input  wire  [2:0] keycus_off,

    // --- video timing out -------------------------------------------------
    output wire        ce_pix,
    output wire [8:0]  hpos,
    output wire [8:0]  vpos,
    output wire        hblank,
    output wire        vblank,
    output wire        hsync,
    output wire        vsync,
    output wire [7:0]  vid_r,
    output wire [7:0]  vid_g,
    output wire [7:0]  vid_b,

    // --- external CG store, only driven when CG_EXTERNAL -------------------
    // Rows of four CG words.  Deliberately says nothing about which memory
    // part answers: the controller belongs to the target.
    output wire        cg_xm_req,
    output wire        cg_xm_we,
    output wire [16:0] cg_xm_addr,
    output wire [15:0] cg_xm_wdata,
    output wire  [1:0] cg_xm_wstrb,
    input  wire        cg_xm_ack,
    input  wire [(16<<CG_LINE_LOG)-1:0] cg_xm_rdata,
    // Second CG store, ROZ only.  Idle unless CG_SPLIT_ROZ.
    output wire        rz_xm_req,
    output wire        rz_xm_we,
    output wire [16:0] rz_xm_addr,
    output wire [15:0] rz_xm_wdata,
    output wire  [1:0] rz_xm_wstrb,
    input  wire        rz_xm_ack,
    input  wire [(16<<CG_ROZ_LINE_LOG)-1:0] rz_xm_rdata,
    output wire [15:0] cg_dbg_row_req,
    output wire [15:0] cg_dbg_row_miss,
    output wire [15:0] cg_dbg_max_wait,
    // The video status word the debug overlay already shows, brought out so a
    // target without an overlay can read it.  Observation only.
    //   [15] sprite list was cut short by the scanline deadline
    //   [14] the line did not finish in time
    //   [13] a C219 sample tick was missed
    //   [12] gfx-bank slip seen during a blit
    output wire [15:0] dbg_video_status,
    output wire [15:0] dbg_sprite_status,

    // --- audio, 16-bit signed, already in board order ----------------------
    output wire signed [15:0] aud_l,
    output wire signed [15:0] aud_r,
    input  wire        dbg_overlay_en,  // TEMPORARY: hardware debug overlay
    // Which instruments the overlay's last eight rows show:
    // 0 = boot/D7, 1 = the C219 sound set.  rtl/video/na2_dbg.sv.
    input  wire        dbg_page_snd,
    // DIAGNOSTIC, branch fix/na12-accuracy: overlay rows 8-31 show the NA12
    // timing probe instead of their usual instruments.  Observation only;
    // see the "NA12 probe" block below and docs/NA12_TIMING_AUDIT.md.
    input  wire        dbg_page_na12,
    // fix/na12-accuracy: 1 = the 68000 takes no wait states for this core's
    // memory latency (rtl/cpu/na2_m68k.sv, "wait states").  0 = as before.
    input  wire        cpu_recover,
    // fix/na12-accuracy: 1 = a palette write while graphics are disabled does
    // not change the backdrop colour until graphics come back on, as MAME
    // (rtl/video/na2_bgpal_hold.sv).  0 = shown immediately, as FBNeo and as
    // this core did before.
    input  wire        pal_hold_en,
    // TEMPORARY, 2026-08-23: makes the sound driver's fade attenuation read as
    // zero, so the melody can be heard returning if that is what silences it.
    // A probe under an OSD switch, removed with the rest of the diagnostics.
    input  wire        snd_fade_off,
    input  wire [15:0] dbg_st_word0,     // TEMPORARY: SDRAM self-test results,
    input  wire [15:0] dbg_st_errs,      //            produced in na2_membus
    input  wire [15:0] dbg_st_dlwords,
    input  wire [15:0] dbg_st_dlindex,
    input  wire [15:0] dbg_st_berrs,

    // --- trace / debug ----------------------------------------------------
    output wire [23:1] dbg_addr,
    output wire        dbg_as_n,
    output wire        dbg_rd,
    output wire        dbg_wr,
    output wire [15:0] dbg_dout,
    output wire [15:0] dbg_din,
    output wire [2:0]  dbg_fc,
    output wire        dbg_ack,
    output wire        dbg_m68k_rst,
    output wire [15:0] dbg_vreg_irq_en,
    output wire        dbg_gfx_enable,
    output wire        dbg_blit_busy,
    output wire        dbg_blit_start,
    output wire [7:0]  dbg_c70_cmd,
    output wire        dbg_c70_done,
    output wire        dbg_unmapped,
    output wire [15:0] dbg_rdata,     // live read mux, valid during ack
    output wire [15:0] dbg_blit_p [0:7], // src/dst fmt, bank, addrs, count
    output wire [15:0] dbg_bus         // handshake state, see below
);

  // =====================================================================
  // 68000
  // =====================================================================
  wire [23:1] cpu_a;
  wire [15:0] cpu_do;
  wire [15:0] cpu_debt;    // na2_m68k's owed clock phases, for the NA12 probe
  wire [31:0] cpu_d7;      // LOCAL: RISK-10 debug, see the D7 watch below
  reg  [15:0] cpu_di;
  wire        cpu_rd, cpu_wr, cpu_uds_n, cpu_lds_n, cpu_as_n;
  wire [2:0]  cpu_fc;
  reg         cpu_ack;
  wire        m68k_rst;
  wire [1:0]  cpu_ds = {~cpu_uds_n, ~cpu_lds_n};
  reg  [2:0]  ipl_n;

  na2_m68k #(.CLK_DIV(`NA2_CPU_DIV)) u_cpu (
      .clk     (clk),
      .rst     (rst),
      .cpu_rst (m68k_rst),
      .ce_en   (ce_en),
      .recover_en(cpu_recover),
      .dbg_debt(cpu_debt),
      .addr    (cpu_a),
      .dbg_d7  (cpu_d7),
      .dout    (cpu_do),
      .din     (cpu_di),
      .rd      (cpu_rd),
      .wr      (cpu_wr),
      .uds_n   (cpu_uds_n),
      .lds_n   (cpu_lds_n),
      .ack     (cpu_ack),
      .ipl_n   (ipl_n),
      .fc      (cpu_fc),
      .as_n    (cpu_as_n),
      .halted_n()
  );

  wire [23:0] a24 = {cpu_a, 1'b0};

  // =====================================================================
  // Address decode
  // MAME_VERIFIED: namcona1_state::namcona1_main_map
  // =====================================================================
  wire sel_workram   = (a24 <= `NA2_WORKRAM_HI);
  wire sel_mailbox   = (a24 >= `NA2_MAILBOX_LO)   && (a24 <= `NA2_MAILBOX_HI);
  wire sel_maskrom   = (a24 >= `NA2_MASKROM_LO)   && (a24 <= `NA2_MASKROM_HI);
  wire sel_prgrom    = (a24 >= `NA2_PRGROM_LO)    && (a24 <= `NA2_PRGROM_HI);
  wire sel_eeprom    = (a24 >= `NA2_EEPROM_LO)    && (a24 <= `NA2_EEPROM_HI);
  wire sel_keycus    = (a24 >= `NA2_KEYCUS_LO)    && (a24 <= `NA2_KEYCUS_HI);
  wire sel_vreg      = (a24 >= `NA2_VREG_LO)      && (a24 <= `NA2_VREG_HI);
  wire sel_palette   = (a24 >= `NA2_PALETTE_LO)   && (a24 <= `NA2_PALETTE_HI);
  wire sel_gfxram    = (a24 >= `NA2_GFXRAM_LO)    && (a24 <= `NA2_GFXRAM_HI);
  wire sel_videoram  = (a24 >= `NA2_VIDEORAM_LO)  && (a24 <= `NA2_VIDEORAM_HI);
  wire sel_scratch   = (a24 >= `NA2_SCRATCH_LO)   && (a24 <= `NA2_SCRATCH_HI);
  wire sel_scroll    = (a24 >= `NA2_SCROLL_LO)    && (a24 <= `NA2_SCROLL_HI);
  wire sel_spriteram = (a24 >= `NA2_SPRITERAM_LO) && (a24 <= `NA2_SPRITERAM_HI);

  wire cpu_mem_sel = sel_workram | sel_maskrom | sel_prgrom;
  wire cpu_vid_sel = sel_palette | sel_gfxram | sel_videoram | sel_scratch |
                     sel_scroll  | sel_spriteram;
  wire cpu_fast_sel = sel_mailbox | sel_keycus | sel_vreg | sel_eeprom;

  // Anything the 68000 touches that we have not decoded.  Never silently
  // return 0 for these -- flag them so simulation reports them (project rule:
  // unmapped accesses get traced, not patched over).
  wire sel_unmapped = ~(cpu_mem_sel | cpu_vid_sel | cpu_fast_sel);
  assign dbg_unmapped = sel_unmapped & (cpu_rd | cpu_wr);

  // =====================================================================
  // C70 stand-in  *** TEMPORARY_HLE ***
  // =====================================================================
  wire vblank_rise;   // from the CRTC, below
  wire [4:0] u_c70_st;   // driven inside the generate below
  wire        c70_req, c70_we, c70_busy;
  wire [1:0]  c70_ds;
  wire [18:1] c70_addr;
  wire [15:0] c70_dout, mbx_dout;
  wire        c70_ack;
  // command-7 parameters as the C70 read them, for the debug overlay.  These
  // used to be declared 200 lines further down, after their first use: Quartus
  // accepted it, ModelSim would not compile the file at all, and the boot
  // testbench had quietly stopped building.
  // c70_dbg_src / c70_dbg_dst removed 2026-08-27: both branches drove them
  // but nothing read them -- the overlay row was never wired. c70_dbg_word
  // IS read (na2_dbg .c70_copy_word) and stays.  docs/DEBUG_LOG.md.
  wire [15:0] c70_dbg_word;

  // RISK-10 stack-slot watch, declared here rather than beside the logic that
  // drives it: na2_top does not set `default_nettype none`, so a name used in
  // the na2_dbg instantiation above its declaration becomes an implicit 1-bit
  // wire and the real declaration is then a duplicate.  The reasoning is with
  // the logic further down.
  wire [15:0] dbg_d7_wr, dbg_d7_rd, dbg_d7_seen, dbg_d7_pc, dbg_d7_pcl;
  wire [15:0] dbg_vec, dbg_vec_pc, dbg_vec_pl;
  wire [15:0] dbg_wr0, dbg_wr1, dbg_wr2, dbg_wr3, dbg_wr4, dbg_wr5;
  wire [15:0] dbg_wa_hi, dbg_wa_lo;
  wire [15:0] dbg_d7v_hi, dbg_d7v_lo, dbg_d7p_hi, dbg_d7p_lo;
  wire [15:0] dbg_enter, dbg_restore;

  // The C219's register port.  With the real C70 this is the sound driver
  // writing the chip; with the stand-in nothing drives it and the chip is
  // silent (KNOWN_ISSUES.md ISSUE-6).
  wire        snd_reg_we;
  wire  [8:0] snd_reg_addr;
  wire  [7:0] snd_reg_din;
  wire [15:0] snd_voice_key;   // the C219's key bits, read back by the C70

  wire [23:0] c70_pc;
  wire  [7:0] c70_opcode;
  wire        c70_halted, c70_unmapped, c70_valid;
  wire [15:0] c70_regA, c70_regB, c70_regX, c70_chan;
  wire [15:0] c70_env_tl, c70_env_ptr, c70_env_rate, c70_env_act;
  wire [15:0] c70_env_fl, c70_chanlist, c70_vec120, c70_fade;

  generate
  if (USE_REAL_C70) begin : g_c70_real

    na2_c70 #(.CLK_DIV(`NA2_CPU_DIV)) u_c70 (
        .clk         (clk),
        .rst         (rst),
        .ce_en       (ce_en),
        .m68k_rst    (m68k_rst),
        .mbx_sel     (sel_mailbox),
        .mbx_addr    (cpu_a[3:1]),
        .mbx_din     (cpu_do),
        .mbx_wr      (cpu_wr & sel_mailbox & cpu_ack),
        .mbx_ds      (cpu_ds),
        .mbx_dout    (mbx_dout),
        .ram_req     (c70_req),
        .ram_we      (c70_we),
        .ram_ds      (c70_ds),
        .ram_addr    (c70_addr),
        .ram_dout    (c70_dout),
        .ram_din     (mem_din),
        .ram_ack     (c70_ack),
        .ram_busy    (c70_busy),
        .bios_addr   (bios_addr),
        .bios_data   (bios_data),
        .in_p1       (in_p1),
        .in_p2       (in_p2),
        .in_p4       (in_p4),
        .in_p3       (in_p3),
        .in_dsw      (in_dsw),
        .vblank_rise (vblank_rise),
        .snd_reg_we  (snd_reg_we),
        .snd_reg_addr(snd_reg_addr),
        .snd_reg_din (snd_reg_din),
        .snd_voice_key(snd_voice_key),
        .dbg_pc      (c70_pc),
        .dbg_opcode  (c70_opcode),
        .dbg_valid   (c70_valid),
        .dbg_halted  (c70_halted),
        .dbg_regA    (c70_regA),
        .dbg_regB    (c70_regB),
        .dbg_regX    (c70_regX),
        .dbg_chan    (c70_chan),
        .dbg_env_tl  (c70_env_tl),
        .dbg_env_ptr (c70_env_ptr),
        .dbg_env_rate(c70_env_rate),
        .dbg_env_act (c70_env_act),
        .dbg_env_fl  (c70_env_fl),
        .dbg_chanlist(c70_chanlist),
        .dbg_vec120  (c70_vec120),
        .dbg_fade    (c70_fade),
        .snd_fade_off(snd_fade_off),
        .dbg_unmapped(c70_unmapped),
        .dbg_unmapped_a(),
        .dbg_mbx4    ()
    );

    assign u_c70_st     = 5'd0;
    // What the overlay shows with the real MCU.  The stand-in put its
    // command-7 parameters here; the MCU has no such thing, and what is worth
    // a screenshot instead is where its program counter is and whether it met
    // an opcode it cannot decode.
    assign dbg_c70_cmd  = c70_opcode;
    // --- liveness, which is the thing a screenshot could not tell us ------
    // Sampling a program counter once a frame cannot distinguish "stopped"
    // from "in a 60 Hz loop, sampled at the same point every time" -- and the
    // C70's task dispatcher IS a 60 Hz loop, so that is not a hypothetical.
    // These two counters answer it: if they move between two screenshots, that
    // side is executing.  Row 2 counts retired MCU instructions in units of
    // 1024; row 12 counts 68000 program fetches and is free to wrap.
    reg  [9:0]  mcu_div;
    reg         mcu_tick;
    reg [15:0]  fetch_cnt;
    always @(posedge clk) begin
      mcu_tick <= 1'b0;
      if (rst) begin
        mcu_div   <= 10'd0;
        fetch_cnt <= 16'd0;
      end else begin
        if (c70_valid) begin
          mcu_div <= mcu_div + 10'd1;
          if (&mcu_div) mcu_tick <= 1'b1;
        end
        // Any completed 68000 bus cycle, supervisor or user, read or write.
        // Filtering on the function code would have made this counter freeze
        // for exactly the reason the counter exists to rule out.
        if (ce_en && cpu_ack && (cpu_rd || cpu_wr))
          fetch_cnt <= fetch_cnt + 16'd1;
      end
    end

    assign dbg_c70_done  = mcu_tick;     // row 2  = MCU instructions / 1024
    assign c70_dbg_word  = fetch_cnt;    // row 12 = 68000 program fetches

  end else begin : g_c70_hle

    na2_c70_hle u_c70 (
        .clk         (clk),
        .rst         (rst),
        .ce_en       (ce_en),
        .m68k_rst    (m68k_rst),
        .mbx_sel     (sel_mailbox),
        .mbx_addr    (cpu_a[3:1]),
        .mbx_din     (cpu_do),
        .mbx_wr      (cpu_wr & sel_mailbox & cpu_ack),
        .mbx_ds      (cpu_ds),
        .mbx_dout    (mbx_dout),
        .ram_req     (c70_req),
        .ram_we      (c70_we),
        .ram_ds      (c70_ds),
        .ram_addr    (c70_addr),
        .ram_dout    (c70_dout),
        .ram_din     (mem_din),
        .ram_ack     (c70_ack),
        .ram_busy    (c70_busy),
        .bios_addr   (bios_addr),
        .bios_data   (bios_data),
        .in_p1       (in_p1),
        .in_p2       (in_p2),
        .in_p4       (in_p4),
        .in_p3       (in_p3),
        .in_dsw      (in_dsw),
        .vblank_rise (vblank_rise),
        .dbg_state   (u_c70_st),
        .dbg_last_cmd(dbg_c70_cmd),
        .dbg_p_src(), .dbg_p_dst(),
        .dbg_copy_word(c70_dbg_word),
        .dbg_cmd_done(dbg_c70_done)
    );

    assign snd_reg_we   = 1'b0;
    assign snd_reg_addr = 9'd0;
    assign snd_reg_din  = 8'd0;
    assign c70_pc       = 24'd0;
    assign c70_opcode   = 8'd0;
    assign c70_halted   = 1'b0;
    assign c70_unmapped = 1'b0;
    assign c70_regA     = 16'd0;
    assign c70_regB     = 16'd0;
    assign c70_regX     = 16'd0;
    assign c70_chan     = 16'd0;
    assign c70_env_tl   = 16'd0;
    assign c70_env_ptr  = 16'd0;
    assign c70_env_rate = 16'd0;
    assign c70_env_act  = 16'd0;
    assign c70_env_fl   = 16'd0;
    assign c70_chanlist = 16'd0;
    assign c70_vec120   = 16'd0;
    assign c70_fade     = 16'd0;
    assign c70_valid    = 1'b0;

  end
  endgenerate

  // =====================================================================
  // Video register file, 0xEFFF00-0xEFFFFF (128 words)
  // MAME_VERIFIED: vreg_w
  // =====================================================================
  reg  [15:0] vreg[0:127];
  wire [6:0]  vreg_i    = cpu_a[7:1];
  wire        vreg_wr   = ce_en & cpu_wr & sel_vreg & cpu_ack;
  // The trigger write, decoded WITHOUT the acknowledge.  cpu_ack is held off
  // while the blitter is busy (see the acknowledge multiplexer below), and
  // vreg_wr depends on cpu_ack, so gating the acknowledge with blit_start
  // itself is a combinational loop -- Quartus reported one of four nodes and
  // the stall could not work.  Decode the attempt, gate the acknowledge with
  // the attempt, and let blit_start stay the accepted write.
  wire        blit_trig_w = ce_en & cpu_wr & sel_vreg &
                            (vreg_i == (`VREG_BLT_TRIGGER >> 1));
  wire        blit_start = vreg_wr & (vreg_i == (`VREG_BLT_TRIGGER >> 1));

  integer k;
  always @(posedge clk) begin
    if (rst) begin
      for (k = 0; k < 128; k = k + 1) vreg[k] <= 16'h0000;
    end else if (vreg_wr) begin
      if (cpu_ds[1]) vreg[vreg_i][15:8] <= cpu_do[15:8];
      if (cpu_ds[0]) vreg[vreg_i][ 7:0] <= cpu_do[ 7:0];
    end
  end

  assign dbg_vreg_irq_en = vreg[`VREG_IRQ_ENABLE >> 1];
  assign dbg_gfx_enable  = |vreg[`VREG_GFX_ENABLE >> 1];
  assign dbg_blit_start  = blit_start;
  // Row 3 and row 13 carry the stall diagnostics this session needs;
  // the blit counter and the byte-write self-test have both done their job
  // (KNOWN_ISSUES.md RISK-8 is resolved and the counter reads 0 every run).
  assign dbg_blit_p[0] = vreg[7'h1];   // source format
  assign dbg_blit_p[1] = vreg[7'h4];   // destination format
  assign dbg_blit_p[2] = vreg[7'h6];   // gfx bank / mode
  assign dbg_blit_p[3] = vreg[7'h7];
  assign dbg_blit_p[4] = vreg[7'h8];
  assign dbg_blit_p[5] = vreg[7'h9];
  assign dbg_blit_p[6] = vreg[7'ha];
  assign dbg_blit_p[7] = vreg[7'hb];   // byte count

  // =====================================================================
  // C219 sound
  // =====================================================================
  // Declared up here because the main-memory arbiter below uses them, and a
  // wire used before its declaration is something Quartus accepts and
  // ModelSim does not -- which is how the boot testbench silently stopped
  // building for two sessions.  See KNOWN_ISSUES.md.
  wire [18:0] snd_addr;
  wire        snd_req;

  // =====================================================================
  // Blitter
  // =====================================================================
  wire        blt_mem_req, blt_vid_req, blt_busy;
  wire        blt_abort;      // a transfer abandoned on an out-of-range address
  wire        vid_dead_wr;    // a write the bank register routes nowhere

  // There is NO honest "trigger lost" flag here any more, and this comment is
  // what replaced one.
  //
  // st_video[12] used to carry `blit_trig_w & blt_busy`, which is the IDENTICAL
  // expression the acknowledge multiplexer stalls on -- so the flag was raised
  // by the stall WORKING, never by a loss.  The game re-triggers into a busy
  // blitter constantly (session 6 measured 36.5 % of kicks), so it could not
  // read 0, and on 2026-08-22 the board's lit cell was read as "a graphic never
  // arrived" when it meant "the CPU waited, as designed".
  //
  // Nor is there a fixed version.  Substituting the definitions:
  //     blit_start      = blit_trig_w & fast_ack & ~(blit_trig_w & blt_busy)
  //                     = blit_trig_w & fast_ack & ~blt_busy
  //     blit_start & blt_busy  ==  0,  identically
  // -- so any flag of that shape is a constant the fitter folds away, and a
  // permanently dark cell is not evidence of anything.  Section 16 mechanism 1
  // is closed BY CONSTRUCTION by the stall, not by measurement, and no flag on
  // this signal can say otherwise.  Writing one anyway is how the mirrored
  // strip and the old blt_trig_lost both happened.
  //
  // So the cell carries something that CAN read either value: blt_slip_seen,
  // the gfx-bank hazard, defined below.

  wire [23:1] blt_mem_addr, blt_vid_addr;
  wire [15:0] blt_vid_dout;
  reg         blt_mem_ack, blt_vid_ack;

  // ---------------------------------------------------------------------
  // The gfx bank a blit uses is the one that was set when it was TRIGGERED.
  //
  // MAME_VERIFIED, and this one matters.  namcona1.cpp:526 calls blit() from
  // inside vreg_w, so the whole transfer -- every transfer_dword, every
  // gfxram_w, and every read of m_vreg[0x0c/2] inside it -- completes before
  // the CPU's write to the trigger register returns.  $0C therefore cannot
  // change part way through a MAME blit: it is frozen by construction.
  //
  // Our blitter takes real time, three clocks a word plus SDRAM latency plus
  // whatever the video arbiter costs, and na2_vram.sv read $0C LIVE.  So a
  // 68000 write to $0C landing mid-transfer re-routes the tail of that blit --
  // and $0C holds 0x0000 for 800 of the 820 writes measured in
  // docs/glitch_investigation.md section 12, which is a value that routes a
  // gfx write to neither CG RAM nor shape RAM.  The tail is silently
  // discarded: a graphic that never arrives, which is section 16's surviving
  // explanation for the spark, and vid_dead_wr is lit on hardware.
  //
  // Latching at the trigger is not a guess about the real board -- it is what
  // makes this blitter agree with the reference it is checked against.  The
  // 68000's own writes to the CG/shape window keep following $0C live, exactly
  // as MAME's gfxram_w does for a CPU write.
  // ---------------------------------------------------------------------
  reg [15:0] blt_bank_l;
  always @(posedge clk) begin
    if (rst)             blt_bank_l <= 16'd0;
    else if (blit_start) blt_bank_l <= vreg[`VREG_GFXBANK >> 1];
  end

  // Does the hazard the latch exists for actually occur on the board?
  //
  // Measured offline against MAME's captured write stream: on 4,400 attract
  // frames, EVERY one of the 96 256-word shape blits has $0C move 3 -> 2 part
  // way through (207 words each, which then land in CG RAM rather than shape
  // RAM), and 77 of 384 512-word CG blits have it move 2 -> 0 (424 words each,
  // routed nowhere).  52,520 words in all.  That is offline arithmetic, so the
  // board gets to confirm it: this counts blits during which the live register
  // moved away from the value the blit captured.
  //
  // Expected NON-zero, and it is the useful half of the pair.  n_dead_wr is the
  // other half and is expected ZERO -- together they say "the hazard happened N
  // times and cost nothing", which is what a working latch looks like.  A flag
  // that can only read one value proves nothing; these two can disagree.
  reg  blt_slip, blt_slip_seen, blt_busy_d;
  reg  [15:0] n_slip;
  always @(posedge clk) begin
    if (rst) begin
      blt_slip <= 1'b0; blt_slip_seen <= 1'b0; blt_busy_d <= 1'b0; n_slip <= 16'd0;
    end else begin
      blt_busy_d <= blt_busy;
      if (blit_start) blt_slip <= 1'b0;
      else if (blt_busy && (vreg[`VREG_GFXBANK >> 1] != blt_bank_l)) blt_slip <= 1'b1;
      if (blt_busy_d && !blt_busy && blt_slip) begin
        n_slip        <= n_slip + 16'd1;      // wraps
        blt_slip_seen <= 1'b1;
      end
    end
  end

  na2_blitter u_blit (
      .clk     (clk),
      .rst     (rst),
      .ce_en   (ce_en),
      .trigger (blit_start),
      .src_fmt (vreg[7'h1]),
      .dst_fmt (vreg[7'h4]),
      .gfxbank (vreg[7'h6]),
      .src_hi  (vreg[7'h7]),
      .src_lo  (vreg[7'h8]),
      .dst_hi  (vreg[7'h9]),
      .dst_lo  (vreg[7'ha]),
      .nbytes  (vreg[7'hb]),
      .busy    (blt_busy),
      .dbg_abort (blt_abort),
      .mem_req (blt_mem_req),
      .mem_addr(blt_mem_addr),
      .mem_din (mem_din),
      .mem_ack (blt_mem_ack),
      .vid_req (blt_vid_req),
      .vid_addr(blt_vid_addr),
      .vid_dout(blt_vid_dout),
      .vid_ack (blt_vid_ack)
  );
  assign dbg_blit_busy = blt_busy;

  // =====================================================================
  // Main-memory arbiter:  C70 > C219 > blitter > 68000
  //
  // The grant is *registered and held* for the whole transaction.  An earlier
  // version selected the master combinationally, which let a higher-priority
  // master take over mid-cycle: the address switched under the memory while
  // the acknowledge was already asserted, so the new master saw an instant
  // (bogus) ack and the old one silently lost its data.  In simulation that
  // corrupted an instruction fetch just after a C70 doorbell and sent the
  // 68000 off into unmapped space.  Hold the grant until mem_ack, then drop
  // it for one cycle so the memory sees a clean request edge.
  //
  // The address is a straight pass-through of the 68000 map; whatever backs
  // mem_* maps work RAM / mask ROM / program ROM onto the download image.
  // =====================================================================
  // The C219 sits second: it asks for at most sixteen bytes per output sample,
  // which is one hundred and twelve of the 1136 clocks in a sample period, and if it does not
  // get them the voice it is on plays the wrong byte for that sample.  It
  // cannot starve anything at that rate.
  wire cpu_mem_req = (cpu_rd | cpu_wr) & cpu_mem_sel;

  localparam GR_NONE = 3'd0, GR_C70 = 3'd1, GR_SND = 3'd2,
             GR_BLT  = 3'd3, GR_CPU = 3'd4;
  reg [2:0] mgrant;

  always @(posedge clk) begin
    if (rst) mgrant <= GR_NONE;
    else if (mgrant == GR_NONE) begin
      // The C219 goes first.  It is the only master here with a hard real-time
      // deadline -- a byte that arrives after its sample period is an audible
      // fault, while the MCU stalling for a few clocks is nothing.  Measured
      // cost (docs/bgm_investigation.md 11.6b): the C70 makes 194 main-memory
      // accesses a frame and loses at most one 7-clock C219 read each, an
      // absolute worst case of 0.16 % of a frame and realistically 0.016 %,
      // against its tightest deadline of 104,128 MCU cycles.  Its handshake
      // holds req until ack with no timeout, so a longer wait adds no state.
      // Worst-case C219 fetch latency 38 -> 26 clocks, worst-case pass
      // 704 -> 512 of 1136, margin 1.61x -> 2.22x.
      if      (snd_req)     mgrant <= GR_SND;
      else if (c70_req)     mgrant <= GR_C70;
      else if (blt_mem_req) mgrant <= GR_BLT;
      else if (cpu_mem_req) mgrant <= GR_CPU;
    end else if (mem_ack)   mgrant <= GR_NONE;
  end

  assign mem_req  = (mgrant != GR_NONE);
  assign mem_addr = (mgrant == GR_C70) ? {5'd0, c70_addr}
                  : (mgrant == GR_SND) ? {5'd0, snd_addr[18:1]}
                  : (mgrant == GR_BLT) ? blt_mem_addr
                                       : cpu_a;
  assign mem_dout = (mgrant == GR_C70) ? c70_dout : cpu_do;
  assign mem_we   = (mgrant == GR_C70) ? c70_we
                  : (mgrant == GR_CPU) ? cpu_wr : 1'b0;
  assign mem_ds   = (mgrant == GR_C70) ? c70_ds
                  : (mgrant == GR_SND) ? 2'b11
                  : (mgrant == GR_BLT) ? 2'b11 : cpu_ds;

  assign c70_ack = mem_ack & (mgrant == GR_C70);
  always @(*) blt_mem_ack = mem_ack & (mgrant == GR_BLT);
  wire cpu_mem_ack = mem_ack & (mgrant == GR_CPU);
  wire snd_ack     = mem_ack & (mgrant == GR_SND);
  // Work RAM holds 68000 words big-endian, so the byte at an even address is
  // the high half -- the same convention na2_membus loads the download with,
  // and the one MAME's C219 reads through (ENDIANNESS_BIG).
  wire [7:0] snd_data = snd_addr[0] ? mem_din[7:0] : mem_din[15:8];

  // =====================================================================
  // Video memories, with the blitter ahead of the 68000
  // =====================================================================
  // The renderer's palette read port.  na2_video decides the pen: a line
  // buffer entry inside the display window, or the backdrop colour outside it.
  wire [11:0] pal_addr;
  wire        pal_alt;
  wire        pal_shadow;
  wire [15:0] pal_word;

  // renderer read ports, driven by na2_video
  wire [16:0] rnd_cg_addr;   wire [15:0] rnd_cg_data;   wire rnd_cg_stall;
  wire [16:0] rnd_cg_pf_addr; wire rnd_cg_pf_req;
  wire [16:0] rnd_rz_cg_addr; wire [15:0] rnd_rz_cg_data; wire rnd_rz_cg_stall;
  wire [13:0] rnd_sh_addr;   wire [15:0] rnd_sh_data;
  wire [10:0] rnd_sp_addr;   wire [15:0] rnd_sp_data;
  wire [14:0] rnd_vr_addr;   wire [15:0] rnd_vr_data;
  wire [10:0] rnd_sc_addr;   wire [15:0] rnd_sc_data;
  // Same held-grant scheme as main memory, for the same reason.
  reg  vid_ack_r;
  wire cpu_vid_req = (cpu_rd | cpu_wr) & cpu_vid_sel;
  // Three bits, not two.  GR_CPU is 3'd4, and in a 2-bit register it truncated
  // to 2'b00 -- the same encoding as GR_NONE.  The arbiter then granted the
  // 68000, immediately believed itself idle again, never raised vid_rq, and
  // never acknowledged: every 68000 access to palette / CG / video / scratch /
  // scroll / sprite RAM hung forever.  Found on hardware in session 6 by
  // latching the bus state at the moment the 68000 gave up waiting
  // (KNOWN_ISSUES.md ISSUE-10), and confirmed in ModelSim.
  reg [2:0] vgrant;

  always @(posedge clk) begin
    if (rst) vgrant <= GR_NONE;
    else if (vgrant == GR_NONE) begin
      if      (blt_vid_req) vgrant <= GR_BLT;
      else if (cpu_vid_req) vgrant <= GR_CPU;
    end else if (vid_ack_r)  vgrant <= GR_NONE;
  end

  wire        vid_owner_blt = (vgrant == GR_BLT);
  wire [23:1] vid_a    = vid_owner_blt ? blt_vid_addr : cpu_a;
  wire [15:0] vid_din  = vid_owner_blt ? blt_vid_dout : cpu_do;
  wire [1:0]  vid_ds   = vid_owner_blt ? 2'b11        : cpu_ds;
  wire [23:0] vid_a24  = {vid_a, 1'b0};

  wire v_pal     = (vid_a24 >= `NA2_PALETTE_LO)   && (vid_a24 <= `NA2_PALETTE_HI);
  wire v_gfx     = (vid_a24 >= `NA2_GFXRAM_LO)    && (vid_a24 <= `NA2_GFXRAM_HI);
  wire v_vram    = (vid_a24 >= `NA2_VIDEORAM_LO)  && (vid_a24 <= `NA2_VIDEORAM_HI);
  wire v_scratch = (vid_a24 >= `NA2_SCRATCH_LO)   && (vid_a24 <= `NA2_SCRATCH_HI);
  wire v_scroll  = (vid_a24 >= `NA2_SCROLL_LO)    && (vid_a24 <= `NA2_SCROLL_HI);
  wire v_spr     = (vid_a24 >= `NA2_SPRITERAM_LO) && (vid_a24 <= `NA2_SPRITERAM_HI);
  wire v_any     = v_pal | v_gfx | v_vram | v_scratch | v_scroll | v_spr;

  wire vid_we = vid_owner_blt ? 1'b1 : (cpu_wr & cpu_vid_sel);
  wire vid_rq = (vgrant != GR_NONE);

  wire [15:0] vram_dout;

  wire cg_a_stall;

  na2_vram #(.CG_EXTERNAL(CG_EXTERNAL), .CG_ROWS(CG_ROWS),
             .CG_CLINES(CG_CLINES), .CG_PREFETCH(CG_PREFETCH),
             .CG_INDEX_XOR_HI(CG_INDEX_XOR_HI),
             .CG_INDEX_XOR_LO(CG_INDEX_XOR_LO),
             .CG_LINE_LOG(CG_LINE_LOG),
             .CG_SPLIT_ROZ(CG_SPLIT_ROZ), .CG_ROZ_LINE_LOG(CG_ROZ_LINE_LOG),
             .CG_ROZ_CLINES(CG_ROZ_CLINES), .CG_ROZ_ROWS(CG_ROZ_ROWS),
             .CG_ROZ_INDEX_XOR_HI(CG_ROZ_INDEX_XOR_HI),
             .CG_ROZ_WRITE_QUEUE(CG_ROZ_WRITE_QUEUE),
             .CG_ROZ_WRITE_QUEUE_URGENT(CG_ROZ_WRITE_QUEUE_URGENT),
             .CG_ROZ_CACHE_WRITE_THROUGH(CG_ROZ_CACHE_WRITE_THROUGH),
             .CG_ROZ_CACHE_LOOKUP_AIM(CG_ROZ_CACHE_LOOKUP_AIM),
             .CG_ROZ_INDEX_XOR_LO(CG_ROZ_INDEX_XOR_LO)) u_vram (
      .rz_cg_addr(rnd_rz_cg_addr), .rz_cg_data(rnd_rz_cg_data),
      .rz_cg_stall(rnd_rz_cg_stall),
      .rz_xm_req(rz_xm_req), .rz_xm_we(rz_xm_we), .rz_xm_addr(rz_xm_addr),
      .rz_xm_wdata(rz_xm_wdata), .rz_xm_wstrb(rz_xm_wstrb),
      .rz_xm_ack(rz_xm_ack), .rz_xm_rdata(rz_xm_rdata),
      .clk          (clk),
      .rst          (rst),
      .a            (vid_a),
      .din          (vid_din),
      .dout         (vram_dout),
      .we           (vid_we & v_any & ce_en),
      .ds           (vid_ds),
      .sel_pal      (v_pal),
      .sel_gfx      (v_gfx),
      .sel_vram     (v_vram),
      .sel_scratch  (v_scratch),
      .sel_scroll   (v_scroll),
      .sel_spr      (v_spr),
      // the blitter uses the bank its trigger captured; the 68000 uses the
      // live register, as MAME's gfxram_w does for a CPU write
      .gfx_bank     (vid_owner_blt ? blt_bank_l : vreg[`VREG_GFXBANK >> 1]),
      .dbg_dead_wr  (vid_dead_wr),
      .r_pal_addr   (pal_addr),    .r_pal_data   (pal_word),
      .r_cg_addr    (rnd_cg_addr), .r_cg_data    (rnd_cg_data),
      .r_cg_stall   (rnd_cg_stall),
      .r_cg_pf_addr (rnd_cg_pf_addr), .r_cg_pf_req (rnd_cg_pf_req),
      .cg_a_stall   (cg_a_stall),
      .cg_xm_req    (cg_xm_req),    .cg_xm_we     (cg_xm_we),
      .cg_xm_addr   (cg_xm_addr),   .cg_xm_wdata  (cg_xm_wdata),
      .cg_xm_wstrb  (cg_xm_wstrb),  .cg_xm_ack    (cg_xm_ack),
      .cg_xm_rdata  (cg_xm_rdata),
      .cg_dbg_row_req (cg_dbg_row_req), .cg_dbg_row_miss(cg_dbg_row_miss),
      .cg_dbg_max_wait(cg_dbg_max_wait),
      .r_shape_addr (rnd_sh_addr), .r_shape_data (rnd_sh_data),
      .r_vram_addr  (rnd_vr_addr), .r_vram_data  (rnd_vr_data),
      .r_scroll_addr(rnd_sc_addr), .r_scroll_data(rnd_sc_data),
      .r_spr_addr   (rnd_sp_addr), .r_spr_data   (rnd_sp_data)
  );

  // one-cycle registered acknowledge for the block RAMs.
  //
  // cg_a_stall extends it when CG lives off-chip and the store has not taken
  // the access yet.  With the on-chip backend it is a constant 0 and this is
  // the original expression again, which is what keeps the MiSTer netlist the
  // same.  Withholding the ack is how the 68000 and the blitter are held: the
  // arbiter above only releases the grant on vid_ack_r.
  always @(posedge clk) begin
    if (rst) vid_ack_r <= 1'b0;
    else     vid_ack_r <= ce_en & vid_rq & ~vid_ack_r & ~cg_a_stall;
  end
  always @(*) blt_vid_ack = vid_ack_r & (vgrant == GR_BLT);
  wire cpu_vid_ack = vid_ack_r & (vgrant == GR_CPU);

  // =====================================================================
  // C359 KEYCUS
  // =====================================================================
  // =====================================================================
  // 28C16 EEPROM
  //
  // The game will not boot past 0xC30932 without it -- see na2_eeprom.sv.
  // =====================================================================
  wire [15:0] eeprom_dout;

  wire eeprom_we = ce_en & cpu_wr & sel_eeprom & cpu_ack & cpu_ds[0];
  assign nv_game_we = eeprom_we;      // the dirty flag the save path watches

  na2_eeprom_array #(
      .DEFAULT_IMAGE (EEPROM_DEFAULT_IMAGE),
      .DEFAULT_FILE  (EEPROM_DEFAULT_FILE)
  ) u_eeprom (
      .clk      (clk),
      .a        (cpu_a[11:1]),
      .din      (cpu_do),
      .dout     (eeprom_dout),
      .we       (eeprom_we),
      // The 68000 owns the array whenever it has a cycle in flight, so a save
      // cannot corrupt a read.  docs/nvram_save.md section 3.
      .cpu_busy (sel_eeprom & (cpu_rd | cpu_wr)),
      .nv_req   (nv_req),
      .nv_we    (nv_we),
      .nv_addr  (nv_addr),
      .nv_din   (nv_din),
      .nv_dout  (nv_dout),
      .nv_gnt   (nv_gnt)
  );

  wire [15:0] keycus_dout;
  reg  fast_ack;
  always @(posedge clk) begin
    if (rst) fast_ack <= 1'b0;
    else     fast_ack <= ce_en & cpu_fast_sel & (cpu_rd | cpu_wr) & ~fast_ack;
  end

  na2_keycus u_keycus (
      .clk    (clk),
      .rst    (rst),
      .ce_en  (ce_en),
      .id     (keycus_id),
      .id_off (keycus_off),
      .sel    (sel_keycus),
      .addr   (cpu_a[3:1]),
      .rd     (cpu_rd & fast_ack),  // exactly one step per completed read
      .dout   (keycus_dout)
  );

  // =====================================================================
  // Pixel output
  //
  // na2_video runs the scanline: per-line scroll, backdrop clear, then the
  // tilemap layers in MAME's priority order, into a double-buffered line
  // buffer that is scanned out one pixel ahead.  ROZ and sprites are not
  // built yet -- see ROADMAP.md Level 4.
  //
  // MAME_VERIFIED: screen_update() fills with (vreg[0xBA] & 0xF) << 8 *before*
  // testing vreg[0x8E] (GFX_ENABLE), so with graphics disabled the screen shows
  // the backdrop colour, not black.  na2_video puts that pen out; blanking is
  // the only thing that forces black here.
  // =====================================================================
  // Renderer health, for the debug overlay.  Sticky, because the interesting
  // event is "did this ever happen", and a per-frame sample would miss it.
  wire        vid_overrun;
  // GATED by the ROZ pass -- see na2_video's port comment.  The raw store
  // stall counts the CPU's writes through the ROZ store while ROZ is idle.
  wire        vid_roz_stall;
  wire [7:0]  vid_spr_hits;
  reg  [7:0]  ovr_lines = 8'd0;   // this frame so far; published at vblank
  reg         vid_overrun_d = 1'b0;
  // ROZ CG stall, per line, worst line of the frame, as a coarse fraction of
  // the line.
  //
  // WHY A BUCKET AND NOT THE NUMBER.  The first version of this published the
  // count in clocks, saturating at 255.  Every frame of the first hardware
  // round read 255 -- including a frame whose overrun count was zero, so the
  // reading could not separate a healthy line from a line spent entirely
  // waiting.  An instrument that returns its maximum whatever it is pointed at
  // is not measuring; it is a stuck bit with extra steps.
  //
  // Two bits over the whole line answer the question that was actually being
  // asked.  A line is 3,184 clocks, so the thresholds below are eighth,
  // quarter and half of one line, and the top bucket means ROZ waited for more
  // of the line than it could possibly have left for the sprite pass.
  reg  [11:0] rz_stall_ln  = 12'd0;
  reg  [1:0]  rz_stall_max = 2'd0;
  wire [1:0]  rz_stall_bkt = (rz_stall_ln >= 12'd1592) ? 2'd3   // half a line
                           : (rz_stall_ln >= 12'd796)  ? 2'd2   // a quarter
                           : (rz_stall_ln >= 12'd398)  ? 2'd1   // an eighth
                                                       : 2'd0;
  reg  [8:0]  hpos_d       = 9'd0;
  wire        vid_spr_ovf;
  // The C219 raises this when a sample tick arrives while the previous pass is
  // still fetching PCM out of shared work RAM.  It has never been watched, and
  // attract mode only proves the fetch path under attract load -- in game the
  // bus carries the 68000, the C70 and a blitter that is now never dropped.
  wire        snd_overrun;
  // How long the C219's fetch pass takes, as a fraction of the 1136-clock
  // sample period.  The overrun flag only says "a tick was missed once"; this
  // says whether the pass is comfortable or living at the edge, which is what
  // decides whether the arbiter change above was necessary or merely tidy.
  //   0 = under a quarter   1 = under a half   2 = under the whole   3 = over
  wire        snd_busy;
  reg  [10:0] snd_pass;      // clocks in the current pass
  reg  [1:0]  snd_band;      // worst band seen, sticky
  reg         snd_busy_d;
  // One pulse per DISCARDED ACCESS, and blitter writes only.
  //
  // `vid_dead_wr` is a level: `we` is high for every clock the grant is held,
  // so counting it directly over-counts by about 2x for the blitter and up to
  // 16x for the 68000, and a count that is wrong by an unknown factor cannot
  // answer "once at a scene change or eight hundred a frame", which is the
  // only reason it exists.  `vid_ack_r` is the single-cycle acknowledge.
  //
  // Blitter only, because a 68000 write into the CG/shape window with $0C set
  // to neither 2 nor 3 is discarded by MAME's gfxram_w in exactly the same way
  // -- that is the reference behaviour, not a fault, and putting it on a fault
  // strip would light a red cell for correct emulation.
  // BGM diagnostics out of the C219 -- see the port comments there.  Declared
  // here rather than beside the instance because the overlay reads them and
  // this file has been bitten twice by declaring a wire below its first use
  // (KNOWN_ISSUES.md lesson 8).
  wire [15:0] snd_key_ever, snd_mute_ever, snd_bank_seen;
  wire [15:0] snd_loud_ever, snd_key_now, snd_oor_ever;
  wire [15:0] snd_pcm0, snd_pcm1, snd_pcm2, snd_pcm3;
  wire [23:0] snd_snap_addr;
  wire [15:0] snd_snap_freq, snd_snap_mv;
  wire [15:0] snd_vol_n, snd_vol_last, snd_vol_nz, snd_vol2_nz;

  // ---------------------------------------------------------------------
  // How long the 68000 waits for the C70 to answer.
  //
  // Work RAM 0x0F60 is the command ACKNOWLEDGE byte.  The 68000 writes a
  // command number there, rings the doorbell at 0x3F8008, and then sits in a
  // two-instruction loop reading 0x0F60 until the C70 clears it
  // (docs/bgm_investigation.md section 6: `cmpi.b #$03,$0F60.w / beq.s`).
  //
  // MAME never exercises that loop.  namcona1.cpp's simulate_mcu() forces
  // m_workram[0xf60/2] = 0 at EVERY vblank, with a FIXME saying it should not
  // be necessary, and it does it even though the real M37710 is running.  So
  // in MAME every handshake succeeds immediately no matter what the MCU did --
  // which means MAME's captured C219 register stream, the thing this whole
  // investigation has used as ground truth for "what the driver does", comes
  // from a machine where a sound command can never block.  Ours can.
  //
  // That is measurable and this is the measurement: how many times the 68000
  // reads that byte.  In MAME the answer would be one or two per command.  A
  // command the C70 never acknowledges shows up here as hundreds or thousands,
  // and it would stall the sound driver for exactly as long as it lasts --
  // which is the shape of a melody that never starts in one scene and is fine
  // in the next.
  //
  // MAME_FBNEO_DIFF.md and docs/c70_rtl_boot.md already record that both
  // emulators are wrong about this address; what is new here is that being
  // right about it costs us a wait they never take.
  // the byte pair at 0x0F60/0x0F61 -- the 68000 reads it with a byte compare,
  // so match the word and let either lane count
  wire f60_rd = ce_en & cpu_rd & cpu_ack & sel_workram
                & (a24[23:1] == 23'h00007B0);   // 0x0F60 >> 1
  reg [15:0] f60_reads;      // total, wrapping
  reg [15:0] f60_frame;      // this frame
  reg [15:0] f60_worst;      // the worst frame so far
  always @(posedge clk) begin
    if (rst) begin
      f60_reads <= 16'd0;
      f60_frame <= 16'd0;
      f60_worst <= 16'd0;
    end else begin
      if (f60_rd) begin
        f60_reads <= f60_reads + 16'd1;
        if (!(&f60_frame)) f60_frame <= f60_frame + 16'd1;
      end
      if (vblank_rise) begin
        if (f60_frame > f60_worst) f60_worst <= f60_frame;
        f60_frame <= 16'd0;
      end
    end
  end

  // ---------------------------------------------------------------------
  // The C70's accumulators at the instruction that computes a voice's volume.
  //
  // The sound driver reaches a voice's volume through
  //     0x00E250  MPY $30,X      A(8) * [$30+X] -> B:A
  // and the high half, in B, becomes the volume byte written to the C219 a few
  // instructions later.  MAME's trace of the scene where the melody is missing
  // has A=0x46BC and B=0x0043 at the next instruction (0x00E253); the board
  // reads A=0x2468 B=0x000A on the same note, at the same sample address and
  // the same pitch.  The multiply is right on both sides -- 0x24 * 0x4A really
  // is 0x0A68 -- so the inputs differ, not the result.
  //
  // A's HIGH byte says which path through the driver produced them, and that
  // turned out to matter more than the product: 8-bit mode leaves it alone, so
  // it still holds whatever the last 16-bit operation left.  MAME's settled
  // channel shows 0x46, the channel base the caller loaded; MAME's RAMPING
  // channel 0x5320 shows 0x03, which is its own level.  The board shows 0x24,
  // its own level -- so the board ramps this channel where MAME never does.
  // docs/bgm_investigation.md section 15.2.
  //
  // Latched on the retire of 0x00E253, which is the first instruction that can
  // see the finished product.
  wire [23:0] VOL_PC = 24'h00E253;
  // Narrowed to the channel that drives C219 voice 0.  Without this the latch
  // showed whichever channel ran last -- healthy values that said nothing about
  // the one that is silent.  MAME's trace: X = 0x4600 here, and a few
  // instructions later `LDX $da` makes it 0x1000, voice 0's register base.
  wire [15:0] MELODY_CHAN = 16'h4600;
  reg [15:0] c70_volA, c70_volB;
  always @(posedge clk) begin
    if (rst) begin
      c70_volA <= 16'd0;
      c70_volB <= 16'd0;
    end else if (c70_valid && c70_pc == VOL_PC && c70_regX == MELODY_CHAN) begin
      c70_volA <= c70_regA;
      c70_volB <= c70_regB;
    end
  end

  // The record latch (0x00E23A) and the advance counter came out with the
  // 2026-08-23 build: both answered.  The board is handed {rate index 0x50,
  // target 0x00} where the working case gets {0x7F, 0xFF}, and it stops
  // advancing while it ramps, which is what a ramping channel does.
  // docs/bgm_investigation.md section 16.2.

  // How often the driver runs the melody channel's envelope routine at all.
  // This is the fact everything else in section 16.4 hangs on and the only one
  // that was never measured: `env_n` counts envelope ADVANCES, which a channel
  // that is merely ramping never does, and the volume latch holds a value that
  // is identical on consecutive passes of a steady note -- attract shows it
  // "frozen" constantly, which is why reading a freeze as "not serviced" was
  // wrong the first time it was tried.
  //
  //   climbing while the level does not move  the channel IS serviced and the
  //                                           level really refuses to change,
  //                                           which a 54/54 opcode bench says
  //                                           it cannot -- suspect the tap
  //   frozen                                  the channel is skipped, and
  //                                           `chanlist` / `vec120` say where
  //
  // 0x00E1AA is the routine's first instruction (SEM), reached through
  // JMP ($0120), and X still holds the channel base there.
  wire [23:0] VISIT_PC = 24'h00E1AA;
  reg [15:0] c70_visits;
  always @(posedge clk) begin
    if (rst)
      c70_visits <= 16'd0;
    else if (c70_valid && c70_pc == VISIT_PC && c70_regX == MELODY_CHAN)
      c70_visits <= c70_visits + 16'd1;          // wraps
  end

  wire dead_wr_ev = vid_dead_wr & vid_ack_r & vid_owner_blt;

  // The three silent ways a graphic can fail to arrive, counted rather than
  // argued about (docs/glitch_investigation.md section 16).  Every other link
  // in ROM -> blitter -> CG/shape RAM -> renderer -> pixel is now verified
  // against MAME's own data, so if the spark is still wrong on the board it is
  // one of these three, and one screenshot during the event names it.
  reg  [15:0] st_video;
  assign dbg_video_status = st_video;

  // THE SPRITE WORD.  st_video says whether the LINE finished; it says
  // nothing about whether the sprites on it did.  A line cut off at the
  // sprite deadline finishes on time and loses its sprites, which is exactly
  // the picture Emeraldia's attract showed on 2026-09-06 while every other
  // number on the grid read healthy.  Two mechanisms drop sprites from a
  // line and this carries both.
  //
  //   [7]   the sprite hit list overflowed somewhere in the last frame
  //   [6:0] how many lines of it lost sprites to the deadline, saturating
  //
  // Saturating, not wrapping: 127 has to keep meaning "at least 127".
  reg  [15:0] st_sprite;
  assign dbg_sprite_status = st_sprite;
  // EIGHT BITS, NOT SEVEN.  A frame has 224 visible lines and this counted to
  // 127.  On hardware the bubble screen pinned it at 127 for builds 39, 40 and
  // 41 alike -- three different bitstreams reporting the same number because
  // the number was the counter's ceiling, not the core's behaviour.  A
  // saturated instrument cannot say whether a fix helped; it can only say the
  // fault is still above the ceiling.  DEBUG_LOG O-ROZ14.
  reg  [7:0]  abort_lines  = 8'd0;
  reg         abort_seen   = 1'b0;   // this line has already been counted
  reg         spr_ovf_frm  = 1'b0;
  wire        vid_spr_abort;
  always @(posedge clk) begin
    if (rst) begin
      st_video   <= 16'd0;
      ovr_lines  <= 8'd0;
      vid_overrun_d <= 1'b0;
      st_sprite  <= 16'd0;
      abort_lines <= 8'd0;
      abort_seen <= 1'b0;   spr_ovf_frm <= 1'b0;
      snd_pass   <= 11'd0;
      snd_band   <= 2'd0;
      snd_busy_d <= 1'b0;
    end
    else begin
      vid_overrun_d <= vid_overrun;

      // Per line: accumulate, bucket it, and keep the frame's worst bucket.
      // The accumulator saturates at 4,095, which is more than the 3,184 a
      // line has, so the top bucket cannot be reached by counting past the end
      // of a line.
      hpos_d <= hpos;
      if (hpos == 9'd0 && hpos_d != 9'd0) begin
        if (rz_stall_bkt > rz_stall_max) rz_stall_max <= rz_stall_bkt;
        rz_stall_ln <= 12'd0;
      end else if (vid_roz_stall && (rz_stall_ln != 12'hFFF)) begin
        rz_stall_ln <= rz_stall_ln + 12'd1;
      end
      // st_video[15:14] is the ROZ stall bucket.  [13:8] is left for the
      // TARGET to fill: on Pocket those six bits carry the second PSRAM's
      // self-test, which is a clk_ram-domain fact this module has no business
      // sampling.  On MiSTer CG is on-chip, vid_roz_stall is constant zero,
      // and all eight bits read zero -- which is the correct report.
      if (vblank_rise) begin
        st_video[15:14] <= rz_stall_max;
        st_video[13:8]  <= 6'd0;
        rz_stall_max    <= 2'd0;
      end

      // ONE PER LINE, NOT ONE PER CLOCK.  sp_abort is a level: it stays high
      // from the deadline to the end of the sprite pass, so counting it
      // directly counts clocks and saturates inside a single line -- the same
      // way the first version of ovr_lines reported "255 of 224".  `abort_seen`
      // makes it an edge per line, cleared where the line is.
      if (hpos == 9'd0 && hpos_d != 9'd0) abort_seen <= 1'b0;
      else if (vid_spr_abort && !abort_seen) begin
        abort_seen <= 1'b1;
        if (abort_lines != 8'd255) abort_lines <= abort_lines + 8'd1;
      end
      if (vid_spr_ovf) spr_ovf_frm <= 1'b1;
      if (vblank_rise) begin
        // The overflow flag moves up out of the byte the Pocket grid shows, so
        // that byte is the whole count.  It is not lost -- MiSTer's na2_dbg
        // reads all sixteen bits -- and it was never independent: measured on
        // build 39, every frame with zero cut lines also had zero overflow, so
        // the two move together and the count is the one that carries size.
        st_sprite   <= {7'd0, spr_ovf_frm, abort_lines};
        abort_lines <= 8'd0;
        spr_ovf_frm <= 1'b0;
      end
      snd_busy_d <= snd_busy;
      if (snd_busy) begin
        snd_pass <= snd_pass + 11'd1;
        if      (snd_pass >= 11'd1136) snd_band <= 2'd3;
        else if (snd_pass >= 11'd568)  snd_band <= (snd_band < 2'd2) ? 2'd2 : snd_band;
        else if (snd_pass >= 11'd284)  snd_band <= (snd_band < 2'd1) ? 2'd1 : snd_band;
      end else if (snd_busy_d) begin
        snd_pass <= 11'd0;
      end
      // snd_band and the blitter/dead-write sticky flags used to live in
      // [12:8].  They are still computed -- the bridge diagnostic word carries
      // them -- but the grid byte is worth more as a ROZ stall figure while
      // this fault is open.
      // st_video[7:0] counts the lines in the LAST COMPLETED FRAME that did
      // not finish in time, and starts again at every vblank.
      //
      // It used to carry the peak sprite count, and bit 14 carried a sticky
      // "a line overran at some point" flag.  Sticky is the wrong shape for
      // the question actually being asked on hardware.  One late line during
      // boot latches it, and from then on the grid reads 1 whether the board
      // is drawing perfectly or dropping every sprite on the screen -- which
      // is exactly what happened: builds 26 and 29 both read line_overrun=1
      // in a scene that looked right and in one that did not, and a whole
      // session was spent treating that as evidence.
      //
      // A count separates the two, and it is in the same unit the simulator
      // reports ("lines that did not finish in time: N of 224"), so the two
      // readouts can finally be compared instead of merely both existing.
      if (vblank_rise) begin
        st_video[7:0] <= ovr_lines;
        ovr_lines     <= 8'd0;
      end else if (vid_overrun && !vid_overrun_d && (ovr_lines != 8'hFF)) begin
        // RISING EDGE, not level.  na2_video holds `overrun` from the line
        // start that observed a late line until the next one, so counting the
        // level counts clocks and saturates at 255 within a line -- which is
        // what the first build of this counter reported: "255 of 224".
        ovr_lines <= ovr_lines + 8'd1;
      end
    end
  end

  // ---------------------------------------------------------------------
  // A lit sticky flag says a thing happened once.  That is not enough to act
  // on: "once, at a scene change" and "eight hundred times a frame" want
  // completely different investigations, and neither the address nor the bank
  // register that caused it survives in one bit.  So the two faults that are
  // actually lit on the board get counted and their first occurrence latched.
  //
  // Counters WRAP rather than saturate.  n_cmd and n_blit below both saturate
  // at 0xFFFF and both read 0xFFFF on every screenshot this project has taken,
  // which makes them useless for exactly the job KNOWN_ISSUES.md lesson 21 says
  // they exist for -- telling a stopped machine from a 60 Hz loop.  A wrapping
  // counter compared between two shots always answers that.
  // ---------------------------------------------------------------------
  reg [15:0] n_dead_wr;        // CG/shape writes the bank register routed nowhere
  reg [15:0] dead_bank;        // vreg $0C as it stood at the FIRST such write
  reg [23:0] dead_a24;         // and the address it was aimed at
  reg        dead_seen;
  reg [23:0] unmap_a24;        // first unmapped 68000 address
  reg        unmap_wr;         // 1 = it was a write
  reg        unmap_seen;

  always @(posedge clk) begin
    if (rst) begin
      n_dead_wr   <= 16'd0;
      dead_bank   <= 16'd0;
      dead_a24    <= 24'd0;
      dead_seen   <= 1'b0;
      unmap_a24   <= 24'd0;
      unmap_wr    <= 1'b0;
      unmap_seen  <= 1'b0;
    end else begin
      if (dead_wr_ev) begin
        n_dead_wr <= n_dead_wr + 16'd1;          // wraps on purpose
        if (!dead_seen) begin
          dead_seen   <= 1'b1;
          // the bank that ACTUALLY routed this write, which since blt_bank_l
          // exists is not the live register for a blitter write.  Latching
          // vreg[$0C] here would report a value that had nothing to do with
          // the write being counted.
          dead_bank   <= vid_owner_blt ? blt_bank_l : vreg[`VREG_GFXBANK >> 1];
          dead_a24    <= vid_a24;
        end
      end
      if (dbg_unmapped && !unmap_seen) begin
        unmap_seen <= 1'b1;
        unmap_a24  <= a24;
        unmap_wr   <= cpu_wr;
      end
    end
  end

  na2_video #(.SPLIT_ROZ_CG(CG_SPLIT_ROZ), .SPR_MARGIN(SPR_MARGIN),
              .SPR_SKIP_EMPTY(SPR_SKIP_EMPTY), .SPR_MAXHIT(SPR_MAXHIT),
              .SPR_EARLY_EVAL(SPR_EARLY_EVAL)) u_video (
      .clk        (clk),
      .rst        (rst),
      .hpos       (hpos),
      .vpos       (vpos),
      .ce_pix     (ce_pix),

      .vreg_xmin  (vreg[`VREG_CRTC_XMIN >> 1]),
      .vreg_xmax  (vreg[`VREG_CRTC_XMAX >> 1]),
      .vreg_ymin  (vreg[`VREG_CRTC_YMIN >> 1]),
      .vreg_ymax  (vreg[`VREG_CRTC_YMAX >> 1]),
      .gfx_enable (dbg_gfx_enable),
      .vreg_pri0  (vreg[(`VREG_PRI0 >> 1) + 7'd0]),
      .vreg_pri1  (vreg[(`VREG_PRI0 >> 1) + 7'd1]),
      .vreg_pri2  (vreg[(`VREG_PRI0 >> 1) + 7'd2]),
      .vreg_pri3  (vreg[(`VREG_PRI0 >> 1) + 7'd3]),
      .vreg_clr0  (vreg[(`VREG_CLR0 >> 1) + 7'd0]),
      .vreg_clr1  (vreg[(`VREG_CLR0 >> 1) + 7'd1]),
      .vreg_clr2  (vreg[(`VREG_CLR0 >> 1) + 7'd2]),
      .vreg_clr3  (vreg[(`VREG_CLR0 >> 1) + 7'd3]),
      .vreg_bg    (vreg[`VREG_CLR_ROZ_IDX]),
      .vreg_bpp   (vreg[`VREG_BPP_FLAGS >> 1]),
      .vreg_sprctl(vreg[`VREG_SPRITE_CTRL >> 1]),
      .spr_bank_sel(spr_bank_sel),
      .vreg_pri_roz(vreg[`VREG_PRI_ROZ >> 1]),
      .vreg_rz_xx (vreg[7'h60]),      // $C0
      .vreg_rz_xy (vreg[7'h61]),      // $C2
      .vreg_rz_yx (vreg[7'h62]),      // $C4
      .vreg_rz_yy (vreg[7'h63]),      // $C6
      .vreg_rz_x0 (vreg[7'h64]),      // $C8
      .vreg_rz_y0 (vreg[7'h65]),      // $CA

      .scroll_addr(rnd_sc_addr), .scroll_data(rnd_sc_data),
      .vram_addr  (rnd_vr_addr), .vram_data  (rnd_vr_data),
      .cg_addr    (rnd_cg_addr), .cg_data    (rnd_cg_data),
      .cg_stall   (rnd_cg_stall),
      // ROZ's own CG port, tied off until a target carries it.  na2_video
      // defaults SPLIT_ROZ_CG to 0 and then ignores these, but an input left
      // dangling is the fifth suspect in SIMULATION_RULES section 11 and this
      // project has lost hardware rounds to one, so it is driven rather than
      // left open.
      .roz_cg_addr (rnd_rz_cg_addr), .roz_cg_data (rnd_rz_cg_data),
      .roz_cg_stall(rnd_rz_cg_stall),
      .cg_pf_addr (rnd_cg_pf_addr), .cg_pf_req (rnd_cg_pf_req),
      .shape_addr (rnd_sh_addr), .shape_data (rnd_sh_data),
      .spr_addr   (rnd_sp_addr), .spr_data   (rnd_sp_data),

      .pal_addr   (pal_addr),
      .pal_alt    (pal_alt),
      .pal_shadow (pal_shadow),
      .dbg_roz_stall    (vid_roz_stall),
      .dbg_line_overrun (vid_overrun),
      .dbg_spr_hits     (vid_spr_hits),
      .dbg_spr_overflow (vid_spr_ovf),
      .dbg_spr_abort    (vid_spr_abort)
  );


  // =====================================================================
  // C219 sound
  // =====================================================================
  // The sample clock: 44100 Hz out of the 50.113 MHz core clock, by adding
  // 44100 every cycle and taking a tick each time the accumulator passes the
  // clock rate.  Exactly 44100 ticks per 50,113,000 cycles, with the jitter
  // spread one cycle at a time rather than piling up.
  //
  // 44100 is MAME's rate for this chip and the rate everything here is checked
  // against; c140.cpp's own notes say the 219's base pitch is the C352's
  // 42667 Hz.  Nothing on the board has been measured.  UNKNOWN.
  // 27 bits to match snd_acc.  `int` here made every accumulator expression
  // 32-bit and truncate back down; the values fit 27 bits with room to spare
  // (snd_acc + SND_HZ peaks at 50,157,100 against 2**27 = 134,217,728).
  localparam [26:0] CLK_HZ  = 27'd50_113_000;
  localparam [26:0] SND_HZ  = 27'd44_100;
  reg [26:0] snd_acc;
  reg        snd_tick;
  always @(posedge clk) begin
    if (rst) begin
      snd_acc  <= 27'd0;
      snd_tick <= 1'b0;
    end else begin
      snd_tick <= 1'b0;
      if (snd_acc + SND_HZ >= CLK_HZ) begin
        snd_acc  <= snd_acc + SND_HZ - CLK_HZ;
        snd_tick <= 1'b1;
      end else begin
        snd_acc <= snd_acc + SND_HZ;
      end
    end
  end

  // The register port is driven by the C70 above: with the real MCU that is
  // the sound driver inside c70.bin, which is what makes this chip audible
  // (KNOWN_ISSUES.md ISSUE-6, ROADMAP Level 8 + 9).
  wire signed [15:0] snd_l, snd_r;

  na2_c219 u_c219 (
      .clk         (clk),
      .rst         (rst),
      .sample_tick (snd_tick),
      .reg_we      (snd_reg_we),
      .reg_addr    (snd_reg_addr),
      .reg_din     (snd_reg_din),
      .snd_addr    (snd_addr),
      .snd_req     (snd_req),
      .snd_ack     (snd_ack),
      .snd_data    (snd_data),
      .audio_l     (snd_l),
      .audio_r     (snd_r),
      .busy        (snd_busy),
      .dbg_overrun (snd_overrun),
      .dbg_voices  (),
      .voice_key   (snd_voice_key),
      .dbg_key_ever  (snd_key_ever),
      .dbg_mute_ever (snd_mute_ever),
      .dbg_bank_seen (snd_bank_seen),
      .dbg_loud_ever (snd_loud_ever),
      .dbg_key_now   (snd_key_now),
      .dbg_oor_ever  (snd_oor_ever),
      // the memory sweeper runs only with the debug overlay on, so a normal
      // session carries none of its traffic
      .sweep_en      (dbg_overlay_en),
      .dbg_pcm_sum0  (snd_pcm0),
      .dbg_pcm_sum1  (snd_pcm1),
      .dbg_pcm_sum2  (snd_pcm2),
      .dbg_pcm_sum3  (snd_pcm3),
      .dbg_snap_addr (snd_snap_addr),
      .dbg_snap_freq (snd_snap_freq),
      .dbg_snap_mv   (snd_snap_mv),
      .dbg_vol_n     (snd_vol_n),
      .dbg_vol_last  (snd_vol_last),
      .dbg_vol_nz    (snd_vol_nz),
      .dbg_vol2_nz   (snd_vol2_nz)
  );

  // The board crosses the chip's outputs on the way to the speakers
  // (MAME_VERIFIED, namcona1.cpp: add_route(0, "speaker", 1.00, 1) and
  // add_route(1, "speaker", 1.00, 0)).  Whether that is the PCB's wiring or
  // MAME compensating for something else is UNKNOWN; it is reproduced because
  // it is what the audio this core is checked against does.
  assign aud_l = snd_r;
  assign aud_r = snd_l;

  wire [7:0]  pal_r, pal_g, pal_b;

  // The backdrop colour while graphics are off: MAME holds palette writes
  // until graphics return (rtl/video/na2_bgpal_hold.sv).  With graphics off
  // na2_video puts nothing but the backdrop pen out (show = gfx_enable && ...),
  // so substituting the word for every pixel is substituting the backdrop.
  // vid_ack_r is the one clock a palette access completes on; vgrant still
  // names its owner then, so address, data and strobes are that write's.
  wire [15:0] bg_hold_word;
  na2_bgpal_hold u_bgpal (
      .clk        (clk),
      .rst        (rst),
      .we         (vid_ack_r & vid_we & v_pal),
      .idx        (vid_a[12:1]),
      .ds         (vid_ds),
      .din        (vid_din),
      .gfx_enable (dbg_gfx_enable),
      .bg_sel     (vreg[`VREG_CLR_ROZ_IDX][3:0]),
      .word       (bg_hold_word)
  );
  wire [15:0] pal_word_shown = (pal_hold_en & ~dbg_gfx_enable) ? bg_hold_word
                                                                : pal_word;

  na2_palette u_pal (
      .clk      (clk),
      .pal_word (pal_word_shown),
      .alt      (pal_alt),
      .shadow   (pal_shadow),
      .r        (pal_r),
      .g        (pal_g),
      .b        (pal_b)
  );

  wire active = ~hblank & ~vblank;
  wire [7:0] bg_r = active ? pal_r : 8'd0;
  wire [7:0] bg_g = active ? pal_g : 8'd0;
  wire [7:0] bg_b = active ? pal_b : 8'd0;

  // --- hardware debug overlay  *** TEMPORARY *** --------------------------
  // There is no debugger on a DE10-Nano.  Without this a black screen tells
  // us nothing; with it, one screenshot says how far the 68000 got.
  wire       dbg_active;
  wire [7:0] dbg_r, dbg_g, dbg_b;

  // =====================================================================
  // Stall snapshot -- who holds the main-memory bus when the 68000 cannot get
  // it.  Session 6 measured the 68000 stopping dead after 2,110 bus cycles
  // while the MCU kept running, which says the CPU is waiting for a grant or
  // an acknowledge that never comes.  A screenshot cannot answer "why" unless
  // the state at the moment of the wedge is latched, because by the time it is
  // read the request has been pending for minutes.
  //
  // Latched once, the first time the 68000's request has been outstanding for
  // 4096 clocks (82 us -- three orders of magnitude longer than any legitimate
  // SDRAM access), and never overwritten after that.
  // =====================================================================
  reg [15:0] stall_snap;
  reg [11:0] stall_cnt;
  reg [15:0] ack_cnt;      // main-memory acknowledges, wrapping: is the bus alive at all?

  always @(posedge clk) begin
    if (rst) begin
      stall_snap <= 16'd0;
      stall_cnt  <= 12'd0;
      ack_cnt    <= 16'd0;
    end else begin
      if (mem_ack) ack_cnt <= ack_cnt + 16'd1;

      // cpu_mem_ack is declared further down with the rest of the arbiter's
      // acknowledges; spell it out here rather than move a declaration.
      // ANY outstanding 68000 access, not just a main-memory one.  The
      // instruction the CPU died on writes to scratch RAM, which is on the
      // video side and does not go through this arbiter at all -- a detector
      // that only watched main memory would never have fired.
      if ((cpu_rd | cpu_wr) && !cpu_ack) stall_cnt <= stall_cnt + 12'd1;
      else                               stall_cnt <= 12'd0;

      if (!stall_snap[15] && (&stall_cnt))
        stall_snap <= {1'b1,                 // [15] captured
                       mgrant,               // [14:12] main-memory grant
                       c70_req,              // [11]
                       snd_req,              // [10]
                       blt_mem_req,          // [9]
                       mem_req,              // [8]
                       mem_ack,              // [7]
                       (vgrant == GR_BLT),   // [6] video grant: blitter
                       (vgrant == GR_CPU),   // [5] video grant: 68000
                       cpu_mem_sel,          // [4]  which target the
                       cpu_vid_sel,          // [3]  68000 was waiting on
                       cpu_fast_sel,         // [2]
                       cpu_wr,               // [1]
                       blt_busy};            // [0]
    end
  end

  // =====================================================================
  // NA12 probe  *** DIAGNOSTIC -- branch fix/na12-accuracy ***
  //
  // The game-level timing tools/mame/na12_golden.lua measures in MAME, measured
  // here the same way so the two can be compared number for number
  // (docs/NA12_TIMING_AUDIT.md).  Observation only: nothing below reaches game
  // logic.  Every word is published at vblank_rise, so one screenshot shows
  // one frame and the rows cannot tear against each other.
  //
  // Numan Athletics' frame, ROM_DISASM_CONFIRMED (IRQ4 C006A2, IRQ3 C006F6,
  // main loop C00AF0):
  //   IRQ3 (posirq)  $2310++ always.  If $230A == 0: $2312++, $230C++, work,
  //                  wait for IRQ4 to clear $2312, then $230A++ (release).
  //   IRQ4 (vblank)  if $2312: clear it; if also $230A == 0: window step
  //                  (C007E2) and sprite bank flip (C00A10).
  //   main loop      waits for $230A != 0, runs the frame, writes $230A = 0.
  // So $2310 - $230C counts frames the main loop did not finish in time, and
  // $230A set -> cleared is the main loop's own work, in lines.
  // The addresses are Numan Athletics'.  On other sets the words mean nothing.
  //
  //   w0  $2310 shadow           w12 {$0FC1 ever-OR, last}  (68000 or MCU)
  //   w1  $230C shadow           w13 {raw B1 presses, raw B2 presses}
  //   w2  {busy max, busy last}  w14 {MCU-deposit B1 presses, B2 presses}
  //   w3  {line/2 $230A set,     w15 frames (vblank_rise), wrapping
  //        line/2 $230A clear}   w16 {MCU deposits to 0x0FFE, 68000 reads}
  //   w4  {line/2 IACK3, IACK4}  w17 $0F60 reads, last frame
  //   w5  bus-pending clocks/16  w18 {line/2 $2312 cleared, clears}
  //   w6  ... max of 32 frames   w19 {$0FC0 ever-OR, last}
  //   w7  bus cycles             w20 {doorbells max-32, last}
  //   w8  est. wait states       w21 {blits max-32, last}
  //   w9  {$0F60 rd max, last}   w22 68000 phase debt, max/frame
  //   w10 $80 writes not 50/E0   w23 MCU instructions, last frame
  //   w11 {$239C writes, low byte}
  // Counts of presses/deposits/writes wrap; per-frame values saturate.
  // =====================================================================
  function automatic [7:0] p12_sat8(input [15:0] v);
    p12_sat8 = (v > 16'd255) ? 8'hFF : v[7:0];
  endfunction

  wire        p12_wr     = ce_en & cpu_wr & cpu_ack & sel_workram & (cpu_ds == 2'b11);
  wire        p12_bwr    = ce_en & cpu_wr & cpu_ack & sel_workram;
  wire        p12_rd     = ce_en & cpu_rd & cpu_ack & sel_workram;
  wire        p12_c70wr  = (mgrant == GR_C70) & mem_we & mem_ack;
  wire        p12_line   = ce_pix & (hpos == 9'd397);          // H_TOTAL - 1
  wire        p12_pend   = cpu_rd | cpu_wr;       // a cycle waiting for its ack
  wire        p12_iack   = (cpu_fc == 3'b111) & ~cpu_as_n;

  reg  [9:0]  p12_bcnt;
  reg         p12_brun;
  reg  [7:0]  p12_blast, p12_bmax;
  reg  [7:0]  p12_setln, p12_clrln, p12_ia3, p12_ia4, p12_c2312ln;
  reg         p12_iack_d;
  reg  [15:0] p12_2310, p12_230c, p12_239c;
  reg  [7:0]  p12_n239c, p12_n2312;
  reg  [19:0] p12_pacc, p12_pmax;
  reg  [15:0] p12_debtmax;
  reg  [4:0]  p12_plen;
  reg  [15:0] p12_cyc, p12_wacc;
  reg  [15:0] p12_vmid;
  reg  [7:0]  p12_fc1, p12_fc1or, p12_fc0, p12_fc0or;
  reg  [7:0]  p12_inp_d, p12_raw1, p12_raw2;
  reg  [7:0]  p12_dep, p12_ndep, p12_mcu1, p12_mcu2, p12_rd0ffe;
  reg  [15:0] p12_dbell, p12_blits, p12_mcuins;
  reg  [7:0]  p12_f60max, p12_dbmax, p12_blmax;
  reg  [15:0] p12_frames;
  reg  [4:0]  p12_win;
  reg  [15:0] p12_w [0:23];

  wire        p12_restart  = (p12_win == 5'd0);
  wire [19:0] p12_pmax_n   = (p12_restart || (p12_pacc > p12_pmax)) ? p12_pacc : p12_pmax;
  wire [7:0]  p12_f60_8    = p12_sat8(f60_frame);
  wire [7:0]  p12_f60max_n = (p12_restart || (p12_f60_8 > p12_f60max)) ? p12_f60_8 : p12_f60max;
  wire [7:0]  p12_db_8     = p12_sat8(p12_dbell);
  wire [7:0]  p12_dbmax_n  = (p12_restart || (p12_db_8 > p12_dbmax)) ? p12_db_8 : p12_dbmax;
  wire [7:0]  p12_bl_8     = p12_sat8(p12_blits);
  wire [7:0]  p12_blmax_n  = (p12_restart || (p12_bl_8 > p12_blmax)) ? p12_bl_8 : p12_blmax;
  wire [7:0]  p12_bcnt_8   = p12_sat8({6'd0, p12_bcnt});

  integer p12_k;
  always @(posedge clk) begin
    if (rst) begin
      p12_brun <= 1'b0;  p12_bcnt <= 10'd0;  p12_blast <= 8'd0;  p12_bmax <= 8'd0;
      p12_setln <= 8'd0; p12_clrln <= 8'd0;  p12_ia3 <= 8'd0;    p12_ia4 <= 8'd0;
      p12_c2312ln <= 8'd0; p12_iack_d <= 1'b0;
      p12_2310 <= 16'd0; p12_230c <= 16'd0;  p12_239c <= 16'd0;
      p12_n239c <= 8'd0; p12_n2312 <= 8'd0;
      p12_pacc <= 20'd0; p12_pmax <= 20'd0;  p12_debtmax <= 16'd0; p12_plen <= 5'd0;
      p12_cyc <= 16'd0;  p12_wacc <= 16'd0;  p12_vmid <= 16'd0;
      p12_fc1 <= 8'd0;   p12_fc1or <= 8'd0;  p12_fc0 <= 8'd0;    p12_fc0or <= 8'd0;
      p12_inp_d <= 8'hFF; p12_raw1 <= 8'd0;  p12_raw2 <= 8'd0;
      p12_dep <= 8'hFF;  p12_ndep <= 8'd0;   p12_mcu1 <= 8'd0;   p12_mcu2 <= 8'd0;
      p12_rd0ffe <= 8'd0;
      p12_dbell <= 16'd0; p12_blits <= 16'd0; p12_mcuins <= 16'd0;
      p12_f60max <= 8'd0; p12_dbmax <= 8'd0; p12_blmax <= 8'd0;
      p12_frames <= 16'd0; p12_win <= 5'd0;
      for (p12_k = 0; p12_k < 24; p12_k = p12_k + 1) p12_w[p12_k] <= 16'd0;
    end else begin
      // --- main-loop work, in lines --------------------------------------
      // The line count comes first so a $230A write in the same clock wins.
      if (p12_line && p12_brun && (p12_bcnt != 10'h3FF)) p12_bcnt <= p12_bcnt + 10'd1;

      if (p12_wr) begin
        case (a24)
          24'h002310: p12_2310 <= cpu_do;
          24'h00230C: p12_230c <= cpu_do;
          24'h00239C: begin p12_239c <= cpu_do; p12_n239c <= p12_n239c + 8'd1; end
          24'h002312: if (cpu_do == 16'd0) begin
                        p12_n2312   <= p12_n2312 + 8'd1;
                        p12_c2312ln <= vpos[8:1];
                      end
          24'h00230A: if (cpu_do != 16'd0) begin
                        // the release; a second one while the main loop is
                        // still busy is a lag frame and keeps the first time
                        if (!p12_brun) begin
                          p12_brun  <= 1'b1;
                          p12_bcnt  <= 10'd0;
                          p12_setln <= vpos[8:1];
                        end
                      end else if (p12_brun) begin
                        p12_brun  <= 1'b0;
                        p12_blast <= p12_bcnt_8;
                        if (p12_bcnt_8 > p12_bmax) p12_bmax <= p12_bcnt_8;
                        p12_clrln <= vpos[8:1];
                      end
          default: ;
        endcase
      end

      // --- how long each 68000 bus cycle waits for its acknowledge --------
      // A real 68000 needs DTACK by S4 to avoid a wait state; fx68k's own
      // comment puts that at about one CPU clock (4 system clocks) after the
      // strobes.  `est. wait states` charges one per 4 clocks beyond that.
      // It is an estimate; w5 (the raw clock count) is the measurement.
      if (ce_en && p12_pend) begin
        if (p12_pacc != 20'hFFFFF) p12_pacc <= p12_pacc + 20'd1;
        if (cpu_ack) begin
          p12_cyc <= p12_cyc + 16'd1;
          if (p12_plen >= 5'd4)
            p12_wacc <= p12_wacc + {11'd0, (p12_plen - 5'd4) >> 2} + 16'd1;
          p12_plen <= 5'd0;
        end else if (p12_plen != 5'd31) begin
          p12_plen <= p12_plen + 5'd1;
        end
      end

      // --- the recovery debt: how far behind its schedule the 68000 got --
      if (cpu_debt > p12_debtmax) p12_debtmax <= cpu_debt;

      // --- which raster line the 68000 takes each interrupt on -----------
      p12_iack_d <= p12_iack;
      if (p12_iack && !p12_iack_d) begin
        if (cpu_a[3:1] == 3'd3) p12_ia3 <= vpos[8:1];
        if (cpu_a[3:1] == 3'd4) p12_ia4 <= vpos[8:1];
      end

      // --- the display window: steps of the collapse, not the two presets -
      if (vreg_wr && (vreg_i == 7'h40) && cpu_ds[0] &&
          (cpu_do != 16'h0050) && (cpu_do != 16'h00E0))
        p12_vmid <= p12_vmid + 16'd1;

      // --- $0FC0/$0FC1, from whichever side writes them -------------------
      if (p12_bwr && (a24[23:1] == 23'h0007E0)) begin
        if (cpu_ds[0]) begin p12_fc1 <= cpu_do[7:0];  p12_fc1or <= p12_fc1or | cpu_do[7:0];  end
        if (cpu_ds[1]) begin p12_fc0 <= cpu_do[15:8]; p12_fc0or <= p12_fc0or | cpu_do[15:8]; end
      end
      if (p12_c70wr && (mem_addr == 23'h0007E0)) begin
        if (mem_ds[0]) begin p12_fc1 <= c70_dout[7:0];  p12_fc1or <= p12_fc1or | c70_dout[7:0];  end
        if (mem_ds[1]) begin p12_fc0 <= c70_dout[15:8]; p12_fc0or <= p12_fc0or | c70_dout[15:8]; end
      end

      // --- the input path: pin, MCU deposit, game read --------------------
      p12_inp_d <= in_p1;
      if (p12_inp_d[4] && !in_p1[4]) p12_raw1 <= p12_raw1 + 8'd1;
      if (p12_inp_d[5] && !in_p1[5]) p12_raw2 <= p12_raw2 + 8'd1;
      // MCU 0x2FFE -> work RAM byte 0x0FFE, the even (high) lane
      if (p12_c70wr && (mem_addr == 23'h0007FF) && mem_ds[1]) begin
        p12_dep  <= c70_dout[15:8];
        p12_ndep <= p12_ndep + 8'd1;
        if (p12_dep[4] && !c70_dout[12]) p12_mcu1 <= p12_mcu1 + 8'd1;
        if (p12_dep[5] && !c70_dout[13]) p12_mcu2 <= p12_mcu2 + 8'd1;
      end
      if (p12_rd && (a24[23:1] == 23'h0007FF)) p12_rd0ffe <= p12_rd0ffe + 8'd1;

      // --- per-frame rates -------------------------------------------------
      if (ce_en && cpu_wr && sel_mailbox && cpu_ack && (cpu_a[3:1] == 3'd4))
        p12_dbell <= p12_dbell + 16'd1;
      if (blit_start) p12_blits  <= p12_blits + 16'd1;
      if (c70_valid)  p12_mcuins <= p12_mcuins + 16'd1;

      // --- publish ---------------------------------------------------------
      if (vblank_rise) begin
        p12_frames <= p12_frames + 16'd1;
        p12_win    <= p12_win + 5'd1;
        p12_pmax   <= p12_pmax_n;
        p12_f60max <= p12_f60max_n;
        p12_dbmax  <= p12_dbmax_n;
        p12_blmax  <= p12_blmax_n;
        if (p12_win == 5'd31) p12_bmax <= 8'd0;

        p12_w[0]  <= p12_2310;
        p12_w[1]  <= p12_230c;
        p12_w[2]  <= {p12_bmax, p12_blast};
        p12_w[3]  <= {p12_setln, p12_clrln};
        p12_w[4]  <= {p12_ia3, p12_ia4};
        p12_w[5]  <= p12_pacc[19:4];
        p12_w[6]  <= p12_pmax_n[19:4];
        p12_w[7]  <= p12_cyc;
        p12_w[8]  <= p12_wacc;
        p12_w[9]  <= {p12_f60max_n, p12_f60_8};
        p12_w[10] <= p12_vmid;
        p12_w[11] <= {p12_n239c, p12_239c[7:0]};
        p12_w[12] <= {p12_fc1or, p12_fc1};
        p12_w[13] <= {p12_raw1, p12_raw2};
        p12_w[14] <= {p12_mcu1, p12_mcu2};
        p12_w[15] <= p12_frames;
        p12_w[16] <= {p12_ndep, p12_rd0ffe};
        p12_w[17] <= f60_frame;
        p12_w[18] <= {p12_c2312ln, p12_n2312};
        p12_w[19] <= {p12_fc0or, p12_fc0};
        p12_w[20] <= {p12_dbmax_n, p12_db_8};
        p12_w[21] <= {p12_blmax_n, p12_bl_8};
        p12_w[22] <= p12_debtmax;
        p12_w[23] <= p12_mcuins;

        p12_pacc   <= 20'd0;
        p12_debtmax <= cpu_debt;
        p12_cyc    <= 16'd0;
        p12_wacc   <= 16'd0;
        p12_dbell  <= 16'd0;
        p12_blits  <= 16'd0;
        p12_mcuins <= 16'd0;
      end
    end
  end

  na2_dbg u_dbg (
      .page_na12 (dbg_page_na12),
      .na12_w    (p12_w),
      .clk       (clk),
      .rst       (rst),
      .cpu_addr  (cpu_a),
      .cpu_fetch (cpu_rd & cpu_ack & (cpu_fc[1:0] == 2'b10)),   // program fetch, supervisor or user
      .m68k_rst  (m68k_rst),
      .unmapped  (sel_unmapped & cpu_ack & (cpu_rd | cpu_wr)),
      .blit_start(blit_start),
      .c70_done  (dbg_c70_done),
      .gfx_enable(dbg_gfx_enable),
      .st_word0  (c70_chan),
      // Session 6: rows 5 and 13 carry the stall diagnostics instead of the
      // memory self-test counters, which have read 0 on every run since RISK-8
      // was resolved.  ack_cnt says whether the main-memory bus is alive at
      // all; stall_snap says who was holding it when the 68000 gave up.
      .st_errs   (ack_cnt),
      .st_dlwords(dbg_st_dlwords),
      .st_dlindex(dbg_st_dlindex),
      .c70_p_src (c70_volA),
      .c70_p_dst (c70_volB),
      .c70_copy_word(c70_dbg_word),
      .st_berrs  (stall_snap),
      .st_video  (st_video),
      .dead_n      (n_dead_wr),
      .dead_bank   (dead_bank),
      .dead_a24    (dead_a24),
      .slip_seen   (blt_slip_seen),
      .slip_n      (n_slip),
      .unmap_a24   (unmap_a24),
      .unmap_wr    (unmap_wr),
      .snd_key_ever  (snd_key_ever),
      .snd_mute_ever (snd_mute_ever),
      .snd_bank_seen (snd_bank_seen),
      .snd_loud_ever (snd_loud_ever),
      .snd_key_now   (snd_key_now),
      .page_snd      (dbg_page_snd),
      .d7_wr         (dbg_d7_wr),
      .d7_rd         (dbg_d7_rd),
      .d7_seen       (dbg_d7_seen),
      .d7_pc         (dbg_d7_pc),
      .d7_pcl        (dbg_d7_pcl),
      .vec           (dbg_vec),
      .vec_pc        (dbg_vec_pc),
      .vec_pl        (dbg_vec_pl),
      .fr0           (dbg_wr0),
      .fr1           (dbg_wr1),
      .fr2           (dbg_wr2),
      .fr3           (dbg_wr3),
      .fr4           (dbg_wr4),
      .fr5           (dbg_wr5),
      .fr_a_hi       (dbg_wa_hi),
      .fr_a_lo       (dbg_wa_lo),
      .d7v_hi        (dbg_d7v_hi),
      .d7v_lo        (dbg_d7v_lo),
      .d7p_hi        (dbg_d7p_hi),
      .d7p_lo        (dbg_d7p_lo),
      .n_enter       (dbg_enter),
      .n_restore     (dbg_restore),
      .snd_oor_ever  (snd_oor_ever),
      .snd_pcm0      (snd_pcm0),
      .snd_pcm1      (snd_pcm1),
      .snd_pcm2      (snd_pcm2),
      .snd_pcm3      (snd_pcm3),
      .snap_addr     (snd_snap_addr),
      .snap_freq     (snd_snap_freq),
      .snap_mv       (snd_snap_mv),
      .vol_n         (snd_vol_n),
      .vol_last      (snd_vol_last),
      .vol_nz        (snd_vol_nz),
      .vol2_nz       (snd_vol2_nz),
      .env_tl        (c70_env_tl),
      .env_ptr       (c70_env_ptr),
      .env_fl        (c70_env_fl),
      .env_act       (c70_env_act),
      .env_visits    (c70_visits),
      .env_fade      (c70_fade),
      .env_list      (c70_chanlist),
      .hpos      (hpos),
      .vpos      (vpos),
      .enable    (dbg_overlay_en),
      .active    (dbg_active),
      .r         (dbg_r),
      .g         (dbg_g),
      .b         (dbg_b)
  );

  // =====================================================================
  // The fault strip is gone (2026-08-23).  It was eight sticky flags drawn over
  // the top-left corner whenever any of them was set, and it sat on the picture
  // permanently once one had ever fired -- `slip_seen` fires in normal play by
  // design, so in practice it was always up.  Everything it carried is on the
  // debug overlay, which has an OSD switch; this did not.
  //
  // If it is ever wanted back, the one thing to keep from it is the comment
  // that used to live here: `fault_bits[i]` IS cell i, and writing
  // `fault_bits[3'd7 - fs_cell]` drew all eight mirrored.  On 2026-08-22 that
  // made the board report "MCU halted / C219 missed a tick / line overrun"
  // when it was actually saying "unmapped access / CG-shape write routed
  // nowhere / blit trigger lost" -- three different investigations.
  // KNOWN_ISSUES.md lesson 10.
  assign vid_r = dbg_active ? dbg_r : bg_r;
  assign vid_g = dbg_active ? dbg_g : bg_g;
  assign vid_b = dbg_active ? dbg_b : bg_b;

  // =====================================================================
  // CRTC
  // =====================================================================

  na2_crtc u_crtc (
      .irq_line_sel(irq_line_sel),
      .clk        (clk),
      .rst        (rst),
      .ce_en      (ce_en),
      .ce_pix     (ce_pix),
      .hpos       (hpos),
      .vpos       (vpos),
      .hblank     (hblank),
      .vblank     (vblank),
      .hsync      (hsync),
      .vsync      (vsync),
      .vblank_rise(vblank_rise)
  );

  // =====================================================================
  // Interrupts
  // MAME_VERIFIED: scanline_interrupt() -- level 4 at vblank, level 3 at the
  // scanline in vreg 0x8A, both gated by ~vreg[0x1A] and by the enable latch
  // that any write to vreg 0x1A sets.  All autovectored.
  // =====================================================================
  reg irq_enabled;
  always @(posedge clk) begin
    if (rst)                                                  irq_enabled <= 1'b0;
    else if (vreg_wr && (vreg_i == (`VREG_IRQ_ENABLE >> 1)))   irq_enabled <= 1'b1;
  end

  wire [15:0] irq_mask = irq_enabled ? ~vreg[`VREG_IRQ_ENABLE >> 1] : 16'h0000;
  // MAME_VERIFIED: scanline_interrupt() takes the raster line as
  // `m_vreg[0x8a / 2] & 0xff` -- EIGHT bits -- and compares it against an
  // unmasked `m_screen->vpos()`.  This compared nine bits, so a game that
  // parked a value with bit 8 set would never take the raster interrupt here
  // and would take it in MAME.  Emeraldia holds 0x00F0 in the HOW TO PLAY
  // screen, where the raster interrupt is what splits the water over its
  // dolphin (docs/emeraldia_dolphin.md), so this game is not affected either
  // way; the mask is here because the reference has it.
  wire        pos_line = (vpos == {1'b0, vreg[`VREG_POSIRQ_LINE >> 1][7:0]});
  reg         pos_line_d;
  always @(posedge clk) pos_line_d <= pos_line;

  reg irq4, irq3;
  always @(posedge clk) begin
    if (rst) begin irq4 <= 1'b0; irq3 <= 1'b0; end
    else if (ce_en) begin
      if (vblank_rise && irq_mask[3])                irq4 <= 1'b1;
      if (pos_line && ~pos_line_d && irq_mask[2])    irq3 <= 1'b1;
      // cleared by the 68000's interrupt acknowledge cycle
      // Clear only the level being acknowledged.  Clearing both threw away a
      // pending raster interrupt whenever a vblank interrupt was taken first,
      // and the stall added above makes that overlap more likely.
      if (cpu_fc == 3'b111) begin
        if (cpu_a[3:1] == 3'd4) irq4 <= 1'b0;
        if (cpu_a[3:1] == 3'd3) irq3 <= 1'b0;
      end
    end
  end

  always @(*) begin
    if      (irq4) ipl_n = ~3'd4;
    else if (irq3) ipl_n = ~3'd3;
    else           ipl_n = 3'b111;
  end

  // =====================================================================
  // Read data and acknowledge back to the 68000
  // The read value is latched on ack so it stays stable while fx68k finishes
  // the bus cycle.
  // =====================================================================
  always @(*) begin
    if      (cpu_mem_sel) cpu_ack = cpu_mem_ack;
    else if (cpu_vid_sel) cpu_ack = cpu_vid_ack;
    // The blitter samples its trigger only when it is idle, and its busy
    // output goes nowhere, so a trigger written while it is running was
    // silently lost.  Measured in MAME: during an event the game re-triggers
    // every 400 system clocks with 16 or 32 words in flight and never polls a
    // status bit -- that allows 12.5 clocks per word, and this blitter needs
    // about 14, so 555 of 1,521 kicks (36.5 %) were dropped.  A dropped shape
    // blit leaves a mask of zeros and a zero mask draws nothing, which is why
    // moving objects vanished while stationary ones were fine: in this game a
    // moving object IS those blits (CG RAM changes by about 1,029 words a
    // frame during an event, sprite RAM barely at all).
    //
    // Holding the acknowledge until the blitter is idle makes the 68000 wait
    // instead, which is what a real board would do with a busy signal wired
    // up.  Costs about 2 % of the CPU's time per frame.
    // docs/glitch_investigation.md
    else if (cpu_fast_sel)cpu_ack = fast_ack & ~(blit_trig_w & blt_busy);
    else                  cpu_ack = cpu_rd | cpu_wr;   // unmapped: never stall
  end

  reg [15:0] rdata_mux;
  always @(*) begin
    if      (cpu_mem_sel) rdata_mux = mem_din;
    else if (cpu_vid_sel) rdata_mux = vram_dout;
    else if (sel_mailbox) rdata_mux = mbx_dout;
    else if (sel_keycus)  rdata_mux = keycus_dout;
    else if (sel_eeprom)  rdata_mux = eeprom_dout;
    else if (sel_vreg)    rdata_mux = vreg[vreg_i];
    else                  rdata_mux = 16'h0000;
  end

  reg [15:0] rd_hold;
  always @(posedge clk) if (cpu_ack && cpu_rd) rd_hold <= rdata_mux;

  // =====================================================================
  // RISK-10: the one stack slot Quiztou's D7 comes from.
  //
  // Measured in MAME (tools/d7_origin.lua): `jsr $c20000.l` at 0xC00028 pushes
  // 0x00C0002E with SP = 0x0007FFFC, and the routine pops it back at
  //
  //     C22636  move.l (sp)+, d7
  //
  // with SP = 0x0007FFF8.  Later, `movea.l d7,a0 ; jmp (a0)` at 0xC2288A goes
  // wherever that value points.  On the board the escape latch says the jump
  // target is 0x000000, so D7 is zero -- and D7's reset value is zero, so
  // either nothing was pushed or the read did not bring it back.
  //
  // This watches that single word.  Two numbers settle it: what the 68000
  // WROTE there, and what it READ back.  Write 0x00C0 and read 0x0000 means
  // memory lost it; no write at all means the push never happened and the
  // fault is further upstream.
  // =====================================================================
  // 23 bits, not 24: this is a word address, cpu_a[23:1].  The value is
  // byte address 0x07FFF8 shifted right one -- written out so the literal
  // is the width of the thing it is compared against.
  localparam [23:1] D7SLOT = 23'h03FFFC;
  wire d7slot_hit = ce_en & cpu_ack & (cpu_a[23:1] == D7SLOT);

  reg [15:0] d7_wr_val, d7_rd_val;
  reg [23:0] d7_rd_pc;
  reg        d7_wr_seen, d7_rd_seen;
  reg  [6:0] d7_wr_n, d7_rd_n;

  // The last program fetch before this cycle -- the instruction the data read
  // belongs to.  na2_dbg keeps its own copy for the once-a-frame sample; this
  // one is here because the latch below needs it at the acknowledge.
  reg [23:0] pc_at_read;
  always @(posedge clk) begin
    if (rst) pc_at_read <= 24'd0;
    else if (ce_en & cpu_rd & cpu_ack & (cpu_fc[1:0] == 2'b10)) pc_at_read <= a24;
  end
  always @(posedge clk) begin
    if (rst) begin
      d7_wr_val <= 16'd0; d7_rd_val <= 16'd0; d7_rd_pc <= 24'd0;
      d7_wr_seen <= 1'b0; d7_rd_seen <= 1'b0;
      d7_wr_n <= 7'd0;    d7_rd_n <= 7'd0;
    end else if (d7slot_hit) begin
      // FIRST, not last.  The first build latched the most recent access and
      // answered nothing: this word is the top of the stack, so it is written
      // and read continuously (27 writes and 31 reads by the second
      // screenshot) and by then the boot values are long gone.  The two that
      // matter are the JSR's push at 0xC00028 -- the first write this address
      // ever takes -- and the `move.l (sp)+,d7` at 0xC22636 that reads it back,
      // the first read.  Both should be 0x00C0, the high half of 0x00C0002E.
      // rdata_mux, NOT rd_hold.  rd_hold is assigned at this same edge by the
      // block above, so reading it here is a non-blocking read of the value
      // from BEFORE this acknowledge -- the previous bus cycle's data.  The
      // first build of this watch did exactly that and reported 0x46FC, which
      // is the opcode word at 0xC22638 that the 68000 prefetched just before
      // the pop: the number a CORRECTLY working CPU produces.  An off-by-one
      // clock in the instrument, dressed up as a memory fault.
      if (cpu_wr && !d7_wr_seen) d7_wr_val <= cpu_do;
      if (cpu_rd && !d7_rd_seen) begin
        d7_rd_val <= rdata_mux;
        d7_rd_pc  <= pc_at_read;      // ... and where the CPU was, to prove
      end                             //     this read is the pop at 0xC22636
      if (cpu_wr) begin
        d7_wr_seen <= 1'b1;
        if (~&d7_wr_n) d7_wr_n <= d7_wr_n + 7'd1;
      end
      if (cpu_rd) begin
        d7_rd_seen <= 1'b1;
        if (~&d7_rd_n) d7_rd_n <= d7_rd_n + 7'd1;
      end
    end
  end

  assign dbg_d7_wr   = d7_wr_val;
  assign dbg_d7_rd   = d7_rd_val;
  assign dbg_d7_seen = {d7_wr_seen, d7_wr_n, d7_rd_seen, d7_rd_n};
  // -------------------------------------------------------------------
  // Which EXCEPTION, if any.
  //
  // Quiztou's vector table (tools/vectors.lua, read out of MAME at frame
  // 1200) installs nothing: vectors 2-15 and the spurious-interrupt vector 24
  // are all 0x00000000, and only autovectors 3 and 4 have handlers.  So any
  // exception this game does not expect jumps to 0x000000 -- which is exactly
  // where the escape latch says the board goes.
  //
  // The 68000 fetches a vector as a SUPERVISOR DATA read (FC = 101) from
  // 0x000000-0x0000FF, so the address names the exception: vector = addr / 4.
  // 8 is a bus error, 0x0C an address error, 0x10 an illegal instruction,
  // 0x60 spurious, 0x64-0x7C the autovectors.
  //
  // Armed only after the pop at 0xC22636 has happened, so the C70's boot-time
  // deposit and the game's own early table writes cannot latch a false answer.
  // -------------------------------------------------------------------
  reg  [7:0]  vec_addr;
  reg [23:0]  vec_pc;
  reg         vec_seen;
  reg  [6:0]  iack_n;
  wire        is_vec_rd = ce_en & cpu_ack & cpu_rd & (cpu_fc == 3'b101)
                          & (a24 < 24'h000100);
  always @(posedge clk) begin
    if (rst) begin
      vec_addr <= 8'd0; vec_pc <= 24'd0; vec_seen <= 1'b0; iack_n <= 7'd0;
    end else begin
      // NOT gated on cpu_ack.  rtl/cpu/na2_m68k.sv builds its cycle signal as
      //     wire cyc = ~cpu_as_n & ~(cpu_lds_n & cpu_uds_n) & ~iack;
      // so cpu_rd and cpu_wr -- and therefore cpu_ack -- are deliberately low
      // for the whole of an interrupt-acknowledge cycle.  A counter that waits
      // for cpu_ack here can never increment: it reads zero whether interrupts
      // are being taken or not.  The first version did exactly that and its
      // zero was read as "no interrupt was involved", which the counter had no
      // way of knowing.  The IRQ clear a few lines up gets this right and is
      // the pattern to copy: fc alone.
      if (ce_en & (cpu_fc == 3'b111) & ~&iack_n) iack_n <= iack_n + 7'd1;
      if (is_vec_rd && d7_rd_seen && !vec_seen) begin
        vec_addr <= a24[7:0];
        vec_pc   <= pc_at_read;
        vec_seen <= 1'b1;
      end
    end
  end

  // -------------------------------------------------------------------
  // The exception stack frame, which carries the address that faulted.
  //
  // A 68000 group-0 exception -- bus error or address error -- pushes seven
  // words before it fetches the vector: a special status word, the 32-bit
  // ACCESS ADDRESS, the instruction register, then SR and PC.  Those pushes
  // are ordinary bus writes, so the address that caused the fault is visible
  // from here even though the register that held it is not.
  //
  // RISK-10 needs exactly that.  The fault is `jmp (a0)` at 0xC2288C with A0
  // odd, and nothing on this side of the CPU can read A0 -- but the frame the
  // CPU pushes contains it.  So: a ring of the last six words written, frozen
  // the moment the vector fetch happens.  Four of them go on rows 20-23,
  // newest first; between them and the 68000's frame layout the faulting
  // address can be read straight off the screen.
  // -------------------------------------------------------------------
  reg [15:0] wr_ring [0:5];
  reg [23:0] wr_addr_l;
  reg        ring_frozen;
  integer    wi;
  always @(posedge clk) begin
    if (rst) begin
      for (wi = 0; wi < 6; wi = wi + 1) wr_ring[wi] <= 16'd0;
      wr_addr_l   <= 24'd0;
      ring_frozen <= 1'b0;
    end else begin
      if (is_vec_rd && d7_rd_seen && !vec_seen) ring_frozen <= 1'b1;
      if (!ring_frozen && ce_en && cpu_ack && cpu_wr) begin
        for (wi = 5; wi > 0; wi = wi - 1) wr_ring[wi] <= wr_ring[wi-1];
        wr_ring[0] <= cpu_do;
        wr_addr_l  <= a24;
      end
    end
  end

  // The ADDRESS of the newest frozen write, and two more of the ring.
  //
  // Four data words alone were ambiguous.  Newest-first they read
  // 0000 4ED6 FFFF 4ED0, and 0x4ED0 is unmistakably `jmp (a0)` -- the
  // instruction register the 68000 stacks -- but that assignment makes the
  // special status word 0x0000, which cannot be right: the SSW encodes R/W,
  // instruction/not and the function code, and none of those are zero for this
  // fault.  So either the frame is laid out differently than assumed or the
  // ring caught something that is not frame.
  //
  // An address settles it.  A group-0 frame goes to consecutive DESCENDING
  // stack addresses, so if the newest write landed on the stack this is the
  // frame and its address is the frame base; if it did not, the four words are
  // not what they looked like.  Two more ring words come along for the ride:
  // under the assumed layout they are SR and the PC's high half, and an SR of
  // roughly 0x2700 with a PC high of 0x00C2 would confirm the whole reading in
  // one glance.
  // -------------------------------------------------------------------
  // Where D7 is written, and to what.
  //
  // The board's exception frame says A0 = 0x0000FFFF at `jmp (a0)`, and A0 is
  // `movea.l d7,a0` one instruction earlier.  MAME writes D7 exactly twice in
  // that whole block -- 0x00000000 at 0xC22600 and 0x00C0002E at the pop at
  // 0xC2263A -- and never again before the jump (tools/d7_track.lua).  So our
  // core writes it somewhere MAME does not, and nothing outside the CPU can
  // see a register.  fx68k now brings D7 out; this keeps the last two changes
  // with the program counter at each.
  // -------------------------------------------------------------------
  // d7_v1 / d7_pc1 removed 2026-08-27: the second stage of this shift was
  // written but never read -- only d7_v0 reaches the overlay (dbg_d7v_*).
  // The comment above promises "the last two changes"; only the last one is
  // actually wired.  Restore from git if the second row is ever added.
  reg [31:0] d7_prev, d7_v0;
  reg [23:0] d7_pc0;
  reg  [7:0] d7_n;
  always @(posedge clk) begin
    if (rst) begin
      d7_prev <= 32'd0; d7_v0 <= 32'd0;
      d7_pc0 <= 24'd0;  d7_n <= 8'd0;
    end else if (ce_en && (cpu_d7 !== d7_prev)) begin
      d7_prev <= cpu_d7;
      d7_v0   <= cpu_d7;  d7_pc0 <= pc_at_read;
      if (~&d7_n) d7_n <= d7_n + 8'd1;
    end
  end

  assign dbg_d7v_hi = d7_v0[31:16];
  assign dbg_d7v_lo = d7_v0[15:0];
  assign dbg_d7p_hi = {d7_n, d7_pc0[23:16]};
  assign dbg_d7p_lo = d7_pc0[15:0];

  // -------------------------------------------------------------------
  // Does the board reach the restore?
  //
  // The routine that destroys D7 is wrapped:
  //     C20728  movem.l d0-d7/a0-a6, -(sp)
  //     C20742  bsr.w   C207EC          <- the EEPROM read, D7 destroyed
  //     C2074A  movem.l (sp)+, d0-d7/a0-a6
  // MAME runs the pair and D7 comes back.  Our D7 ends destroyed, and the
  // movem itself is proven correct in simulation, so the question is whether
  // 0xC2074A is ever fetched -- and how often 0xC20728 is, which separates
  // "stopped before the restore" from "went round again".
  // -------------------------------------------------------------------
  reg [15:0] n_enter, n_restore;
  always @(posedge clk) begin
    if (rst) begin
      n_enter <= 16'd0; n_restore <= 16'd0;
    end else if (ce_en & cpu_rd & cpu_ack & (cpu_fc[1:0] == 2'b10)) begin
      if (a24 == 24'hC20728 && ~&n_enter)   n_enter   <= n_enter   + 16'd1;
      if (a24 == 24'hC2074A && ~&n_restore) n_restore <= n_restore + 16'd1;
    end
  end
  assign dbg_enter   = n_enter;
  assign dbg_restore = n_restore;

  assign dbg_wa_hi = {8'd0, wr_addr_l[23:16]};
  assign dbg_wa_lo = wr_addr_l[15:0];
  assign dbg_wr4   = wr_ring[4];
  assign dbg_wr5   = wr_ring[5];

  assign dbg_wr0 = wr_ring[0];
  assign dbg_wr1 = wr_ring[1];
  assign dbg_wr2 = wr_ring[2];
  assign dbg_wr3 = wr_ring[3];

  assign dbg_vec    = {vec_seen, iack_n, vec_addr};
  assign dbg_vec_pc = vec_pc[23:8];
  assign dbg_vec_pl = {8'd0, vec_pc[7:0]};

  assign dbg_d7_pc   = d7_rd_pc[23:8];
  assign dbg_d7_pcl  = {8'd0, d7_rd_pc[7:0]};
  always @(*) cpu_di = rd_hold;

  // =====================================================================
  // Trace taps
  // =====================================================================
  assign dbg_addr     = cpu_a;
  assign dbg_as_n     = cpu_as_n;
  assign dbg_rd       = cpu_rd;
  assign dbg_wr       = cpu_wr;
  assign dbg_dout     = cpu_do;
  assign dbg_din      = cpu_di;
  assign dbg_rdata    = rdata_mux;
  assign dbg_fc       = cpu_fc;
  assign dbg_ack      = cpu_ack;
  assign dbg_m68k_rst = m68k_rst;

  // Compact handshake snapshot for the stall diagnostics in the testbench.
  //  [1:0] main-memory grant   [3:2] video grant     [8:4] C70 state
  //  [9]   blitter busy        [10]  68000 ASn       [11] mem_req
  //  [12]  mem_ack             [13]  cpu_ack         [15:14] IPLn low bits
  // NOTE: this concatenation is 17 bits wide against a 16-bit port, so the MSB
  // -- ipl_n[1] -- is dropped and every field above mgrant sits one place lower
  // than the comment above says.  mgrant is 3 bits (GR_CPU = 3'd4), not the 2
  // the comment assumes, which is where the extra bit comes from.
  // Written out explicitly here so the width is intentional rather than a
  // silent truncation; the bit placement is UNCHANGED from before.
  // Consequence, recorded not fixed: tb_na2_boot.sv reads dbg_bus[1:0] as the
  // grant, which cannot tell GR_CPU (3'b100) from GR_NONE.  Diagnostic only --
  // dbg_bus reaches no game logic.  See docs/DEBUG_LOG.md.
  assign dbg_bus = {ipl_n[0], cpu_ack, mem_ack, mem_req, cpu_as_n,
                    blt_busy, u_c70_st, {(vgrant == GR_BLT), (vgrant == GR_CPU)}, mgrant};

  // synthesis translate_off
  // Report unmapped accesses, but cap the noise: Numan Athletics deliberately
  // scans past the end of work RAM while sizing it, which would otherwise
  // drown the log.  The count and address range are reported by the testbench.
  integer unmapped_n = 0;
  always @(posedge clk) if (ce_en && cpu_ack && (cpu_rd | cpu_wr) && sel_unmapped) begin
      if (unmapped_n < 16)
          $display("[%0t] NA2 UNMAPPED %s @ %06h ds=%b", $time,
                   cpu_wr ? "WR" : "RD", a24, cpu_ds);
      else if (unmapped_n == 16)
          $display("[%0t] NA2 UNMAPPED: further reports suppressed", $time);
      unmapped_n = unmapped_n + 1;
  end
  // synthesis translate_on

endmodule

`default_nettype wire
