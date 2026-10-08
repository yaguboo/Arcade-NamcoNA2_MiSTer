//============================================================================
//  Namco NA-2 (Numan Athletics) for MiSTer -- board wrapper
//
//  Wires the board-independent core in rtl/na2_top.sv to the MiSTer framework:
//  clocks, HPS I/O, ROM download, SDRAM and video.
//
//  STATE OF PLAY: the scanline sequencer and the four tilemap layers are in and
//  verified in simulation against a golden model; ROZ, sprites and sound are
//  not.  On real hardware the core loads, the ROMs download and the 68000 runs,
//  but it does not stay on the rails -- the debug overlay reports the
//  unmapped-access counter saturated, and the same is true of the build before
//  the renderer existed.  Simulation runs this RTL correctly against a
//  behavioural memory, so the suspect is SDRAM.  Do not read the colours on
//  screen as game state until that is closed out.
//  See ROADMAP.md Level 4, STATUS.md, and tools/read_overlay.py.
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 3 of the License, or (at your option)
//  any later version.
//============================================================================

module emu
(
	`include "sys/emu_ports.vh"
);

///////// Ports this core does not use /////////

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
// DDRAM is driven by screen_rotate, in the VIDEO section below.

// The framebuffer is only ever the rotator's; nothing else here blanks it.
assign FB_FORCE_BLANK = 0;

assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

wire signed [15:0] aud_l, aud_r;

// The C219 is wired up and checked against MAME's own audio, but nothing
// drives its registers yet: the sound driver runs on the C70, and na2_c70_hle
// does not execute c70.bin.  So this is real silence, not a stub.
assign AUDIO_S   = 1;          // signed samples
assign AUDIO_L   = aud_l;
assign AUDIO_R   = aud_r;
assign AUDIO_MIX = 0;

assign LED_DISK  = 0;
assign LED_POWER = 0;
// BUTTONS is tied off in the DIAGNOSTIC section near the bottom.

//////////////////////////////////////////////////////////////////

wire [1:0] ar = status[122:121];

// 304 x 224 visible.  MAME_VERIFIED: set_visarea(0, 38*8-1, 4*8, 32*8-1);
// FBNEO_CROSSCHECK: the FBNeo driver declares 304x224 with a 4:3 aspect.
// Aspect, when the Aspect ratio option is left on Original.
//
//   not rotated   4:3, which is what this board drove and what both emulators
//                 declare for its 304x224.
//   Rotate On     the picture's OWN ratio, 224 wide by 304 tall, so the pixels
//                 stay square and nothing is stretched to reach a number.
//   Rotate On     3:4 -- the 4:3 cabinet stood on end.  Offered because it is
//     (4:3)       what a real rotated monitor shows, and it is a matter of
//                 taste which of the two a player wants.
//
// Forcing 3:4 was the first attempt and it is the wrong default: it takes a
// 224x304 picture and stretches it to fill a shape it does not have.
// These are the aspect of the picture AS DISPLAYED, so when the picture has
// been turned they are TALL.  MiSTer does not swap them for us: `video_rotated`
// tells the HPS the picture is rotated, and the framebuffer screen_rotate
// hands over is already 224 wide by 304 high, but the ratio the scaler uses is
// whatever the core puts here.
//
//   Rotate Off        4:3, what this board drove and what both emulators
//                     declare for its 304x224.
//   Rotate On         224:304 -- the turned picture's own ratio, so the pixels
//                     stay square and nothing is stretched.
//   Rotate On (4:3)   3:4, the 4:3 cabinet stood on end.
//
// This was briefly "corrected" to the unrotated numbers on 2026-08-26 and that
// was wrong.  The report it was based on -- "both settings look 4:3" -- came
// from a board whose .mra still had three bytes because a deploy had been cut
// short, so board_vertical was 0 and Rotate could not engage at all.  A 4:3
// picture with the feature switched off was read as a 4:3 picture with the
// feature working.  Measure the thing you think you are measuring.
assign VIDEO_ARX = (!ar) ? (no_rotate ? 12'd4 : (rot_43 ? 12'd3 : 12'd224))
                         : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? (no_rotate ? 12'd3 : (rot_43 ? 12'd4 : 12'd304))
                         : 12'd0;

`include "build_id.v"
localparam CONF_STR = {
	"NamcoNA2;;",
	"-;",
	"O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"O[4:2],Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
	// Two of the thirteen NA-1 games are ROT90 in MAME -- Fighter & Attacker
	// and F/A -- because their cabinets are on their side.  The core draws the
	// raster the board draws, unrotated, which is what a real NA-1 puts out
	// whichever way the monitor is bolted in; this turns the PICTURE for a
	// player whose monitor is not.
	//
	// H0 hides both items on a game that is not vertical, which is eleven of
	// the thirteen NA-1 sets and every NA-2 set.  Whether a set is vertical is
	// not compiled in: it arrives in the .mra's ioctl_index 1 descriptor, next
	// to the KEYCUS identity, for the same reason -- it is a property of the
	// board being described, not of this core.  On = the cabinet's own
	// orientation, i.e. vertical; Off = the raster as the board emits it.
	"H0O[16:15],Rotate,Off,On,On (4:3);",
	"H0O[17],Flip,Off,On;",
	"-;",
	// *** Default OFF, 2026-08-23. ***
	// It defaulted ON while the core was being brought up, because the overlay
	// was how a session got read at all and MiSTer clears every status bit on a
	// fresh core load -- so the default IS what a session starts with.
	//
	// That stopped being right the moment a second game loaded.  Each .mra gets
	// its own settings, so Emeraldia's first boot on hardware came up with the
	// overlay painting over the picture, and the screenshot showed the overlay
	// rather than the game.  A debug aid that hides the thing being debugged is
	// worse than no default at all.
	//
	// The option order is normal now (Off first), so status[10]=0 means Off and
	// dbg_overlay_en takes status[10] directly rather than inverting it.
	"H1P1O[10],Debug overlay,Off,On;",
	// Rows 24-31 of the overlay carry two different instrument sets and
	// this chooses which.  It exists because swapping them used to mean
	// editing rtl/video/na2_dbg.sv and waiting for the fitter.
	"H1P1O[18],Overlay rows 24-31,Boot / D7,C219 sound;",
	// DIAGNOSTIC, branch fix/na12-accuracy.  Bit 20 because 10-19 are taken
	// and .mra files store OSD defaults by bit number (OSD_POLICY 4).
	"H1P1O[20],Overlay rows 8-31,Normal,NA12 timing probe;",
	// fix/na12-accuracy.  status[21]==0 is the default and means Recover: the
	// 68000 takes no wait states for this core's memory latency (na2_m68k.sv).
	// Legacy is the wrapper as it was, kept so the two can be compared on a
	// board without a rebuild.
	"H1P1O[21],68000 bus waits,Recover,Legacy;",
	// fix/na12-accuracy.  status[22]==0 is the default and means Hold: a
	// palette write while graphics are off leaves the backdrop colour alone
	// until graphics return (MAME namcona1_v.cpp:131-143).  Live is FBNeo's
	// behaviour and this core's before; kept for an A/B on the board.
	"H1P1O[22],Palette while gfx off,Hold (MAME),Live (FBNeo);",
	"H1P1O[12:11],Vblank IRQ line,256 (end of picture),224 (MAME),240 (FBNeo);",
	"H1P1O[13],Sprite bank,Live,Latch per frame;",
	// *** OFF by default again, 2026-08-23 evening. ***
	// This was the workaround that carried the melody while the cause was
	// unknown.  The cause is now fixed -- the shared window's 16-bit byte order
	// depends on the ALIGNMENT of the access, and an odd-aligned one follows the
	// byte rule (rtl/mcu/na2_c70.sv, docs/bgm_investigation.md section 23) -- so
	// the attenuation is correct on its own and must not be overridden: the game
	// really does fade, and with this on it would not.
	// Kept only so the two can still be compared by ear.
	// The C70 computes the melody's volume correctly (0x43, MAME's own value)
	// and then loses it five instructions later:
	//     00E25A  SBCB $05,X   ; volume - esram[0x4005], the fade attenuation
	//     00E260  LDB #$00     ;   borrowed -> silence
	// MAME's attenuation is 0x00 and it never enters the fade path in any of
	// five captures; ours reads 0x50, which zeroes every volume under it.
	// Turning this on makes that one byte read as zero, so the chain can be
	// confirmed BY EAR in the same build that measures it.  It is a probe:
	// the real repair is whatever asks for the fade.
	// docs/bgm_investigation.md section 18.
	"H1P1O[14],BGM fade attenuation,Normal,Ignore (old workaround);",
	"-;",
	"DIP;",
	// The four P1 items above are probes, not settings a player wants, so they
	// sit on a "Debug" submenu at the BOTTOM of the top level and the first
	// screen stays Aspect ratio, Scandoubler Fx and DIP.  The status bits are
	// untouched -- this is where the items are SHOWN, not what they mean -- so
	// a saved config keeps working.  Asked for 2026-08-26, once ISSUE-1 was
	// settled and Vblank IRQ line stopped being a question anyone must answer.
	"H1P1,Debug;",
	"-;",
	// --- 표준 OSD, docs/OSD_POLICY.md -------------------------------------
	// `On` 을 먼저 쓴다.  MiSTer 는 코어 로드 때 status 를 전부 0 으로 지우므로
	// status[19]==0 이 곧 기본값이고, 이 순서라야 그 기본이 On 이다.
	// 비트 19 인 이유는 10~18 이 이미 쓰이고 있어서다 -- .mra 가 OSD 기본값을
	// 비트 번호로 저장하므로 기존 비트를 옮기지 않는다 (OSD_POLICY 4절).
	"O[19],Pause when OSD open,On,Off;",
	"-;",
	"T[0],Reset;",
	"R[0],Reset and close OSD;",
	// Pause 가 J1 에 없었다.  .mra 는 처음부터
	//   names="...,Start,Coin,Pause" default="A,B,X,Start,Select,R"
	// 로 Pause 를 선언하고 있었는데 여기 없어서 **어느 패드로도 닿을 수
	// 없었다** -- .mra 와 J1 은 순서로 매핑되므로 조용히 어긋난다.
	"J1,Button 1,Button 2,Button 3,Start,Coin,Pause;",
	"jn,A,B,X,Start,Select,R;",
	"V,v",`BUILD_DATE
};

wire        forced_scandoubler;
wire  [1:0] buttons;
wire [127:0] status;
wire [10:0] ps2_key;

wire        ioctl_download;
wire        ioctl_upload;

// --- 28C16 nvram, saved and restored through the HPS (ioctl index 2) -------
// docs/nvram_save.md.  na2_eeprom_nvport speaks the ioctl side; the array
// itself is inside na2_top, so its second port is brought out there.
wire        nv_req, nv_we, nv_gnt, nv_upload_req, nv_game_we;
wire [10:0] nv_addr;
wire  [7:0] nv_din, nv_dout, nv_ioctl_din;
wire  [7:0] nv_upload_index;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire [15:0] ioctl_index;

wire [31:0] joystick_0, joystick_1, joystick_2, joystick_3;
wire [21:0] gamma_bus;
wire        ioctl_wait;
wire  [7:0] dsw_hps;
wire [15:0] st_word0, st_errs, st_berrs, st_dlwords;

// Which ioctl_index the HPS actually used for the ROM.  na2_membus only
// accepts index 0, so if the ROM never lands this is the first thing to look
// at -- and it is not visible from anywhere else.
// bit 15 records whether the core-reset signals were asserted at any point
// while the HPS was downloading -- the reset fault of session 3, caught in the
// act.
//
// Only the *first* transfer counts.  The MRA's <switches> block arrives
// afterwards as its own download at index 254, and latching every transfer
// made the overlay report "download index = 254, core reset was asserted"
// on a run where the ROM had in fact landed perfectly -- a diagnostic that
// cries wolf is worse than no diagnostic.        HW_CONFIRMED (session 4)
reg  [7:0] st_dlidx_lo = 8'hFF;
reg        st_dlrst    = 1'b0;
wire [15:0] st_dlindex = {st_dlrst, 7'd0, st_dlidx_lo};

// Board feature flags and the debug unlock.  Declared here because
// status_menumask below reads them; what DRIVES them lives further down with
// the rest of the .mra descriptor decoding, next to the KEYCUS it arrives with.
reg  [7:0] board_flags = 8'h00;
reg        osd_unlock  = 1'b0;
// What the OSD options should come up as for this set, from the .mra.
reg [23:0] mra_status      = 24'd0;
reg        mra_status_seen = 1'b0;
wire       board_vertical = board_flags[0];
wire       debug_menu_en  = board_flags[2] | osd_unlock;

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	// H0 hides the vertical-only video items, H1 the whole Debug page.
	// A mask BIT SET means "hide", so each is the negation of the thing that
	// should show it.  docs/na1_verification.md section 3.
	.status_menumask({14'd0, ~debug_menu_en, ~board_vertical}),

	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_index(ioctl_index),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.joystick_2(joystick_2),
	.joystick_3(joystick_3),

	.ioctl_wait(ioctl_wait),

	.ps2_key(ps2_key),

	// hps_io declares these inputs without a default value, so leaving them
	// open makes them float.  ioctl_wait is the one that actually bites --
	// it goes straight onto HPS_BUS[37] as "core not ready", and unconnected
	// it hung the HPS partway through the ROM load with the log stopped at
	// "Identified 8BIT core / Found SDRAM config: 3".  The rest are tied off
	// explicitly so the same class of bug cannot come back.
	.joystick_0_rumble(16'd0),
	.joystick_1_rumble(16'd0),
	.joystick_2_rumble(16'd0),
	.joystick_3_rumble(16'd0),
	.joystick_4_rumble(16'd0),
	.joystick_5_rumble(16'd0),
	.ps2_kbd_clk_in(1'b0),
	.ps2_kbd_data_in(1'b0),
	.ps2_kbd_led_status(3'd0),
	.ps2_kbd_led_use(3'd0),
	.ps2_mouse_clk_in(1'b0),
	.ps2_mouse_data_in(1'b0),
	.video_rotated(video_rotated),
	.new_vmode(1'b0),
	.status_in({104'd0, mra_status[23:1], 1'b0}),
	.status_set(mra_status_set),
	.info_req(1'b0),
	.info(8'd0),
	.sd_lba('{default:32'd0}),
	.sd_blk_cnt('{default:6'd0}),
	.sd_rd(1'b0),
	.sd_wr(1'b0),
	.sd_buff_din('{default:8'd0}),
	// The 28C16 save path.  na2_eeprom_nvport, below, drives these four and
	// reads ioctl_wr/ioctl_addr/ioctl_dout/ioctl_index; the array it talks to
	// is na2_top.u_eeprom, whose second port comes back out through na2_top's
	// nv_* ports.  ioctl index 2, 2048 bytes -- docs/nvram_save.md.
	.ioctl_upload(ioctl_upload),
	.ioctl_upload_req(nv_upload_req),
	.ioctl_upload_index(nv_upload_index),
	.ioctl_din(nv_ioctl_din)
);

///////////////////////   CLOCKS   ///////////////////////////////
//
// The NA-2 board runs from a single 50.113 MHz oscillator (HW_CONFIRMED from
// the PCB notes).  outclk_1 is the same frequency, phase shifted for SDRAM_CLK.
//
// 50.113 MHz is not reachable from a 50 MHz reference with an *integer* PLL,
// and the first attempt to get close with one is the single worst bug this
// project has had.  It asked for 50.161290 MHz (N=10 M=311 C=31, +0.096 %),
// Quartus accepted the parameters, synthesis passed, the fitter passed, and
// TimeQuest reported a clean 19.933 ns core clock -- because TimeQuest takes
// the frequency from the megafunction's parameter *string*.  What the fitter
// had actually programmed was M=2 N=2 C=31: a 50 MHz VCO against a 600 MHz
// minimum, so the PLL could not lock and clk_sys never ran on hardware.
//
// Nothing downstream can survive that.  sys_top.v generates the HPS handshake
// acknowledge in the clk_sys domain, so MiSTer's main spun forever on its very
// first word to the core and the whole machine hung -- which is exactly what
// happened, and it cost two sessions of looking at the wrong things.
//
// So this is a fractional PLL now, like every PLL in sys/ (pll_hdmi and
// pll_audio are both fractional and both work on this board), and it asks for
// the real 50.113 MHz rather than an approximation.  outclk_1 is the same
// frequency shifted half a period for SDRAM_CLK.
//
// tools/check_pll.py reads the Fitter's PLL Usage Summary -- the counters as
// programmed, not as requested -- and ./build.sh fails if the VCO is out of
// range or the output is off frequency.  Do not remove that check.

wire clk_sys, clk_sdram, pll_locked;

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.outclk_1(clk_sdram),
	.locked(pll_locked)
);

assign SDRAM_CLK = clk_sdram;

// The core is held in reset while ROMs load.  The memory bus must NOT be:
// it is the thing doing the loading.  Holding it reset kept dl_req at 0, so
// not one byte of the 10.5 MB would ever have reached SDRAM.
wire rst_sys  = RESET | status[0] | buttons[1] | ~pll_locked;

// The memory bus gets a *different* reset, and this matters more than it looks.
//
// The HPS holds the core in reset while it loads the ROMs -- that is what RESET
// and status[0] are for -- and na2_membus is the thing doing the loading.  Held
// in reset it never sets dl_req, so not one byte reaches SDRAM, ioctl_wait never
// rises, the HPS finishes happily, and the 68000 comes out of reset to execute
// zeros.  Which is exactly what happened: the on-screen self-test read SDRAM
// word 0 as 0x0000 instead of 0x46FC, with the download word counter at 0 and
// the work-RAM read-back test passing 256/256.
//
// Session 2 fixed half of this by taking ioctl_download out of the memory bus's
// reset.  This is the other half.  The memory bus is only ever reset by the PLL
// coming up.
wire rst_mem  = ~pll_locked;

// The core also stays in reset while na2_membus reads the SDRAM back and checks
// it -- see the self-test there; it needs the memory to itself.
wire st_busy;
wire reset    = rst_sys | ioctl_download | st_busy;

///////////////////////   INPUTS   ///////////////////////////////
//
// Bit order per player port, active low:
//   0 right  1 left  2 down  3 up  4 button1  5 button2  6 button3  7 start
// MAME_VERIFIED: INPUT_PORTS_START(namcona1_joy).
//
// MiSTer joystick_N bit order is  0 right 1 left 2 down 3 up 4.. buttons,
// which already matches bits 0-3; buttons and start are placed to match the
// PORT_BIT assignments above rather than MiSTer's default order.

function [7:0] na2_port(input [31:0] j, input start_bit);
	na2_port = ~{ start_bit, j[6], j[5], j[4], j[3], j[2], j[1], j[0] };
endfunction

// --- Pause -----------------------------------------------------------------
// 이 보드에는 pause 선이 없다.  코어 편의 기능이므로 na2_top 의 ce_en 을
// 게이트한다 -- 68000·MCU·사운드가 서 있고 비디오는 계속 스캔한다.
// docs/OSD_POLICY.md 2절.
//
// ioctl_download 에서 반드시 푼다.  ROM 로드 내내 CPU 는 리셋에 잡혀 있고,
// 로드 뒤 pause 가 남아 있으면 화면상 행과 구별되지 않는다.
wire pause_btn = joystick_0[9] | joystick_1[9];   // J1 7번째 = Pause = bit 9
reg  pause_btn_d, pause_latch;
always @(posedge clk_sys) begin
	pause_btn_d <= pause_btn;
	if (ioctl_download)                  pause_latch <= 1'b0;
	else if (~pause_btn_d & pause_btn)   pause_latch <= ~pause_latch;
end
wire pause_core = pause_latch | (OSD_STATUS & ~status[19]);

wire [7:0] in_p1 = na2_port(joystick_0, joystick_0[7]);
wire [7:0] in_p2 = na2_port(joystick_1, joystick_1[7]);
wire [7:0] in_p4 = na2_port(joystick_3, joystick_3[7]);
// Player 3 does NOT go through the digital mux: its eight bits arrive one
// per A-D input, scrambled, and rtl/mcu/na2_m37702_periph.sv unscrambles
// them.  Until 2026-08-26 the wrapper had no port for it and the analog
// inputs were tied idle, so player 3 pressed nothing on a 4-player game.
wire [7:0] in_p3 = na2_port(joystick_2, joystick_2[7]);

// DSW port, active low:
//   0 freeze  1 test  2..5 coin4..coin1  6 service  7 service1
//
// The DIP switches come from the MRA <switches> block; coins come from the
// joystick "Coin" buttons.  MAME 0.289 exposes three switches on this port and
// so do we now -- Freeze on bit 0, Test on bit 1, Service Mode on bit 6, all
// defaulting Off, identical across all 21 sets (`mame -listxml`).
//
// Bit 6 was tied inactive here until 2026-08-31.  Driving it from the MRA
// cannot change how anything boots, because bit 2 of dsw_hps reads 1 on every
// path into this wire: dsw_reg initialises to 0xFF below, so it is 1 before
// the HPS transfers anything, and every .mra in this family -- old ones that
// declare no Service switch included -- says default="FF", so it is still 1
// after.  ~1 inverted back is 1, and 1 on this active-low port means the
// switch is off.  The behaviour is new only when an operator turns it on.
wire coin1 = joystick_0[8];
wire coin2 = joystick_1[8];
wire [7:0] in_dsw = ~{ 1'b0, ~dsw_hps[2], coin1, coin2, 2'b00, ~dsw_hps[1], ~dsw_hps[0] };

// Which KEYCUS chip is on this game's ROM board.  The .mra sends three bytes
// at ioctl_index 1 -- identity high, identity low, word offset -- because the
// key chip is part of the board being described, exactly like the ROMs are.
// tools/build_rom.py writes them from the table in tools/verify_roms.py.
//
// The default is C359 at word offset 1, so a Numan Athletics .mra written
// before this existed still runs: it simply sends no index 1 and gets the
// chip it always got.  A set that needs a different chip and is loaded from
// such an .mra fails its own protection check rather than running wrong,
// which is the failure that is easy to recognise.
//
// A FOURTH byte was added on 2026-08-26: board feature flags.  The decoder
// below always accepted four bytes and byte 3 fell through `default`, so an
// .mra written before this exists sends three and the flags stay at 0 -- which
// is "not vertical, no debug menu", the right answer for every set that was
// shipping until now.
//
//   bit 0  this cabinet is VERTICAL -- offer Rotate/Flip in the OSD
//   bit 1  ... and turn it counter-clockwise rather than clockwise
//   bit 2  show the Debug page
//
// Why the .mra and not a parameter: which way a cabinet stands is a property
// of the board being described, exactly like the KEYCUS and the ROMs, and
// putting it here means a set that needs it costs an .mra edit rather than a
// twenty-minute fitter run.  The same goes for the debug menu: turning the
// instruments on used to mean rebuilding.
reg [15:0] keycus_id  = 16'h0167;
reg  [2:0] keycus_off = 3'd1;
always @(posedge clk_sys) begin
	// !ioctl_addr[24:3] so only the first EIGHT bytes can land here.  The mask
	// has to match the block length exactly: a longer block would otherwise
	// alias byte 8 onto byte 0 and rewrite the identity with whatever followed
	// it.  The DIP switch decoder below had exactly that bug (it matched
	// addresses 0 and 1 both) and it cost a session.
	if (ioctl_wr && (ioctl_index[7:0] == 8'd1) && !ioctl_addr[24:3]) begin
		case (ioctl_addr[2:0])
			3'd0: keycus_id[15:8]  <= ioctl_dout[7:0];
			3'd1: keycus_id[7:0]   <= ioctl_dout[7:0];
			3'd2: keycus_off       <= ioctl_dout[2:0];
			3'd3: board_flags      <= ioctl_dout[7:0];
			// Bytes 4-6: what the OSD options should COME UP as for this set.
			// Sent to MiSTer through status_in/status_set below, so they are
			// starting values and not locks -- the player can still change
			// anything afterwards.
			3'd4: mra_status[23:16] <= ioctl_dout[7:0];
			3'd5: mra_status[15:8]  <= ioctl_dout[7:0];
			3'd6: begin mra_status[7:0] <= ioctl_dout[7:0]; mra_status_seen <= 1'b1; end
			default: ;
		endcase
	end
end

// Hand the .mra's option defaults to MiSTer once, when the block that carried
// them has finished arriving.
//
// hps_io latches status_in on the rising edge of status_set and sends it up as
// the status word, so this SEEDS the OSD rather than overriding it: whatever
// the player selects afterwards wins, and MiSTer clears status on a fresh
// arcade load anyway, which is exactly the moment worth seeding.
//
// Bit 0 is Reset and is masked off unconditionally.  Setting it here would
// hold the core in reset from the moment the .mra finished loading, which is
// a very confusing way to discover a typo in an .mra.
reg        ioctl_dl_d      = 1'b0;
reg        mra_status_done = 1'b0;
reg        mra_status_set  = 1'b0;
always @(posedge clk_sys) begin
	ioctl_dl_d     <= ioctl_download;
	mra_status_set <= 1'b0;
	if (ioctl_dl_d && !ioctl_download && mra_status_seen && !mra_status_done) begin
		// Only when NO saved settings were loaded.  MiSTer loads <setname>.CFG
		// into status BEFORE the ROM download, and status_set makes Main replace
		// all 128 bits with status_in (Main_MiSTer user_io.cpp
		// check_status_change) -- so pushing the .mra defaults unconditionally
		// overwrote every saved OSD setting on every load ("settings do not
		// save", reported 2026-10-07).  A saved file has some bit of [127:1]
		// set; a fresh load has none.  [0] is the reset bit Main pulses.
		mra_status_set  <= ~|status[127:1];
		mra_status_done <= 1'b1;
	end
end

// The Debug page: shown when the .mra asks for it, and also when the player
// opens the OSD twice in quick succession.
//
// The second route is the one that matters day to day.  Every instrument in
// this core -- the overlay, the vblank line, the sprite bank -- used to be one
// menu item away, which is fine until the menu is the thing being kept clean;
// and moving them behind a rebuild is how an afternoon gets spent fitting a
// bitstream to read one counter.  Two OSD presses inside two seconds is a
// gesture nobody makes by accident and everybody can remember.  It is sticky
// until the core is reloaded, which is also how it gets turned off.
localparam int OSD_DBL_CLKS = 100_000_000;      // ~2 s at 50 MHz
reg         osd_prev   = 1'b0;
reg  [26:0] osd_gap    = 27'd0;
reg         osd_armed  = 1'b0;
always @(posedge clk_sys) begin
	osd_prev <= OSD_STATUS;
	if (osd_gap != 27'd0) osd_gap <= osd_gap - 27'd1;
	else                  osd_armed <= 1'b0;
	if (OSD_STATUS && !osd_prev) begin           // the OSD just opened
		if (osd_armed) osd_unlock <= 1'b1;
		else begin
			osd_armed <= 1'b1;
			osd_gap   <= OSD_DBL_CLKS[26:0];
		end
	end
	if (rst_sys) begin osd_unlock <= 1'b0; osd_armed <= 1'b0; osd_gap <= 27'd0; end
end

reg dl_was = 1'b0;      // a download has been seen
reg dl_over = 1'b0;     // ... and it has finished, so later ones are not the ROM
always @(posedge clk_sys) begin
	if (ioctl_download) begin
		dl_was <= 1'b1;
		if (!dl_over) begin
			st_dlidx_lo <= ioctl_index[7:0];
			if (rst_sys) st_dlrst <= 1'b1;
		end
	end else if (dl_was) dl_over <= 1'b1;
end

// DIP switches arrive over ioctl_index 254 (the MRA <switches> mechanism).
//
// The HPS sends EIGHT bytes here, not one.  The MRA declares a single DIP
// byte -- <switches default="FF" base="0"> -- but main transfers the whole
// 64-bit switch word, so byte 0 is 0xFF and bytes 1-7 are 0x00.  Every MiSTer
// arcade core in .vendor/ decodes it the same way and sizes the array to
// match: `reg [7:0] sw[8]` with `!ioctl_addr[24:3]` and `sw[ioctl_addr[2:0]]`
// (Arcade-IremM90, Arcade-IremM92, Arcade-Robotron, Arcade-TaitoF2, zerowing
// -- the last one even comments it "8 dip switches of 8 bits").
//
// This used to be a single register latched on `!ioctl_addr[24:1]`, which
// matches address 0 *and* address 1, so byte 1 landed on top of byte 0 and
// dsw_hps ended up 0x00.  in_dsw is then 0xFC: bit 1 is the TEST switch and
// bit 0 is FREEZE, both active low, both asserted.  The game's response to
// TEST is to boot into the operator menu instead of the game --
//
//     NA-1/2 SELF TEST Rev.2.12
//     FLIP [OFF] / SWITCH Test / SOUND Test / COIN Options / GAME Options /
//     UARTs Test / OBJECT Test / SCROLL Test / A.D.S. / COLOR Test /
//     CONVERGENCE Test
//
// -- which is exactly the screen this core came up in on hardware.
//
// MAME_VERIFIED, measured rather than argued: MAME driven with DSW = 0xFC,
// the value this core was presenting, draws that menu and parks the 68000 at
// C2E4F0/C2E4F4 (the self-test dispatcher) from frame 500 on; the same run
// with DSW = 0xFF reaches attract mode by frame 900.  Both were done with a
// blank EEPROM, so the EEPROM had nothing to do with it.
reg [7:0] dsw_reg[0:7];
integer   dsw_i;
initial for (dsw_i = 0; dsw_i < 8; dsw_i = dsw_i + 1) dsw_reg[dsw_i] = 8'hFF;
always @(posedge clk_sys) begin
	if (ioctl_wr && (ioctl_index == 254) && !ioctl_addr[24:3])
		dsw_reg[ioctl_addr[2:0]] <= ioctl_dout;
end
assign dsw_hps = dsw_reg[0];

///////////////////////   CORE   /////////////////////////////////

wire [23:1] mem_addr;
wire [15:0] mem_dout, mem_din;
wire        mem_req, mem_we, mem_ack;
wire  [1:0] mem_ds;
wire [13:0] bios_addr;
wire  [7:0] bios_data;

wire        ce_pix, hblank, vblank, hsync, vsync;
wire  [8:0] hpos, vpos;
wire  [7:0] vid_r, vid_g, vid_b;

na2_eeprom_nvport nvport
(
	.clk                (clk_sys),
	// rst_mem, not reset: the nvram port has to run *during* the download,
	// for the same reason na2_membus does.
	.rst                (rst_mem),
	.nvs_download       (ioctl_download),
	.nvs_upload         (ioctl_upload),
	.nvs_wr             (ioctl_wr),
	.nvs_addr           (ioctl_addr),
	.nvs_data_in        (ioctl_dout),
	.nvs_index          (ioctl_index),
	.nvs_data_out       (nv_ioctl_din),
	.nvs_save_req       (nv_upload_req),
	.nvs_save_index     (nv_upload_index),
	.nv_req             (nv_req),
	.nv_we              (nv_we),
	.nv_addr            (nv_addr),
	.nv_din             (nv_din),
	.nv_dout            (nv_dout),
	.nv_gnt             (nv_gnt),
	.game_we            (nv_game_we),
	.nv_loaded          (),
	.nv_dirty           ()
);

// EEPROM_DEFAULT_FILE is resolved by Quartus against the *project* directory,
// which is this one -- targets/mister -- not the board's.  na2_top's default is
// written for a project sitting at the board root, so every target has to pass
// its own path.  pocket_probe already does the same thing.  Getting this wrong
// does not fail the build: $readmemh just leaves the array at 0xFF and the
// power-on contents silently go missing, so the build log is checked for
// "can't open" after every move.  docs/PORTABILITY.md section 3.
na2_top #(.EEPROM_DEFAULT_FILE("../../rtl/memory/na2_eeprom_default.hex")) na2
(
	.clk        (clk_sys),
	.rst        (reset),
	.ce_en      (~pause_core),

	// CG lives on-chip in this target (CG_EXTERNAL defaults to 0), so the
	// external row port is not used.  Tied off explicitly rather than left
	// open: an unconnected input floats, and this project has lost time to a
	// port that looked connected and was not.
	.cg_xm_ack   (1'b0),
	.cg_xm_rdata (64'd0),
    // ROZ's second CG store is Pocket-only; MiSTer keeps CG on chip and
    // CG_SPLIT_ROZ defaults off, so these are driven idle rather than left
    // dangling.
    .rz_xm_req(), .rz_xm_we(), .rz_xm_addr(), .rz_xm_wdata(), .rz_xm_wstrb(),
    .rz_xm_ack(1'b0), .rz_xm_rdata('0),
	.aud_l      (aud_l),
	.aud_r      (aud_r),

	.mem_addr   (mem_addr),
	.mem_dout   (mem_dout),
	.mem_din    (mem_din),
	.mem_req    (mem_req),
	.mem_we     (mem_we),
	.mem_ds     (mem_ds),
	.mem_ack    (mem_ack),

	.bios_addr  (bios_addr),
	.bios_data  (bios_data),

	.in_p1      (in_p1),
	.in_p2      (in_p2),
	.in_p4      (in_p4),
	.in_p3      (in_p3),
	.in_dsw     (in_dsw),
	// Which raw scanline raises the vblank interrupt.  An open question
	// (KNOWN_ISSUES.md ISSUE-1) that the person in front of the board can
	// settle by looking, so it is an OSD option rather than a parameter.
	.irq_line_sel(status[12:11]),
	.spr_bank_sel(status[13]),
	.keycus_id(keycus_id),
	.keycus_off(keycus_off),
	.nv_req(nv_req), .nv_we(nv_we), .nv_addr(nv_addr),
	.nv_din(nv_din), .nv_dout(nv_dout), .nv_gnt(nv_gnt),
	.nv_game_we(nv_game_we),

	.ce_pix     (ce_pix),
	.hpos       (hpos),
	.vpos       (vpos),
	.hblank     (hblank),
	.vblank     (vblank),
	.hsync      (hsync),
	.vsync      (vsync),
	.vid_r      (vid_r),
	.vid_g      (vid_g),
	.vid_b      (vid_b),
	// Labels are "On,Off" as of 2026-08-22, so status bit 0 means ON and the
	// bit is inverted here.  The two have to move together: flipping the labels
	// without flipping this is what made the option read backwards once before.
	// DIAGNOSTIC, fix/na12-accuracy: with the NA12 probe page selected
	// (status[20]) the overlay also shows while the pad's Pause has the core
	// frozen, so a player can reach a scene with the picture clear, press R,
	// and have the probe read off the frozen frame.  Counters stop with the
	// core (ce_en), so the reading does not move under the screenshot.
	.dbg_overlay_en(status[10] | (pause_latch & status[20])),
	.dbg_page_snd  (status[18]),
	.dbg_page_na12 (status[20]),
	.cpu_recover   (~status[21]),
	.pal_hold_en   (~status[22]),
	.snd_fade_off  (status[14]),
	.dbg_st_word0(st_word0),
	.dbg_st_errs (st_errs),
	.dbg_st_dlwords(st_dlwords),
	.dbg_st_dlindex(st_dlindex),
	.dbg_st_berrs(st_berrs),

	.dbg_addr(), .dbg_as_n(), .dbg_rd(), .dbg_wr(), .dbg_dout(), .dbg_din(),
	.dbg_fc(), .dbg_ack(), .dbg_m68k_rst(), .dbg_vreg_irq_en(),
	.dbg_gfx_enable(), .dbg_blit_busy(), .dbg_blit_start(), .dbg_c70_cmd(),
	.dbg_c70_done(), .dbg_unmapped(), .dbg_rdata(), .dbg_blit_p(), .dbg_bus()
);

///////////////////////   MEMORY   ///////////////////////////////

wire [24:0] sd_addr;
wire [15:0] sd_din, sd_dout;
wire        sd_req, sd_we, sd_ack;
wire  [1:0] sd_ds;

na2_membus membus
(
	.clk             (clk_sys),
	.rst             (rst_mem),

	.mem_addr        (mem_addr),
	.mem_dout        (mem_dout),
	.mem_din         (mem_din),
	.mem_req         (mem_req),
	.mem_we          (mem_we),
	.mem_ds          (mem_ds),
	.mem_ack         (mem_ack),

	.bios_addr       (bios_addr),
	.bios_data       (bios_data),

	.ld_download     (ioctl_download && (ioctl_index == 0)),
	.ld_wr           (ioctl_wr),
	.ld_addr         (ioctl_addr[24:0]),
	.ld_data         (ioctl_dout),

	.sd_addr         (sd_addr),
	.sd_din          (sd_din),
	.sd_dout         (sd_dout),
	.sd_req          (sd_req),
	.sd_we           (sd_we),
	.sd_ds           (sd_ds),
	.sd_ack          (sd_ack),
	.ld_wait         (ioctl_wait),

	.st_busy         (st_busy),
	.st_word0        (st_word0),
	.st_errs         (st_errs),
	.st_berrs        (st_berrs),
	.st_dlwords      (st_dlwords)
);

// Run the SDRAM init sequence once after the PLL locks.
reg [3:0] sdram_init_cnt = 0;
wire      sdram_init = ~sdram_init_cnt[3];
always @(posedge clk_sys) begin
	if (!pll_locked) sdram_init_cnt <= 0;
	else if (sdram_init) sdram_init_cnt <= sdram_init_cnt + 1'd1;
end

na2_sdram sdram
(
	.clk        (clk_sys),
	.init       (sdram_init),

	.addr       (sd_addr),
	.din        (sd_din),
	.dout       (sd_dout),
	.req        (sd_req),
	.we         (sd_we),
	.ds         (sd_ds),
	.ack        (sd_ack),

	.SDRAM_A    (SDRAM_A),
	.SDRAM_BA   (SDRAM_BA),
	.SDRAM_DQ   (SDRAM_DQ),
	.SDRAM_DQML (SDRAM_DQML),
	.SDRAM_DQMH (SDRAM_DQMH),
	.SDRAM_nCS  (SDRAM_nCS),
	.SDRAM_nWE  (SDRAM_nWE),
	.SDRAM_nRAS (SDRAM_nRAS),
	.SDRAM_nCAS (SDRAM_nCAS),
	.SDRAM_CKE  (SDRAM_CKE)
);

///////////////////////   VIDEO   ////////////////////////////////

wire [2:0] fx = status[4:2];

arcade_video #(.WIDTH(304), .DW(24)) arcade_video
(
	.*,
	.clk_video(clk_sys),
	.ce_pix(ce_pix),
	.RGB_in({vid_r, vid_g, vid_b}),
	.HBlank(hblank),
	.VBlank(vblank),
	.HSync(hsync),
	.VSync(vsync),
	.fx(fx)
);

// --- rotation ---------------------------------------------------------------
// screen_rotate takes the picture arcade_video just produced on the VGA_*
// outputs, writes it turned into DDRAM, and hands the scaler an FB_* buffer to
// read back.  It is instantiated unconditionally; with no_rotate high it holds
// FB_EN low and the ordinary VGA path is what reaches the screen, so the
// default costs a player nothing.
//
// video_rotated goes back to hps_io so MiSTer knows the picture has been
// turned -- that is what makes the OSD and the aspect ratio come out the right
// way up rather than sideways on top of a rotated game.
// Rotate is On/Off: On means the cabinet's own orientation, which for this
// family is always ROT90.  Which way to turn is the BOARD's business, not the
// player's, so it comes from the .mra rather than from a second menu item.
// A set that is not marked vertical cannot rotate at all, so a stale saved
// config cannot turn a horizontal game on its side.
wire       no_rotate  = ~(board_vertical & (status[16:15] != 2'd0));
wire       rot_43     = (status[16:15] == 2'd2);
wire       rotate_ccw = board_flags[1];
wire       flip       = board_vertical & status[17];
wire       video_rotated;

screen_rotate screen_rotate
(
	.CLK_VIDEO(CLK_VIDEO),
	.CE_PIXEL(CE_PIXEL),
	.VGA_R(VGA_R), .VGA_G(VGA_G), .VGA_B(VGA_B),
	.VGA_HS(VGA_HS), .VGA_VS(VGA_VS), .VGA_DE(VGA_DE),
	.rotate_ccw(rotate_ccw),
	.no_rotate(no_rotate),
	.flip(flip),
	.video_rotated(video_rotated),
	.FB_EN(FB_EN), .FB_FORMAT(FB_FORMAT),
	.FB_WIDTH(FB_WIDTH), .FB_HEIGHT(FB_HEIGHT),
	.FB_BASE(FB_BASE), .FB_STRIDE(FB_STRIDE),
	.FB_VBL(FB_VBL), .FB_LL(FB_LL),
	.DDRAM_CLK(DDRAM_CLK), .DDRAM_BUSY(DDRAM_BUSY),
	.DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_ADDR(DDRAM_ADDR),
	.DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE),
	.DDRAM_WE(DDRAM_WE), .DDRAM_RD(DDRAM_RD)
);

assign LED_USER = ioctl_download;

///////////////////////   DIAGNOSTIC   ///////////////////////////
//
// BUTTONS is the core's channel of last resort, and it is worth knowing about.
//
// When the HPS handshake wedges, every normal way of asking the core anything
// is wedged with it: MiSTer's main spins in its FPGA-manager bit-bang routine
// waiting for gp_in[17] (io_ack), which sys_top.v produces as
//
//     always @(posedge clk_sys)
//       if(~(io_wait | vs_wait) | io_strobe) begin rack <= io_clk; io_ack <= rack; end
//
// so exactly three things can hold it off: clk_sys not running, io_wait (which
// is HPS_BUS[37], our ioctl_wait) stuck high, or vs_wait stuck high.
//
// BUTTONS escapes all of that.  sys_top.v wires it straight into gp_in:
//     gp_in[30] = btn_user | btn[1]
//     gp_in[29] = btn_osd  | btn[0]
// with no clock and no handshake in the way, so whatever the core drives onto
// it stays readable from a completely hung system with `devmem 0xFF706014 32`.
// That is how the PLL was caught: a heartbeat here stayed frozen, which said
// clk_sys was not running, which is a thing no report in the build had said.
//
// To reinstate it, replace the tie-off below with:
//
//     reg [23:0] dbg_hb = 0;
//     always @(posedge clk_sys) dbg_hb <= dbg_hb + 1'd1;
//     assign BUTTONS = {dbg_hb[23], ioctl_wait};   // ~3 Hz heartbeat, ioctl_wait
//
// and read it with tools/mister/na2_gpi.sh.  Do not leave it in: bit 1 is the
// user button, so a running heartbeat makes MiSTer reset the core several times
// a second, and bit 0 is the OSD button.
assign BUTTONS = 0;

endmodule
