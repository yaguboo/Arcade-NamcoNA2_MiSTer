//============================================================================
//  Namco NA-2 -- 28C16 parallel EEPROM, with the MiSTer nvram save path
//
//  2 KiB, byte wide, on the 68000's D7-D0 lane at 0xE00000-0xE00FFF.
//  MAME_VERIFIED: namcona1.cpp namcona1_main_map has
//      map(0xe00000, 0xe00fff).rw("eeprom", ...).umask16(0x00ff);
//  and the machine config instantiates EEPROM_2816.  So 0x1000 bytes of
//  address space hold 0x800 EEPROM bytes: one per *word*, on the odd address.
//
//  This is not a nicety.  Numan Athletics initialises its EEPROM at boot and
//  will not go any further until the write sticks:
//
//      C308EE  lea.l  $E00100, a0        228 bytes, from a table at $144(a5)
//      C308FE  move.b $1(a0), d0         read what is there
//      C30906  move.b (a1)+, d1          what it should be
//      C30908  bsr.w  $C30918            ... which, if they differ:
//      C3092A     move.b d1, $1(a0)          writes it
//      C30932     clr.b  $fbf.w
//      C30936     move.b $1(a0), d2          reads it back
//      C3093E     cmp.b  d1, d2
//      C30940     bne.b  $C30932             and spins until it matches
//
//  Without an EEPROM that loop never exits, which is exactly where the core
//  sat on hardware -- the debug overlay's sampled fetch address read
//  0xC30934..0xC30942 on three screenshots in a row, with zero unmapped
//  accesses.  ROM_DISASM_CONFIRMED.
//
//  NOT MODELLED: the real part's 1-10 ms write cycle and its data-polling /
//  toggle-bit status during it.  MAME does not model it either, and the game's
//  own loop above tolerates an instant write.  `INFERRED` that nothing depends
//  on the delay.
//
//  ---------------------------------------------------------------------
//  Power-on contents
//
//  A real erased 28C16 reads 0xFF everywhere, and numanath ships no default
//  EEPROM image in MAME (unlike Cosmo Gang the Puzzle and others, which ship
//  one "otherwise game would lock up on 1st boot").  So a blank part is the
//  faithful power-on state, and the game copes with it: at
//
//      C2D940  lea.l   $C29954(pc), a0      "12Feb93REVISION6"
//      C2D944  lea.l   $E00000.l, a1
//      C2D94A  moveq   #$3, d6              4 longs = 16 bytes
//      C2D94C  movep.l $1(a1), d0           the odd lane, 4 bytes at a time
//      C2D950  cmp.l   (a0)+, d0
//      C2D952  bne.b   $C2D95C              -> first-boot init path
//
//  a mismatch sends it to C2D95C, which writes the whole settings block --
//  signature, coin options and game options -- and carries straight on to
//  C2D9A6 where it reads the block back.  ROM_DISASM_CONFIRMED.
//
//  MAME_VERIFIED, measured this session and not read out of a source file:
//  starting numanath in MAME with no nvram file at all, the 2 KiB behind
//  0xE00001 has become byte-for-byte identical to nvram/numanath/eeprom by
//  frame 200 (0 differing bytes of 2048), and the game reaches attract mode
//  on frame 900 with the same 68000 PC trace as a run that started from the
//  saved image.  A blank part costs one extra screen at the very first boot,
//  nothing more.
//
//  DEFAULT_IMAGE therefore loads that same settings block as the array's
//  power-up contents.  It is a **convenience**, tagged `INFERRED` /
//  convenience and not `HW_CONFIRMED`: a real board with a fresh 28C16 shows
//  the first-boot init screen, and this one will not.  The bytes are not
//  invented -- they are exactly what this game writes into a blank part on
//  its own, captured from a local MAME run (see LICENSE_NOTES.md), so the
//  machine state after the first frames is the same either way.
//
//  2026-10-06: DEFAULT_IMAGE now defaults to 0 -- the faithful erased part --
//  and the MiSTer target ships that way.  The image put Numan Athletics'
//  block into every set's EEPROM, which no real board of another game has.
//  Verified on the DE10-Nano the same day, each set loaded under a fresh name
//  (no saved .nvm) with 2 KiB of 0xFF sent at ioctl index 2: all fifteen sets
//  that ship no factory EEPROM reach attract mode from blank, numanath and
//  numanathj included.  The six that ship one (cgangpzl/j, exvania/j,
//  knckheadjp, quiztou) do NOT boot from blank -- NOTICE or black, frozen --
//  and boot normally with their factory image, which their .mra keeps
//  sending, so this default never reaches them.  docs/VALIDATION.md.
//  The Pocket target still passes EEPROM_DEFAULT_IMAGE = 1 until it is
//  measured there.
//
//  To get the faithful blank part back, set DEFAULT_IMAGE = 0 on the
//  instance.  Deleting rtl/memory/na2_eeprom_default.hex also works and is
//  louder about it: $readmemh then leaves the 0xFF fill in place, with a
//  warning from the tool.
//
//  ---------------------------------------------------------------------
//  Persistence -- the MiSTer nvram path
//
//  na2_eeprom_array carries a second, HPS-side access port so the 2 KiB can
//  be filled from a saved file at load time and read back out when the HPS
//  asks for a save.  na2_eeprom_nvport is the state machine that speaks the
//  host save protocol on the other end of it.  save index 2, size 2048;
//  see docs/nvram_save.md for the whole design and the index survey it came
//  from.  na2_top brings the array's second port out through its nv_* ports
//  and NamcoNA2.sv instantiates na2_eeprom_nvport against them.
//
//  ONE WRITE PORT.  HANDOFF.md section 4 item 3: an array with two write
//  ports does not go into M10K however mutually exclusive the two writers
//  are at run time, and this project has already lost a 20-minute fitter run
//  to that.  So the HPS does not get a port of its own -- the single existing
//  port is muxed, the 68000 wins every cycle it wants it, and the HPS takes
//  the cycles that are left.  That is not a fairness nicety either: na2_top
//  latches the 68000's EEPROM read out of this array's `q` on exactly one
//  clock edge per bus cycle (rd_hold <= rdata_mux on cpu_ack & cpu_rd, with
//  cpu_ack being the one-cycle fast_ack), so a single stolen cycle would hand
//  the game the wrong byte.  cpu_busy is what makes that impossible.
//============================================================================
`default_nettype none

// ---------------------------------------------------------------------------
//  The array itself: 2048 x 8, one write port, two requesters.
// ---------------------------------------------------------------------------
module na2_eeprom_array #(
    // 1 = power up holding the settings block this game writes for itself.
    // 0 = power up as a real erased 28C16 does, 0xFF everywhere.
    parameter bit DEFAULT_IMAGE = 1'b0,
    parameter     DEFAULT_FILE  = "rtl/memory/na2_eeprom_default.hex"
) (
    input  wire        clk,

    // --- 68000 side -------------------------------------------------------
    input  wire [11:1] a,          // 68000 word address inside 0xE00000
    input  wire [15:0] din,
    output reg  [15:0] dout,
    input  wire        we,         // one clock, low byte lane
    input  wire        cpu_busy,   // 1 while the 68000 has a cycle in flight

    // --- HPS side (see na2_eeprom_nvport) ---------------------------------
    input  wire        nv_req,     // wants the port this clock
    input  wire        nv_we,      // ... to write rather than read
    input  wire [10:0] nv_addr,
    input  wire  [7:0] nv_din,
    output wire  [7:0] nv_dout,    // valid the clock after nv_gnt
    output wire        nv_gnt      // request granted this clock
);

  // Declarations before first use: Quartus tolerates the other order, ModelSim
  // refuses to compile the file at all (HANDOFF.md section 4 item 8).
  reg  [7:0] mem[0:2047];
  reg  [7:0] q;
  reg        nv_req_s;
  integer    i;

  // nv_req is sanitised before anything selects on it.  In synthesis it is an
  // ordinary driven input and nv_req_s is exactly nv_req -- the cleanup sits
  // inside `synthesis translate_off`.  In simulation it can be left
  // unconnected by a testbench written before this port existed
  // (sim/tb_na2_boot.sv instantiates na2_top without it), and an unconnected
  // input is 'z': `z & ~cpu_busy` is X, the port mux below then selects on X,
  // and the entire EEPROM reads X.  The game does not boot past 0xC30932
  // without this array, so that failure is total and silent, and it is folded
  // to 0 here rather than left to every testbench to remember.
  always @(*) begin
    nv_req_s = nv_req;
    // synthesis translate_off
    if (nv_req !== 1'b1) nv_req_s = 1'b0;
    // synthesis translate_on
  end

  // The 68000 owns the port whenever it is in a bus cycle.  Everything else
  // the HPS may have.
  assign nv_gnt = nv_req_s & ~cpu_busy;

  wire [10:0] p_addr = nv_gnt ? nv_addr  : a;
  wire  [7:0] p_din  = nv_gnt ? nv_din   : din[7:0];
  wire        p_we   = nv_gnt ? nv_we    : we;

  // Power-up contents.  Quartus honours an initial block for an array this
  // size and turns it into the M10K's init data; the 0xFF fill runs 2048
  // iterations, well inside the 5000-iteration cap in HANDOFF.md section 4
  // item 4.  $readmemh then overwrites all 2048 of them when the default
  // image is wanted -- the hex file holds the erased 0xFF bytes too, so the
  // two are not layered, the fill is only there so that DEFAULT_IMAGE = 0
  // and a missing hex file both give a properly blank part.
  //
  // The path is relative to the project root, which is the working directory
  // both for ./build.sh (Quartus resolves $readmemh against the project
  // directory) and for sim/run_*.sh.
  initial begin
    for (i = 0; i < 2048; i = i + 1) mem[i] = 8'hFF;
    if (DEFAULT_IMAGE) $readmemh(DEFAULT_FILE, mem);
  end

  always @(posedge clk) begin
    if (p_we) mem[p_addr] <= p_din;
    q <= mem[p_addr];
  end

  assign nv_dout = q;

  // umask16(0x00ff): only D7-D0 is the EEPROM.  The rest of the word is not
  // driven by anything, and MAME hands back zero for it.
  always @(*) dout = {8'h00, q};

endmodule


// ---------------------------------------------------------------------------
//  There used to be a five-port `na2_eeprom` wrapper here, with the nvram side
//  tied to constants, for as long as rtl/na2_top.sv still instantiated it.
//  na2_top now instantiates na2_eeprom_array directly and passes the port
//  through, so the wrapper is gone.
//
//  If you ever need to stage this again: do NOT add the ports to a module and
//  leave them unconnected at the instantiation.  Measured on Quartus 17.0, an
//  unconnected input port is undefined rather than zero and the whole array is
//  then optimised away -- a build that passes with the EEPROM silently gone,
//  which is the failure mode this project already lost three sessions to with
//  the PLL.  Ports and their connections belong in the same commit.  The same
//  goes the other way in simulation: an unconnected nv_req is 'z', so
//  na2_eeprom_array folds anything that is not a solid 1 down to 0 before it
//  reaches the port mux (see nv_req_s above).
// ---------------------------------------------------------------------------

`default_nettype wire
