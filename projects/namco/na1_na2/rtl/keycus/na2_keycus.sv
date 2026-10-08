//============================================================================
//  Namco NA-1/NA-2 KEYCUS -- the custom key chip on the ROM board
//
//  Sits at 68000 0xE40000-0xE4000F.  Two jobs: identify the game (so a ROM
//  swap alone will not run), and act as a random number source.
//
//  Every NA-1/NA-2 set is one chip from the same family, and which one is a
//  property of the ROM BOARD, not of this core -- so the identity arrives as
//  an input rather than being compiled in.  That is what lets a single
//  bitstream run Numan Athletics, Emeraldia and Knuckle Heads: the .mra says
//  which chip is socketed, exactly as it says which ROMs are.
//
//  What Numan Athletics actually does with it (ROM_DISASM_CONFIRMED, from
//  tools/disasm68k.py --find-io over the program ROM):
//    - word offset 2 (0xE40004) is read in ~106 places, always twice in a row
//      8 bytes apart, and the result is consumed as a random value.
//    - the identity word is read through the pointer set up by
//      "LEA $E40000,A0" at 0xC2D9D4.
//  The game never writes the chip.
//
//  MAME_VERIFIED: namcona1.cpp custom_key_r is, for every set in this family,
//  two rules and nothing else --
//      offset == the set's identity offset  ->  the identity word
//      anything else                        ->  machine().rand()
//  (the switch returns the identity for one offset, and the line after the
//  switch is "return machine().rand()&0xffff").  m_count, which some cases
//  name explicitly, is the same random value with a guarantee that two
//  consecutive reads differ.  So the whole family needs no per-game logic
//  here, only the two numbers below.
//
//  The identity values are the chip's own part number -- C348 -> 0x015C,
//  C349 -> 0x015D, ... C359 -> 0x0167, C360 -> 0x0168 -- and that regularity
//  across eleven chips is strong evidence they really are part numbers rather
//  than arbitrary constants.  tools/verify_roms.py NA_GAMES holds the table.
//
//  TINKLE PIT (C367) is the one chip in the family that is not a constant,
//  and it is handled below rather than left out.  Its chip runs a 32-bit
//  register read back through a fixed bitswap, stepped and conditionally
//  inverted on every read of word offset 3, and cleared by a read of offset 4.
//
//  MAME_VERIFIED + FBNEO_CROSSCHECK, and the two are independent: MAME
//  (namcona1.cpp, NAMCO_TINKLPIT) runs the algorithm, while FBNeo
//  (d_namcona1.cpp tinklpit_keycus_read) hard-codes the first 32 outputs as a
//  table -- somebody there ran it once and wrote the answers down.  Running
//  MAME's algorithm reproduces FBNeo's table exactly, 32 of 32, so the shift
//  order, the bitswap order and the feedback condition are all confirmed twice
//  over.  tools/keycus_tinklpit.py is that check, and sim/tb_na2_keycus.sv
//  runs the same 32 values against this RTL.
//
//  Which chip is socketed still arrives from the .mra, so Tinkle Pit needs no
//  new descriptor field and no new core input: the identity 0x016F IS the
//  selector.  No other chip in the family carries that number.
//
//  UNKNOWN: the internal structure of the random generator.  MAME uses the
//  host RNG and additionally forces two consecutive reads to differ; FBNeo
//  uses its own "guaranteed_rand".  Neither is hardware evidence.  This module
//  is therefore a game-functional model of the random half, not a hardware
//  model -- see RESEARCH.md "C359" and KNOWN_ISSUES.md.
//============================================================================
`default_nettype none

module na2_keycus (
    input  wire        clk,
    input  wire        rst,
    input  wire        ce_en,

    // Which chip is on this ROM board.  Held stable outside reset; the game
    // reads the identity once at boot, long after the .mra has been streamed.
    input  wire [15:0] id,          // e.g. 0x0167 for C359
    input  wire [2:0]  id_off,      // the word offset that answers with it

    input  wire        sel,
    input  wire [2:0]  addr,        // word offset within 0xE40000
    input  wire        rd,
    output wire [15:0] dout
);

  // 32-bit maximal-length Galois LFSR (taps 32,22,2,1).  Chosen because it
  // never repeats a value on consecutive reads, which is the only property
  // MAME's implementation actually guarantees.  INFERRED, not hardware.
  reg [31:0] lfsr;
  wire       step = sel & rd & ce_en;

  always @(posedge clk) begin
    if (rst)       lfsr <= 32'h1357_9BDF;          // any non-zero seed
    else if (step) lfsr <= {lfsr[30:0], 1'b0} ^ (lfsr[31] ? 32'h04C1_1DB7 : 32'd0);
  end

  // --- Tinkle Pit's C367 -------------------------------------------------
  //
  // offset 4 read : clear the register
  // offset 3 read : return bitswap16(keyval), THEN keyval >>= 1, and if the
  //                 shifted value is zero or has odd parity over the mask
  //                 0x58000C00, flip bit 31.
  //
  // The order matters and is MAME's: the value handed back is computed from
  // the register BEFORE the step, and the feedback test looks at the register
  // AFTER it.  Getting either way round wrong still produces a plausible
  // stream of numbers, which is why this is checked against 32 known answers
  // rather than by inspection.
  //
  // Verilog's reduction XOR is a parity, which is what std::popcount(x) & 1
  // means here.
  localparam [15:0] TINKLPIT_ID = 16'h016F;
  wire tp = (id == TINKLPIT_ID);

  reg  [31:0] keyval;
  wire        tp_rd    = tp & sel & rd & ce_en;
  wire        tp_clear = tp_rd & (addr == 3'd4);
  wire        tp_step  = tp_rd & (addr == 3'd3);

  wire [31:0] keyval_sh = {1'b0, keyval[31:1]};
  wire        keyval_fb = (keyval_sh == 32'd0) | (^(keyval_sh & 32'h5800_0C00));

  always @(posedge clk) begin
    if (rst)            keyval <= 32'd0;
    else if (tp_clear)  keyval <= 32'd0;
    else if (tp_step)   keyval <= keyval_fb ? (keyval_sh ^ 32'h8000_0000)
                                            : keyval_sh;
  end

  // bitswap<16>(keyval, 22,26,31,23,18,20,16,30,24,21,25,19,17,29,28,27):
  // the first source bit listed becomes the MSB of the result.
  wire [15:0] tp_dout = {keyval[22], keyval[26], keyval[31], keyval[23],
                         keyval[18], keyval[20], keyval[16], keyval[30],
                         keyval[24], keyval[21], keyval[25], keyval[19],
                         keyval[17], keyval[29], keyval[28], keyval[27]};

  // The identity wins, as it does in MAME's switch: Tinkle Pit's identity is
  // at offset 7 and its algorithm at offset 3, so the two never collide.
  assign dout = (addr == id_off)     ? id
              : (tp && addr == 3'd3) ? tp_dout
                                     : lfsr[15:0];

endmodule

`default_nettype wire
