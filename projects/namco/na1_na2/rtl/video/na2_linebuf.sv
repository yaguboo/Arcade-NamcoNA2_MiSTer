//============================================================================
//  Namco NA-2 -- double-buffered scanline buffer
//
//  One half is written by the renderer while the other is scanned out; they
//  swap every line.  Each entry carries the pen and the priority value that
//  wrote it, which is what MAME keeps in its separate `screen.priority()`
//  bitmap: layers are drawn in increasing priority order and each stamps its
//  own priority, then sprites draw over a pixel only when the sprite's
//  priority is at least the value already there
//  (MAME_VERIFIED: pdraw_tile, `if (pri[x] <= priority)`).
//
//  **Two pixels per clock.**  The tilemap renderer produces two pixels per
//  clock (one 16-bit CG RAM word = two 8bpp pixels), and a single write port
//  would not fit the line budget: four layers at one pixel per clock is 3120
//  of the 3184 system clocks in a line, leaving nothing for sprites.
//
//  The two pixels of a pair are always at adjacent x, so exactly one of them
//  is even.  Each half of the buffer is therefore split into an even-x array
//  and an odd-x array, and the incoming pair is *routed by parity* so that
//  each array has exactly ONE write port.
//
//  That routing is not cosmetic.  The first version selected the array with
//     if (wr_a && !addr_a[0]) even[...] <= ...;
//     if (wr_b && !addr_b[0]) even[...] <= ...;
//  which is correct at run time -- only one of the two can ever fire -- but it
//  gives each array two *structural* write ports.  An M10K has two ports in
//  total and one of them is needed for reading, so Quartus could not infer
//  block RAM and began building 4 x 512 x 16 bits out of flip-flops:
//  Analysis & Synthesis ran for twenty minutes and 17 GB before it was killed.
//  One write port per array is what makes this map to memory.
//
//  304 visible pixels; 256 entries per parity array, so the index is a plain
//  8 bits and writes pushed off the edge by fine scrolling land harmlessly
//  out of view.
//
//  The shadow flag is not in those arrays.  A hardware-shadow sprite pixel
//  (MAME_VERIFIED: pdraw_tile, `dest[x] = palette_shadow_table[dest[x]]`)
//  darkens whatever is already on the line and must leave the pen alone, so
//  its write has to set one bit without rewriting the entry -- which a memory
//  with one write port cannot do.  One flip-flop per pixel per half, 1024 of
//  them, written independently of the pen port.
//============================================================================
`default_nettype none

module na2_linebuf (
    input  wire        clk,

    input  wire        bank,          // which half the renderer owns

    // --- renderer write port A ---------------------------------------------
    input  wire        wr_a,
    input  wire [8:0]  addr_a,
    input  wire [12:0] pen_a,
    input  wire [2:0]  pri_a,
    input  wire        hwsh_we_a,    // write the shadow flag (independent of wr_a)
    input  wire        hwsh_a,

    // --- renderer write port B (always adjacent to addr_a) -----------------
    input  wire        wr_b,
    input  wire [8:0]  addr_b,
    input  wire [12:0] pen_b,
    input  wire [2:0]  pri_b,
    input  wire        hwsh_we_b,
    input  wire        hwsh_b,

    // --- display read port -------------------------------------------------
    input  wire [8:0]  rd_addr,
    output wire [12:0] rd_pen,
    output wire [2:0]  rd_pri,
    output wire        rd_hwsh
);

  // Route the pair by parity so each array sees a single write port.
  wire a_is_even = ~addr_a[0];

  wire        ev_we   = a_is_even ? wr_a           : wr_b;
  wire [7:0]  ev_addr = a_is_even ? addr_a[8:1]    : addr_b[8:1];
  wire [15:0] ev_data = a_is_even ? {pri_a, pen_a} : {pri_b, pen_b};

  wire        od_we   = a_is_even ? wr_b           : wr_a;
  wire [7:0]  od_addr = a_is_even ? addr_b[8:1]    : addr_a[8:1];
  wire [15:0] od_data = a_is_even ? {pri_b, pen_b} : {pri_a, pen_a};

  // {pri[2:0], pen[12:0]}, one array per (half, parity)
  reg [15:0] buf0_ev[0:255];
  reg [15:0] buf0_od[0:255];
  reg [15:0] buf1_ev[0:255];
  reg [15:0] buf1_od[0:255];

  // synthesis translate_off
  integer ii;
  initial for (ii = 0; ii < 256; ii = ii + 1) begin
    buf0_ev[ii] = 16'h0000; buf0_od[ii] = 16'h0000;
    buf1_ev[ii] = 16'h0000; buf1_od[ii] = 16'h0000;
  end
  // synthesis translate_on

  reg [15:0] q0_ev, q0_od, q1_ev, q1_od;

  always @(posedge clk) begin
    if (ev_we && !bank) buf0_ev[ev_addr] <= ev_data;
    q0_ev <= buf0_ev[rd_addr[8:1]];
  end
  always @(posedge clk) begin
    if (od_we && !bank) buf0_od[od_addr] <= od_data;
    q0_od <= buf0_od[rd_addr[8:1]];
  end
  always @(posedge clk) begin
    if (ev_we && bank) buf1_ev[ev_addr] <= ev_data;
    q1_ev <= buf1_ev[rd_addr[8:1]];
  end
  always @(posedge clk) begin
    if (od_we && bank) buf1_od[od_addr] <= od_data;
    q1_od <= buf1_od[rd_addr[8:1]];
  end

  // ---- the shadow flag, in flip-flops -----------------------------------
  // Split by parity for the same reason the pen arrays are, though for a
  // different cost: these are registers, so two write ports are legal, but
  // two 1-of-512 decoders are about twice the logic of four 1-of-256 ones.
  wire       hw_ev_we = a_is_even ? hwsh_we_a : hwsh_we_b;
  wire [7:0] hw_ev_i  = a_is_even ? addr_a[8:1] : addr_b[8:1];
  wire       hw_ev_d  = a_is_even ? hwsh_a : hwsh_b;
  wire       hw_od_we = a_is_even ? hwsh_we_b : hwsh_we_a;
  wire [7:0] hw_od_i  = a_is_even ? addr_b[8:1] : addr_a[8:1];
  wire       hw_od_d  = a_is_even ? hwsh_b : hwsh_a;

  reg [255:0] hwsh0_ev, hwsh0_od, hwsh1_ev, hwsh1_od;
  always @(posedge clk) begin
    if (hw_ev_we) begin
      if (bank) hwsh1_ev[hw_ev_i] <= hw_ev_d; else hwsh0_ev[hw_ev_i] <= hw_ev_d;
    end
    if (hw_od_we) begin
      if (bank) hwsh1_od[hw_od_i] <= hw_od_d; else hwsh0_od[hw_od_i] <= hw_od_d;
    end
  end

  reg       bank_d;
  reg [8:0] rd_addr_d;
  always @(posedge clk) begin
    bank_d    <= bank;
    rd_addr_d <= rd_addr;
  end

  // the display reads whichever half the renderer does not own
  wire [15:0] q = bank_d ? (rd_addr_d[0] ? q0_od : q0_ev)
                         : (rd_addr_d[0] ? q1_od : q1_ev);
  assign {rd_pri, rd_pen} = q;
  assign rd_hwsh = bank_d ? (rd_addr_d[0] ? hwsh0_od[rd_addr_d[8:1]]
                                           : hwsh0_ev[rd_addr_d[8:1]])
                          : (rd_addr_d[0] ? hwsh1_od[rd_addr_d[8:1]]
                                          : hwsh1_ev[rd_addr_d[8:1]]);

endmodule

`default_nettype wire
