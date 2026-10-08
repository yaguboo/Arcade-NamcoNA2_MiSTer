//============================================================================
//  Namco NA-1 / NA-2 -- CG storage boundary
//
//  CG RAM is 131,072 x 16 bit = 256 KiB, written by the 68000 and the blitter
//  and read by the tilemap, sprite and ROZ engines through one shared port.
//  On this board it is one true-dual-port array; on a device that cannot spare
//  256 M10K blocks it has to live off-chip.  This module is the seam between
//  those two facts, so that "where the CG bytes are" is one decision in one
//  file rather than a change spread through verified renderer code.
//
//  Two backends, chosen by a parameter.  Neither carries a word of platform
//  vocabulary: the external port below talks about rows of CG, not about any
//  particular memory part.  The part-specific controller lives in the target.
//
//    EXTERNAL = 0   on-chip true dual port.  What this board has always done.
//                   r_stall and a_stall are constant 0 and fold away, so the
//                   MiSTer netlist is unchanged.  This is the default.
//
//    EXTERNAL = 1   the bytes live in an external store reached through xm_*.
//                   Two 4-word row buffers stand in front of it.
//
//  THE CONTRACT TOWARDS THE BOARD
//
//    port A   68000 / blitter.  Read and write, byte enables, data one clock
//             after the address -- unless a_stall is high, which means "this
//             access has not happened yet, hold the bus".
//
//    port B   renderer.  Read only, data one clock after the address -- unless
//             r_stall is high, which means "the word for the address you are
//             presenting is not ready; hold everything".
//
//  WHY ROWS OF FOUR
//
//  A CG tile is 32 words: 8 rows of 4.  Every renderer engine reads exactly
//  one 4-word row at a time and consumes all four -- na2_tilemap walks
//  cg_base+0..3 in states L0..L3, na2_sprite does the same in S_C0..S_C3, and
//  na2_roz picks one word out of a row per pixel.  Four words is therefore the
//  natural transfer, and anything larger fetches bytes nobody asked for.
//
//  A STALL IS NOT A LOSS BY ITSELF
//
//  The renderer fills a line buffer and has until the sprite deadline at
//  H_TOTAL-16 to do it; what reaches the screen is decided by whether the line
//  finished, not by whether anybody waited.  A backend may stall freely as long
//  as it still supplies every row before that deadline.  na2_video's
//  dbg_line_overrun (st_video[14]) is the instrument that says whether it did,
//  and na2_sprite's dbg_overflow says whether the sprite list was cut short.
//
//  COHERENCY
//
//  The rule is: a read sees every write accepted before it.  That is kept two
//  ways.  Writes are blocking -- a_stall is held until the store has taken the
//  word -- so no write is ever in flight behind a later read.  And a write to a
//  row that is sitting in a buffer updates that buffer in the same cycle it is
//  accepted, so the buffers can never go stale.  There is deliberately no write
//  FIFO: the ordering argument above is short enough to check by reading it,
//  and a FIFO would replace it with a longer one plus an overflow case.
//============================================================================
`default_nettype none

module na2_cgmem #(
    parameter int AW       = 17,   // 131,072 words
    parameter bit EXTERNAL = 1'b0,
    // Row buffers in the EXTERNAL backend.  Power of two, >= 2.  Meaningless
    // on-chip, where the block RAM answers every cycle and there is no cache.
    parameter int ROWS     = 2,
    // Rows held in the block-RAM row cache that sits between those buffers and
    // the external store.  0 disables it and the backend behaves exactly as it
    // did before the cache existed.  Power of two.  Each line is one four-word
    // row, so LINES=8192 holds 32,768 of the 131,072 CG words.
    //
    // WHY THIS EXISTS.  Measured on the Numan Athletics title screen rendered
    // through the Pocket CG path: the sprite pass wants 1,667 clocks of a
    // line's 3,184 and is cut off at 3,056, and 63 of those clocks-per-line are
    // sprite CG row fetches at 9 clocks each.  Every one of those 63 rows is a
    // row the same portraits asked for on the previous frame.  Widening the
    // register row buffers does not reach that -- 2, 4, 8 and 16 rows changed
    // the aborted-line count by 84, 84, 84, 76 -- because 16 rows is 64 words
    // against a working set of thousands.  Block RAM is where a working set
    // that size can live.
    parameter int CLINES   = 0,
    // Act on the prefetch hint.  Off, the port and its N-way compare disappear
    // -- which at 99 % ALM is a real amount of room, and the question is
    // whether that room buys more as row buffers.
    parameter bit PREFETCH = 1'b1,
    // Candidate direct-map hash: distribute each of the two tag bits over
    // selected index bits.  Zero masks preserve the straight low-bit index.
    // Words per cache/row-buffer line, as a log2.  2 = four words = one
    // PSRAM burst as shipped; 3 = eight words, which covers a row and its
    // sibling in a single burst.  FA reads the sibling of 18,496 of its
    // 18,586 distinct rows -- 99 % -- so an eight-word line halves the misses
    // that pass through, at the cost of a longer burst each time.  Which way
    // that trades is a measurement, not an argument.
    // WRITE_FIRST -- serve a pending port-A write before the renderer's read.
    //
    // The default is reads first, which is right for the store the renderer
    // depends on: a late read misses the line deadline, a late write only
    // makes the 68000 wait.  It is wrong for a DUPLICATE store.
    //
    // With CG_SPLIT_ROZ the ROZ store holds a second copy and na2_top's port-A
    // acknowledge waits for both stores, so a write that cannot start parks
    // the CPU with the video grant still held -- and ROZ needs that grant to
    // fetch the VRAM that would let its read finish.  Each waits for the
    // other.  On hardware that is the watchdog firing during the boot ROM
    // check, over and over, which is what build 35 did.
    //
    // Measured on em3/f880 at the shipped parameters: 192,641 clocks of port-A
    // stall in one frame against 39,429 before the acknowledge was widened --
    // 27 % of the frame with the CPU held.  Reads first is not survivable on
    // this store.  The ROZ read can afford the ten clocks a write costs,
    // because ROZ is the only reader of this copy.
    parameter bit WRITE_FIRST = 1'b0,
    // CACHE HIT WITHOUT THE EXTRA STALL CYCLE.
    //
    // A row-buffer miss that the block-RAM cache can answer costs two stall
    // cycles: one in S_IDLE noticing the miss and issuing the lookup, one in
    // S_LOOK while the tag and data come back and the row buffer is filled.
    // The renderer only sees the line on the third cycle, because r_hit is
    // computed from the registered row buffer.
    //
    // The data is already there in S_LOOK.  Handing the renderer its word
    // straight out of c_data_q in that cycle, and dropping r_stall with it,
    // removes one of the two.  Measured on emrot/f720 (DEBUG_LOG O-ROZ2):
    // the ROZ pass takes 304 row-buffer misses a line -- one per pixel,
    // because a rotated sample has no row-buffer locality -- so this is one
    // cycle per ROZ pixel, not one per fetch.
    //
    // EXTERNAL only.  The on-chip branch never stalls, so MiSTer cannot see
    // this parameter at all; it exists so the sweep can A/B it.
    parameter bit LOOK_BYPASS = 1'b1,
    // POSTED PORT-A WRITES.  0 = blocking, which is what this module has always
    // done and what the shared store must keep doing.
    //
    // WHAT IT IS FOR.  With CG_SPLIT_ROZ there are two stores holding the same
    // words, and na2_vram wires only the shared one's a_stall to the board
    // (`.a_stall()`, deliberately -- waiting for both parks the CPU for 26 % of
    // a frame and build 35 watchdogged on it).  So a write this store cannot
    // take in the cycle it is offered is simply LOST: 3,374 of 131,072 words
    // measured on emrot/f720.  And WRITE_FIRST, which exists to shorten that
    // window, pays for it out of the renderer -- every CPU write preempts the
    // ROZ fetch, 62 times a line during a CG upload, which is the same
    // magnitude as the deficit that costs a line its sprites.
    //
    // A queue answers both at once, which is why it is one change and not two:
    // the write is accepted the cycle it is offered (nothing is lost) and
    // drained when the renderer is not asking (nothing is preempted).
    //
    // WHAT IT COSTS IN CORRECTNESS.  A posted write is visible to the renderer
    // late.  That is not a defect -- the CPU and the video hardware are not
    // synchronised on real hardware either, and MAME draws the frame-end
    // snapshot -- but it does mean the old gate cannot judge this.  Comparing
    // the word the renderer takes against a golden model updated at ACCEPT
    // time scores the posting itself as error, which is why three different
    // designs scored identically to the digit (DEBUG_LOG O-ROZ10).  The gate
    // is sim/pocket/run_store_gate.sh: the end-of-frame picture and the
    // end-of-frame store contents.
    //
    // WHAT IT COSTS IN LOGIC.  AW+18 bits an entry.  dbg_q_drop is how the
    // depth is chosen: nobody listens to this store's a_stall, so a write the
    // queue refuses is lost exactly as before.  Non-zero means too shallow.
    parameter int WRITE_QUEUE = 0,
    // How full the queue gets before a drain is allowed to preempt the
    // renderer.  0 = half full.  WRITE_QUEUE itself means "only when the
    // next write would otherwise be dropped", which is the least
    // preemption a queue that must not lose a word can do.
    //
    // THIS IS THE PARAMETER THAT DECIDES WHETHER A QUEUE IS WORTH IT, and
    // it has to be separate from the depth because the two are different
    // questions.  Depth asks how much traffic can be held back; this asks
    // what it costs the renderer to hold it.  Measured on emrot/f720 at
    // depth 8 and half full: aborted lines went 8 -> 27, worse than the
    // blocking store it replaced, while q_drop stayed 0 -- so the queue was
    // keeping every word and paying for it out of exactly the budget the
    // defect is made of.  A depth sweep alone would have read that as
    // "queues do not work".
    parameter int WRITE_QUEUE_URGENT = 0,
    // A PORT-A WRITE UPDATES THE CACHE LINE INSTEAD OF THROWING IT AWAY.
    //
    // Today a write invalidates the row it lands on, so the next renderer
    // read of that row is a miss and pays a full external fetch.  During a
    // CG upload that is 62 forced misses a line at about 22 clocks each,
    // out of a line that has roughly 160 clocks of slack.  Measured going
    // the other way: giving the store a queue so that no write is dropped
    // any more cost 554 cache hits on emrot/f720 and took aborted lines
    // from 8 to 13 -- the queue was not slow, it was CORRECT, and being
    // correct meant performing the invalidations the shipped store skips
    // by losing the write.
    //
    // The data to keep the line is already in hand: c_data_q was read for
    // the tag compare.  Patching the written word into it and writing the
    // line back leaves the cache correct AND warm, which is strictly
    // better than either -- provided the lookup was aimed at the write,
    // which is the second half of this parameter.  See c_rd_idx.
    parameter bit CACHE_WRITE_THROUGH = 1'b0,
    // Aim the cache lookup at the write without changing what the write
    // then does to the line.  Implied by CACHE_WRITE_THROUGH, which cannot
    // work without it.  It exists on its own so the two halves can be told
    // apart: aiming makes the invalidation CORRECT, which should cost hits;
    // the write-through makes it unnecessary, which should save them.  One
    // parameter carrying both would have reported the sum and named it
    // after whichever half was written first.
    parameter bit CACHE_LOOKUP_AIM = 1'b0,
    parameter int LINE_LOG = 2,
    parameter [31:0] INDEX_XOR_HI = 32'd0,
    parameter [31:0] INDEX_XOR_LO = 32'd0
) (
    input  wire            clk,
    input  wire            rst,

    // --- port A: 68000 / blitter -----------------------------------------
    input  wire [AW-1:0]   a_addr,
    input  wire [15:0]     a_din,
    output wire [15:0]     a_dout,
    input  wire            a_we,
    input  wire [1:0]      a_ds,        // {upper byte, lower byte}
    input  wire            a_sel,       // this access is for CG (read or write)
    output wire            a_stall,     // hold the bus, not done yet

    // --- port B: renderer, read only --------------------------------------
    input  wire [AW-1:0]   r_addr,
    output wire [15:0]     r_data,
    output wire            r_stall,

    // --- port B prefetch hint ---------------------------------------------
    // "I will ask for this row shortly."  It never stalls anyone and never
    // returns data: it only lets the backend start the fill in cycles the
    // renderer was not going to use the port, so the read that follows is a
    // hit.  Ignored on-chip, where nothing is ever late.
    input  wire [AW-1:0]   pf_addr,
    input  wire            pf_req,

    // --- external CG store, only driven when EXTERNAL ---------------------
    output wire            xm_req,      // hold until xm_ack
    output wire            xm_we,       // 1 = write one word, 0 = fetch a row
    output wire [AW-1:0]   xm_addr,     // word address; a fetch uses [AW-1:LINE_LOG]
    output wire [15:0]     xm_wdata,
    output wire [1:0]      xm_wstrb,
    input  wire            xm_ack,      // transaction complete this cycle
    input  wire [(16<<LINE_LOG)-1:0] xm_rdata,  // line words, word 0 in [15:0]

    // --- observation only, never gates anything ---------------------------
    output wire [15:0]     dbg_row_req,     // rows asked of the store
    output wire [15:0]     dbg_row_miss,    // renderer reads that had to wait
    output wire [15:0]     dbg_max_wait,    // longest single wait, in clocks
    // Port-A writes the queue could not accept.  Always 0 when WRITE_QUEUE=0,
    // where a write cannot be refused because it is never posted.  Non-zero
    // means writes were dropped -- on the ROZ store nobody is listening to
    // a_stall, so a refused write is a lost word, not a stalled one.
    output wire [15:0]     dbg_q_drop
);

  localparam int LW   = LINE_LOG;        // address bits inside a line
  localparam int LWD  = 16 << LW;        // line width in bits
  localparam int TW   = AW - LW;         // row tag width
  localparam int RSEL = (ROWS <= 2) ? 1 : (ROWS <= 4) ? 2 : (ROWS <= 8) ? 3
                      : (ROWS <= 16) ? 4 : (ROWS <= 32) ? 5 : 6;
  // Index and tag split for the block-RAM cache.  CIDX counts the index bits.
  localparam int CIDX = (CLINES <= 1)     ? 1 : (CLINES <= 2)     ? 1
                      : (CLINES <= 4)     ? 2 : (CLINES <= 8)     ? 3
                      : (CLINES <= 16)    ? 4 : (CLINES <= 32)    ? 5
                      : (CLINES <= 64)    ? 6 : (CLINES <= 128)   ? 7
                      : (CLINES <= 256)   ? 8 : (CLINES <= 512)   ? 9
                      : (CLINES <= 1024)  ? 10 : (CLINES <= 2048) ? 11
                      : (CLINES <= 4096)  ? 12 : (CLINES <= 8192) ? 13 : 14;
  localparam int CTAG = TW - CIDX;       // tag bits kept per line

  generate
  if (!EXTERNAL) begin : g_onchip

    na2_dpram #(.AW(AW)) u_ram (
        .clk    (clk),
        .a_addr (a_addr),
        .a_din  (a_din),
        .a_dout (a_dout),
        .a_we   (a_we),
        .a_ds   (a_ds),
        .b_addr (r_addr),
        .b_dout (r_data)
    );

    // Block RAM is never late.
    assign r_stall = 1'b0;
    assign a_stall = 1'b0;

    assign xm_req   = 1'b0;
    assign xm_we    = 1'b0;
    assign xm_addr  = '0;
    assign xm_wdata = 16'd0;
    assign xm_wstrb = 2'b00;

    assign dbg_row_req  = 16'd0;
    assign dbg_row_miss = 16'd0;
    assign dbg_max_wait = 16'd0;
    assign dbg_q_drop   = 16'd0;

    // pf_addr and pf_req mean nothing here: block RAM is never late, so there
    // is nothing to start early.
    //
    // xm_ack, xm_rdata, a_sel and rst are genuinely unused in this backend.
    // They are NOT tied into a dummy `_unused` wire: that trick silences one
    // warning by creating another ("assigned but never read"), which is how
    // this file picked up two of its own.

  end else begin : g_external

    // ---- row buffers, ROWS of them --------------------------------------
    //
    // Two was a starting point, not a decision, and it is shared by four
    // tilemap layers, the ROZ engine and the sprite engine.  A line of
    // repeated background tiles refetches the same row over and over.
    // Measured cost: the render FSM used 3,065 of a line's 3,184 clocks,
    // leaving 932 where the sprite pass needs 941, and 84 lines of 224 lost
    // their sprites.
    reg            rb_val [0:ROWS-1];
    reg [TW-1:0]   rb_tag [0:ROWS-1];
    reg [15:0]     rb_w   [0:ROWS-1][0:(1<<LW)-1];
    reg [RSEL-1:0] rb_next;             // round robin; which to replace next

    wire [TW-1:0] r_row = r_addr[AW-1:LW];
    wire [LW-1:0] r_sel = r_addr[LW-1:0];
    wire [TW-1:0] a_row = a_addr[AW-1:LW];
    wire [LW-1:0] a_sub = a_addr[LW-1:0];

    // One-hot hit vectors and the index of the hit.  ROWS is small and a power
    // of two, so this is a comparator per row and a priority encoder -- the
    // same shape the two-row version had, written for N.
    wire [ROWS-1:0] r_hit_v, a_hit_v;
    genvar gi;
    for (gi = 0; gi < ROWS; gi = gi + 1) begin : g_tags
      assign r_hit_v[gi] = rb_val[gi] && (rb_tag[gi] == r_row);
      assign a_hit_v[gi] = rb_val[gi] && (rb_tag[gi] == a_row);
    end

    wire r_hit = |r_hit_v;
    wire a_hit = |a_hit_v;

    function automatic [RSEL-1:0] first_set(input [ROWS-1:0] v);
      integer fi;
      begin
        first_set = {RSEL{1'b0}};
        for (fi = ROWS-1; fi >= 0; fi = fi - 1)
          if (v[fi]) first_set = fi[RSEL-1:0];
      end
    endfunction

    wire [RSEL-1:0] r_hit_i = first_set(r_hit_v);
    wire [RSEL-1:0] a_hit_i = first_set(a_hit_v);

    // Registered one clock behind the address, exactly as the block RAM was.
    reg [15:0] r_data_r, a_data_r;
    assign r_data = r_data_r;
    assign a_dout = a_data_r;

    // =====================================================================
    // Block-RAM row cache
    //
    // Direct mapped, one four-word row per line, write-invalidate.  It sits
    // behind the register row buffers, so the three-of-four words that hit a
    // buffer still cost nothing; what it shortens is the fourth.
    //
    // Valid bits cannot be cleared in one cycle in block RAM, so after reset
    // the cache walks every line writing valid=0 and reports miss until it is
    // done.  That is CLINES clocks, once, while the board is still in reset.
    // =====================================================================
    localparam int CN = (CLINES < 1) ? 1 : CLINES;   // never declare [0:-1]

    reg [LWD-1:0]   c_data [0:CN-1];
    reg [CTAG:0]    c_tag  [0:CN-1];      // {valid, tag}
    reg [CIDX-1:0]  c_init_i;
    reg             c_init_done;
    reg [LWD-1:0]   c_data_q;
    reg [CTAG:0]    c_tag_q;
    reg [CTAG-1:0]  c_look_tag;

    // Index with the two high row/tag bits XORed into the set number.
    //
    // The straight low-bit index is the worst case for ROZ: a rotated layer
    // walks CG with a stride, and a stride that shares factors with CLINES
    // maps a whole run onto a handful of lines while the rest of the cache
    // sits idle.  Doubling the cache hid that -- 8192 -> 16384 took ROZ stall
    // from 1,371 to 692 and made the external path byte-identical -- but the
    // second 512 Kbit does not fit (fitter: needs more than 308 M10K).
    //
    // Folding costs a row of XOR gates and no memory.  If most of the win was
    // conflict rather than capacity, this recovers it for free; the sweep says
    // which, and it is not worth arguing about beforehand.
    function automatic [CIDX-1:0] cfold(input [TW-1:0] row);
      reg [CIDX-1:0] idx;
      begin
        idx = row[CIDX-1:0];
        // Slice the masks to the index width.  They are ints, and an int
        // ANDed with a CIDX-wide value is a 32-bit expression assigned to a
        // CIDX-wide target -- correct in value, and four 10230 truncation
        // warnings in a file the Pocket baseline records as zero.
        idx = idx ^ ({CIDX{row[TW-1]}} & INDEX_XOR_HI[CIDX-1:0]);
        idx = idx ^ ({CIDX{row[TW-2]}} & INDEX_XOR_LO[CIDX-1:0]);
        cfold = idx;
      end
    endfunction

    wire [CIDX-1:0] r_cidx = cfold(r_row);
    wire [CTAG-1:0] r_ctag = r_row[TW-1:CIDX];
    wire [CIDX-1:0] a_cidx = cfold(a_row);
    wire [CTAG-1:0] a_ctag = a_row[TW-1:CIDX];

    // The hint, and whether it is worth acting on.  A hint for a row already in
    // a buffer is dropped; there is nothing to fetch.
    wire [TW-1:0]   p_row = pf_addr[AW-1:LW];
    wire [ROWS-1:0] p_hit_v;
    for (gi = 0; gi < ROWS; gi = gi + 1) begin : g_ptags
      assign p_hit_v[gi] = rb_val[gi] && (rb_tag[gi] == p_row);
    end
    wire p_want = PREFETCH && pf_req && !(|p_hit_v);

    // Where the next fill lands.  Named because the back-to-back path below
    // needs it in the same cycle it retires the current one.
    wire [RSEL-1:0] nx_fill = (x_fill_sel == (ROWS-1)) ? {RSEL{1'b0}}
                                                       : x_fill_sel + 1'b1;
    wire            c_look_hit = c_tag_q[CTAG] && (c_tag_q[CTAG-1:0] == c_look_tag);

    // ---- the one transaction in flight ----------------------------------
    // S_LOOK is the block-RAM lookup: one clock for the tag and data to come
    // back, then either the row buffer is filled from the cache or the
    // external store is asked, exactly as before.
    localparam [1:0] S_IDLE = 2'd0, S_RD = 2'd1, S_WR = 2'd2, S_LOOK = 2'd3;
    reg [1:0]      st;
    reg [AW-1:0]   x_addr;
    reg [CIDX-1:0] x_cidx;
    reg [CTAG-1:0] x_ctag;
    reg            x_for_a;    // this fetch is for port A, do not cache it
    reg            x_for_r;    // ...and this one is the renderer's, so the
                               // S_LOOK bypass may answer it.  A prefetch
                               // also clears x_for_a and must NOT: it is
                               // asking about a row the renderer is not on.
    reg [15:0]     x_wdata;
    reg [1:0]      x_wstrb;
    integer        fw;
    reg [RSEL-1:0] x_fill_sel;          // buffer the fetch lands in
    reg            x_inv_pending;

    // S_LOOK is internal: nothing is asked of the external store during it.
    assign xm_req   = (st == S_RD) || (st == S_WR);
    assign xm_we    = (st == S_WR);
    assign xm_addr  = x_addr;
    assign xm_wdata = x_wdata;
    assign xm_wstrb = x_wstrb;

    // The renderer has a deadline and the 68000 does not, so a renderer miss
    // is served first.  A write already in flight still finishes first --
    // there is only ever one transaction -- which is what keeps ordering.
    wire want_r = !r_hit;

    // The cache answered in S_LOOK and the line is the renderer's, so its
    // word goes out of c_data_q now instead of out of the row buffer one
    // cycle later.  See LOOK_BYPASS at the top of the module.
    wire look_ans_r = LOOK_BYPASS && (st == S_LOOK) && x_for_r && c_look_hit;

    assign r_stall = want_r && !look_ans_r;

    // A WRITE HAS TO BE ABLE TO FINISH.
    //
    // This used to be
    //
    //     wire want_a  = a_sel && (a_we ? 1'b1 : !a_hit);
    //     assign a_stall = a_sel && (want_a || (st != S_IDLE) || want_r);
    //
    // and want_a is 1 for every write, unconditionally, so for a write the
    // stall had no path to zero.  na2_top acknowledges only when the stall is
    // low (vid_ack_r <= ... & ~cg_a_stall) and its arbiter holds the grant
    // until that acknowledge, so the first CG write by the 68000 or the
    // blitter parked the board permanently.  The word did reach the store and
    // was then rewritten every few cycles, which is why the symptom was a
    // black screen rather than a corrupt one: CG RAM never advanced past the
    // first address, so every tilemap stayed empty.
    //
    // This only ever existed with EXTERNAL=1.  The on-chip branch ties a_stall
    // to 0, which is why MiSTer was never affected and why nothing here was
    // suspected for three hardware rounds.
    //
    // The release condition is the transaction completing, not a "want" flag:
    //   write -- the cycle the store takes it
    //   read  -- the buffers hold the word and the store is free
    // The read term is exactly the old expression rewritten, so reads are
    // unchanged.  sim/pocket/tb_na2_cgmem_ext.sv is the test that was missing.
    //
    // a_wr_done is the seam: `(st == S_WR) && xm_ack` when writes are
    // blocking, and "the queue has it" when they are posted.
    wire a_done = a_sel && (a_we ? a_wr_done
                                 : (a_hit && (st == S_IDLE) && !want_r));
    assign a_stall = a_sel && !a_done;

    // na2_top registers its acknowledge from a_stall, so the grant is released
    // one cycle AFTER the stall falls -- by which time st is back in S_IDLE and
    // a_sel is still high.  Without this the same write starts a second time.
    reg a_wr_just;

    // =====================================================================
    // WHERE THE NEXT PORT-A WRITE COMES FROM
    //
    // One seam with two implementations, so the FSM below is written once.
    // WRITE_QUEUE=0 resolves every signal here back to the expression it
    // replaced, so that configuration is the shipped netlist unchanged -- and
    // it has to be, because it is the control every sweep is read against.
    //
    // The parameter split is a GENERATE, not a condition inside a procedural
    // block.  `always @(*) if (WRITE_QUEUE > 0) ...` folds at elaboration and
    // takes the signal references in the condition with it, so the block never
    // runs at all and its outputs stay X.  That happened here on 2026-09-07
    // and only the control column caught it, because the control was the
    // configuration reporting 224 lines overrunning out of 224 (O-ROZ8).
    // =====================================================================
    wire            w_valid;    // there is a write for the store to do
    wire [AW-1:0]   w_addr;
    wire [15:0]     w_data;
    wire [1:0]      w_ds;
    wire            w_urgent;   // do it BEFORE serving the renderer
    wire            a_wr_done;  // port A may retire its write now
    wire            q_push;     // a write is entering the queue this cycle
    wire [TW-1:0]   w_row = w_addr[AW-1:LW];
    wire [LW-1:0]   w_sub = w_addr[LW-1:0];

    // The FSM takes a write in S_IDLE when it is urgent, or when the renderer
    // is not asking for anything.  Written here because the queue's pop needs
    // it and it must be the same expression the case statement uses.
    wire w_take = (st == S_IDLE) && w_valid && (w_urgent || !want_r);

    if (WRITE_QUEUE > 0) begin : g_wq
      localparam int QN = WRITE_QUEUE;
      localparam int QW = (QN <= 1) ? 1 : (QN <= 2) ? 1 : (QN <= 4) ? 2
                        : (QN <= 8) ? 3 : (QN <= 16) ? 4 : (QN <= 32) ? 5 : 6;

      reg [AW-1:0] q_addr [0:QN-1];
      reg [15:0]   q_data [0:QN-1];
      reg [1:0]    q_dsq  [0:QN-1];
      reg [QW-1:0] q_rd, q_wr;
      reg [QW:0]   q_cnt;
      reg [15:0]   q_drop_r;
      // ONE PUSH PER ASSERTION, NOT PER CYCLE.  a_sel stays high until the
      // SHARED store acknowledges, which is many cycles, and this store has
      // its own idea of when it is done.  Without this latch one CPU write
      // becomes a queue full of copies of itself.
      reg          a_wr_held;
      reg          a_sel_d;

      wire q_full  = (q_cnt == QN[QW:0]);
      wire q_empty = (q_cnt == {(QW+1){1'b0}});

      assign q_push = a_sel && a_we && !a_wr_held && !q_full;

      assign w_valid = !q_empty;
      assign w_addr  = q_addr[q_rd];
      assign w_data  = q_data[q_rd];
      assign w_ds    = q_dsq [q_rd];

      // HALF FULL IS WHEN THE WRITER STOPS BEING POLITE.
      //
      // Draining only when the renderer is idle is the whole point, but ROZ
      // misses the row buffer on every pixel of its pass -- 304 of them, one
      // per pixel, because a rotated sample has no locality (O-ROZ2) -- so
      // "idle" can be scarce for a long stretch while writes keep arriving at
      // 62 a line.  Without this the queue would have to be as deep as a whole
      // ROZ pass; with it, a shallow queue is enough and the renderer is
      // preempted only when the alternative is losing a word.  That is still
      // far less often than WRITE_FIRST, which preempts on every write.
      localparam int UAT = (WRITE_QUEUE_URGENT > 0) ? WRITE_QUEUE_URGENT
                                                    : ((QN + 1) / 2);
      assign w_urgent = (q_cnt >= UAT[QW:0]);

      // The write retires as soon as it is in the queue.  Nothing on this
      // store's a_stall is wired to the board, so this matters only to a
      // testbench that does wire it -- but it is what makes the queue a store
      // buffer rather than a delay line.
      assign a_wr_done = a_wr_held || q_push;
      assign dbg_q_drop = q_drop_r;

      always @(posedge clk) begin
        if (rst) begin
          q_rd <= {QW{1'b0}}; q_wr <= {QW{1'b0}}; q_cnt <= {(QW+1){1'b0}};
          a_wr_held <= 1'b0; a_sel_d <= 1'b0; q_drop_r <= 16'd0;
        end else begin
          a_sel_d <= a_sel && a_we;
          if (!(a_sel && a_we)) a_wr_held <= 1'b0;
          else if (q_push)      a_wr_held <= 1'b1;

          // A WRITE THAT WAS NEVER TAKEN IS A LOST WORD, NOT A LATE ONE.
          // Counted where the offer ends, not where the queue is full: a_sel
          // is held for many cycles and a full queue early in that window
          // still has time to drain.  Counting the full cycles would report
          // losses that never happened.
          if (a_sel_d && !(a_sel && a_we) && !a_wr_held)
            q_drop_r <= q_drop_r + 16'd1;

          if (q_push) begin
            q_addr[q_wr] <= a_addr;
            q_data[q_wr] <= a_din;
            q_dsq [q_wr] <= a_ds;
            q_wr <= q_wr + 1'b1;
          end
          if (w_take) q_rd <= q_rd + 1'b1;
          case ({q_push, w_take})
            2'b10:   q_cnt <= q_cnt + 1'b1;
            2'b01:   q_cnt <= q_cnt - 1'b1;
            default: ;
          endcase
        end
      end

    end else begin : g_nowq

      // Exactly the expressions the FSM used before the seam existed.
      assign q_push     = 1'b0;
      assign w_valid    = a_sel && a_we && !a_wr_just;
      assign w_addr     = a_addr;
      assign w_data     = a_din;
      assign w_ds       = a_ds;
      assign w_urgent   = WRITE_FIRST;
      assign a_wr_done  = (st == S_WR) && xm_ack;
      assign dbg_q_drop = 16'd0;

    end

    // ---- counters, observation only -------------------------------------
    reg [15:0] c_row_req, c_row_miss, c_max_wait, c_wait, c_hit, c_pf;
    assign dbg_row_req  = c_row_req;
    assign dbg_row_miss = c_row_miss;
    assign dbg_max_wait = c_max_wait;

    // The cache RAMs.  Written on a fill, on an invalidate, and during the
    // post-reset sweep; read whenever a lookup is issued.  Kept in its own
    // process so the tools infer block RAM rather than registers.
    // THE LOOKUP INDEX HAS TO MATCH WHO IS ASKING.
    //
    // This read used to be hardwired to r_cidx.  A prefetch asks about a
    // different row, so it was handed the tag and data of the renderer's line
    // instead of its own -- and when that tag happened to match, a wrong row
    // went into the buffer.  Measured: prefetch alone left 1,557 wrong pixels
    // on the title frame, prefetch plus an 8,192-line cache left 5,619, and a
    // frame that was pixel-exact with the cache alone came back with 398.
    // Two changes that are each correct can be wrong together; this is the
    // seam between them.
    // With WRITE_QUEUE=0 this is `a_sel && a_we && !a_wr_just` and cfold(w_row)
    // is a_cidx, so the expression is unchanged from what shipped.
    wire a_write_start = (st == S_IDLE) && w_valid;
    // THE LOOKUP HAS TO BE AIMED AT WHOEVER THE FSM IS ABOUT TO SERVE.
    //
    // The shipped expression aims at the renderer whenever the renderer
    // wants anything -- but with WRITE_FIRST the FSM serves the WRITE in
    // exactly that case.  So the write is issued with x_cidx set to its
    // own line while c_tag_q/c_data_q hold the RENDERER's line, the tag
    // compare in S_WR compares two unrelated lines, and the invalidation
    // it guards almost never fires.  The ROZ store runs with WRITE_FIRST
    // on, so on that store a write during a ROZ pass leaves the cache
    // holding the old row.  Not fixed in the shipped branch on purpose:
    // it is the control, and changing it would move the zero point of
    // every measurement taken against it (root CLAUDE.md 1.3).  It is
    // written down in DEBUG_LOG O-ROZ12 and measured as a candidate.
    localparam bit C_AIM = CACHE_WRITE_THROUGH || CACHE_LOOKUP_AIM;
    wire [CIDX-1:0] c_rd_idx =
        C_AIM               ? (w_take   ? cfold(w_row)
                             : want_r  ? r_cidx : cfold(p_row))
                            : (want_r  ? r_cidx
                             : a_write_start ? cfold(w_row) : cfold(p_row));

    // The cache line with the written word patched into it.  c_data_q is
    // the line the tag compare just read, so this costs a mux and no
    // extra memory port.
    wire [LWD-1:0] c_wr_merge;
    if (CACHE_WRITE_THROUGH) begin : g_cwm
      for (gi = 0; gi < (1<<LW); gi = gi + 1) begin : g_cwm_w
        wire sel_w = (gi[LW-1:0] == x_addr[LW-1:0]);
        assign c_wr_merge[gi*16   +: 8] =
            (sel_w && x_wstrb[0]) ? x_wdata[7:0]  : c_data_q[gi*16   +: 8];
        assign c_wr_merge[gi*16+8 +: 8] =
            (sel_w && x_wstrb[1]) ? x_wdata[15:8] : c_data_q[gi*16+8 +: 8];
      end
    end else begin : g_no_cwm
      assign c_wr_merge = {LWD{1'b0}};
    end

    always @(posedge clk) begin
      if (CLINES > 0) begin
        c_data_q <= c_data[c_rd_idx];
        c_tag_q  <= c_tag [c_rd_idx];
        if (!c_init_done) begin
          c_tag[c_init_i] <= {(CTAG+1){1'b0}};
        end else if ((st == S_RD) && xm_ack && !x_for_a) begin
          c_data[x_cidx] <= xm_rdata;
          c_tag [x_cidx] <= {1'b1, x_ctag};
        end else if ((st == S_WR) && x_inv_pending && c_look_hit) begin
          // Write-invalidate only the row being written.  Clearing the set
          // unconditionally also evicts an unrelated row with another tag;
          // sequential port-A writes made that false eviction frequent enough
          // to change renderer deadlines after the set hash changed.
          //
          // CACHE_WRITE_THROUGH keeps the line instead, with the written
          // word patched in.  A parameter inside a clocked block is safe --
          // the O-ROZ8 trap was an `always @(*)` losing its sensitivity when
          // the condition folded, and this block's sensitivity is the clock.
          if (CACHE_WRITE_THROUGH) begin
            c_data[x_cidx] <= c_wr_merge;
            c_tag [x_cidx] <= {1'b1, x_ctag};
          end else begin
            c_tag[x_cidx] <= {(CTAG+1){1'b0}};
          end
        end
      end
    end

    integer i;
    always @(posedge clk) begin
      if (rst) begin
        st         <= S_IDLE;
        for (i = 0; i < ROWS; i = i + 1) rb_val[i] <= 1'b0;
        rb_next    <= {RSEL{1'b0}};
        c_row_req  <= 16'd0;
        c_row_miss <= 16'd0;
        c_hit      <= 16'd0;
        c_pf       <= 16'd0;
        c_max_wait <= 16'd0;
        c_wait     <= 16'd0;
        r_data_r   <= 16'd0;
        a_data_r   <= 16'd0;
        a_wr_just   <= 1'b0;
        x_inv_pending <= 1'b0;
        c_init_i    <= {CIDX{1'b0}};
        c_init_done <= (CLINES == 0);
      end else begin
        if (!c_init_done) begin
          if (c_init_i == CN[CIDX-1:0] - 1'b1) c_init_done <= 1'b1;
          c_init_i <= c_init_i + 1'b1;
        end
        a_wr_just <= (st == S_WR) && xm_ack;
        if ((st == S_WR) && x_inv_pending) x_inv_pending <= 1'b0;

        // --- serve reads out of the buffers, registered like the BRAM was --
        if (look_ans_r)  r_data_r <= c_data_q[r_sel*16 +: 16];
        else if (r_hit)  r_data_r <= rb_w[r_hit_i][r_sel];
        if (a_hit) a_data_r <= rb_w[a_hit_i][a_sub];

        // --- longest wait, and how many renderer reads had to wait --------
        if (r_stall) begin
          c_wait <= c_wait + 16'd1;
          if (c_wait + 16'd1 > c_max_wait) c_max_wait <= c_wait + 16'd1;
        end else begin
          c_wait <= 16'd0;
        end

        case (st)
          S_IDLE: begin
            if (w_urgent && w_valid) begin
              // Blocking (WRITE_QUEUE=0): w_urgent is WRITE_FIRST and w_valid
              // is the old `a_sel && a_we && !a_wr_just`, so this is the same
              // branch it was -- on a duplicate store the write goes first, or
              // the CPU and the renderer wait for each other.
              //
              // Posted: this is the queue past half full, where the cost of
              // making the renderer wait is less than the cost of refusing a
              // word nobody will offer twice.
              x_addr  <= w_addr;
              x_wdata <= w_data;
              x_wstrb <= w_ds;
              x_for_r    <= 1'b0;
              x_cidx  <= cfold(w_row);
              x_ctag  <= w_row[TW-1:CIDX];
              c_look_tag <= w_row[TW-1:CIDX];
              x_inv_pending <= 1'b1;
              st      <= S_WR;
              for (i = 0; i < ROWS; i = i + 1) begin
                if (rb_val[i] && (rb_tag[i] == w_row)) begin
                  if (w_ds[0]) rb_w[i][w_sub][7:0]  <= w_data[7:0];
                  if (w_ds[1]) rb_w[i][w_sub][15:8] <= w_data[15:8];
                end
              end
            end else if (want_r) begin
              // Ask the cache first when there is one; the lookup was issued
              // this cycle by the RAM process above.
              x_addr     <= {r_row, {LW{1'b0}}};
              x_fill_sel <= rb_next;
              x_cidx     <= r_cidx;
              x_ctag     <= r_ctag;
              x_for_a    <= 1'b0;
              x_for_r    <= 1'b1;
              c_look_tag <= r_ctag;
              st         <= (CLINES > 0 && c_init_done) ? S_LOOK : S_RD;
              c_row_req  <= c_row_req + 16'd1;
              c_row_miss <= c_row_miss + 16'd1;
            end else if (w_valid) begin
              // Blocking: a write is blocking, so nothing can slip past it.
              // Posted: the renderer is not asking, so this cycle is free.
              x_addr  <= w_addr;
              x_wdata <= w_data;
              x_wstrb <= w_ds;
              x_for_r    <= 1'b0;
              x_cidx  <= cfold(w_row);
              x_ctag  <= w_row[TW-1:CIDX];
              c_look_tag <= w_row[TW-1:CIDX];
              x_inv_pending <= 1'b1;
              st      <= S_WR;
              // Keep the buffers true in the same breath: a row that holds
              // this word is updated, byte by byte, rather than invalidated,
              // so a renderer that is walking that row does not take a miss
              // it did not need to.
              //
              // On a posted write this ALSO repairs the buffer.  Between the
              // accept and here a miss on that row can have refilled it from
              // the backing store, which does not have the word yet -- so the
              // patch done at accept time was overwritten by stale data.  The
              // cache line is invalidated by x_inv_pending below for the same
              // reason.  Both are why a posted read can be late but not wrong.
              for (i = 0; i < ROWS; i = i + 1) begin
                if (rb_val[i] && (rb_tag[i] == w_row)) begin
                  if (w_ds[0]) rb_w[i][w_sub][7:0]  <= w_data[7:0];
                  if (w_ds[1]) rb_w[i][w_sub][15:8] <= w_data[15:8];
                end
              end
            end else if (a_sel && !a_we && !a_hit) begin
              x_addr     <= {a_row, {LW{1'b0}}};
              x_fill_sel <= rb_next;
              x_for_a    <= 1'b1;
              x_for_r    <= 1'b0;
              st         <= S_RD;
              c_row_req  <= c_row_req + 16'd1;
            end else if (p_want) begin
              // WHY THIS IS WORTH A STATE.
              //
              // Measured on the Numan Athletics title screen through the Pocket
              // CG path: 63 sprite CG row fetches on the worst line, one per
              // cell, at 9 clk_sys each -- 567 of the 3,056 clocks a line has
              // before the sprite pass is cut off.  Nothing removes those
              // fetches; every one is a different tile row.  What can be
              // removed is the WAITING, because na2_sprite spends six clocks
              // per cell writing pixels and asks the CG port for nothing in
              // four of them.
              x_addr     <= {p_row, {LW{1'b0}}};
              x_fill_sel <= rb_next;
              x_cidx     <= cfold(p_row);
              x_ctag     <= p_row[TW-1:CIDX];
              x_for_a    <= 1'b0;
              x_for_r    <= 1'b0;
              c_look_tag <= p_row[TW-1:CIDX];
              st         <= (CLINES > 0 && c_init_done) ? S_LOOK : S_RD;
              c_row_req  <= c_row_req + 16'd1;
              c_pf       <= c_pf + 16'd1;
            end
          end

          // ---- block-RAM cache lookup ------------------------------------
          S_LOOK: begin
            if (c_look_hit) begin
              rb_tag[x_fill_sel]  <= x_addr[AW-1:LW];
              rb_val[x_fill_sel]  <= 1'b1;
              for (fw = 0; fw < (1<<LW); fw = fw + 1)
                rb_w[x_fill_sel][fw] <= c_data_q[fw*16 +: 16];
              rb_next <= (x_fill_sel == (ROWS-1)) ? {RSEL{1'b0}}
                                                  : x_fill_sel + 1'b1;
              c_hit   <= c_hit + 16'd1;
              st      <= S_IDLE;
            end else begin
              st <= S_RD;
            end
          end

          // KEEP THE REQUEST UP IF THERE IS ANOTHER ONE.
          //
          // Dropping xm_req to pass through S_IDLE costs two clocks of
          // turnaround, and with the sprite prefetch running there is almost
          // always another row wanted.  Measured with requests never stopping:
          // 11 clk_ram from row to row when the request goes down between
          // them, 9 when it does not.  pkt_cg_psram takes a new address while
          // the request is still up because a different address is a different
          // request.
          S_RD: if (xm_ack) begin
            rb_tag[x_fill_sel] <= x_addr[AW-1:LW];
            rb_val[x_fill_sel] <= 1'b1;
            for (fw = 0; fw < (1<<LW); fw = fw + 1)
              rb_w[x_fill_sel][fw] <= xm_rdata[fw*16 +: 16];
            rb_next <= nx_fill;

            // The row that just landed is not in rb_val yet -- these tests use
            // the tag that is being written this same edge, so a hint for the
            // row we just fetched is not chased again.
            if (want_r && ({r_row, {LW{1'b0}}} != x_addr)) begin
              x_addr     <= {r_row, {LW{1'b0}}};
              x_fill_sel <= nx_fill;
              x_cidx     <= r_cidx;
              x_ctag     <= r_ctag;
              x_for_a    <= 1'b0;
              x_for_r    <= 1'b1;
              c_look_tag <= r_ctag;
              c_row_req  <= c_row_req + 16'd1;
              c_row_miss <= c_row_miss + 16'd1;
              st         <= S_RD;
            end else if (p_want && ({p_row, {LW{1'b0}}} != x_addr)) begin
              x_addr     <= {p_row, {LW{1'b0}}};
              x_fill_sel <= nx_fill;
              x_cidx     <= cfold(p_row);
              x_ctag     <= p_row[TW-1:CIDX];
              x_for_a    <= 1'b0;
              x_for_r    <= 1'b0;
              c_look_tag <= p_row[TW-1:CIDX];
              c_row_req  <= c_row_req + 16'd1;
              c_pf       <= c_pf + 16'd1;
              st         <= S_RD;
            end else begin
              st <= S_IDLE;
            end
          end

          S_WR: if (xm_ack) st <= S_IDLE;

          default: st <= S_IDLE;
        endcase

        // ---- accept-time visibility, posted writes only ------------------
        //
        // q_push is constant 0 when WRITE_QUEUE=0 and this whole block folds
        // away, so the blocking store is untouched.
        //
        // A posted write is allowed to reach the backing store late; it is not
        // allowed to make the renderer read the OLD word out of a buffer that
        // is sitting right here holding it.  Patching on accept costs nothing
        // and shortens the stale window from "until the queue drains" to zero
        // for every row the renderer is currently walking -- which is the only
        // row that can be read soon enough to matter.
        //
        // AFTER the case on purpose.  A fill landing in the same cycle writes
        // the whole line from the backing store, which does not have this word
        // yet; last assignment wins in a non-blocking block, and the value that
        // has to win is the new one.
        if (q_push) begin
          for (i = 0; i < ROWS; i = i + 1) begin
            if (rb_val[i] && (rb_tag[i] == a_row)) begin
              if (a_ds[0]) rb_w[i][a_sub][7:0]  <= a_din[7:0];
              if (a_ds[1]) rb_w[i][a_sub][15:8] <= a_din[15:8];
            end
          end
        end
      end
    end

  end
  endgenerate

endmodule

`default_nettype wire
