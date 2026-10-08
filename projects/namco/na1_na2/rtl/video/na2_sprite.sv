//============================================================================
//  Namco NA-2 -- sprites, one scanline
//
//  Runs after every tilemap for the line, exactly where MAME's screen_update()
//  calls draw_sprites().  Written from the pseudocode in RESEARCH.md section
//  10b, which is transcribed from MAME draw_sprites() / pdraw_tile().
//
//  This module is a pure *producer*: it emits candidate pixel pairs and lets
//  na2_video decide whether they land.  The two rules that decide come from
//  pdraw_tile and neither of them is local to one sprite:
//
//      if (pri[x] <= priority)  dest[x] = pen;      // versus the background
//      pri[x] = 0xff;                               // whether it drew or not
//
//  The second line is the one that is easy to miss: a sprite that loses to the
//  background still blocks every later sprite at that pixel, so sprite 0 always
//  beats sprite 1 and the priority field only ever settles sprite-versus-
//  background.  na2_video keeps the occupancy bit and does the comparison.
//
//  Per line, in two passes:
//
//    1. Evaluation, one sprite issued per clock.  Word 0 alone decides whether
//       a sprite touches this line -- it carries the height as well as the y --
//       so this is 256 reads and not 1024.  The cell row that covers the line
//       is closed-form (RESEARCH.md 10b): every row of a sprite has the same
//       residue mod 8, so exactly one row delta mod 64 can cover a given line.
//       Hits go into a list of at most MAXHIT entries.
//
//    2. Drawing.  For each hit: read words 1..3 (5 clocks), then one 8-pixel
//       cell per column, two pixels per clock, six clocks per cell.
//
//       Six and not eight because the shape read for the *next* column is
//       issued underneath this column's pixel writes, where the shape port is
//       idle, and lands exactly when the next column starts.  The mask is then
//       never registered: shape_addr does not move again until the next
//       column's fourth clock, so shape_data is stable across the whole cell.
//       The same trick as na2_tilemap's six-clock group.
//
//  Budget: 258 + hits * (5 + 6 * width) clocks of the 3184 in a line, against
//  about 1420 already spent on the clear, four tilemap layers and ROZ.  Two things bound it: MAXHIT
//  caps the hit list, and na2_video raises `abort` when the line runs out of
//  time.  Either one sets `dbg_overflow`, because a silently short sprite list
//  looks exactly like a sprite that is missing for some subtler reason.
//
//  UNKNOWN: what the real hardware's per-line sprite limit is.  Both emulators
//  draw all 256 sprites in zero time, so neither says.  The limit here is a
//  consequence of the line budget, not a model of the PCB.
//
//  APPROXIMATION -- MAXHIT = 128 (64 until 2026-10-06).  Sized to what the
//  SOFTWARE asks for, measured over 6000 frames of every shipped set with
//  tools/spr_peak.lua: Tinkle Pit 112, the four Emeraldia sets 72, every other
//  set 43 or less.  64 dropped sprites in five shipped sets (KNOWN_ISSUES
//  RISK-9); Emeraldia's high-score table lost visible text against MAME.
//  Above roughly 125 width-1 hits the line deadline binds before this list
//  does, so a longer list buys nothing.  The one set that asks for more,
//  Bakuretsu Quiz Ma-Q, does it only while its sprite RAM is uninitialised at
//  power-on (all 256 entries parked on one line, frames 92-599).  Worst
//  shipped line measured against the deadline: tinklpit f1157, 112 hits, the
//  pass ends at hpos 370 of the 382 allowed.  hit_n[6:0] below is what caps
//  MAXHIT at 128 -- widen the index before going higher.
//
//  Read latency: every address here is a register, so an address assigned in
//  cycle T reaches the memory port in T+1 and its data is back in T+2.  That
//  two-cycle shape is the same one na2_tilemap uses.
//
//  Two different shadows, both from pdraw_tile (MAME_VERIFIED):
//
//    colour 0x0F (8bpp) or 0xFF (4bpp)   dest[x] = shadow_table[dest[x]]
//    any other shadow-flagged sprite     dest[x] = pal_base + c + 0x1000
//
//  The second is pen bit 12, which na2_palette decodes as the alternate
//  reading.  The first darkens what is already on the line and carries no pen
//  of its own: `px_hwshadow` says so and na2_video turns it into a flag in the
//  line buffer, leaving the pen underneath in place.
//
//  UNKNOWN: whether the real hardware darkens by MAME's 0.6.  What is
//  measured is that MAME does -- a frame of the attract mode disagreed with
//  this renderer on 422 pixels, every one of them on a sprite with colour
//  word 0x7FFE, until this was implemented (tools/mame_check.sh).
//============================================================================
`default_nettype none

module na2_sprite #(
    parameter int H_VISIBLE = 304,
    parameter int MAXHIT    = 128,   // at most 128: see the header
    // A zero shape mask produces no candidate pixels, so an external-CG target
    // can skip that cell before issuing any CG read.  Off by default to retain
    // the established MiSTer schedule exactly.
    parameter bit SKIP_EMPTY = 1'b0
) (
    input  wire        clk,
    input  wire        rst,

    // --- job ---------------------------------------------------------------
    input  wire        prepare,        // evaluate before the draw slot
    input  wire        start,          // one-cycle pulse
    output reg         busy,
    input  wire        abort,          // the line is out of time; stop now
    input  wire [8:0]  line,           // raw scanline
    input  wire        bank,           // vreg $22 bit 0: alternate sprite RAM

    // --- sprite RAM read port (word addressed, 2048 words) -----------------
    output reg  [10:0] spr_addr,
    input  wire [15:0] spr_data,

    // --- CG / shape read ports --------------------------------------------
    output reg  [16:0] cg_addr,
    // The row this engine will ask for next, and a flag saying so.  A hint
    // only: it never returns data and it must never stall this engine, which
    // is why it cannot ride on cg_addr -- cg_stall is derived from that.
    // na2_cgmem starts the fill in a cycle nothing else wants the port, so the
    // read that follows is a hit instead of a nine-clock wait.
    output wire [16:0] cg_pf_addr,
    output wire        cg_pf_req,
    input  wire [15:0] cg_data,
    // The CG backend is not ready with cg_data.  Held BELOW the abort test on
    // purpose: the scanline deadline keeps running while the engine waits, so
    // a line that runs out of time still drops its remaining sprites rather
    // than hanging.  na2_cgmem.sv.
    input  wire        cg_stall,
    output reg  [13:0] shape_addr,
    input  wire [15:0] shape_data,

    // --- candidate pixels, two per clock, always adjacent ------------------
    output reg         px_a,
    output reg  [8:0]  px_xa,
    output reg  [12:0] px_pena,
    output reg         px_b,
    output reg  [8:0]  px_xb,
    output reg  [12:0] px_penb,
    output reg  [2:0]  px_pri,
    output reg         px_hwshadow,   // darken what is there, do not write a pen

    // --- diagnostics -------------------------------------------------------
    output reg  [7:0]  dbg_hits,       // cells' worth of sprites on the last line
    output reg         dbg_overflow    // a line needed more than MAXHIT
);

  localparam int NSPR = 256;

  // =====================================================================
  // Hit list, filled by the evaluation pass
  // =====================================================================
  reg [7:0] h_spr  [0:MAXHIT-1];   // sprite index
  reg [2:0] h_row  [0:MAXHIT-1];   // cell row, after the flipy fold
  reg [2:0] h_srow [0:MAXHIT-1];   // source line inside the cell
  reg [7:0] hit_n;
  reg [7:0] hit_i;

  // =====================================================================
  // Evaluation
  // =====================================================================
  reg [8:0] ev_i;                  // index being issued, 0..256
  reg [1:0] ev_v;                  // validity of the two reads in flight
  reg [7:0] ev_ix1, ev_ix2;        // and their indices

  wire [15:0] ypos    = spr_data;
  wire [3:0]  e_hgt   = {1'b0, ypos[14:12]} + 4'd1;         // 1..8
  wire        e_flipy = ypos[15];
  wire [8:0]  e_raw   = ypos[8:0] + 9'd2;                   // -30 + 32
  wire [8:0]  e_m     = e_raw + 9'd7 - line;                // 9-bit wrap
  wire [5:0]  e_d     = 6'd0 - e_m[8:3];                    // -(m >> 3) mod 64
  wire        e_hit   = (e_d < {2'd0, e_hgt});
  wire [2:0]  e_yin   = 3'd7 - e_m[2:0];
  wire [3:0]  e_row4  = e_hgt - 4'd1 - {1'b0, e_d[2:0]};    // 4-bit: height can be 8
  wire [2:0]  e_row   = e_flipy ? e_row4[2:0] : e_d[2:0];
  wire [2:0]  e_srow  = e_flipy ? (3'd7 - e_yin) : e_yin;

  // =====================================================================
  // The sprite being drawn
  // =====================================================================
  reg  [11:0] d_tile;
  reg         d_opaque;
  reg  [15:0] d_color;
  reg  [8:0]  d_xbase;
  reg  [2:0]  d_row, d_srow;
  reg  [1:0]  d_col;

  wire [1:0]  d_wm1    = d_color[13:12];        // width - 1
  wire        d_flipx  = d_color[15];
  wire        d_shadow = d_color[14];
  wire        d_bpp4   = d_color[3];
  wire [2:0]  d_pri    = d_color[2:0];

  // MAME_VERIFIED, pdraw_tile: the hardware shadow table is selected only when
  // the colour handed to it is exactly 0x0F in 8bpp or 0xFF in 4bpp, where
  // that colour is palbase -- d_color[7:4] in 8bpp, {d_color[7:4],
  // d_color[11:8]} in 4bpp.
  // MAME_VERIFIED, and this is the order that matters: pdraw_tile() tests
  // bOpaque FIRST, and that branch never looks at bShadow.  An opaque sprite
  // that also carries the shadow bit is painted normally; it does not darken
  // what is underneath and it does not take the alternate palette.  Reading
  // the two flags independently, as this did, sends such a sprite down the
  // hardware-shadow path and it effectively disappears.
  //
  // Measured: ten frames of the Turbo Dash event across four MAME runs, 731
  // differing pixels each, all of them on opaque sprites carrying the shadow
  // bit -- 0 with this one line.  Of the 103 frame dumps this project has,
  // 67 contain opaque sprites and every one of them renders correctly; the
  // only dumps where an opaque sprite is ALSO shadow-flagged are the 43 taken
  // during events, which is why five sessions of attract-mode comparison
  // never saw it.  docs/glitch_investigation.md
  wire        d_shadow_eff = d_shadow && !d_opaque;

  wire        d_hwshadow = d_shadow_eff && (d_color[7:4] == 4'hF) &&
                           (!d_bpp4 || (d_color[11:8] == 4'hF));

  // code = (tile + (row << 6) + col) & 0xFFF, which is MAME's
  // `code % gfx->elements()`: CG RAM holds exactly 0x1000 cells of 64 bytes.
  wire [11:0] d_code  = d_tile + {3'd0, d_row, 6'd0} + {10'd0, d_col};
  wire [11:0] d_code1 = d_code + 12'd1;                 // the next column
  wire [16:0] cg_base = {d_code, 5'd0} + {12'd0, d_srow, 2'd0};

  // shaperam byte = code*8 + srow  ->  word = code*4 + (srow >> 1)
  wire [13:0] sh_of_col  = {d_code,  2'd0} + {12'd0, d_srow[2:1]};
  wire [13:0] sh_of_next = {d_code1, 2'd0} + {12'd0, d_srow[2:1]};

  // MAME: tile-word bit 15 forces an opaque cell, which this folds in.  Not
  // registered -- see the note about shape_data being stable for a whole cell.
  wire [7:0]  d_mask = d_opaque ? 8'hFF
                                : (d_srow[0] ? shape_data[7:0] : shape_data[15:8]);

  // sx for this column.  MAME: sx = (xpos & 0x1ff) - 10 + delta*8, then
  // ((sx + 16) & 0x1ff) - 8.  -10 + 16 = +6, so the wrap is on xbase + 6.
  wire [1:0]  d_dcol = d_flipx ? (d_wm1 - d_col) : d_col;
  wire [8:0]  d_sxw  = d_xbase + 9'd6 + {4'd0, d_dcol, 3'd0};
  wire signed [10:0] d_sx = $signed({2'b00, d_sxw}) - 11'sd8;

  // =====================================================================
  // Sequencer
  // =====================================================================
  localparam S_IDLE = 4'd0,
             S_EV   = 4'd1,
             S_D0   = 4'd2,   // issue word 1
             S_D1   = 4'd3,   // issue word 2
             S_D2   = 4'd4,   // word 1 back, issue word 3
             S_D3   = 4'd5,   // word 2 back
             S_D4   = 4'd6,   // word 3 back, start the first column
             S_C0   = 4'd7,   // shape is on the bus; issue cg pair 0
             S_C1   = 4'd8,   // issue cg pair 1
             S_C2   = 4'd9,   // issue cg pair 2, write pair 0
             S_C3   = 4'd10,  // issue cg pair 3, write pair 1
             S_C4   = 4'd11,  // issue the NEXT column's shape, write pair 2
             S_C5   = 4'd12,  // write pair 3, then next column / hit / done
             S_HOLD = 4'd13;  // early evaluation done; wait for draw slot

  reg [3:0] st;
  reg [2:0] pn;                    // even pixel of the pair being written: 0/2/4/6
  reg       skip_cell;
  reg       draw_pending;

  // --- the pair being written -------------------------------------------
  wire signed [10:0] dxa = d_sx + $signed({8'd0, pn});
  wire signed [10:0] dxb = dxa + 11'sd1;

  // Under flipx the cell is walked backwards, but a screen pair still lives
  // inside one CG word: word k holds source pixels 2k and 2k+1, and the
  // mirrored pair (7-pn, 6-pn) is word 3-(pn>>1) with its halves exchanged.
  wire [7:0]  byte_a = d_flipx ? cg_data[7:0]  : cg_data[15:8];
  wire [7:0]  byte_b = d_flipx ? cg_data[15:8] : cg_data[7:0];

  // the shape bit for source index s is d_mask[7-s]
  wire mask_a = d_flipx ? d_mask[pn]         : d_mask[3'd7 - pn];
  wire mask_b = d_flipx ? d_mask[pn + 3'd1]  : d_mask[3'd6 - pn];

  // pen = {shadow, colour, pixel}; in 4bpp the colour offset sits between them
  wire [12:0] pen_a = {d_shadow_eff, d_color[7:4],
                       d_bpp4 ? {d_color[11:8], byte_a[3:0]} : byte_a};
  wire [12:0] pen_b = {d_shadow_eff, d_color[7:4],
                       d_bpp4 ? {d_color[11:8], byte_b[3:0]} : byte_b};

  wire vis_a = (dxa >= 0) && (dxa < H_VISIBLE) && mask_a;
  wire vis_b = (dxb >= 0) && (dxb < H_VISIBLE) && mask_b;

  wire consuming = !skip_cell && ((st == S_C2) || (st == S_C3) ||
                                  (st == S_C4) || (st == S_C5));

  // The next column's first CG word.  d_code1 is already here for the shape
  // read, which uses this same six-clock window for the same reason.
  wire [16:0] cg_base_next = {d_code1, 5'd0} + {12'd0, d_srow, 2'd0};
  assign cg_pf_addr = cg_base_next;
  assign cg_pf_req  = busy && (d_col != d_wm1) &&
                      (st >= S_C0) && (st <= S_C5);

  // CG word index for pair p, mirrored under flipx
  function automatic [1:0] cgw_i(input [1:0] p);
    cgw_i = d_flipx ? (2'd3 - p) : p;
  endfunction

  // MAME's draw_sprites() does `source += 0x400` on a u16*, i.e. the bank
  // selects a 0x400-WORD half of object RAM, not 0x200.  With the shift one
  // place short, sprite n in bank 1 read MAME's sprite n-128 and MAME's
  // sprites 128-255 were never read at all.
  // MAME_VERIFIED, and measured: on Emeraldia frame 900 every sprite with
  // index < 128 was pixel-exact and all 21 with index >= 128 were missing;
  // fixing this alone took that frame from 2,227 differing pixels to 0.
  // Numan Athletics cannot see it -- it uses bank 0 on four of the six dumped
  // frames and only sprites 0-23 on the other two.  docs/level11_second_game.md
  wire [10:0] spr_base = {bank, 10'd0};

  // the sprite whose words are being fetched
  wire [7:0] cur_spr = h_spr[hit_i[6:0]];

  always @(posedge clk) begin
    if (rst) begin
      st           <= S_IDLE;
      busy         <= 1'b0;
      px_a         <= 1'b0;
      px_b         <= 1'b0;
      px_hwshadow  <= 1'b0;
      ev_v         <= 2'd0;
      hit_n        <= 8'd0;
      dbg_hits     <= 8'd0;
      dbg_overflow <= 1'b0;
      skip_cell    <= 1'b0;
      draw_pending <= 1'b0;
    end else if (abort && busy) begin
      // Out of line time.  Stopping here drops the sprites that had not been
      // reached, which is what a real sprite limit does; `dbg_overflow` is how
      // that gets said out loud, because a silently short sprite list looks
      // exactly like a sprite that is missing for some subtler reason.
      px_a         <= 1'b0;
      px_b         <= 1'b0;
      busy         <= 1'b0;
      // If the deadline arrives during early evaluation, publish the partial
      // count instead of leaving dbg_hits at the previous line's value.  The
      // top-level peak counter can then identify the line that was truncated.
      dbg_hits     <= hit_n;
      dbg_overflow <= 1'b1;
      skip_cell    <= 1'b0;
      draw_pending <= 1'b0;
      st           <= S_IDLE;
    // A STALL MUST NOT SWALLOW `start`.
    //
    // `start` is a one-clock pulse.  Freezing the whole state machine on
    // cg_stall freezes S_IDLE too, so the pulse passes while the engine is
    // held and the pass is silently skipped -- no busy, no overflow flag, no
    // line-overrun flag, nothing.  And the stall that does it is not even this
    // engine's: na2_video muxes cg_addr by whose turn it is, so the instant
    // this engine's window opens the port carries ITS idle address, which is
    // not in na2_cgmem's row cache, and cg_stall is high before the engine has
    // asked for anything.
    //
    // Measured 2026-08-30 on a real MAME frame through the Pocket CG path:
    // sprite starts 448, busy 0 cycles, sp_cg_stall high for all 896 cycles of
    // the window, peak sprites 0.  On hardware: no characters, no objects,
    // correct text and backgrounds.
    //
    // MiSTer ties cg_stall to 0 (na2_cgmem's on-chip branch), so this line has
    // never done anything there.
    end else if (cg_stall && busy && !(st == S_HOLD && start)) begin
      px_a <= 1'b0;
      px_b <= 1'b0;
    end else begin
      px_a <= 1'b0;
      px_b <= 1'b0;

      if (start && busy) draw_pending <= 1'b1;

      case (st)
        S_IDLE: if (prepare || start) begin
          busy   <= 1'b1;
          ev_i   <= 9'd0;
          ev_v   <= 2'd0;
          hit_n  <= 8'd0;
          // Per-line result.  na2_top owns the sticky history used by the
          // hardware grid, so retaining an old overflow here only makes a
          // later clean line look truncated to simulation/debug clients.
          dbg_overflow <= 1'b0;
          skip_cell <= 1'b0;
          draw_pending <= start;
          st     <= S_EV;
        end

        // ---- evaluation, one sprite issued per clock -------------------
        S_EV: begin
          spr_addr <= spr_base + {1'b0, ev_i[7:0], 2'b00};
          ev_v     <= {ev_v[0], (ev_i < NSPR)};
          ev_ix1   <= ev_i[7:0];
          ev_ix2   <= ev_ix1;
          if (ev_i < NSPR) ev_i <= ev_i + 9'd1;

          // the word on spr_data now belongs to the index issued two cycles
          // ago, which is ev_ix2, and ev_v[1] says whether there was one
          if (ev_v[1] && e_hit) begin
            if (hit_n == MAXHIT[7:0]) dbg_overflow <= 1'b1;
            else begin
              h_spr [hit_n[6:0]] <= ev_ix2;
              h_row [hit_n[6:0]] <= e_row;
              h_srow[hit_n[6:0]] <= e_srow;
              hit_n              <= hit_n + 8'd1;
            end
          end

          if ((ev_i >= NSPR) && (ev_v == 2'd0)) begin
            dbg_hits <= hit_n;
            hit_i    <= 8'd0;
            if (draw_pending || start) begin
              draw_pending <= 1'b0;
              if (hit_n == 8'd0) begin
                busy <= 1'b0;
                st   <= S_IDLE;
              end else
                st <= S_D0;
            end else
              st <= S_HOLD;
          end
        end

        S_HOLD: if (start) begin
          draw_pending <= 1'b0;
          if (hit_n == 8'd0) begin
            busy <= 1'b0;
            st   <= S_IDLE;
          end else
            st <= S_D0;
        end

        // ---- one sprite: words 1, 2, 3 --------------------------------
        S_D0: begin spr_addr <= spr_base + {1'b0, cur_spr, 2'd1}; st <= S_D1; end
        S_D1: begin spr_addr <= spr_base + {1'b0, cur_spr, 2'd2}; st <= S_D2; end
        S_D2: begin
          spr_addr <= spr_base + {1'b0, cur_spr, 2'd3};
          d_tile   <= spr_data[11:0];            // word 1
          d_opaque <= spr_data[15];
          d_row    <= h_row [hit_i[6:0]];
          d_srow   <= h_srow[hit_i[6:0]];
          d_col    <= 2'd0;
          st       <= S_D3;
        end
        S_D3: begin
          d_color    <= spr_data;                // word 2
          // the first column's shape read, two clocks before S_C0 needs it
          shape_addr <= sh_of_col;
          st         <= S_D4;
        end
        S_D4: begin
          d_xbase <= spr_data[8:0];              // word 3
          st      <= S_C0;
        end

        // ---- one 8x8 cell, six clocks ---------------------------------
        S_C0: begin
          if (SKIP_EMPTY && (d_mask == 8'h00)) begin
            // shape_addr for this column was issued two clocks ago.  With no
            // enabled pixels, CG data cannot affect occupancy or the line
            // buffer.  Reuse C4/C5 to pipeline the next shape read and advance
            // the column, but suppress their normal pixel outputs.
            skip_cell <= 1'b1;
            st        <= S_C4;
          end else begin
            skip_cell <= 1'b0;
            cg_addr   <= cg_base + {15'd0, cgw_i(2'd0)};
            pn        <= 3'd0;
            st        <= S_C1;
          end
        end
        S_C1: begin cg_addr <= cg_base + {15'd0, cgw_i(2'd1)}; st <= S_C2; end
        S_C2: begin cg_addr <= cg_base + {15'd0, cgw_i(2'd2)}; st <= S_C3; end
        S_C3: begin cg_addr <= cg_base + {15'd0, cgw_i(2'd3)}; st <= S_C4; end
        S_C4: begin
          // the next column's shape goes out here, where the port is idle;
          // it lands exactly as that column's first pixels are written
          shape_addr <= sh_of_next;
          st         <= S_C5;
        end
        S_C5: begin
          skip_cell <= 1'b0;
          if (d_col != d_wm1) begin
            d_col <= d_col + 2'd1;
            st    <= S_C0;
          end else if (hit_i + 8'd1 == hit_n) begin
            busy <= 1'b0;
            draw_pending <= 1'b0;
            st   <= S_IDLE;
          end else begin
            hit_i <= hit_i + 8'd1;
            st    <= S_D0;
          end
        end

        default: st <= S_IDLE;
      endcase

      if (consuming) begin
        px_a    <= vis_a;
        px_xa   <= dxa[8:0];
        px_pena <= pen_a;
        px_b    <= vis_b;
        px_xb   <= dxb[8:0];
        px_penb <= pen_b;
        px_pri  <= d_pri;
        px_hwshadow <= d_hwshadow;
        pn      <= pn + 3'd2;
      end
    end
  end

endmodule

`default_nettype wire
