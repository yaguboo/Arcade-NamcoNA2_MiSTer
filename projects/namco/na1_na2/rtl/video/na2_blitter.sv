//============================================================================
//  Namco NA-2 -- blitter
//
//  A DMA engine driven from the video register file at 0xEFFF00.  Reads from
//  main memory (mask ROM, program ROM, work RAM) and writes into the video
//  memories.  Numan Athletics uses it during boot to clear sprite RAM and to
//  move tile and shape data, so nothing appears on screen until it works.
//
//  Registers  (MAME_VERIFIED: blit() in src/mame/namco/namcona1.cpp,
//              FBNEO_CROSSCHECK: blit() in d_namcona1.cpp -- identical)
//    vreg[0x1]      source format selector
//    vreg[0x4]      destination format selector
//    vreg[0x6]      gfx bank; 3 selects the "tile data" format table
//    vreg[0x7,0x8]  source address as a dword offset      (byte addr = 2*v)
//    vreg[0x9,0xa]  destination address as a dword offset
//    vreg[0xb]      byte count
//    write 0xEFFF18 starts the transfer
//
//  INFERRED, not MAME_VERIFIED: the format tables below are a lookup of magic
//  constants in both emulators, with a fallback formula.  The real field
//  encoding is unknown -- see KNOWN_ISSUES.md ISSUE-3.  The constants are
//  reproduced exactly because the game depends on them, but they are not a
//  hardware model.
//
//  UNKNOWN: how long a real blit takes and whether the 68000 is stalled during
//  it.  Both emulators complete instantly.  This implementation takes one
//  word per main-memory access, which is at least a plausible lower bound, and
//  raises `busy` so the timing can be made accurate later without changing the
//  interface.  See KNOWN_ISSUES.md ISSUE-4.
//============================================================================
`default_nettype none

module na2_blitter (
    input  wire        clk,
    input  wire        rst,
    input  wire        ce_en,

    // --- start ------------------------------------------------------------
    input  wire        trigger,          // pulse: 68000 wrote 0xEFFF18
    input  wire [15:0] src_fmt,          // vreg[0x1]
    input  wire [15:0] dst_fmt,          // vreg[0x4]
    input  wire [15:0] gfxbank,          // vreg[0x6]
    input  wire [15:0] src_hi,           // vreg[0x7]
    input  wire [15:0] src_lo,           // vreg[0x8]
    input  wire [15:0] dst_hi,           // vreg[0x9]
    input  wire [15:0] dst_lo,           // vreg[0xa]
    input  wire [15:0] nbytes,           // vreg[0xb]
    output wire        busy,
    output reg         dbg_abort,

    // --- main memory read port (work RAM / mask ROM / program ROM) ---------
    output reg         mem_req,
    output reg  [23:1] mem_addr,
    input  wire [15:0] mem_din,
    input  wire        mem_ack,

    // --- video memory write port ------------------------------------------
    output reg         vid_req,
    output reg  [23:1] vid_addr,
    output reg  [15:0] vid_dout,
    input  wire        vid_ack
);

  // ------------------------------------------------------------------
  // Format decode.
  // MAME blit_setup(): "bytes per row" and "pitch" for a format word.
  // ------------------------------------------------------------------
  //
  // The `default` rows are `(64 - (format >> n)) << m` in MAME, computed in
  // `int`.  A format big enough to make that negative is not an error there:
  // `offset >= bytes_per_row` is then true on the first word, so the C starts
  // a new row on *every* word.  In unsigned RTL the same expression underflows
  // to about 65528 and the row never wraps at all -- opposite behaviour.
  // Clamping the negative case to zero reproduces the C exactly, because
  // `off + 2 >= 0` is also always true.  sim/tb_na2_blitter.sv covers both
  // branches; without the clamp its last two cases fail.   MAME_VERIFIED
  function automatic [31:0] fmt_decode(input [15:0] f, input is_tile);
    reg [15:0] row, pitch;
    begin
      if (is_tile) begin
        case (f)
          16'h0001: begin row = 16'h1000;                     pitch = 16'h1000; end
          16'h0081: begin row = 16'h0020;                     pitch = 16'h0120; end
          default:  begin
            row   = ((f >> 2) > 16'd64) ? 16'd0 : ((16'd64 - (f >> 2)) << 3);
            pitch = 16'h0200;
          end
        endcase
      end else begin
        case (f)
          16'h0000: begin row = 16'h0010;                     pitch = 16'h0000; end // Numan: clears sprite RAM
          16'h0001: begin row = 16'h1000;                     pitch = 16'h1000; end
          16'h008d: begin row = 16'h0008;                     pitch = 16'h0120; end // Numan
          16'h00bd: begin row = 16'h0004;                     pitch = 16'h0120; end // Numan
          16'h0401: begin row = 16'h0100;                     pitch = 16'h0900; end // F/A
          default:  begin
            row   = ((f >> 5) > 16'd64) ? 16'd0 : ((16'd64 - (f >> 5)) << 6);
            pitch = 16'h1000;
          end
        endcase
      end
      fmt_decode = {row, pitch};
    end
  endfunction

  wire is_tile = (gfxbank == 16'd3);

  // ------------------------------------------------------------------
  // Address classification.
  // MAME transfer_dword() accepts exactly these ranges and aborts the whole
  // blit on anything else.  We do the same rather than inventing a fallback.
  // ------------------------------------------------------------------
  function automatic src_ok(input [24:0] a);
    src_ok = ((a >= 25'h0400000) && (a < 25'h0C00000))    // mask ROM
          || ((a >= 25'h0C00000) && (a < 25'h0E00000))    // program ROM
          || ((a >= 25'h0001000) && (a < 25'h0080000));   // work RAM
  endfunction

  function automatic dst_ok(input [24:0] a);
    dst_ok = ((a >= 25'h0F00000) && (a < 25'h0F02000))    // palette
          || ((a >= 25'h0F40000) && (a < 25'h0F80000))    // CG / shape RAM
          || ((a >= 25'h0FF0000) && (a < 25'h0FFC000))    // video RAM
          || ((a >= 25'h0FFF000) && (a < 25'h1000000));   // sprite RAM
    // KNOWN DIVERGENCE, deliberately not patched here.  namcona1.cpp:355 also
    // accepts `dest >= 0xf00000*2 && dest < 0xf02000*2` -- 0x1E00000-0x1E03FFF
    // -- which its own comment calls an assumed palette mirror for xday2, and
    // it TRANSLATES that address back to palette RAM before writing.  Adding
    // the range to this test alone would let the blit run and then drop every
    // word, because nothing downstream decodes 0x1E00000: a silent dead write
    // in place of an honest abort, which is worse than the divergence.  Doing
    // it properly means a destination-address rewrite in the transfer path.
    // Numan Athletics never aims here (the abort flag reads 0 on hardware), so
    // it is recorded rather than guessed at.  Level 11's second game is where
    // it would first matter.  MAME_FBNEO_DIFF.md.
  endfunction

  localparam S_IDLE = 3'd0, S_RD = 3'd1, S_RD_W = 3'd2,
             S_WR   = 3'd3, S_WR_W = 3'd4, S_STEP = 3'd5, S_ABORT = 3'd6;

  reg [2:0]  st;
  reg [24:0] src_base, dst_base;
  reg [15:0] src_off,  dst_off;
  reg [15:0] src_row,  src_pitch, dst_row, dst_pitch;
  reg [16:0] count;                  // one bit wider: nbytes is rounded up
  reg [15:0] data;

  assign busy = (st != S_IDLE);

  wire [24:0] src_cur = src_base + {9'd0, src_off};
  wire [24:0] dst_cur = dst_base + {9'd0, dst_off};

  // "dest and source are provided as dword offsets": byte address = 2 * value,
  // with the value first masked to 24 bits.  MAME_VERIFIED: blit().
  wire [23:0] src_reg24 = {src_hi[7:0], src_lo};
  wire [23:0] dst_reg24 = {dst_hi[7:0], dst_lo};
  wire [24:0] src_start = {src_reg24, 1'b0};
  wire [24:0] dst_start = {dst_reg24, 1'b0};

  always @(posedge clk) begin
    if (rst) begin
      dbg_abort <= 1'b0;
      st      <= S_IDLE;
      mem_req <= 1'b0;
      vid_req <= 1'b0;
    end else if (ce_en) begin
      case (st)
        S_IDLE: if (trigger) begin
          src_base <= src_start;
          // MAME folds a destination below 0xF00000 up into the gfx window
          // once, before the loop.  This used to be re-tested on every word,
          // which was only harmless because dst_ok() aborts long before the
          // 25-bit address could wrap back under 0xF00000.
          dst_base <= (dst_start < 25'h0F00000) ? (dst_start + 25'h0F40000)
                                                : dst_start;
          {src_row, src_pitch} <= fmt_decode(src_fmt, is_tile);
          {dst_row, dst_pitch} <= fmt_decode(dst_fmt, is_tile);
          count    <= {1'b0, nbytes} + {16'd0, nbytes[0]};   // round odd up
          src_off  <= 16'd0;
          dst_off  <= 16'd0;
          st       <= S_RD;
        end

        S_RD: begin
          if (count == 0) begin
            st <= S_IDLE;
          end else if (!src_ok(src_cur) || !dst_ok(dst_cur)) begin
            st <= S_ABORT;
          end else begin
            mem_req  <= 1'b1;
            mem_addr <= src_cur[23:1];
            st       <= S_RD_W;
          end
        end

        S_RD_W: if (mem_ack) begin
          mem_req <= 1'b0;
          data    <= mem_din;
          st      <= S_WR;
        end

        S_WR: begin
          vid_req  <= 1'b1;
          vid_addr <= dst_cur[23:1];
          vid_dout <= data;
          st       <= S_WR_W;
        end

        S_WR_W: if (vid_ack) begin
          vid_req <= 1'b0;
          st      <= S_STEP;
        end

        S_STEP: begin
          count <= count - 17'd2;
          if (dst_off + 16'd2 >= dst_row) begin
            dst_base <= dst_base + {9'd0, dst_pitch};
            dst_off  <= 16'd0;
          end else
            dst_off <= dst_off + 16'd2;

          if (src_off + 16'd2 >= src_row) begin
            src_base <= src_base + {9'd0, src_pitch};
            src_off  <= 16'd0;
          end else
            src_off <= src_off + 16'd2;

          st <= (count <= 17'd2) ? S_IDLE : S_RD;
        end

        S_ABORT: begin
          dbg_abort <= 1'b1;
          // synthesis translate_off
          $display("[%0t] BLITTER abort: src=%07h dst=%07h remaining=%0d",
                   $time, src_cur, dst_cur, count);
          // synthesis translate_on
          st <= S_IDLE;
        end

        default: st <= S_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
