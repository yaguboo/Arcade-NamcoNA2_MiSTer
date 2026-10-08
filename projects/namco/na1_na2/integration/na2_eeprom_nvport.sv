//============================================================================
//  Namco NA-1/NA-2 -- the host end of the 28C16 save path.
//
//  Split out of rtl/memory/na2_eeprom.sv during the platform-boundary
//  refactor.  The array itself is board hardware and stayed there; this is
//  the transport that moves those 2 KiB to and from whatever the platform
//  uses for persistent storage.  It is instantiated at the target wrapper
//  level, not inside na2_top.
//
//  The module body below is unchanged from the original file.
//============================================================================
`default_nettype none



// ---------------------------------------------------------------------------
//  The HPS end of the save path.
//
//  Download: the HPS sends the saved file at NV_INDEX before the core comes
//  out of reset, one byte per nvs_wr.  Those bytes go straight into the
//  array and beat the DEFAULT_IMAGE contents, which is the whole point --
//  the default is only ever the state of a card that has never saved.
//
//  Upload: hps_io samples nvs_data_out on the same clock edge that it advances
//  nvs_addr, so nvs_data_out has to be the byte for the address as it stands
//  *now*.  This module keeps a read request standing for the whole upload and
//  re-latches every granted result, so nvs_data_out tracks mem[nvs_addr] with
//  two clocks of latency.  HPS strobes are hundreds of clocks apart, so two
//  is free.
//
//  WHEN TO ASK FOR A SAVE.  nvs_save_req is a request to write a file to
//  the SD card, so it must not follow the game's writes one for one: the
//  first-boot init path alone writes 286 bytes (58 settings + 228 table) in a
//  burst, and an operator walking through COIN Options rewrites the block on
//  every keypress.  So: a dirty flag, set by any 68000-side write, and a
//  settle timer reloaded by every one of them.  The request goes out when the
//  timer expires -- SETTLE_CLKS clocks with no EEPROM write at all -- which is
//  2 s at 50.113 MHz by default.
//
//  Two seconds rather than one: the read-back poll at C30932 spins on the
//  same byte between writes, so a burst is not evenly spaced, and a human
//  moving through the operator menu leaves far longer than 2 s between
//  changes.  It also bounds the save rate at one file per two seconds for a
//  game that writes continuously, which nothing here does but a later NA-2
//  title might.  `INFERRED`: no measurement says two is better than one,
//  only that it has to be comfortably longer than a write burst.
// ---------------------------------------------------------------------------
module na2_eeprom_nvport #(
    parameter [7:0] NV_INDEX    = 8'd2,
    parameter int   SETTLE_CLKS = 100_226_000   // 2 s at 50.113 MHz
) (
    input  wire        clk,
    input  wire        rst,

    // --- hps_io ------------------------------------------------------------
    input  wire        nvs_download,
    input  wire        nvs_upload,
    input  wire        nvs_wr,
    input  wire [26:0] nvs_addr,
    input  wire  [7:0] nvs_data_in,
    input  wire [15:0] nvs_index,
    output wire  [7:0] nvs_data_out,
    output reg         nvs_save_req,
    output wire  [7:0] nvs_save_index,

    // --- array -------------------------------------------------------------
    output reg         nv_req,
    output reg         nv_we,
    output reg  [10:0] nv_addr,
    output reg   [7:0] nv_din,
    input  wire  [7:0] nv_dout,
    input  wire        nv_gnt,

    // --- dirty tracking ----------------------------------------------------
    input  wire        game_we,    // the 68000 wrote a byte

    // --- diagnostics -------------------------------------------------------
    output wire        nv_loaded,  // a save file was downloaded this session
    output wire        nv_dirty    // unsaved 68000 writes are outstanding
);

  localparam int SW = $clog2(SETTLE_CLKS + 1);

  reg  [7:0]  up_data;
  reg         gnt_d;
  reg         wpend;
  reg  [10:0] wpend_a;
  reg  [7:0]  wpend_d;
  reg         dirty;
  reg  [SW-1:0] settle;
  reg         loaded;

  wire dl_sel = nvs_download && (nvs_index == {8'd0, NV_INDEX})
                               && (nvs_addr[26:11] == 16'd0);

  assign nvs_data_out          = up_data;
  assign nvs_save_index = NV_INDEX;
  assign nv_loaded          = loaded;
  assign nv_dirty           = dirty;

  // -------------------------------------------------------------------
  // Port requests.  A download write is held until it is granted; an upload
  // read is simply re-issued every clock.
  // -------------------------------------------------------------------
  always @(posedge clk) begin
    if (rst) begin
      wpend  <= 1'b0;
      loaded <= 1'b0;
    end else begin
      if (nvs_wr && dl_sel) begin
        wpend   <= 1'b1;
        wpend_a <= nvs_addr[10:0];
        wpend_d <= nvs_data_in;
        loaded  <= 1'b1;
      end else if (wpend && nv_gnt && nv_we) begin
        wpend   <= 1'b0;
      end
    end
  end

  always @(*) begin
    if (wpend) begin
      nv_req  = 1'b1;
      nv_we   = 1'b1;
      nv_addr = wpend_a;
      nv_din  = wpend_d;
    end else begin
      nv_req  = nvs_upload;
      nv_we   = 1'b0;
      nv_addr = nvs_addr[10:0];
      nv_din  = 8'h00;
    end
  end

  always @(posedge clk) begin
    gnt_d <= nv_gnt & ~nv_we;
    if (gnt_d) up_data <= nv_dout;
  end

  // -------------------------------------------------------------------
  // Dirty flag and settle timer -- see the header for why it is shaped this
  // way rather than one request per write.
  // -------------------------------------------------------------------
  always @(posedge clk) begin
    nvs_save_req <= 1'b0;
    if (rst) begin
      dirty  <= 1'b0;
      settle <= '0;
    end else if (game_we) begin
      dirty  <= 1'b1;
      // SETTLE_CLKS is an int (32-bit); settle is SW = $clog2 bits wide.
      // The value fits by construction -- SW is derived from it.
      settle <= SW'(SETTLE_CLKS);
    end else if (settle != 0) begin
      settle <= settle - 1'b1;
    end else if (dirty && !nvs_download && !nvs_upload) begin
      dirty            <= 1'b0;
      nvs_save_req <= 1'b1;
    end
  end

endmodule

`default_nettype wire
