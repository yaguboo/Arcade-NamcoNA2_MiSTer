// MiSTer convenience pause: the board has no physical pause input.
// Only game execution is stopped; na2_top keeps video timing running.
`default_nettype none

module na2_pause (
    input  wire clk,
    input  wire rst,
    input  wire download,
    input  wire button,
    input  wire osd_open,
    input  wire osd_pause_off,
    output reg  button_pause = 1'b0,
    output wire paused
);
  reg button_d = 1'b0;

  always @(posedge clk) begin
    // Track a held button during reset/download so it does not become a
    // fresh press when loading finishes. Reset also releases a pad pause.
    button_d <= button;
    if (rst || download)       button_pause <= 1'b0;
    else if (button && !button_d) button_pause <= ~button_pause;
  end

  assign paused = button_pause | (osd_open & ~osd_pause_off);
endmodule

`default_nettype wire
