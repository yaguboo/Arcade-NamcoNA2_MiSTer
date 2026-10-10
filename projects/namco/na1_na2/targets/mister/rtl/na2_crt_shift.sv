// MiSTer display compensation, not a claim about the original PCB timing.
// INFERRED: delaying VSYNC relative to unchanged pixels moves a native CRT
// picture upwards. The correction magnitude still needs a real CRT check.
// All choices keep the three-line pulse wholly inside vertical blanking.
`default_nettype none

module na2_crt_shift (
    input  wire       clk,
    input  wire       rst,
    input  wire       ce_pix,
    input  wire [8:0] hpos,
    input  wire [8:0] vpos,
    input  wire       vsync_in,
    input  wire       enable,
    input  wire [2:0] selection,
    output wire       vsync_out
);
    reg [2:0] active_selection;
    reg [8:0] vpos_previous;
    reg       shifted_vsync;
    reg [8:0] sync_start;

    // Existing CRTC: 262 lines, VSYNC at 258..260, active at 32..255.
    // Only the output phase changes: +2/+4/+8/+12/+16/+20/+24 lines.
    // The +2 choice straddles the counter wrap (260, 261, 0).
    always @* begin
        case (active_selection)
            3'd1: sync_start = 9'd260;
            3'd2: sync_start = 9'd0;
            3'd3: sync_start = 9'd4;
            3'd4: sync_start = 9'd8;
            3'd5: sync_start = 9'd12;
            3'd6: sync_start = 9'd16;
            3'd7: sync_start = 9'd20;
            default: sync_start = 9'd258;
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            active_selection <= 3'd0;
            vpos_previous    <= 9'd0;
            shifted_vsync    <= 1'b0;
        end else begin
            // ce_pix reaches this adapter one clock after the CRTC updated
            // its counters. Retain the same pre-update line used by its VSYNC.
            vpos_previous <= vpos;
            if (ce_pix) begin
                if (active_selection == 3'd1)
                    shifted_vsync <= (vpos_previous >= 9'd260) ||
                                     (vpos_previous == 9'd0);
                else
                    shifted_vsync <= (vpos_previous >= sync_start) &&
                                     (vpos_previous < sync_start + 9'd3);

                // Every possible pulse is finished by line 23. Commit menu
                // changes at line 32 so they cannot cut or duplicate a pulse.
                // The CRTC continues during OSD/pad pause, so this also works
                // while the menu has stopped game execution.
                if ((hpos == 9'd0) && (vpos == 9'd32))
                    active_selection <= enable ? selection : 3'd0;
            end
        end
    end

    // Original is a direct bypass, including reset/download. The adjusted
    // path adds one system clock of pipeline delay, under 1/1000 of a line.
    assign vsync_out = (active_selection == 3'd0) ? vsync_in : shifted_vsync;
endmodule

`default_nettype wire
