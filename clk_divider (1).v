`timescale 1ps / 1ps
// =============================================================================
// Module : clk_divider
// Phase  : 1 (Corrected)
// Desc   : Divides 5 GHz master clock to one of 4 target frequencies
//          based on a 2-bit config input. Outputs a single-cycle HIGH pulse
//          (clock-enable) on scl_p at the target frequency, as required by
//          the spec ("derived synchronous clock-enables").
//          Detects config changes and cleanly resets the counter to avoid
//          glitched output pulses.
// -----------------------------------------------------------------------------
// Corrections vs. original:
//   1. scl_p is now a single-cycle clock-enable PULSE (high for exactly one
//      scl_m cycle at the target rate) instead of a toggling divided clock.
//      This matches the spec's "synchronous clock-enable" requirement and is
//      safe for downstream gated-clock / enable logic in the top-level SoC.
//   2. toggle_val renamed to period_val for clarity (counts full periods, not
//      half-periods, since we no longer toggle).
// =============================================================================
module clk_divider (
    input  wire        rst_n,        // Active-low async reset
    input  wire        scl_m,        // 5 GHz master clock (period = 200 ps)
    input  wire [1:0]  config_bits,  // Frequency select for this port
    output reg         scl_p         // Derived clock-enable pulse output
);

    reg [9:0] counter;
    reg [1:0] config_bits_prev;

    // -------------------------------------------------------------------------
    // Combinational lookup: full-period count per target frequency.
    // period_val = f_master / f_target  (counts master cycles per output pulse)
    //   00 -> 2.5  MHz : 5000 MHz / 2.5  MHz = 2000 cycles
    //   01 -> 20   MHz : 5000 MHz / 20   MHz = 250  cycles
    //   10 -> 500  MHz : 5000 MHz / 500  MHz = 10   cycles
    //   11 -> 1.25 GHz : 5000 MHz / 1250 MHz = 4    cycles
    // -------------------------------------------------------------------------
    function [10:0] get_period_val(input [1:0] cfg);
        case (cfg)
            2'b00: get_period_val = 11'd2000; // 2.5  MHz
            2'b01: get_period_val = 11'd250;  // 20   MHz
            2'b10: get_period_val = 11'd10;   // 500  MHz
            2'b11: get_period_val = 11'd4;    // 1.25 GHz
            default: get_period_val = 11'd2000;
        endcase
    endfunction

    wire [10:0] period_val = get_period_val(config_bits);

    // Counter widened to 11 bits to hold max value of 2000
    reg [10:0] cnt;

    always @(posedge scl_m or negedge rst_n) begin
        if (!rst_n) begin
            cnt              <= 11'd0;
            scl_p            <= 1'b0;
            config_bits_prev <= 2'b00;
        end else begin
            config_bits_prev <= config_bits;

            if (config_bits != config_bits_prev) begin
                // Config changed: flush counter, suppress output to avoid glitch
                cnt   <= 11'd0;
                scl_p <= 1'b0;
            end else if (cnt >= period_val - 1) begin
                // Period elapsed: fire a single-cycle enable pulse and reset
                cnt   <= 11'd0;
                scl_p <= 1'b1;
            end else begin
                cnt   <= cnt + 1;
                scl_p <= 1'b0;    // Pulse is only high for one master cycle
            end
        end
    end

endmodule
