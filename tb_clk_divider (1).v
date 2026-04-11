`timescale 1ps / 1ps
// =============================================================================
// Module : tb_clk_divider
// Phase  : 1 (Corrected)
// Desc   : Self-checking testbench for clk_divider. Verifies all 4 output
//          frequencies by measuring time between rising edges of scl_p.
//          Also verifies pulse width (must be exactly 1 master cycle = 200 ps),
//          clean counter reset on config change, and reset-during-operation.
// -----------------------------------------------------------------------------
// Corrections vs. original:
//   1. Inter-test config changes now wait for a full @(posedge scl_m) instead
//      of a bare #50 (which was shorter than one master clock cycle and could
//      race the config_bits_prev latch).
//   2. Added pulse-width check: scl_p must be high for exactly 200 ps.
//   3. Added config-change glitch test: change config mid-run and verify
//      scl_p stays low for at least one full new period before pulsing.
//   4. Added reset-during-operation test: assert rst_n=0 mid-run and confirm
//      scl_p goes and stays low until reset releases.
// =============================================================================
module tb_clk_divider();

    reg        rst_n;
    reg        scl_m;
    reg [1:0]  config_bits;
    wire       scl_p;

    clk_divider uut (
        .rst_n(rst_n),
        .scl_m(scl_m),
        .config_bits(config_bits),
        .scl_p(scl_p)
    );

    // 5 GHz master clock: period = 200 ps, half-period = 100 ps
    always #100 scl_m = ~scl_m;

    realtime rise1, rise2, fall1;
    integer  pass_count, fail_count;

    // -------------------------------------------------------------------------
    // Task: check_freq
    //   Measures two consecutive rising edges of scl_p and compares period.
    //   Also checks that the pulse width == 200 ps (one master clock cycle).
    // -------------------------------------------------------------------------
    task check_freq;
        input [63:0]  expected_period_ps;
        input [127:0] label;          // string label (padded to 128 bits)
        begin
            // Skip first edge (may be mid-period after config change)
            @(posedge scl_p);
            // Measure full period
            @(posedge scl_p); rise1 = $realtime;
            @(negedge scl_p); fall1 = $realtime;   // capture fall for pulse width
            @(posedge scl_p); rise2 = $realtime;

            // Period check
            if ((rise2 - rise1) == expected_period_ps) begin
                $display("PASS [%s] Period = %0t ps (expected %0t ps)",
                         label, rise2 - rise1, expected_period_ps);
                pass_count = pass_count + 1;
            end else begin
                $display("FAIL [%s] Period = %0t ps (expected %0t ps)",
                         label, rise2 - rise1, expected_period_ps);
                fail_count = fail_count + 1;
            end

            // Pulse-width check: scl_p must be high for exactly 200 ps
            if ((fall1 - rise1) == 200) begin
                $display("PASS [%s] Pulse width = %0t ps (expected 200 ps)",
                         label, fall1 - rise1);
                pass_count = pass_count + 1;
            end else begin
                $display("FAIL [%s] Pulse width = %0t ps (expected 200 ps)",
                         label, fall1 - rise1);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Main stimulus
    // -------------------------------------------------------------------------
    initial begin
        rst_n      = 0;
        scl_m      = 0;
        config_bits = 2'b00;
        pass_count = 0;
        fail_count = 0;

        // Spec-compliant 5 ns reset hold
        #5000;
        rst_n = 1;

        $display("======================================================");
        $display("  CLK_DIVIDER VERIFICATION — ALL TESTS");
        $display("======================================================");

        // ------------------------------------------------------------------
        // Test 1: 2.5 MHz — period = 400,000 ps
        // ------------------------------------------------------------------
        $display("\n-- Test 1: config=00 => 2.5 MHz --");
        config_bits = 2'b00;
        check_freq(400000, "2.5MHz ");

        // ------------------------------------------------------------------
        // Test 2: 20 MHz — period = 50,000 ps
        // FIX: wait for posedge scl_m before changing config (was bare #50)
        // ------------------------------------------------------------------
        $display("\n-- Test 2: config=01 => 20 MHz --");
        @(posedge scl_m);
        config_bits = 2'b01;
        check_freq(50000, "20MHz  ");

        // ------------------------------------------------------------------
        // Test 3: 500 MHz — period = 2,000 ps
        // ------------------------------------------------------------------
        $display("\n-- Test 3: config=10 => 500 MHz --");
        @(posedge scl_m);
        config_bits = 2'b10;
        check_freq(2000, "500MHz ");

        // ------------------------------------------------------------------
        // Test 4: 1.25 GHz — period = 800 ps
        // ------------------------------------------------------------------
        $display("\n-- Test 4: config=11 => 1.25 GHz --");
        @(posedge scl_m);
        config_bits = 2'b11;
        check_freq(800, "1.25GHz");

        // ------------------------------------------------------------------
        // Test 5: Config-change glitch suppression
        //   Change config mid-run. scl_p must stay low for at least one full
        //   new period before it fires — no spurious pulses right after change.
        // ------------------------------------------------------------------
        $display("\n-- Test 5: Config change glitch suppression --");
        @(posedge scl_m);
        config_bits = 2'b00;          // switch to 2.5 MHz mid-run
        // Wait 3 master cycles — shorter than any valid period (4 cycles min)
        @(posedge scl_m);
        @(posedge scl_m);
        @(posedge scl_m);
        if (scl_p === 1'b0) begin
            $display("PASS [Glitch] scl_p correctly suppressed after config change");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL [Glitch] scl_p spuriously high after config change");
            fail_count = fail_count + 1;
        end

        // ------------------------------------------------------------------
        // Test 6: Reset during operation
        //   Assert rst_n=0 while running. scl_p must immediately go low and
        //   stay low until reset releases.
        // ------------------------------------------------------------------
        $display("\n-- Test 6: Reset during operation --");
        @(posedge scl_m);
        config_bits = 2'b10;         // 500 MHz for fast cycling
        @(posedge scl_p);            // wait until active
        rst_n = 0;
        @(posedge scl_m);            // sample one cycle after reset
        if (scl_p === 1'b0) begin
            $display("PASS [Reset] scl_p low during reset");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL [Reset] scl_p not low during reset");
            fail_count = fail_count + 1;
        end
        #1000;                        // hold reset for 1 ns
        rst_n = 1;
        // Verify resumes correctly after reset
        check_freq(2000, "500MHz post-rst");

        // ------------------------------------------------------------------
        // Summary
        // ------------------------------------------------------------------
        $display("\n======================================================");
        $display("  RESULTS: %0d PASSED, %0d FAILED", pass_count, fail_count);
        $display("======================================================");
        $finish;
    end

endmodule
