`timescale 1ps / 1ps
// =============================================================================
// Module : tb_top_system
// Desc   : Corrected & expanded testbench for top_system
// -----------------------------------------------------------------------------
// Fixes vs. submitted version:
//
//   FIX 1 — MISSING WRITE TRANSACTION TEST:
//     Original only tested flush and power management. A full 12-byte write
//     transaction (preamble + header + 8-byte payload + parity + tail) is now
//     verified, including acknowledgement assertion and data_valid_out timing.
//
//   FIX 2 — FLUSH TEST TIMING:
//     Original checked current_state after only 15 cycles. STATE_FLUSH needs
//     one extra cycle to see data_valid_in=0 and transition to IDLE. Added
//     one more @(posedge scl_m) guard cycle before the state check.
//
//   FIX 3 — WAVEFORM DUMP:
//     Added $dumpfile / $dumpvars so the simulation produces a VCD that can
//     be opened in GTKWave for the required waveform deliverables.
//
//   FIX 4 — PASS/FAIL COUNTER:
//     Added a global pass_count / fail_count summary consistent with the
//     clk_divider testbench style.
//
//   FIX 5 — LOOPBACK TEST ADDED:
//     Added a basic loopback transaction (mode=1, master target) with a check
//     on loop_test_pass.
// =============================================================================
module tb_top_system();

    // -------------------------------------------------------------------------
    // DUT Ports
    // -------------------------------------------------------------------------
    reg        rst_n, scl_m, get_ready_in, data_valid_in, mode;
    wire [3:0] scl_p, get_ready_out, data_valid_out;
    wire       acknowledgement, loop_test_pass;

    // Tri-state master data line
    wire       sda_m;
    reg        sda_m_drv;
    reg        sda_m_out_tb;
    assign sda_m = sda_m_drv ? sda_m_out_tb : 1'bz;

    // Tri-state peripheral data lines
    wire [3:0] sda_p;
    reg  [3:0] sda_p_drv;
    reg  [3:0] sda_p_out_tb;
    assign sda_p[0] = sda_p_drv[0] ? sda_p_out_tb[0] : 1'bz;
    assign sda_p[1] = sda_p_drv[1] ? sda_p_out_tb[1] : 1'bz;
    assign sda_p[2] = sda_p_drv[2] ? sda_p_out_tb[2] : 1'bz;
    assign sda_p[3] = sda_p_drv[3] ? sda_p_out_tb[3] : 1'bz;

    // -------------------------------------------------------------------------
    // DUT instantiation — TIMEOUT parameters overridden for fast simulation
    // -------------------------------------------------------------------------
    top_system #(
        .TIMEOUT_1MS(33'd100),
        .TIMEOUT_1S (33'd300)
    ) uut (
        .rst_n(rst_n), .scl_m(scl_m),
        .get_ready_in(get_ready_in), .data_valid_in(data_valid_in), .mode(mode),
        .scl_p(scl_p), .get_ready_out(get_ready_out),
        .data_valid_out(data_valid_out), .acknowledgement(acknowledgement),
        .loop_test_pass(loop_test_pass), .sda_m(sda_m), .sda_p(sda_p)
    );

    // 5 GHz master clock — period = 200 ps
    always #100 scl_m = ~scl_m;

    integer pass_count, fail_count;

    // -------------------------------------------------------------------------
    // Task: check_signal — compare actual vs expected, log result
    // -------------------------------------------------------------------------
    task check_signal;
        input        actual;
        input        expected;
        input [255:0] label;
        begin
            if (actual === expected) begin
                $display("  PASS [%s]", label);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL [%s] expected %0b, got %0b", label, expected, actual);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Task: send_byte — drive sda_m LSB-first, one bit per posedge scl_m
    // -------------------------------------------------------------------------
    task send_byte;
        input [7:0] data;
        integer i;
        begin
            sda_m_drv = 1'b1;
            for (i = 0; i <= 7; i = i + 1) begin
                sda_m_out_tb = data[i];
                @(posedge scl_m);
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Task: send_full_packet — transmit a complete 12-byte write packet
    //   port_addr : 2-bit port address (Header[1:0])
    //   is_write  : 1 = Write (Master→Port), 0 = Read (Port→Master)
    //   payload   : 8 bytes of data
    //   parity    : pre-computed parity byte
    // -------------------------------------------------------------------------
    task send_full_packet;
        input [1:0] port_addr;
        input       is_write;
        input [63:0] payload;
        input [7:0]  parity;
        integer i;
        reg [7:0] header;
        begin
            // Header: [7:5]=000, [4]=0, [3]=R/W, [2]=0, [1:0]=port_addr
            header = {3'b000, 1'b0, is_write, 1'b0, port_addr};

            data_valid_in = 1'b1;

            // Byte 0: Preamble (8'hAC)
            send_byte(8'hAC);

            // Byte 1: Header
            send_byte(header);

            // Bytes 2-9: Payload (8 bytes), LSB of each byte first
            for (i = 0; i < 8; i = i + 1)
                send_byte(payload[8*i +: 8]);

            // Byte 10: Parity
            send_byte(parity);

            // Byte 11: End marker (8'hCB)
            send_byte(8'hCB);

            data_valid_in = 1'b0;
            sda_m_drv     = 1'b0;
        end
    endtask

    // -------------------------------------------------------------------------
    // Task: run_boot_config — assert reset then send freq_reg over sda_m
    //   freq_bits : value to load into freq_reg (e.g. 8'b11100100)
    //               bits[1:0]=port0 cfg, [3:2]=port1, [5:4]=port2, [7:6]=port3
    // -------------------------------------------------------------------------
    task run_boot_config;
        input [7:0] freq_bits;
        begin
            $display("--- Boot: Reset + Frequency Config ---");
            rst_n = 1'b0;
            #5000;          // 5 ns reset hold per spec
            rst_n = 1'b1;

            // Send 8-bit frequency register immediately after reset release
            send_byte(freq_bits);
            sda_m_drv = 1'b0;

            // Idle gap
            repeat(10) @(posedge scl_m);
        end
    endtask

    // =========================================================================
    // MAIN STIMULUS
    // =========================================================================
    initial begin
        // Waveform dump (FIX 3)
        $dumpfile("tb_top_system.vcd");
        $dumpvars(0, tb_top_system);

        // Initialise
        rst_n         = 1'b0;
        scl_m         = 1'b0;
        get_ready_in  = 1'b0;
        data_valid_in = 1'b0;
        mode          = 1'b0;
        sda_m_drv     = 1'b0;
        sda_m_out_tb  = 1'b0;
        sda_p_drv     = 4'b0000;
        sda_p_out_tb  = 4'b0000;
        pass_count    = 0;
        fail_count    = 0;

        $display("==========================================================");
        $display("  TOP_SYSTEM TESTBENCH");
        $display("==========================================================");

        // ------------------------------------------------------------------
        // TEST 0: Boot Configuration
        // Freq reg = 8'b11_10_01_00
        //   Port0 → 2'b00 → 2.5 MHz
        //   Port1 → 2'b01 → 20  MHz
        //   Port2 → 2'b10 → 500 MHz
        //   Port3 → 2'b11 → 1.25 GHz
        // ------------------------------------------------------------------
        run_boot_config(8'b11_10_01_00);
        $display("\n-- TEST 0: freq_reg loaded correctly --");
        check_signal((uut.freq_reg == 8'b11_10_01_00), 1'b1, "freq_reg value");
        check_signal((uut.current_state == 4'd1),       1'b1, "IDLE after boot");

        // ------------------------------------------------------------------
        // TEST 1: Full Write Transaction (Normal Mode, Port 2)
        // FIX 1 — this entire test was missing from the original testbench.
        // ------------------------------------------------------------------
        $display("\n-- TEST 1: Normal Write Transaction to Port 2 --");

        // Wake the interface (10 ns = 50 cycles)
        get_ready_in = 1'b1;
        repeat(50) @(posedge scl_m);
        get_ready_in = 1'b0;
        @(posedge scl_m);

        // Send a 12-byte write packet to port 2
        // Payload = 64'hDEAD_BEEF_CAFE_BABE, parity is illustrative here
        fork
            // Master drives the packet
            send_full_packet(
                2'b10,               // port 2
                1'b1,                // Write
                64'hDEAD_BEEF_CAFE_BABE,
                8'hA5                // parity (illustrative)
            );

            // Check data_valid_out[2] goes high during payload phase
            begin
                // Wait until PORT_WAKE is active (get_ready_out[2] high)
                @(posedge get_ready_out[2]);
                $display("  INFO: get_ready_out[2] pulsed (port wake)");
                // Wait for data_valid_out[2] to go high (payload transmission)
                @(posedge data_valid_out[2]);
                check_signal(data_valid_out[2], 1'b1, "data_valid_out[2] high during TX");
                // Confirm non-targeted ports stay low (selective port gating)
                check_signal(data_valid_out[0], 1'b0, "data_valid_out[0] gated off");
                check_signal(data_valid_out[1], 1'b0, "data_valid_out[1] gated off");
                check_signal(data_valid_out[3], 1'b0, "data_valid_out[3] gated off");
            end
        join

        // Wait for acknowledgement
        @(posedge acknowledgement);
        check_signal(acknowledgement, 1'b1, "acknowledgement pulsed after packet");

        // Wait for port sleep pulse
        @(posedge get_ready_out[2]);
        $display("  INFO: get_ready_out[2] pulsed (port idle alert)");
        @(negedge get_ready_out[2]);

        // Back to IDLE
        repeat(5) @(posedge scl_m);
        check_signal((uut.current_state == 4'd1), 1'b1, "back to IDLE after write");

        // ------------------------------------------------------------------
        // TEST 2: Mid-Packet Flush Abort
        // FIX 2 — added one extra guard cycle before the state check.
        // ------------------------------------------------------------------
        $display("\n-- TEST 2: Mid-Packet Flush (abort after header) --");

        data_valid_in = 1'b1;
        send_byte(8'hAC);       // Preamble
        send_byte(8'h0A);       // Header: port=2, R/W=Write
        // Send 2 bits of payload then abort
        sda_m_drv    = 1'b1;
        sda_m_out_tb = 1'b1;
        @(posedge scl_m);
        @(posedge scl_m);
        data_valid_in = 1'b0;   // Abort mid-packet

        // FIX 2 — wait for FLUSH state + one transition cycle
        repeat(16) @(posedge scl_m);   // was 15 in original; +1 for safety
        check_signal((uut.current_state == 4'd1), 1'b1, "Flushed to IDLE");
        check_signal(uut.sda_p_dir,  4'b0000, "sda_p_dir cleared after flush");
        check_signal(uut.sda_m_dir,  1'b0,    "sda_m_dir cleared after flush");

        sda_m_drv = 1'b0;

        // ------------------------------------------------------------------
        // TEST 3: Power Management — 3 ns idle alert & timeout transitions
        // ------------------------------------------------------------------
        $display("\n-- TEST 3: Power Management --");

        // 3 ns idle alert from master (15 cycles)
        get_ready_in = 1'b1;
        repeat(15) @(posedge scl_m);
        get_ready_in = 1'b0;
        @(posedge scl_m);
        check_signal((uut.power_state == 2'b01), 1'b1,
                     "3ns get_ready_in -> LOW_POWER");

        // Reset idle timer with a brief data_valid_in pulse
        data_valid_in = 1'b1;
        @(posedge scl_m);
        data_valid_in = 1'b0;

        // Wait past TIMEOUT_1MS (100 cycles)
        repeat(110) @(posedge scl_m);
        check_signal((uut.power_state == 2'b01), 1'b1,
                     "1ms timeout -> LOW_POWER");

        // Wait past TIMEOUT_1S (300 total cycles from last activity)
        repeat(200) @(posedge scl_m);
        check_signal((uut.power_state == 2'b10), 1'b1,
                     "1s timeout -> HIBERNATE");

        // ------------------------------------------------------------------
        // TEST 4: Loopback — Master-Internal (loopback_target = 3'b100)
        // FIX 5 — loopback test was completely absent from original.
        // ------------------------------------------------------------------
        $display("\n-- TEST 4: Loopback Test (Master target) --");

        // Reset and re-boot for a clean loopback run
        rst_n = 1'b0; #5000; rst_n = 1'b1;
        send_byte(8'b11_10_01_00);
        sda_m_drv = 1'b0;
        repeat(10) @(posedge scl_m);

        mode = 1'b1; // Enable loopback mode

        // Wake interface
        get_ready_in = 1'b1;
        repeat(50) @(posedge scl_m);
        get_ready_in = 1'b0;
        @(posedge scl_m);

        // Send loopback packet — Header[7:5]=100 (master target)
        // Header = {3'b100, 1'b0, 1'b1 (write), 1'b0, 2'b00}
        data_valid_in = 1'b1;
        send_byte(8'hAC);           // Preamble
        send_byte({3'b100, 1'b0, 1'b1, 1'b0, 2'b00}); // Header: LB→master
        // Payload
        repeat(8) send_byte(8'hA5);
        // Parity
        send_byte(8'h00);
        // Tail
        send_byte(8'hCB);
        data_valid_in = 1'b0;
        sda_m_drv = 1'b0;

        // Wait for loop_test_pass
        repeat(100) @(posedge scl_m);
        check_signal(loop_test_pass, 1'b1, "loop_test_pass asserted");

        mode = 1'b0;

        // ------------------------------------------------------------------
        // Summary
        // ------------------------------------------------------------------
        $display("\n==========================================================");
        $display("  RESULTS: %0d PASSED, %0d FAILED", pass_count, fail_count);
        $display("==========================================================");
        $finish;
    end

endmodule
