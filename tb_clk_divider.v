`timescale 1ps / 1ps

module tb_clk_divider();
    reg rst_n, scl_m;
    reg [1:0] config_bits;
    wire scl_p;

    clk_divider uut (
        .rst_n(rst_n), .scl_m(scl_m),
        .config_bits(config_bits), .scl_p(scl_p)
    );

    always #100 scl_m = ~scl_m;

    time rise1, rise2;

    initial begin
        rst_n = 0; scl_m = 0; config_bits = 2'b00;
        
        // 5ns spec-compliant reset
        #5000;
        rst_n = 1;
        
        $display("--- STARTING CLOCK DIVIDER VERIFICATION ---");

        // Test 1: 2.5 MHz (Period = 400,000 ps)
        @(posedge uut.scl_p);                    // skip first (post-reset alignment)
        @(posedge uut.scl_p); rise1 = $time;
        @(posedge uut.scl_p); rise2 = $time;
        if ((rise2 - rise1) == 400000)
            $display("PASS:  2.5 MHz | Period = %0d ps", rise2 - rise1);
        else
            $display("FAIL:  2.5 MHz | Expected 400000 ps, got %0d ps", rise2 - rise1);

        // Test 2: 20 MHz (Period = 50,000 ps)
        #50;                                      // land between clock edges
        config_bits = 2'b01;
        @(posedge uut.scl_p);                    // skip first post-change edge
        @(posedge uut.scl_p); rise1 = $time;
        @(posedge uut.scl_p); rise2 = $time;
        if ((rise2 - rise1) == 50000)
            $display("PASS:   20 MHz | Period = %0d ps", rise2 - rise1);
        else
            $display("FAIL:   20 MHz | Expected 50000 ps, got %0d ps", rise2 - rise1);

        // Test 3: 500 MHz (Period = 2,000 ps)
        #50;
        config_bits = 2'b10;
        @(posedge uut.scl_p);
        @(posedge uut.scl_p); rise1 = $time;
        @(posedge uut.scl_p); rise2 = $time;
        if ((rise2 - rise1) == 2000)
            $display("PASS:  500 MHz | Period = %0d ps", rise2 - rise1);
        else
            $display("FAIL:  500 MHz | Expected 2000 ps, got %0d ps", rise2 - rise1);

        // Test 4: 1.25 GHz (Period = 800 ps)
        #50;
        config_bits = 2'b11;
        @(posedge uut.scl_p);
        @(posedge uut.scl_p); rise1 = $time;
        @(posedge uut.scl_p); rise2 = $time;
        if ((rise2 - rise1) == 800)
            $display("PASS: 1.25 GHz | Period = %0d ps", rise2 - rise1);
        else
            $display("FAIL: 1.25 GHz | Expected 800 ps, got %0d ps", rise2 - rise1);

        $display("--- VERIFICATION COMPLETE ---");
        $finish;
    end
endmodule