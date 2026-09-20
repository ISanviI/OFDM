// Update the time delay between tests to make sure that it is at least 50µs. Or it waits until bitstream_ready is high before sending the next byte. This ensures that the modulator has enough time to process the previous byte before receiving the next one.

`timescale 1ns / 1ps

module tb_modulator;
    reg         clk;
    reg         rst;
    reg         ifft_ready;
    reg  [1:0]  sel;

    // Same ordering as the DUT:
    // bitstream[0] = first / MSB bit
    // bitstream[7] = last / LSB bit
    reg  [0:7]  bitstream;
    reg         bitstream_valid;
    wire signed [15:0] I_out;
    wire signed [15:0] Q_out;
    wire               symbol_valid;
    wire               bitstream_ready;

    adaptive_modulator dut (
        .clk             (clk),
        .rst             (rst),
        .ifft_ready      (ifft_ready),
        .sel             (sel),
        .bitstream       (bitstream),
        .bitstream_valid (bitstream_valid),
        .I_out           (I_out),
        .Q_out           (Q_out),
        .symbol_valid    (symbol_valid),
        .bitstream_ready (bitstream_ready)
    );

    // Clock generation
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    // Monitor
    always @(posedge clk) begin
        #1;
        if (symbol_valid && ifft_ready) begin
            $display(
                "TIME = %0t | SEL = %b | SYMBOL VALID = %b | I = %0d | Q = %0d", $time, sel, symbol_valid, I_out, Q_out
            );
        end
    end

    // Task: Send one 8-bit bitstream chunk
    // The user holds:
    //     bitstream_valid = 1
    // until:
    //     bitstream_ready = 1
    // Once accepted, the user can remove valid.
    task send_byte;
        input [0:7] data;
        begin
            // Put data on the input
            bitstream = data;
            bitstream_valid = 1'b1;
            // Wait until modulator can accept it
            while (!bitstream_ready)
                @(posedge clk);
            // Allow the acceptance clock to occur
            @(posedge clk);
            // Remove valid after acceptance
            bitstream_valid = 1'b0;
        end
    endtask

    // Main test
    initial begin
        // Initial values
        rst           = 1'b1;
        ifft_ready      = 1'b1;
        sel             = 2'b00;
        bitstream       = 8'b0;
        bitstream_valid = 1'b0;

        // Reset
        #20;
        rst = 1'b0;
        #10;

        // TEST 1 : QPSK
        // sel = 00
        // 2 bits / symbol
        $display("");
        $display("========================================");
        $display("TEST 1 : QPSK");
        $display("========================================");
        sel = 2'b00;
        // Four QPSK symbols:
        // bitstream[0:7] =
        // 0 0 0 1 1 0 1 1
        send_byte(8'b00011011);
        #30;

        // TEST 2 : 16-QAM
        // sel = 01
        // 4 bits / symbol
        $display("");
        $display("========================================");
        $display("TEST 2 : 16-QAM");
        $display("========================================");
        sel = 2'b01;
        // Two 16-QAM symbols:
        // 0001
        // 1011
        send_byte(8'b00011011);
        #30;

        // TEST 3 : 64-QAM
        // sel = 10
        // 6 bits / symbol
        $display("");
        $display("========================================");
        $display("TEST 3 : 64-QAM");
        $display("========================================");
        sel = 2'b10;
        // First 64-QAM symbol:
        // 000110
        // Remaining 2 bits: 11
        // Next byte begins with: 0100
        // Therefore the second symbol becomes: 11 0100
        send_byte(8'b00011011);
        send_byte(8'b01000000);
        #30;

        // TEST 4 : 256-QAM
        // sel = 11
        // 8 bits / symbol
        $display("");
        $display("========================================");
        $display("TEST 4 : 256-QAM");
        $display("========================================");
        sel = 2'b11;
        // One complete 256-QAM symbol = all 8 bits
        send_byte(8'b00011011);
        send_byte(8'b10100101);
        #30;


        // TEST COMPLETE
        $display("");
        $display("========================================");
        $display("ALL MODULATOR TESTS COMPLETE");
        $display("========================================");
        $finish;
    end
endmodule