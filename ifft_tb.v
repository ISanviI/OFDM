`timescale 1ns / 1ps

module tb_adaptive_dif_ifft;
    parameter MAX_N = 2048;

    reg         clk;
    reg         rst;
    reg         start;
    reg [1:0]   select_line;

    reg signed [15:0] input_real;
    reg signed [15:0] input_imag;
    reg               input_valid;
    wire              input_ready;

    wire signed [15:0] output_real [0:2047];
    wire signed [15:0] output_imag [0:2047];

    wire output_valid;
    wire busy;
    wire done;

    // DUT
    adaptive_dif_ifft dut (
        .clk(clk),
        .rst(rst),
        .start(start),

        .select_line(select_line),

        .input_real(input_real),
        .input_imag(input_imag),
        .input_valid(input_valid),
        .input_ready(input_ready),

        .output_real(output_real),
        .output_imag(output_imag),
        .output_valid(output_valid),

        .busy(busy),
        .done(done)
    );

    // Clock
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    // Test
    integer i;
    integer output_cycles;

    initial begin
        rst         = 1'b1;
        start       = 1'b0;
        select_line = 2'b11;
        input_real  = 16'sd0;
        input_imag  = 16'sd0;
        input_valid = 1'b0;
        output_cycles = 0;
        // Reset
        #20;
        rst = 1'b0;

        // Wait for input interface to become ready.
        @(posedge clk);
        wait (input_ready == 1'b1);

        // --------------------------------------------------------
        // Input loading
        //
        // 2048-point transform selected:
        // select_line = 11
        //
        // One complex sample is presented at a time.
        // --------------------------------------------------------
        for (i = 0; i < MAX_N; i = i + 1) begin
            @(posedge clk);
            input_valid <= 1'b1;

            // Simple test input.
            // Impulse at n = 0.
            if (i == 0) begin
                input_real <= 16'sd16384;
                input_imag <= 16'sd0;
            end
            else begin
                input_real <= 16'sd0;
                input_imag <= 16'sd0;
            end
            while (input_ready == 1'b0)
                @(posedge clk);
        end

        @(posedge clk);
        input_valid <= 1'b0;
        input_real  <= 16'sd0;
        input_imag  <= 16'sd0;

        // Start IFFT computation.
        @(posedge clk);
        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;

        // Wait until computation is complete.
        wait (output_valid == 1'b1);
        $display("==============================================");
        $display("IFFT computation completed.");
        $display("Parallel output is active.");
        $display("==============================================");

        // Check parallel output.
        while (output_valid == 1'b1) begin
            @(posedge clk);
            output_cycles = output_cycles + 1;
            $display("Output cycle = %0d, valid = %b, done = %b",
                output_cycles,
                output_valid,
                done);
        end

        // Display selected output samples.
        $display("==============================================");
        $display("Selected parallel output samples");
        $display("==============================================");

        $display("output[0]    = %d + j%d",
                 output_real[0],
                 output_imag[0]);
        $display("output[1]    = %d + j%d",
                 output_real[1],
                 output_imag[1]);
        $display("output[2]    = %d + j%d",
                 output_real[2],
                 output_imag[2]);
        $display("output[3]    = %d + j%d",
                 output_real[3],
                 output_imag[3]);
        $display("output[1024] = %d + j%d",
                 output_real[1024],
                 output_imag[1024]);
        $display("output[2047] = %d + j%d",
                 output_real[2047],
                 output_imag[2047]);

        // Verify output flags are low after output period.
        @(posedge clk);
        if ((output_valid == 1'b0) &&
            (done == 1'b0)) begin
            $display("==============================================");
            $display("PASS: Output flags returned to zero.");
            $display("==============================================");
        end
        else begin
            $display("==============================================");
            $display("FAIL: Output flags did not return to zero.");
            $display("==============================================");
        end
        $finish;
    end
endmodule