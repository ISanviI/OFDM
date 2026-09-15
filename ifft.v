// start logic has a race/sequence problem
// The combinational array copy is extremely large between next_array and input_array
// output state flag
// 'done' a blocking assignment?
// Sign extension for multiplication width
// Understand flags

`timescale 1ns / 1ps

module adaptive_dif_ifft (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,

    // 00 = 256
    // 01 = 512
    // 10 = 1024
    // 11 = 2048
    input  wire [1:0]  select_line,

    // One complex input sample at a time.
    // Each real/imag component is initially Q2.14, 16 bits.
    input  wire signed [15:0] input_real,
    input  wire signed [15:0] input_imag,
    input  wire               input_valid,
    output reg                input_ready,

    // Complex output.
    // Final samples are Q2.14 = 16 bits.
    output reg signed [15:0] output_real,
    output reg signed [15:0] output_imag,
    output reg               output_valid,

    output reg                busy,
    output reg                done
);
    // ============================================================
    // Maximum transform size
    // ============================================================

    parameter MAX_N = 2048;
    parameter ADDR_W = 11;

    // ============================================================
    // Input/output memories
    // After quantization, internal samples are represented using 16 bits : Q2.14
    // The initial and every complete stage uses this format.
    // ============================================================

    reg signed [15:0] input_array_real [0:MAX_N-1];
    reg signed [15:0] input_array_imag [0:MAX_N-1];

    reg signed [15:0] output_array_real [0:MAX_N-1];
    reg signed [15:0] output_array_imag [0:MAX_N-1];

    // ============================================================
    // Temporary combinational arrays
    //
    // One complete stage is calculated combinationally.
    // At the clock edge, the entire stage is written to memory.
    // ============================================================

    reg signed [15:0] next_array_real [0:MAX_N-1];
    reg signed [15:0] next_array_imag [0:MAX_N-1];

    // ============================================================
    // Twiddle ROM Q2.14 representation.
    // The ROM contains: conjugate(W_2048^k) for k = 0 ... 1023.
    //
    // For a smaller transform:
    // 256  : ROM address = k * 8
    // 512  : ROM address = k * 4
    // 1024 : ROM address = k * 2
    // 2048 : ROM address = k
    //
    // The actual ROM values are expected in:
    //     twiddle_real.mem
    //     twiddle_imag.mem
    // ============================================================

    reg signed [15:0] twiddle_real [0:1023];
    reg signed [15:0] twiddle_imag [0:1023];

    initial begin
        $readmemh("twiddle_real.mem", twiddle_real);
        $readmemh("twiddle_imag.mem", twiddle_imag);
    end

    // ============================================================
    // Transform-size control
    // ============================================================

    reg [11:0] N;
    reg [3:0]  total_stages;

    always @(*) begin
        case (select_line)
            2'b00: begin
                N           = 12'd256;
                total_stages = 4'd8;
            end
            2'b01: begin
                N           = 12'd512;
                total_stages = 4'd9;
            end
            2'b10: begin
                N           = 12'd1024;
                total_stages = 4'd10;
            end
            2'b11: begin
                N           = 12'd2048;
                total_stages = 4'd11;
            end
        endcase
    end
    // ============================================================
    // Adaptive bit reversal
    // First reverse all 11 address bits.
    // Then: (>>> -> Arithmetic Right Shift)
    //
    // 256  -> >>> 3
    // 512  -> >>> 2
    // 1024 -> >>> 1
    // 2048 -> >>> 0
    //
    // This produces the required active-width bit reversal.
    // ============================================================

    reg [10:0] bit_reverse_full;
    reg [10:0] bit_reverse_address;
    reg [11:0] input_count;

    integer br;
    always @(*) begin
        bit_reverse_full = 11'd0;
        for (br = 0; br < 11; br = br + 1) begin
            bit_reverse_full[10-br] = input_count[br];  // check
        end
        case (select_line)
            2'b00: bit_reverse_address = bit_reverse_full >> 3;
            2'b01: bit_reverse_address = bit_reverse_full >> 2;
            2'b10: bit_reverse_address = bit_reverse_full >> 1;
            2'b11: bit_reverse_address = bit_reverse_full;
        endcase
    end

    // ============================================================
    // Input loading
    // The external input arrives in natural index order: 0,1,2,...,N-1
    // It is stored at the bit-reversed address required by the DIF IFFT.
    // ============================================================

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            input_count <= 12'd0;
            input_ready <= 1'b1;
        end
        else begin
            if (start) begin
                input_count <= 12'd0;
                input_ready <= 1'b1;
            end
            if (input_valid && input_ready) begin
                input_array_real[bit_reverse_address] <= input_real;    // check
                input_array_imag[bit_reverse_address] <= input_imag;

                if (input_count == N-1) begin
                    input_ready <= 1'b0;
                end
                else begin
                    input_count <= input_count + 1'b1;
                end
            end
        end
    end

    // ============================================================
    // Stage control
    // ============================================================

    reg [3:0] stage;

    integer chunk;
    integer point_in_chunk;
    integer current_index;
    integer half_size;
    integer chunk_size;
    integer num_chunks;

    // ============================================================
    // Twiddle addressing
    // ============================================================

    integer twiddle_index;
    integer twiddle_stride;

    // ============================================================
    // Intermediate arithmetic

    // Maximum supported operation:
    // Q2.14 data * Q2.14 twiddle = Q4.28
    // Then addition/subtraction  = Q5.28

    // The first stage begins with Q2.14 data and therefore has
    // the smaller Q4.28/Q5.28 widths described in the algorithm.
    // Using the larger widths here safely accommodates all stages.
    // ============================================================

    reg signed [31:0] mult_rr;
    reg signed [31:0] mult_ii;
    reg signed [31:0] mult_ri;
    reg signed [31:0] mult_ir;

    reg signed [32:0] twiddled_real;
    reg signed [32:0] twiddled_imag;

    reg signed [32:0] butterfly_sum_real;
    reg signed [32:0] butterfly_sum_imag;

    reg signed [32:0] butterfly_diff_real;
    reg signed [32:0] butterfly_diff_imag;

    reg signed [31:0] scaled_sum_real;
    reg signed [31:0] scaled_sum_imag;
    reg signed [31:0] scaled_diff_real;
    reg signed [31:0] scaled_diff_imag;

    reg signed [15:0] final_sum_real;
    reg signed [15:0] final_sum_imag;
    reg signed [15:0] final_diff_real;
    reg signed [15:0] final_diff_imag;

    // ============================================================
    // Rounding function
    // Convert Q4.28 -> Q4.14

    // Positive: +2^13 and Negative: -2^13
    // followed by arithmetic >>> 14.
    // ============================================================

    function signed [15:0] round_q428_to_q214;
        input signed [31:0] value;
        reg signed [31:0] rounded_value;
        reg signed [17:0] shifted_value;

        begin
            // For rounding to nearest, add half of the divisor (2^13), before removing 14 fractional bits
            if (value >= 0)
                rounded_value = value + 32'sd8192;
            else
                rounded_value = value - 32'sd8192;

            // Q4.28 -> Q4.14
            shifted_value = rounded_value >>> 14;

            // Check whether removed bits are redundant sign bits.
            // Q4.14 -> Q2.14
            if ((shifted_value[17:16] != {2{shifted_value[15]}}))
                $display("WARNING: Q2.14 overflow");

            // Remove redundant integer/guard bits.
            // Keep sign + 1 integer bit + 14 fractional bits.
            round_q428_to_q214 = shifted_value[15:0];
        end
    endfunction

    // ============================================================
    // Combinational stage calculation
    // The loops are elaborated/synthesized as combinational hardware.
    // Only ONE stage is calculated for the current value of "stage".
    // The resulting entire stage is registered on the next rising clock edge.
    // ============================================================

    integer i;
    always @(*) begin
        // Default: preserve current contents.
        for (i = 0; i < MAX_N; i = i + 1) begin
            next_array_real[i] = input_array_real[i];
            next_array_imag[i] = input_array_imag[i];
        end

        // --------------------------------------------------------
        // Current stage parameters
        // --------------------------------------------------------

        if (stage != 0 ) begin  // Not needed, but added just in case.
            half_size  = 1 << (stage - 1);
            chunk_size = 1 << stage;
            num_chunks = N >> stage;
        end

        // --------------------------------------------------------
        // Process every chunk
        // --------------------------------------------------------

        for (chunk = 0;
             chunk < num_chunks;
             chunk = chunk + 1) begin

            // ----------------------------------------------------
            // First operation:
            // Multiply the second half by the conjugate twiddle.
            // BEFORE the addition/subtraction.
            // ----------------------------------------------------

            for (point_in_chunk = half_size;
                 point_in_chunk < chunk_size;
                 point_in_chunk = point_in_chunk + 1) begin

                current_index = chunk * chunk_size + point_in_chunk;

                // k = point_in_chunk - half_size

                twiddle_stride = 2048 >> stage; // check
                twiddle_index = (point_in_chunk - half_size) * twiddle_stride;

                // Complex multiplication:
                // (a + jb)(c + jd)
                // real = ac - bd
                // imag = ad + bc
                // twiddle ROM contains the conjugated twiddle.

                mult_rr =
                    $signed(input_array_real[current_index]) * $signed(twiddle_real[twiddle_index]);
                mult_ii =
                    $signed(input_array_imag[current_index]) * $signed(twiddle_imag[twiddle_index]);
                mult_ri =
                    $signed(input_array_real[current_index]) * $signed(twiddle_imag[twiddle_index]);
                mult_ir =
                    $signed(input_array_imag[current_index]) * $signed(twiddle_real[twiddle_index]);

                twiddled_real = mult_rr - mult_ii;
                twiddled_imag = mult_ri + mult_ir;

                // ------------------------------------------------
                // Butterfly
                // First half : A + B_twiddled
                // Second half: A - B_twiddled
                // ------------------------------------------------

                butterfly_sum_real =
                    $signed(input_array_real[current_index - half_size]) + twiddled_real;
                butterfly_sum_imag =
                    $signed(input_array_imag[current_index - half_size]) + twiddled_imag;
                butterfly_diff_real = 
                    $signed(input_array_real[current_index - half_size]) - twiddled_real;
                butterfly_diff_imag =
                    $signed(input_array_imag[current_index - half_size]) - twiddled_imag;

                // ------------------------------------------------
                // Scaling by 1/2 (Q5.28 -> Q4.28)
                //
                // Arithmetic right shift by ONE bit.
                // ------------------------------------------------

                scaled_sum_real = butterfly_sum_real >>> 1;
                scaled_sum_imag = butterfly_sum_imag >>> 1;
                scaled_diff_real = butterfly_diff_real >>> 1;
                scaled_diff_imag = butterfly_diff_imag >>> 1;

                // ------------------------------------------------
                // Q4.28 -> Q4.14
                // Round-to-nearest and remove 14 fractional bits.
                // ------------------------------------------------

                final_sum_real = round_q428_to_q214(scaled_sum_real);
                final_sum_imag = round_q428_to_q214(scaled_sum_imag);
                final_diff_real = round_q428_to_q214(scaled_diff_real);
                final_diff_imag = round_q428_to_q214(scaled_diff_imag);

                // ------------------------------------------------
                // Write butterfly outputs into temporary array.
                // ------------------------------------------------

                next_array_real[current_index - half_size] = final_sum_real;
                next_array_imag[current_index - half_size] = final_sum_imag;
                next_array_real[current_index] = final_diff_real;
                next_array_imag[current_index] = final_diff_imag;
            end
        end
    end

    // ============================================================
    // Stage register
    // Exactly one stage is committed on each rising edge.
    // ============================================================

    integer s;
    reg output_active;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            stage        <= 4'd1;
            busy         <= 1'b0;
            done         <= 1'b0;
            output_valid <= 1'b0;

            output_real  <= 16'sd0;
            output_imag  <= 16'sd0;
        end
        else begin
            done         <= 1'b0;
            output_valid <= 1'b0;

            // ----------------------------------------------------
            // Start processing after input loading.
            // ----------------------------------------------------
            if (start && !input_ready) begin
                stage <= 4'd1;
                busy  <= 1'b1;
            end

            // ----------------------------------------------------
            // One stage per clock.
            // ----------------------------------------------------

            else if (busy) begin
                for (s = 0; s < MAX_N; s = s + 1) begin
                    input_array_real[s] <= next_array_real[s];
                    input_array_imag[s] <= next_array_imag[s];
                end

                // ------------------------------------------------
                // Final stage for selected transform size.
                // ------------------------------------------------

                if (stage == total_stages) begin
                    for (s = 0; s < MAX_N; s = s + 1) begin
                        output_array_real[s] <= next_array_real[s];
                        output_array_imag[s] <= next_array_imag[s];
                    end
                    busy <= 1'b0;
                    done <= 1'b1;
                    output_active <= 1'b1;
                    output_count <= 12'd0;
                end
                else begin
                    stage <= stage + 1'b1;
                end
            end
        end
    end

    // ============================================================
    // Output interface
    // The output array contains the selected number of samples:
    //
    // 256  -> 0 ... 255
    // 512  -> 0 ... 511
    // 1024 -> 0 ... 1023
    // 2048 -> 0 ... 2047
    //
    // Since the algorithm stops after the selected stage, the
    // resulting array is already the selected IFFT result.
    // ============================================================

    reg [11:0] output_count;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            output_count <= 12'd0;
            output_valid <= 1'b0;
        end
        else begin
            output_valid <= 1'b0;
            if (done) begin
                output_count <= 12'd0;
            end
            if (output_active) begin
                output_real <= output_array_real[output_count];
                output_imag <= output_array_imag[output_count];
                output_valid <= 1'b1;
                if (output_count == N-1) begin
                    output_active <= 1'b0;
                    output_valid  <= 1'b0;
                    done          <= 1'b1;
                end
                else begin
                    output_count <= output_count + 1'b1;
                end
            end
        end
    end
endmodule