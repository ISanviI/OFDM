`timescale 1ns / 1ps

module ofdm_top (
    input  wire        clk,
    input  wire        rst,

    // ----------------------------------------------------
    // Modulator select
    // 00 = QPSK; 01 = 16QAM; 10 = 64QAM; 11 = 256QAM
    // ----------------------------------------------------
    input  wire [1:0]  modulator_select,

    // ----------------------------------------------------
    // IFFT select
    // 00 = 256; 01 = 512; 10 = 1024; 11 = 2048
    // ----------------------------------------------------
    input  wire [1:0]  ifft_select,

    // ----------------------------------------------------
    // Input bitstream from user
    // bitstream[0] = first/MSB bit
    // bitstream[7] = last/LSB bit
    //
    // The user must hold bitstream and bitstream_valid until bitstream_ready becomes HIGH.
    // ----------------------------------------------------
    input  wire [0:7]  bitstream,
    input  wire        bitstream_valid,
    output wire        bitstream_ready,

    // ----------------------------------------------------
    // Final IFFT output
    //
    // The IFFT produces up to 2048 complex samples.
    // Only the first N samples are valid according to ifft_select:
    //
    // 256  -> [0:255]; 512  -> [0:511]; 1024 -> [0:1023]; 2048 -> [0:2047]
    // ----------------------------------------------------
    output wire signed [15:0] output_real [0:2047],
    output wire signed [15:0] output_imag [0:2047],
    output wire               output_valid,
    output wire               busy
);

    // Modulator output
    wire signed [15:0] I_out;
    wire signed [15:0] Q_out;
    wire               symbol_valid;

    // IFFT input handshake
    wire               ifft_ready;

    // MODULATOR
    adaptive_modulator u_modulator (
        .clk              (clk),
        .rst            (rst),

        // IFFT tells the modulator when it can accept another complex symbol.
        .ifft_ready       (ifft_ready),
        .sel              (modulator_select),

        // Input bitstream comes directly from the user.
        .bitstream        (bitstream),
        .bitstream_valid  (bitstream_valid),

        // User-side input handshake.
        .bitstream_ready  (bitstream_ready),

        // Complex QAM symbol.
        .I_out            (I_out),
        .Q_out            (Q_out),

        // Indicates that a complete modulation symbol is available at I_out/Q_out.
        .symbol_valid     (symbol_valid)
    );

    // IFFT
    adaptive_dif_ifft u_ifft (
        .clk          (clk),
        .rst          (rst),
        .select_line  (ifft_select),

        // Modulator I/Q outputs become IFFT inputs.
        .input_real   (I_out),
        .input_imag   (Q_out),

        // symbol_valid directly corresponds to IFFT input_valid.
        .input_valid  (symbol_valid),

        // IFFT tells the modulator when it is ready to accept another complex sample.
        .input_ready  (ifft_ready),

        // Final OFDM time-domain output.
        .output_real  (output_real),
        .output_imag  (output_imag),
        .output_valid (output_valid),

        .busy         (busy)
    );
endmodule