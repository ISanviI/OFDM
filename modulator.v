// Check - Function synthesis
// Remove 'incoming_bits' and use 'bitstream' directly in the next_bit_buffer assignment

module modulator (
    input  wire        clk,
    input  wire        reset,

    input  wire [1:0]  sel,        // 00:QPSK 01:16QAM 10:64QAM 11:256QAM
    // (data (3GPP) -> MSB arrives first -> leftmost)
    // (syntax -> leftmost-MSB, rightmost-LSB; leftmost-first received and rightmost-last received)
    input  wire [0:7]  bitstream,  // 8-bit input chunk
    input  wire        bitstream_valid,

    output reg  signed [15:0] I_out,
    output reg  signed [15:0] Q_out,
    output reg                symbol_valid,
    output wire               bitstream_ready
);
    // ----------------------------------------------------
    // Number of bits consumed for one modulation symbol
    // ----------------------------------------------------
    reg [3:0] bits_per_symbol;
    always @(*) begin
        case (sel)
            2'b00: bits_per_symbol = 4'd2;  // QPSK
            2'b01: bits_per_symbol = 4'd4;  // 16QAM
            2'b10: bits_per_symbol = 4'd6;  // 64QAM
            2'b11: bits_per_symbol = 4'd8;  // 256QAM
            default: bits_per_symbol = 4'd2;
        endcase
    end

    // ----------------------------------------------------
    // 32-bit bit buffer
    // The oldest available bit is always at the MSB side.

    // Example:
    // bitstream = b0 b1 b2 b3 b4 b5 b6 b7
    // buffer:
    // [31] [30] [29] [28] [27] [26] [25] [24]
    //  b0   b1   b2   b3   b4   b5   b6   b7
    // ----------------------------------------------------

    reg [31:0] bit_buffer;
    reg [5:0]  valid_bits;

    // ----------------------------------------------------
    // Convert [0:7] input ordering into normal [7:0] representation while preserving:
    //
    // bitstream[0] = b0 = MSB/first received bit
    // bitstream[7] = b7 = LSB/last received bit
    // ----------------------------------------------------
    wire [7:0] incoming_bits;
    assign incoming_bits = {
        bitstream[0],
        bitstream[1],
        bitstream[2],
        bitstream[3],
        bitstream[4],
        bitstream[5],
        bitstream[6],
        bitstream[7]
    };

    // ----------------------------------------------------
    // Current symbol bits
    //
    // The first 8 bits of the buffer are always the oldest available bits.
    // ----------------------------------------------------
    reg [0:7] current_bits;
    always @(*) begin
        current_bits = bit_buffer[31:24];
    end

    // ----------------------------------------------------
    // Symbol is available when enough bits are present
    // ----------------------------------------------------
    always @(*) begin
        if (valid_bits >= bits_per_symbol)
            symbol_valid = 1'b1;
        else
            symbol_valid = 1'b0;
    end

    // ----------------------------------------------------
    // Input buffer ready
    // We can accept another 8-bit input whenever there is room for it.
    // If a symbol is being consumed in the same clock, the freed space is also taken into account.
    // ----------------------------------------------------
    assign bitstream_ready = (valid_bits <= 6'd24) ||
            ((valid_bits >= bits_per_symbol) && ((valid_bits - bits_per_symbol) <= 6'd24));

    // ----------------------------------------------------
    // Next-state logic for bit buffer
    // ----------------------------------------------------
    reg [31:0] next_bit_buffer;
    reg [5:0]  next_valid_bits;
    reg [5:0] remaining_bits;
    reg [5:0] append_shift;

    always @(*) begin
        next_bit_buffer = bit_buffer;
        next_valid_bits = valid_bits;
        // First remove the bits used by the current symbol
        if (symbol_valid) begin
            next_bit_buffer = bit_buffer << bits_per_symbol;
            next_valid_bits = valid_bits - bits_per_symbol;
        end
        // Then append a new 8-bit input chunk
        // The new bits are placed immediately after the currently valid bits.
        // To consider the current clock cycle input bitstream arrival and consumption, we write logic using the 'next_valid_bits' value and not old 'valid_bits'.
        if (bitstream_valid && bitstream_ready) begin
            append_shift = 6'd32 - next_valid_bits - 6'd8;
            next_bit_buffer = next_bit_buffer | (incoming_bits << append_shift);
            next_valid_bits = next_valid_bits + 6'd8;
        end
    end

    // ----------------------------------------------------
    // Sequential bit-buffer update
    // ----------------------------------------------------
    always @(posedge clk) begin
        if (reset) begin
            bit_buffer <= 32'd0;
            valid_bits <= 6'd0;
        end
        else begin
            bit_buffer <= next_bit_buffer;
            valid_bits <= next_valid_bits;
        end
    end

    // ----------------------------------------------------
    // Gray-coded amplitude LUT for one PAM axis,
    // given N select bits (MSB..LSB)
    // ----------------------------------------------------
    function signed [15:0] pam_level;
        input [3:0] bits;   // up to 4 bits
        input [1:0] nbits;  // how many bits actually used
        reg signed [15:0] lvl;

        begin
            case (nbits)
                // 1/root(2) * 2^14 = 11585
                2'd1: lvl = bits[0] ? -16'sd11585 : 16'sd11585;

                // 1/root(10) * 2^14 = 5181
                2'd2:
                    case (bits[1:0])
                        2'b00: lvl =  16'sd5181;    // +1
                        2'b01: lvl =  16'sd15543;   // +3
                        2'b10: lvl = -16'sd5181;    // -1
                        2'b11: lvl = -16'sd15543;   // -3
                    endcase
                // 1/root(42) * 2^14 = 2528
                2'd3:
                    case (bits[2:0])
                        3'b000: lvl =  16'sd7584;   // +3
                        3'b001: lvl =  16'sd2528;   // +1
                        3'b010: lvl =  16'sd12641;  // +5
                        3'b011: lvl =  16'sd17697;  // +7
                        3'b100: lvl = -16'sd7584;   // -3
                        3'b101: lvl = -16'sd2528;   // -1
                        3'b110: lvl = -16'sd12641;  // -5
                        3'b111: lvl = -16'sd17697;  // -7
                    endcase
                // 1/root(170) * 2^14 = 1257
                2'd4:
                    case (bits[3:0])
                        4'b0000: lvl =  16'sd6283;   // +5
                        4'b0001: lvl =  16'sd8796;   // +7
                        4'b0010: lvl =  16'sd3770;   // +3
                        4'b0011: lvl =  16'sd1257;   // +1
                        4'b0100: lvl =  16'sd13823;  // +11
                        4'b0101: lvl =  16'sd11309;  // +9
                        4'b0110: lvl =  16'sd16336;  // +13
                        4'b0111: lvl =  16'sd18849;  // +15
                        4'b1000: lvl = -16'sd6283;   // -5
                        4'b1001: lvl = -16'sd8796;   // -7
                        4'b1010: lvl = -16'sd3770;   // -3
                        4'b1011: lvl = -16'sd1257;   // -1
                        4'b1100: lvl = -16'sd13823;  // -11
                        4'b1101: lvl = -16'sd11309;  // -9
                        4'b1110: lvl = -16'sd16336;  // -13
                        4'b1111: lvl = -16'sd18849;  // -15
                    endcase
                default:
                    lvl = 16'sd0;
            endcase
            pam_level = lvl;
        end
    endfunction

    // ----------------------------------------------------
    // QAM mapping
    // current_bits[0] = b(0) = first/MSB bit
    // current_bits[1] = b(1)
    // current_bits[2] = b(2)
    // ...
    //
    // Only the number of bits required by the selected modulation is consumed.
    // ----------------------------------------------------
    always @(*) begin
        I_out = 16'sd0;
        Q_out = 16'sd0;
        case (sel)
            2'b00: begin
                // QPSK: 1 bit/axis
                I_out = pam_level({3'b000, current_bits[0]}, 2'd1);
                Q_out = pam_level({3'b000, current_bits[1]}, 2'd1);
            end
            2'b01: begin
                // 16QAM: 2 bits/axis
                I_out = pam_level({2'b00, current_bits[0], current_bits[2]}, 2'd2);
                Q_out = pam_level({2'b00, current_bits[1], current_bits[3]}, 2'd2);
            end
            2'b10: begin
                // 64QAM: 3 bits/axis
                I_out = pam_level({1'b0, current_bits[0], current_bits[2], current_bits[4]}, 2'd3);
                Q_out = pam_level({1'b0, current_bits[1], current_bits[3], current_bits[5]}, 2'd3);
            end
            2'b11: begin
                // 256QAM: 4 bits/axis
                I_out = pam_level({current_bits[0], current_bits[2], current_bits[4], current_bits[6]}, 2'd4);
                Q_out = pam_level({current_bits[1], current_bits[3], current_bits[5], current_bits[7]}, 2'd4);
            end
            default: begin
                I_out = 16'sd0;
                Q_out = 16'sd0;
            end
        endcase
    end
endmodule