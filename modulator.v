// Remaining - Bitstream shift
// Check - Function synthesis

module modulator (
    input  wire [1:0]  sel,        // 00:QPSK 01:16QAM 10:64QAM 11:256QAM
    // (data (3GPP) -> MSB arrives first -> leftmost) 
    // (syntax -> leftmost-MSB, rightmost-LSB; leftmost-first received and rightmost-last received)
    input  wire [0:7]  bitstream,  // up to 8 bits consumed per symbol
    output reg  signed [15:0] I_out,
    output reg  signed [15:0] Q_out
);

    // Gray-coded amplitude LUT for one PAM axis, given N select bits (MSB..LSB)
    function signed [15:0] pam_level;
        input [3:0] bits;   // up to 4 bits -> 8 levels (256QAM per axis) for I and Q separately
        input [1:0] nbits;  // how many bits actually used (1,2,3)
        reg signed [15:0] lvl;
        begin
            case (nbits)
                // 1/root(2) * ​2^14 = 11585
                2'd1: lvl = bits[0] ? -16'sd11585 : 16'sd11585;        // QPSK axis (-1:+1)
                // 1/root(10) * ​2^14 = 5181
                2'd2: case (bits[1:0])                                 // 16QAM axis
                        2'b00: lvl =  16'sd5181;    // +1
                        2'b01: lvl =  16'sd15543;   // +3
                        2'b10: lvl = -16'sd5181;    // -1
                        2'b11: lvl = -16'sd15543;   // -3
                      endcase
                // 1/root(42) * ​2^14 = 2528
                2'd3: case (bits[2:0])                                 // 64QAM axis
                        3'b000: lvl =  16'sd7584;   // +3
                        3'b001: lvl =  16'sd2528;   // +1
                        3'b010: lvl =  16'sd12641;  // +5
                        3'b011: lvl =  16'sd17697;  // +7
                        3'b100: lvl = -16'sd7584;   // -3
                        3'b101: lvl = -16'sd2528;   // -1
                        3'b110: lvl = -16'sd12641;  // -5
                        3'b111: lvl = -16'sd17697;  // -7
                      endcase
                // 1/root(170) * ​2^14 = 1257
                2'd4: case (bits[3:0])                                 // 256QAM axis
                        4'b0000: lvl =  16'sd6283;  // +5
                        4'b0001: lvl =  16'sd8796;  // +7
                        4'b0010: lvl =  16'sd3770;  // +3
                        4'b0011: lvl =  16'sd1257;  // +1
                        4'b0100: lvl =  16'sd13823; // +11
                        4'b0101: lvl =  16'sd11309; // +9
                        4'b0110: lvl =  16'sd16336; // +13
                        4'b0111: lvl =  16'sd18849; // +15
                        4'b1000: lvl = -16'sd6283;  // -5
                        4'b1001: lvl = -16'sd8796;  // -7
                        4'b1010: lvl = -16'sd3770;  // -3
                        4'b1011: lvl = -16'sd1257;  // -1
                        4'b1100: lvl = -16'sd13823; // -11
                        4'b1101: lvl = -16'sd11309; // -9
                        4'b1110: lvl = -16'sd16336; // -13
                        4'b1111: lvl = -16'sd18849; // -15
                      endcase
                default: lvl = 0;
            endcase
            pam_level = lvl;
        end
    endfunction

    always @(*) begin
        case (sel)
            2'b00: begin // QPSK: 1 bit/axis
                I_out = pam_level({3'b000, bitstream[0]}, 2'd1);
                Q_out = pam_level({3'b000, bitstream[1]}, 2'd1);
            end
            2'b01: begin // 16QAM: 2 bits/axis
                I_out = pam_level({2'b00, bitstream[0], bitstream[2]}, 2'd2);
                Q_out = pam_level({2'b00, bitstream[1], bitstream[3]}, 2'd2);
            end
            2'b10: begin // 64QAM: 3 bits/axis
                I_out = pam_level({1'b0, bitstream[0], bitstream[2], bitstream[4]}, 2'd3);
                Q_out = pam_level({1'b0, bitstream[1], bitstream[3], bitstream[5]}, 2'd3);
            end
            2'b11: begin // 256QAM: 4 bits/axis
                I_out = pam_level({bitstream[0], bitstream[2], bitstream[4], bitstream[6]}, 2'd4);
                Q_out = pam_level({bitstream[1], bitstream[3], bitstream[5], bitstream[7]}, 2'd4);
            end
            default: begin
                I_out = 16'sd0; Q_out = 16'sd0;
            end
        endcase
    end

endmodule