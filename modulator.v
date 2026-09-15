// Remaining - Normalization, Bitstream shift, MSB-LSB on Tx and Rx (MSB arrives first in 3GPP)
//             Increase I_out and Q_out size/capacity of bits 
// Check - Function synthesis

module modulator (
    input  wire [1:0]  sel,        // 00:QPSK 01:16QAM 10:64QAM 11:256QAM
    input  wire [0:7]  bitstream,  // up to 8 bits consumed per symbol (MSB arrives first)
    output reg  signed [3:0] I_out,
    output reg  signed [3:0] Q_out
);

    // Gray-coded amplitude LUT for one PAM axis, given N select bits (MSB..LSB)
    function signed [3:0] pam_level;
        input [3:0] bits;   // up to 4 bits -> 8 levels (256QAM per axis)
        input [1:0] nbits;  // how many bits actually used (1,2,3)
        reg signed [3:0] lvl;
        begin
            case (nbits)
                2'd1: lvl = bits[0] ? -1 : 1;                          // QPSK axis
                2'd2: case (bits[1:0])                                 // 16QAM axis
                        2'b00: lvl =  1; 2'b01: lvl =  3;
                        2'b10: lvl = -1; 2'b11: lvl = -3;
                      endcase
                2'd3: case (bits[2:0])                                 // 64QAM axis
                        3'b000: lvl =  3; 3'b001: lvl =  1;
                        3'b010: lvl =  5; 3'b011: lvl =  7;
                        3'b100: lvl = -3; 3'b101: lvl = -1;
                        3'b110: lvl = -5; 3'b111: lvl = -7;
                      endcase
                2'd4: case (bits[3:0])                                 // 256QAM axis
                        4'b0000: lvl =  5; 4'b0001: lvl =  7;
                        4'b0010: lvl =  3; 4'b0011: lvl =  1;
                        4'b0100: lvl = 11; 4'b0101: lvl =  9;
                        4'b0110: lvl = 13; 4'b0111: lvl = 15;
                        4'b1000: lvl = -5; 4'b1001: lvl = -7;
                        4'b1010: lvl = -3; 4'b1011: lvl = -1;
                        4'b1100: lvl =-11; 4'b1101: lvl = -9;
                        4'b1110: lvl =-13; 4'b1111: lvl =-15;
                      endcase
                default: lvl = 0;
            endcase
            pam_level = lvl;
        end
    endfunction

    always @(*) begin
        case (sel)
            2'b00: begin // QPSK: 1 bit/axis
                I_out = pam_level({2'b0, bitstream[0]}, 2'd1);
                Q_out = pam_level({2'b0, bitstream[1]}, 2'd1);
            end
            2'b01: begin // 16QAM: 2 bits/axis
                I_out = pam_level({1'b0, bitstream[0, 2]}, 2'd2);
                Q_out = pam_level({1'b0, bitstream[1, 3]}, 2'd2);
            end
            2'b10: begin // 64QAM: 3 bits/axis
                I_out = pam_level(bitstream[0, 2, 4], 2'd3);
                Q_out = pam_level(bitstream[1, 3, 5], 2'd3);
            end
            2'b11: begin // 256QAM: 4 bits/axis
                I_out = pam_level(bitstream[0, 2, 4, 6], 2'd4);
                Q_out = pam_level(bitstream[1, 3, 5, 7], 2'd4);
            end
            default: begin
                I_out = 0; Q_out = 0;
            end
        endcase
    end

endmodule