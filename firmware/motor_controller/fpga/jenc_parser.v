`timescale 1ns / 1ns
module jenc_parser
(input c,
 input [7:0] rxd,
 input rxdv,
 input direction,
 output [31:0] enc_usecs,
 output [31:0] enc_angle,
 output [31:0] enc_vel,
 output [15:0] enc_raw);

wire [6:0] byte_cnt;
r #(7) byte_cnt_r
(.c(c), .en(rxdv), .rst(~rxdv), .d(byte_cnt+1'b1), .q(byte_cnt));

wire [31:0] shift;
r #(32) shift_r
(.c(c), .rst(1'b0), .en(1'b1), .d({rxd, shift[31:8]}), .q(shift));

r #(32) enc_usecs_r
(.c(c), .rst(1'b0), .en(byte_cnt == 7'd8), .d(shift), .q(enc_usecs));

r #(32) enc_angle_r
(.c(c), .rst(1'b0), .en(byte_cnt == 7'd20), .d(shift), .q(enc_angle));

r #(32) enc_vel_r
(.c(c), .rst(1'b0), .en(byte_cnt == 7'd24), .d(shift), .q(enc_vel));

r #(16) enc_raw_r
(.c(c), .rst(1'b0), .en(byte_cnt == 7'd36), 
 .d(direction ? 16'd16383 - shift[15:0] : shift[15:0]), .q(enc_raw));

endmodule
