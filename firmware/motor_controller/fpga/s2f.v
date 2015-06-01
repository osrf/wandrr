`timescale 1ns/1ns
module s2f // short (16-bit) integer to float32
(input c,
 input [15:0] d,
 output [31:0] q,
 input start,
 output done);

int16_float32 int16_float32_inst
(.clock(c), .dataa(d), .result(q));

wire [2:0] cnt;
r #(3) cnt_r
(.c(c), .rst(1'b0), .en(|cnt | start), .d(start ? 3'h1 : cnt+1'b1), .q(cnt));

assign done = cnt == 3'd4; // not sure what this should be

endmodule
