`timescale 1ns/1ns
module op_sin
(input c,
 input  [31:0] d,
 output [31:0] q,
 input start,
 output done);

// register the inputs to help timing
wire [31:0] d_i;
d1 #(32) d_i_r(.c(c), .d(d), .q(d_i));

sin sin_inst
(.clock(c), .data(d_i), .result(q));

wire [5:0] cnt;
r #(6) cnt_r
(.c(c), .rst(1'b0), .en(|cnt | start), .d(start ? 4'h1 : cnt+1'b1), .q(cnt));

assign done = cnt == 6'd38; // not sure what this should be

endmodule
 

