`timescale 1ns/1ns
module mul
(input c,
 input  [31:0] a,
 input  [31:0] b,
 output [31:0] q,
 input start,
 output done);

// register the inputs to help timing
wire [31:0] a_i, b_i;
d1 #(32) a_i_r(.c(c), .d(a), .q(a_i));
d1 #(32) b_i_r(.c(c), .d(b), .q(b_i));

mult mult_inst
(.clock(c), .dataa(a_i), .datab(b_i), .result(q));

wire [3:0] cnt;
r #(4) cnt_r
(.c(c), .rst(1'b0), .en(|cnt | start), .d(start ? 4'h1 : cnt+1'b1), .q(cnt));

assign done = cnt == 4'd9; // not sure what this should be

endmodule
 
