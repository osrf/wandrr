`timescale 1ns/1ns
module sqrt
(input c,
 input [31:0] d,
 output [31:0] q,
 input start,
 output done);

fp_sqrt fp_sqrt_inst(.clock(c), .data(d), .result(q));

wire [4:0] cnt;
r #(5) cnt_r
(.c(c), .rst(1'b0), .en(|cnt | start), .d(start ? 5'h1 : cnt+1'b1), .q(cnt));

assign done = cnt == 5'd29; // not sure what this should be

endmodule
