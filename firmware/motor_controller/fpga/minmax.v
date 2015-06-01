`timescale 1ns/1ns
module minmax
(input c,
 input [31:0] a,
 input [31:0] b,
 output [31:0] q,
 input minmax,
 input start,
 output done);

// register the inputs to help timing
wire [31:0] a_i, b_i;
d1 #(32) a_i_r(.c(c), .d(a), .q(a_i));
d1 #(32) b_i_r(.c(c), .d(b), .q(b_i));

wire [2:0] cnt;
r #(3) cnt_r
(.c(c), .rst(1'b0), .en(|cnt | start), .d(start ? 3'h1 : cnt+1'b1), .q(cnt));

assign done = cnt == 3'h5;  // maybe earlier? need to check it again

wire aleb;
fp_compare fp_compare_inst
(.clock(c), .dataa(a_i), .datab(b_i), .aleb(aleb));

r #(32) q_r
(.c(c), .rst(1'b0), .en(cnt == 3'h4), 
 .d(((aleb & minmax) | (~aleb & ~minmax)) ? a : b), .q(q));
      
endmodule

