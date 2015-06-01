`timescale 1ns/1ns
module clamp_signed
(input c,
 input [15:0] clamp,
 input signed [15:0] d,
 input dv,
 output signed [15:0] q,
 output qv);

wire signed [15:0] clamp_pos;
r #(16) clamp_pos_r
(.c(c), .rst(1'b0), .en(dv), 
 .d(d > $signed(clamp) ? $signed(clamp) : d), .q(clamp_pos));

wire dv_d1;
d1 dv_d1_r(.c(c), .d(dv), .q(dv_d1));

r #(16) clamp_neg_r
(.c(c), .rst(1'b0), .en(dv_d1), 
 .d(clamp_pos < -$signed(clamp) ? -$signed(clamp) : clamp_pos), .q(q));

d1 dv_d2_r(.c(c), .d(dv_d1), .q(qv));

endmodule
