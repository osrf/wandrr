`timescale 1ns/1ns
module sync
#(parameter W=1,
  parameter S=2) // number of synchronizer stages 
(input [W-1:0] in,
 input clk,
 output [W-1:0] out);

wire [W*S-1:0] shift;
r #(W*S) shift_r
  (.c(clk), .rst(1'b0), .en(1'b1),
   .d({shift[W*(S-1)-1:0], in}), .q(shift));
assign out = shift[W*S-1:W*(S-1)];

endmodule
