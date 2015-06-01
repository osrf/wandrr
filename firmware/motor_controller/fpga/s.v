`timescale 1ns/1ns
// generic synthronizer
module s
#(parameter W=1,
  parameter S=2) // number of synchronizer stages 
(input c,
 input [W-1:0] d,
 output [W-1:0] q);

wire [W*S-1:0] shift;
r #(W*S) shift_r
  (.c(c), .rst(1'b0), .en(1'b1),
   .d({shift[W*(S-1)-1:0], d}), .q(shift));
assign q = shift[W*S-1:W*(S-1)];

endmodule
