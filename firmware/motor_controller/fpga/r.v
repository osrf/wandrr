`timescale 1ns/1ns
module r
#(parameter WIDTH=1,
  parameter INIT={WIDTH{1'b0}},
  parameter ASYNC_RESET=0)
(input c,
 input [WIDTH-1:0] d,
 input rst,
 input en,
 output reg [WIDTH-1:0] q);

initial q = INIT;

generate if (ASYNC_RESET)
  always @(posedge c or posedge rst) begin
    if (rst)
      q <= INIT;
    else if (en)
      q <= d;
  end
else
  always @(posedge c) begin
    if (rst)
      q <= INIT;
    else if (en)
      q <= d;
  end
endgenerate

endmodule
