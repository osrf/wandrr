`timescale 1ns/1ns
module sat32
(input  c,
 input  [63:0] d,
 output [31:0] q);

// pick the "middle" 32 bits from this 64-bit vector, and saturate as needed

reg [31:0] s; // saturated output
always @(posedge c) begin
  if (d[63:47] == {17{1'b0}} ||
      d[63:47] == {17{1'b1}})
    s = d[47:16];
  else if (d[63])
    s = 32'h8000_0000; // most negative number
  else
    s = 32'h7fff_0000; // most positive number
end
d1 #(32) out_r(.c(c), .d(s), .q(q));

endmodule
