`timescale 1ns/1ns
module sat
(input c,
 input [47:0] d,
 output [15:0] q);

// we need to pick the "middle" 16 bits from this 48-bit vector, and saturate
// as needed

reg [15:0] s; // saturated
always @* begin
  if (d[47:23] == {25{1'b0}} ||
      d[47:23] == {25{1'b1}})
    s = d[23:8];
  else if (d[47])
    s = 16'h8000; // most negative number
  else
    s = 16'h7fff; // most positive number
end
d1 #(16) out_r(.c(c), .d(s), .q(q));

endmodule
