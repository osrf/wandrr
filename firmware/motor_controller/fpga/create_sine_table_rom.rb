#!/usr/bin/env ruby

File.open('sine_table_11bit_float32.v','w') do |f|
  f.puts <<EOHEADER
`timescale 1ns/1ns
module sine_table_11bit_float32
(input c,
 input      [10:0] angle,
 output reg [31:0] sine);

always @(posedge c) begin
  case (angle)
EOHEADER
  2048.times do |i|
    s = Math.sin(i/2048.0*2.0*3.1415926) 
    f.puts "    11'd#{i}: sine = 32'h#{[s].pack('F').unpack('L')[0].to_s(16)};"
  end
  f.puts "  endcase"
  f.puts "end"
  f.puts "endmodule"
end
