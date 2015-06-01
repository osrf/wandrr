`timescale 1ns/1ns
module menc_sincos
(input c,
 input [13:0] menc,
 input [10:0] offset,
 input [7:0] poles,
 output [31:0] sin,
 output [31:0] cos);

wire toggle;
r toggle_r(.c(c), .en(1'b1), .rst(1'b0), .d(~toggle), .q(toggle));

wire [10:0] sin_addr, cos_addr;
d1 #(11) sin_addr_r(.c(c), .d(menc + offset), .q(sin_addr));
d1 #(11) cos_addr_r(.c(c), .d(menc + offset + 11'd512), .q(cos_addr));

wire [10:0] table_addr = toggle ? sin_addr : cos_addr;

wire [31:0] table_q;
sine_table_11bit_float32 table_inst
(.c(c), .angle(table_addr), .sine(table_q));

r #(32) sin_r(.c(c), .en(~toggle), .rst(1'b0), .d(~(~table_q)), .q(sin));
r #(32) cos_r(.c(c), .en( toggle), .rst(1'b0), .d(~(~table_q)), .q(cos));

endmodule

////////////////////////////////////////////////////////////////

`ifdef test_menc_sincos

module menc_sincos_tb();

wire c;
sim_clk #(125) clk_125(.clk(c));

reg [10:0] offset;
reg [10:0] menc;
wire [31:0] sin, cos;

menc_sincos dut(.*);
initial begin
  $dumpfile("menc_sincos.lxt");
  $dumpvars();
  offset = 11'h0;
  menc = 11'h50;
  $display("testing...");
  wait(c);
  wait(~c);
  #1000 $finish();
end
endmodule

`endif
