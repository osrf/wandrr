`timescale 1ns/1ns
module div
(input c,
 input [31:0] a,
 input [31:0] b,
 output [31:0] q,
 input start,
 output done);

fp_div fp_div_inst(.clock(c), .dataa(a), .datab(b), .result(q));

wire [4:0] cnt;
r #(5) cnt_r
(.c(c), .rst(1'b0), .en(|cnt | start), .d(start ? 5'h1 : cnt+1'b1), .q(cnt));

assign done = cnt == 5'd16; // not sure what this should be

endmodule

////////////////////////////////////////////////////////////////////

`ifdef test_div
module div_tb();
reg [31:0] a, b;
wire [31:0] q;
reg start;
wire done;
wire c;
sim_clk #(125) clk_125(.clk(c));
div dut(.*);
initial begin
  $dumpfile("div.lxt");
  $dumpvars();
  start = 1'b0;
  a = 32'h3f80_0000; // 1.0
  b = 32'h3fcc_b55a; // 1.59928441047
  wait(~c);
  wait(c);
  start <= 1'b1;
  wait(~c);
  wait(c);
  start <= 1'b0;
  #10000;
  $finish();
end
endmodule
`endif
