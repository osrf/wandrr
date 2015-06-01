`timescale 1ns/1ns
module op_usmod
(input c,
 input [15:0] a,
 input [15:0] b,
 output [15:0] q,
 input start,
 output done);

// register the inputs to help timing
wire [15:0] a_i, b_i;
d1 #(16) a_i_r(.c(c), .d(a), .q(a_i));
d1 #(16) b_i_r(.c(c), .d(b), .q(b_i));

lpm_divide 
#(.lpm_widthn(16),
  .lpm_widthd(16),
  .lpm_nrepresentation("UNSIGNED"),
  .lpm_drepresentation("UNSIGNED"),
  .lpm_hint("LPM_REMAINDERPOSITIVE=TRUE"),
  .lpm_pipeline(8) // not sure what's best here
 ) div_inst
(.numer(a_i), .denom(b_i), .clock(c), .clken(1'b1), .aclr(1'b0),
 .quotient(), .remain(q));

wire [4:0] cnt;
r #(5) cnt_r
(.c(c), .rst(1'b0), .en(|cnt | start), .d(start ? 5'h1 : cnt+1'b1), .q(cnt));

assign done = cnt == 5'd10; // not sure what this should be

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
