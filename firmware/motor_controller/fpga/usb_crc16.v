`timescale 1ns/1ns
//-----------------------------------------------------------------------------
// Copyright (C) 2009 OutputLogic.com 
// This source file may be used and distributed without restriction 
// provided that this copyright statement is not removed from the file 
// and that any derivative work contains the original copyright notice 
// and the associated disclaimer. 
// 
// THIS SOURCE FILE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS 
// OR IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED  
// WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE. 
//-----------------------------------------------------------------------------
// CRC module for data[7:0] ,   crc[15:0]=1+x^2+x^15+x^16;
//-----------------------------------------------------------------------------
//
// MQ 3/12/2015: minor variable name tweaks, flip bit ordering,
// add testbench, invert output, synchronous reset, etc.

module usb_crc16(
  input [7:0] d,
  input dv,
  output [15:0] crc,
  input rst,
  input c);

  reg [15:0] crc_q, crc_d;

  // flip the input and output bit vectors, and invert the output...
  wire [7:0] df = { d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7] };
  assign crc = { ~crc_q[ 0], ~crc_q[ 1], ~crc_q[ 2], ~crc_q[ 3],
                 ~crc_q[ 4], ~crc_q[ 5], ~crc_q[ 6], ~crc_q[ 7],
                 ~crc_q[ 8], ~crc_q[ 9], ~crc_q[10], ~crc_q[11],
                 ~crc_q[12], ~crc_q[13], ~crc_q[14], ~crc_q[15]  };

  always @(*) begin
    crc_d[0] = crc_q[8] ^ crc_q[9] ^ crc_q[10] ^ crc_q[11] ^ crc_q[12] ^ crc_q[13] ^ crc_q[14] ^ crc_q[15] ^ df[0] ^ df[1] ^ df[2] ^ df[3] ^ df[4] ^ df[5] ^ df[6] ^ df[7];
    crc_d[1] = crc_q[9] ^ crc_q[10] ^ crc_q[11] ^ crc_q[12] ^ crc_q[13] ^ crc_q[14] ^ crc_q[15] ^ df[1] ^ df[2] ^ df[3] ^ df[4] ^ df[5] ^ df[6] ^ df[7];
    crc_d[2] = crc_q[8] ^ crc_q[9] ^ df[0] ^ df[1];
    crc_d[3] = crc_q[9] ^ crc_q[10] ^ df[1] ^ df[2];
    crc_d[4] = crc_q[10] ^ crc_q[11] ^ df[2] ^ df[3];
    crc_d[5] = crc_q[11] ^ crc_q[12] ^ df[3] ^ df[4];
    crc_d[6] = crc_q[12] ^ crc_q[13] ^ df[4] ^ df[5];
    crc_d[7] = crc_q[13] ^ crc_q[14] ^ df[5] ^ df[6];
    crc_d[8] = crc_q[0] ^ crc_q[14] ^ crc_q[15] ^ df[6] ^ df[7];
    crc_d[9] = crc_q[1] ^ crc_q[15] ^ df[7];
    crc_d[10] = crc_q[2];
    crc_d[11] = crc_q[3];
    crc_d[12] = crc_q[4];
    crc_d[13] = crc_q[5];
    crc_d[14] = crc_q[6];
    crc_d[15] = crc_q[7] ^ crc_q[8] ^ crc_q[9] ^ crc_q[10] ^ crc_q[11] ^ crc_q[12] ^ crc_q[13] ^ crc_q[14] ^ crc_q[15] ^ df[0] ^ df[1] ^ df[2] ^ df[3] ^ df[4] ^ df[5] ^ df[6] ^ df[7];

  end // always

  always @(posedge c) begin
    if(rst)
      crc_q <= {16{1'b1}};
    else 
      crc_q <= dv ? crc_d : crc_q;
  end // always
endmodule // crc

`ifdef TEST_USB_CRC16
module tb();

wire c;
task tick;
  begin
    wait(c);
    wait(~c);
  end
endtask

reg [7:0] d;
wire [15:0] crc;
reg dv, rst;
sim_clk #(125) clk_125_r(.clk(c));

usb_crc16 usb_crc16_inst(.*);

initial begin
  $dumpfile("crc16.lxt");
  $dumpvars();
  rst <= 1'b0;
  dv <= 1'b0;
  #100;
  tick();
  rst <= 1'b1;
  tick();
  rst <= 1'b0;
  tick();
  dv <= 1'b1;
  d <= 8'h0;
  tick();
  d <= 8'h1;
  tick();
  d <= 8'h2;
  tick();
  d <= 8'h3;
  tick();
  dv <= 1'b0;
  tick();
  tick();
  $finish();
end
endmodule
`endif

