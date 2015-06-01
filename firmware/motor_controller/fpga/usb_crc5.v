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
// CRC module for data[10:0] ,   crc[4:0]=1+x^2+x^5;
//-----------------------------------------------------------------------------
// MQ 3/7/2015: invert output
module usb_crc5(
  input [10:0] data_in,
  input crc_en,
  output [4:0] crc_out,
  input rst,
  input clk);

  reg [4:0] lfsr_q,lfsr_c;

  assign crc_out = ~lfsr_q;

  always @(*) begin
    lfsr_c[0] = lfsr_q[0] ^ lfsr_q[3] ^ lfsr_q[4] ^ data_in[0] ^ data_in[3] ^ data_in[5] ^ data_in[6] ^ data_in[9] ^ data_in[10];
    lfsr_c[1] = lfsr_q[0] ^ lfsr_q[1] ^ lfsr_q[4] ^ data_in[1] ^ data_in[4] ^ data_in[6] ^ data_in[7] ^ data_in[10];
    lfsr_c[2] = lfsr_q[0] ^ lfsr_q[1] ^ lfsr_q[2] ^ lfsr_q[3] ^ lfsr_q[4] ^ data_in[0] ^ data_in[2] ^ data_in[3] ^ data_in[6] ^ data_in[7] ^ data_in[8] ^ data_in[9] ^ data_in[10];
    lfsr_c[3] = lfsr_q[1] ^ lfsr_q[2] ^ lfsr_q[3] ^ lfsr_q[4] ^ data_in[1] ^ data_in[3] ^ data_in[4] ^ data_in[7] ^ data_in[8] ^ data_in[9] ^ data_in[10];
    lfsr_c[4] = lfsr_q[2] ^ lfsr_q[3] ^ lfsr_q[4] ^ data_in[2] ^ data_in[4] ^ data_in[5] ^ data_in[8] ^ data_in[9] ^ data_in[10];

  end // always

  always @(posedge clk, posedge rst) begin
    if(rst) begin
      lfsr_q <= {5{1'b1}};
    end
    else begin
      lfsr_q <= crc_en ? lfsr_c : lfsr_q;
    end
  end // always
endmodule // crc

`ifdef TEST_USB_CRC5
module tb();
reg [10:0] data_in;
reg crc_en;
wire [4:0] crc_out;
reg rst;
wire clk;
sim_clk #(125) clk_125_r(.clk(clk));
usb_crc5 dut(.*);
wire [4:0] to_wire = ~crc_out; //{ crc_out[0], crc_out[1], crc_out[2], crc_out[3], crc_out[4] };
initial begin
  $dumpfile("crc5.lxt");
  $dumpvars();
  rst <= 1'b0;
  crc_en <= 1'b0;
  wait(clk);
  wait(~clk);
  rst <= 1'b1;
  wait(clk);
  wait(~clk);
  rst <= 1'b0;
  wait(clk);
  wait(~clk);
  data_in <= 11'b10000000000;
  crc_en <= 1'b1;
  wait(clk);
  wait(~clk);
  rst <= 1'b1;
  crc_en <= 1'b0;
  wait(clk);
  wait(~clk);
  rst <= 1'b0;
  crc_en <= 1'b1;
  data_in <= 11'b01000000000;
  #1000;
  $finish();
end
endmodule
`endif

