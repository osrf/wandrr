`timescale 1ns/1ns
module foc_consts_32x32
(input c,
 input      [ 4:0] addr,
 output reg [31:0] q);

`include "ops.v"

initial q = 32'h0;

always @(posedge c) begin
  case (addr)
    5'h00: q = 32'hc680_0000; // -16384.0
    5'h01: q = 32'h3bd5_5555; //  0.00651041667 = "ADC ticks" to amps scalar
    5'h02: q = 32'h3f2a_aaab; //  0.66667
    5'h03: q = 32'hbeaa_aaab; // -0.33333
    5'h04: q = 32'h3f13_cd36; //  0.57735
    5'h05: q = 32'hbf13_cd36; // -0.57735
    5'h06: q = 32'hbf80_0000; // -1.0
    5'h07: q = 32'hbf00_0000; // -0.5
    5'h08: q = 32'h3f5d_ddde; // 0.86667
    5'h09: q = 32'h451f_f000; // 2559 (the 0-volt output level)
    5'h0a: q = 32'h424c_b852; // 2559 / 50volt = 51.18
    5'h0b: q = 32'h459f_f800; // 5119 (maximum pwm value)
    5'h0c: q = 32'h3856_bf95; // adc cycle time @ 19.531 Hz = 51.2 usec
    5'h0d: q = 32'h3f00_0000; // 0.5
    5'h0e: q = 32'h4680_0000; // 16384.0
    5'h0f: q = 32'h40c9_0fdb; // 2 * pi = 6.28318
    5'h10: q = 32'h3fc9_0fdb; // pi/2 = 1.570796
    5'h11: q = 32'h39c9_0fdb; // 2*pi/16384 = 3.83495e-4 
    5'h12: q = 32'h0;
    5'h13: q = 32'h0;
    5'h14: q = 32'h0;
    5'h15: q = 32'h0;
    5'h16: q = 32'h0;
    5'h17: q = 32'h0;
    5'h18: q = 32'h0;
    5'h19: q = 32'h0;
    5'h1a: q = 32'h0;
    5'h1b: q = 32'h0;
    5'h1c: q = 32'h0;
    5'h1d: q = 32'h0;
    5'h1e: q = 32'h0;
    5'h1f: q = 32'h0; // leave this as a zero constant
  endcase
end
endmodule
