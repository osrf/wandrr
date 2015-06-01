`timescale 1ns/1ns
module usb_crossbar
(input c,
 input [15:0] sel, // each nibble says which data source to hear
 input [39:0] d,   // data streaming out of the USB hosts
 input [4:0] dv,   // data-valid from the USB hosts
 output [31:0] q,  // output data to the parsers
 output [3:0] qv); // output data-valids to the parsers

wire [31:0] dmux_z;
wire [ 3:0] dvmux_z;

gmux #(.DWIDTH(8), .SELWIDTH(3)) dmux0
(.d({24'h0, d}), .sel(sel[2:0]), .z(dmux_z[7:0]));

gmux #(.DWIDTH(8), .SELWIDTH(3)) dmux1
(.d({24'h0, d}), .sel(sel[6:4]), .z(dmux_z[15:8]));

gmux #(.DWIDTH(8), .SELWIDTH(3)) dmux2
(.d({24'h0, d}), .sel(sel[10:8]), .z(dmux_z[23:16]));

gmux #(.DWIDTH(8), .SELWIDTH(3)) dmux3
(.d({24'h0, d}), .sel(sel[14:12]), .z(dmux_z[31:24]));

///////////////////////

gmux #(.DWIDTH(1), .SELWIDTH(3)) dvmux0
(.d({3'h0, dv}), .sel(sel[2:0]), .z(dvmux_z[0]));

gmux #(.DWIDTH(1), .SELWIDTH(3)) dvmux1
(.d({3'h0, dv}), .sel(sel[6:4]), .z(dvmux_z[1]));

gmux #(.DWIDTH(1), .SELWIDTH(3)) dvmux2
(.d({3'h0, dv}), .sel(sel[10:8]), .z(dvmux_z[2]));

gmux #(.DWIDTH(1), .SELWIDTH(3)) dvmux3
(.d({3'h0, dv}), .sel(sel[14:12]), .z(dvmux_z[3]));

///////////////////////

d1 #(32) q_d1_r(.c(c), .d(dmux_z), .q(q));

d1 #(4) qv_d1_r(.c(c), .d(dvmux_z),. q(qv));

endmodule
