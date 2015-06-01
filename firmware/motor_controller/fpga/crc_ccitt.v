module crc_ccitt 
#(parameter INIT = 16'hffff)
(input clk, input rst, input [7:0] d, input dv, output [15:0] crc);

wire [15:0] next_crc;
wire [15:0] crc_d = (rst ? INIT : next_crc);
r #(16) crc_r(.c(clk), .rst(1'b0), .en(dv | rst), .d(crc_d), .q(crc));

assign next_crc[0]  = crc[8] ^ crc[12] ^ d[0] ^ d[4];
assign next_crc[1]  = crc[9] ^ crc[13] ^ d[1] ^ d[5];
assign next_crc[2]  = crc[10] ^ crc[14] ^ d[2] ^ d[6];
assign next_crc[3]  = crc[11] ^ crc[15] ^ d[3] ^ d[7];
assign next_crc[4]  = crc[12] ^ d[4];
assign next_crc[5]  = crc[8] ^ crc[12] ^ crc[13] ^ d[0] ^ d[4] ^ d[5];
assign next_crc[6]  = crc[9] ^ crc[13] ^ crc[14] ^ d[1] ^ d[5] ^ d[6];
assign next_crc[7]  = crc[10] ^ crc[14] ^ crc[15] ^ d[2] ^ d[6] ^ d[7];
assign next_crc[8]  = crc[0] ^ crc[11] ^ crc[15] ^ d[3] ^ d[7];
assign next_crc[9]  = crc[1] ^ crc[12] ^ d[4];
assign next_crc[10] = crc[2] ^ crc[13] ^ d[5];
assign next_crc[11] = crc[3] ^ crc[14] ^ d[6];
assign next_crc[12] = crc[4] ^ crc[8] ^ crc[12] ^ crc[15] ^ d[0] ^ d[4] ^ d[7];
assign next_crc[13] = crc[5] ^ crc[9] ^ crc[13] ^ d[1] ^ d[5];
assign next_crc[14] = crc[6] ^ crc[10] ^ crc[14] ^ d[2] ^ d[6];
assign next_crc[15] = crc[7] ^ crc[11] ^ crc[15] ^ d[3] ^ d[7];

endmodule
