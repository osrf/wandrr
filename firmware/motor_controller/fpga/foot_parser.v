`timescale 1ns / 1ns
module foot_parser
(input c,
 input [7:0] rxd,
 input rxdv,
 output [31:0]  usecs,
 output [255:0] pressures);

wire [6:0] byte_cnt;
r #(7) byte_cnt_r
(.c(c), .en(rxdv), .rst(~rxdv), .d(byte_cnt+1'b1), .q(byte_cnt));

wire [31:0] shift;
r #(32) shift_r
(.c(c), .rst(1'b0), .en(1'b1), .d({rxd, shift[31:8]}), .q(shift));

r #(32) usecs_r
(.c(c), .rst(1'b0), .en(byte_cnt == 7'd8), .d(shift), .q(usecs));

localparam NUM_PRESSURE_BYTES = 32;
genvar i;
generate
for (i = 0; i < NUM_PRESSURE_BYTES; i = i + 1) begin: gen_pressure_bytes
  r #(8) pressure_byte
  (.c(c), .rst(1'b0), .en(byte_cnt == 7'd8 + i), 
   .d(rxd), .q(pressures[(i+1)*8-1:i*8]));
end
endgenerate

endmodule
