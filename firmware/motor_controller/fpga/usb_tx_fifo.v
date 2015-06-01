`timescale 1ns/1ns
module usb_tx_fifo
(input c,
 input c_48,
 input [7:0] d,
 input dv,
 input read,
 output q,
 output empty);

wire fifo_read, fifo_empty;
wire fifo_q;

dcfifo_mixed_widths
#(.lpm_width(8), .lpm_width_r(1), .lpm_widthu(7), .lpm_widthu_r(10),
  .lpm_numwords(128), .lpm_showahead("ON"),
  .use_eab("ON"), .intended_device_family("CYCLONE V")) xclk_fifo
(.wrclk(c), .data(d), .wrreq(dv),
 .rdclk(c_48), .rdreq(read), .q(q), .rdempty(empty),
 .aclr(1'b0));

endmodule
