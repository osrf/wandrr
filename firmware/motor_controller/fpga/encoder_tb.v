`timescale 1ns/1ns
module encoder_tb();

initial begin
  $dumpfile("encoder.lxt");
  $dumpvars();
  #2_000_000 $finish();
end

wire clk_125, clk_48;
sim_clk #(125) sim_clk_125_inst(.clk(clk_125));
sim_clk #(48)  sim_clk_48_inst (.clk(clk_48));

wire [7:0] ep1_rxd;
wire ep1_rxdv;
wire usb_vp, usb_vm, usb_oe;
fsusb fsusb_inst
(.c(clk_125), .c_48(clk_48), .en(1'b1), 
 .vp(usb_vp), .vm(usb_vm), 
 .oe_n(usb_oe), .pwr(),
 .ep1_rxd(ep1_rxd), .ep1_rxdv(ep1_rxdv));

wire usb_dp, usb_dm;
sim_fsusb_phy sim_usb_phy
(.vp(usb_vp), .vm(usb_vm), .oe_n(usb_oe), .pwr(1'b1),
 .dp(usb_dp), .dm(usb_dm));

sim_fsusb_encoder sim_enc(.dp(usb_dp), .dm(usb_dm));

menc_parser menc_parser_inst
(.c(clk_125), .rxd(ep1_rxd), .rxdv(ep1_rxdv));

endmodule
