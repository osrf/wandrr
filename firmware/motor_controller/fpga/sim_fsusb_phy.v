`timescale 1ps / 1ps
module sim_fsusb_phy
(inout dp,  // to usb cable
 inout dm,  // to usb cable
 input oe_n,
 input pwr,
 inout vp,  // to controller chip
 inout vm); // to controller chip

assign dp = ~oe_n ? vp : 1'bz;
assign dm = ~oe_n ? vm : 1'bz;

pullup(dp);
pulldown(dm);

assign vp = oe_n ? dp : 1'bz;
assign vm = oe_n ? dm : 1'bz;

endmodule
