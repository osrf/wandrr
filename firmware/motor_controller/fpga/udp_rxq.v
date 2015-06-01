`timescale 1ns / 1ns
module udp_rxq
(input clk,
 input [7:0]  rxd,  input rxdv, input rxlast,
 output [7:0] qrxd, output qrxdv, output qrxlast);

// this module accepts the off-the-wire UDP payload and buffers it until
// we can determine if this was a valid UDP payload or not. (i.e., if the
// ethernet FCS matched, and if the IP and UDP headers were sane)

//////////////////////////////////////////////////////////////////
// input section
 
// we're getting one word per 4 clocks.
wire [3:0] rxdv_shift;
wire rxdv_d4 = rxdv_shift[3];
r #(4) rxdv_shift_r
(.c(clk), .d({rxdv_shift[2:0], rxdv}), .rst(1'b0), .en(1'b1), .q(rxdv_shift));
wire rxdv_end = ~rxdv & rxdv_d4;

wire [31:0] rxd_shift;
wire [7:0] rxd_d4 = rxd_shift[31:24];
r #(32) rxd_shift_r
(.c(clk), .rst(1'b0), .en(1'b1), .d({rxd_shift[23:0], rxd}), .q(rxd_shift));

wire rxlast_d1, rxlast_d2, rxlast_d3, rxlast_d4;
r rxlast_d1_r(.c(clk), .rst(1'b0), .en(1'b1), .d(rxlast), .q(rxlast_d1));
r rxlast_d2_r(.c(clk), .rst(1'b0), .en(1'b1), .d(rxlast_d1), .q(rxlast_d2));
r rxlast_d3_r(.c(clk), .rst(1'b0), .en(1'b1), .d(rxlast_d2), .q(rxlast_d3));
r rxlast_d4_r(.c(clk), .rst(1'b0), .en(1'b1), .d(rxlast_d3), .q(rxlast_d4));

wire [10:0] dfifo_usedw;
wire dfifo_empty;
wire dfifo_rdreq;
wire [8:0] dfifo_q;
scfifo #(.lpm_width(9),
         .lpm_numwords(2048),
         .lpm_widthu(11),
         .lpm_showahead("ON"),
         .use_eab("ON"),
         .intended_device_family("CYCLONE V")) dfifo // data fifo
  (.clock(clk),
   .wrreq(rxdv_d4), .data({rxdv_end, rxd_d4}),
   .rdreq(dfifo_rdreq), .q(dfifo_q),
   .empty(dfifo_empty),
   .usedw(dfifo_usedw),
   .aclr(1'b0), .sclr(1'b0));

assign qrxd = dfifo_q[7:0];
// shift through 22 samples of rxlast in case we see any that are high
// which indicates that (after potentially up to 22 octets on short packets)
// the FCS and length fields all checked out OK
// (of course this could be short-circuited for long packets; revisit this
// design if that ever matters).
localparam RXLAST_SHIFT_LEN = 26;
wire [RXLAST_SHIFT_LEN-1:0] rxlast_shift;
r #(RXLAST_SHIFT_LEN) rxlast_shift_r
  (.c(clk), .en(1'b1), .rst(1'b0), 
   .d({rxlast_shift[RXLAST_SHIFT_LEN-2:0], rxlast_d4}), .q(rxlast_shift));
wire [RXLAST_SHIFT_LEN-1:0] rxdv_end_shift;
r #(RXLAST_SHIFT_LEN) rxdv_end_shift_r
  (.c(clk), .en(1'b1), .rst(1'b0),
   .d({rxdv_end_shift[RXLAST_SHIFT_LEN-2:0], rxdv_end}),
   .q(rxdv_end_shift));

wire vfifo_wrreq = rxdv_end_shift[RXLAST_SHIFT_LEN-1];
wire found_rxlast = |rxlast_shift;
wire vfifo_q;

wire vfifo_rdreq, vfifo_empty;
wire [4:0] vfifo_usedw;
scfifo #(.lpm_width(1), 
         .lpm_numwords(32), 
         .lpm_widthu(5), 
         .lpm_showahead("ON"),
         .intended_device_family("CYCLONE V")) vfifo // valid fifo
  (.clock(clk),
   .wrreq(vfifo_wrreq), .rdreq(vfifo_rdreq),
   .empty(vfifo_empty), .usedw(vfifo_usedw),
   .data(found_rxlast), .q(vfifo_q), 
   .aclr(1'b0), .sclr(1'b0));

//////////////////////////////////////////////////////////////////
// output section

wire pkt_draining, pkt_draining_en;
r pkt_draining_r(.c(clk), .rst(1'b0), .en(1'b1),
                 .d(pkt_draining_en), .q(pkt_draining));

assign pkt_draining_en = pkt_draining ? 
                         ~dfifo_q[8] : // stop when you hit a prior rxdv_end
                         ~vfifo_empty;
assign qrxdv   = pkt_draining & vfifo_q; // mask qrxdv on its rxlast flag
assign qrxlast = pkt_draining & dfifo_q[8] & vfifo_q;
assign dfifo_rdreq = pkt_draining;
assign vfifo_rdreq = pkt_draining & dfifo_q[8];
  
endmodule
