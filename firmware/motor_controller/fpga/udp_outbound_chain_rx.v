`timescale 1ns/1ns
module udp_outbound_chain_rx
(input         clk_50,
 input         clk_100,
 input   [7:0] rxd,
 input         rxdv,
 input         rxlast, // fires HIGH when udp_rx verifies the checksum
 output [15:0] hop_count,
 output  [7:0] submsg_rxd,
 output        submsg_rxdv,
 output        submsg_rxlast);

// buffer the incoming packet here until we get an rxlast signal, which
// indicates the UDP packet was sane.
wire [7:0] qrxd;
wire qrxdv, qrxlast;
udp_rxq udp_rxq_inst(.clk(clk_50), .rxd(rxd), .rxdv(rxdv), .rxlast(rxlast),
                     .qrxd(qrxd), .qrxdv(qrxdv), .qrxlast(qrxlast));

//////////////////////////////////////////////////////////////////////////

localparam SW = 5, CW = 5;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r
  (.c(clk_50), .rst(1'b0), .en(1'b1), .d(next_state), .q(state));

localparam ST_IDLE           = 5'h0;
localparam ST_PROTO_VER      = 5'h1;
localparam ST_HOP_COUNT      = 5'h2;
localparam ST_SUBMSG_ADDR    = 5'h3;
localparam ST_SUBMSG_LEN     = 5'h4;
localparam ST_SUBMSG_PAYLOAD = 5'h5;
localparam ST_SUBMSG_WAIT_FOR_RXLAST = 5'h6; // wait for pad + CRC check
localparam ST_MFIFO_DRAIN    = 5'h7; // send rx submsg to rest of chip
localparam ST_DISCARD        = 5'h8; // bogus

wire rx_cnt_rst;
wire [10:0] rx_cnt;
r #(11) rx_cnt_r
(.c(clk_50), .rst(rx_cnt_rst), .en(1'b1), .d(rx_cnt+1'b1), .q(rx_cnt));
assign rx_cnt_rst = ctrl[0];

wire [7:0] rxd_d1;
d1 #(8) rxd_d1_r(.c(clk_50), .d(qrxd), .q(rxd_d1));
wire [15:0] rx_16bit = { qrxd, rxd_d1 };

r #(16) hop_count_r
(.c(clk_50), .rst(1'b0), .en(state == ST_HOP_COUNT), 
 .d(rx_16bit), .q(hop_count));

wire [15:0] submsg_addr;
r #(16) submsg_addr_r
(.c(clk_50), .rst(1'b0), .en(state == ST_SUBMSG_ADDR), 
 .d(rx_16bit), .q(submsg_addr));

wire [15:0] submsg_len;
r #(16) submsg_len_r
(.c(clk_50), .rst(1'b0), .en(state == ST_SUBMSG_LEN), .d(rx_16bit), .q(submsg_len));

wire submsg_received;
wire wire_submsg_rxdv = ~submsg_received & qrxdv & 
                         hop_count == submsg_addr & 
                         state == ST_SUBMSG_PAYLOAD;
wire [7:0] wire_submsg_rxd = qrxd;

wire wire_submsg_rxdv_d1;
d1 wire_submsg_rxdv_d1_r(.c(clk_50), .d(wire_submsg_rxdv), .q(wire_submsg_rxdv_d1));

r submsg_received_r
(.c(clk_50), .rst(state == ST_IDLE), 
 .en(wire_submsg_rxdv_d1 & ~wire_submsg_rxdv), 
 .d(1'b1), .q(submsg_received));

wire rx_rxlast;
r rx_rxlast_r
(.c(clk_50), .rst(state == ST_IDLE), .en(qrxlast), .d(1'b1), .q(rx_rxlast));

wire mfifo_almost_empty;

always @* begin
  case (state)
    ST_IDLE:
      if (qrxdv)                  ctrl = { ST_PROTO_VER  , 5'b00000 };
      else                       ctrl = { ST_IDLE       , 5'b00001 };
    ST_PROTO_VER: 
      if (~qrxdv)                 ctrl = { ST_IDLE       , 5'b00000 };
      else if (rx_16bit == 16'h4321)ctrl = { ST_HOP_COUNT  , 5'b00001 };
      else                       ctrl = { ST_DISCARD    , 5'b00000 };
    ST_HOP_COUNT:
      if (~qrxdv)                 ctrl = { ST_IDLE       , 5'b00000 };
      else if (rx_cnt == 16'h1)  ctrl = { ST_SUBMSG_ADDR, 5'b00001 };
      else                       ctrl = { ST_HOP_COUNT  , 5'b00000 };
    ST_SUBMSG_ADDR:
      if (~qrxdv)                 ctrl = { ST_SUBMSG_WAIT_FOR_RXLAST, 5'b00001 };
      else if (rx_cnt == 16'h1)  ctrl = { ST_SUBMSG_LEN , 5'b00001 };
      else                       ctrl = { ST_SUBMSG_ADDR, 5'b00000 };
    ST_SUBMSG_LEN:
      if (~qrxdv)                 ctrl = { ST_SUBMSG_WAIT_FOR_RXLAST, 5'b00001 };
      else if (rx_cnt == 16'h1)  ctrl = { ST_SUBMSG_PAYLOAD, 5'b00001 };
      else                       ctrl = { ST_SUBMSG_LEN    , 5'b00000 };
    ST_SUBMSG_PAYLOAD:
      if (~qrxdv)                 ctrl = { ST_SUBMSG_WAIT_FOR_RXLAST, 5'b00001 };
      else if (rx_cnt + 1'b1 == submsg_len) 
                                 ctrl = { ST_SUBMSG_ADDR   , 5'b00001 };
      else                       ctrl = { ST_SUBMSG_PAYLOAD, 5'b00000 };
    ST_SUBMSG_WAIT_FOR_RXLAST:
      if (~submsg_received)      ctrl = { ST_MFIFO_DRAIN   , 5'b00000 };
      else if (qrxlast)           ctrl = { ST_MFIFO_DRAIN   , 5'b00000 };
      else if (rx_cnt == 16'h30) ctrl = { ST_MFIFO_DRAIN  , 5'b00000 };
      else                       ctrl = { ST_SUBMSG_WAIT_FOR_RXLAST, 5'b00000 };
    ST_MFIFO_DRAIN:
      if (mfifo_drained)         ctrl = { ST_IDLE          , 5'b00000 };
      else                       ctrl = { ST_MFIFO_DRAIN   , 5'b00000 };
    ST_DISCARD:
      if (~qrxdv)                 ctrl = { ST_IDLE       , 5'b00000 };
      else                       ctrl = { ST_DISCARD    , 5'b00000 };
    default:                     ctrl = { ST_IDLE       , 5'b00000 };
  endcase
end

wire mfifo_empty;
wire mfifo_draining;
wire [10:0] mfifo_rdusedw;
dcfifo #(.lpm_width(8),
         .lpm_numwords(2048),
         .lpm_widthu(11),
         .lpm_showahead("ON"),
         .use_eab("ON"),
         .intended_device_family("CYCLONE V")) mfifo // submessage fifo
  (.wrclk(clk_50), .wrreq(wire_submsg_rxdv), .data(wire_submsg_rxd),
   .rdclk(clk_100), .rdreq(mfifo_draining & ~mfifo_empty), .q(submsg_rxd),
   .rdempty(mfifo_empty),
   .rdusedw(mfifo_rdusedw),
   .aclr(1'b0));

wire submsg_received_clk100;
sync submsg_received_sync_r(.in(submsg_received), .clk(clk_100), .out(submsg_received_clk100));

wire mfifo_drained;
sync mfifo_drained_sync_r
(.in(mfifo_empty), .clk(clk_50), .out(mfifo_drained));

wire mfifo_drain_req_clk100;
sync mfifo_drain_req_sync_r
(.in(state == ST_MFIFO_DRAIN), .clk(clk_100), .out(mfifo_drain_req_clk100));

r mfifo_draining_r
(.c(clk_100), .rst(mfifo_empty), .d(1'b1), .q(mfifo_draining), 
 .en(mfifo_drain_req_clk100));

assign submsg_rxdv = mfifo_draining & submsg_received_clk100 & ~mfifo_empty;
assign submsg_rxlast = mfifo_draining & submsg_received_clk100 & mfifo_rdusedw == 11'h2;

endmodule

`ifdef test_udp_outbound_chain_rx

module udp_outbound_chain_rx_tb();

wire clk_100, clk_50;
sim_clk #(100) clk_100_inst(clk_100);
sim_clk #( 50) clk_50_inst (clk_50 );

reg [7:0] rxd;
reg rxdv;
reg rxlast;

wire [7:0] submsg_rxd;
wire submsg_rxdv;
wire submsg_rxlast;

udp_outbound_chain_rx dut(.*);

localparam PKT_LEN = 16;
reg [7:0] pkt [PKT_LEN-1:0];
integer i;

initial begin
  $dumpfile("udp_outbound_chain_rx.lxt");
  $dumpvars();

  pkt[0] = 8'h1; // protocol low
  pkt[1] = 8'h0; // protocol high
  pkt[2] = 8'h0; // hop count low
  pkt[3] = 8'h0; // hop count high
  pkt[4] = 8'h0; // submsg addr low
  pkt[5] = 8'h0; // submsg addr high
  pkt[6] = 8'h2; // submsg len low
  pkt[7] = 8'h0; // submsg len high
  pkt[8] = 8'h12; // payload byte 0
  pkt[9] = 8'h34; // payload byte 1
  ////
  pkt[10] = 8'h1; // submsg addr low
  pkt[11] = 8'h0; // submsg addr high
  pkt[12] = 8'h2; // submsg len low
  pkt[13] = 8'h0; // submsg len high
  pkt[14] = 8'h56; // payload byte 0
  pkt[15] = 8'h78; // payload byte 1
  ////

  rxd = 1'b0;
  rxdv = 8'h0;
  rxlast = 1'b0;
  #100;
  wait(~clk_50);
  wait(clk_50);
  rxdv <= 1'b1;
  for (i = 0; i < PKT_LEN; i = i + 1) begin
    rxd <= pkt[i];
    if (i == PKT_LEN - 1)
      rxlast = 1;
    wait(~clk_50);
    wait(clk_50);
  end
  rxdv = 1'b0;
  rxd = 8'h0;
  rxlast = 1'b0;

  #2000;
  $finish();
end
endmodule

`endif
