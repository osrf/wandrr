`timescale 1ns/1ns
module udp_inbound_chain
#(parameter CHAIN_UDP_PORT=16'd11300)
(input        clk_100, // from our on-chip application processing stuff
 input [7:0]  outbox_txd,
 input        outbox_txdv,
 input        outbox_txe,  // end of TX burst to outbox. send the message.
 input        clk_50,  // from the ethernet RX data
 input [7:0]  eth_rxd,
 input        eth_rxdv,
 input        eth_rxe,
 output [1:0] phy_txd,
 output       phy_txen);

localparam SW = 4, CW = 3;
localparam ST_IDLE         = 4'd0;
localparam ST_MACS         = 4'd1;
localparam ST_ETHERTYPE    = 4'd2;
localparam ST_HEADERS      = 4'd3;
localparam ST_CHAIN_VER    = 4'd4;
localparam ST_HOP_COUNT    = 4'd5;
localparam ST_MSG_SENDER   = 4'd6;
localparam ST_MSG_LEN      = 4'd7;
localparam ST_MSG_PAYLOAD  = 4'd8;
localparam ST_UNUSED_BLOCK = 4'd9;
localparam ST_TX_FCS       = 4'd10;
localparam ST_TX_FLUSH     = 4'd11;

wire [23:0] eth_rxdv_cnt;
r #(24) eth_rxdv_cnt_r
(.c(clk_50), .en(1'b1), .rst(eth_rxdv),
 .d(eth_rxdv_cnt+1'b1), .q(eth_rxdv_cnt));
wire idle_reset = eth_rxdv_cnt == 24'hff_ffff; // sanity check state machine

reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r
(.c(clk_50), .rst(idle_reset), .en(1'b1), .d(next_state), .q(state));

wire idle = state == ST_IDLE;

// rxd shifter, useful for assembling 16- and 32-bit values
wire [31:0] rxd_shift;
r #(32) rxd_shift_r
(.c(clk_50), .en(eth_rxdv), .rst(idle & ~eth_rxdv),
 .d({rxd_shift[23:0], eth_rxd}), .q(rxd_shift));

wire [10:0] rx_cnt;
r #(11) rx_cnt_r
(.c(clk_50), .en(eth_rxdv), .rst(ctrl[0]), .d(rx_cnt + 1'b1), .q(rx_cnt));

wire [15:0] ethertype;
r #(16) ethertype_r
(.c(clk_50), .en(state == ST_HEADERS & rx_cnt == 11'h0), .rst(idle),
 .d(rxd_shift[15:0]), .q(ethertype));

wire [7:0] ip_proto;
r #(8) ip_proto_r
(.c(clk_50), .en(state == ST_HEADERS & rx_cnt == 11'ha), .rst(idle),
 .d(rxd_shift[7:0]), .q(ip_proto));

wire is_udp = ethertype == 16'h0800 & ip_proto == 8'h11;

wire [15:0] rx_udp_port;
r #(16) udp_port_r
(.c(clk_50), .en(state == ST_HEADERS & rx_cnt == 11'h18), .rst(idle),
 .d(rxd_shift[15:0]), .q(rx_udp_port));

wire [15:0] udp_payload_len;
r #(16) udp_payload_len_r
(.c(clk_50), .en(state == ST_HEADERS & rx_cnt == 11'h1a), .rst(idle),
 .d(rxd_shift[15:0] - 16'd8), .q(udp_payload_len));

wire [15:0] chain_ver;
r #(16) chain_ver_r
(.c(clk_50), .en(state == ST_HOP_COUNT & rx_cnt == 11'h0), .rst(idle),
 .d(rxd_shift[15:0]), .q(chain_ver));

wire is_chain = is_udp & rx_udp_port == CHAIN_UDP_PORT & chain_ver == 16'h2143;

wire [7:0] tx_hop_downcount; // todo: care about 16-bit hop counts.
r #(8) tx_hop_count_r
(.c(clk_50), .en(state == ST_HOP_COUNT & rx_cnt == 11'h1), .rst(idle),
 .d(rxd_shift[7:0] - 1'b1), .q(tx_hop_downcount));

wire [15:0] pkt_dibit_cnt;
r #(16) pkt_dibit_cnt_r
(.c(clk_50), .en(1'b1), .rst(ctrl[1]),
 .d(pkt_dibit_cnt+1'b1), .q(pkt_dibit_cnt));

wire [15:0] rx_msg_len;
r #(16) rx_msg_len_r
(.c(clk_50), .en(state == ST_MSG_LEN & rx_cnt == 11'h1 & eth_rxdv), .rst(idle),
 .d({eth_rxd, rxd_shift[7:0]}), .q(rx_msg_len));

wire [7:0] flush_cnt;
r #(8) flush_cnt_r
(.c(clk_50), .en(1'b1), .rst(ctrl[0]), .d(flush_cnt+1'b1), .q(flush_cnt));



always @* begin
  case (state)
    ST_IDLE:
      if (eth_rxdv)                    ctrl = { ST_MACS     , 3'b000 };
      else                             ctrl = { ST_IDLE     , 3'b011 };
    ST_MACS:
      if (rx_cnt == 11'd11 & eth_rxdv) ctrl = { ST_ETHERTYPE, 3'b001 };
      else                             ctrl = { ST_MACS     , 3'b000 };
    ST_ETHERTYPE:
      if (rx_cnt == 11'd1 & eth_rxdv)  ctrl = { ST_HEADERS  , 3'b001 };
      else                             ctrl = { ST_ETHERTYPE, 3'b000 };
    ST_HEADERS:
      if (rx_cnt == 11'd27 & eth_rxdv) ctrl = { ST_CHAIN_VER, 3'b001 };
      else                             ctrl = { ST_HEADERS  , 3'b000 };
    ST_CHAIN_VER:
      if (rx_cnt == 11'd1 & eth_rxdv)  ctrl = { ST_HOP_COUNT, 3'b001 };
      else                             ctrl = { ST_CHAIN_VER, 3'b000 };
    ST_HOP_COUNT:
      if (rx_cnt == 11'd1 & eth_rxdv)  ctrl = { ST_MSG_SENDER , 3'b001 };
      else                             ctrl = { ST_HOP_COUNT  , 3'b000 };
    ST_MSG_SENDER:
      if (rx_cnt == 11'd1 & eth_rxdv)  
        if ({rxd_shift[7:0], eth_rxd} == 16'hffff)
                                       ctrl = { ST_UNUSED_BLOCK, 3'b000 };
        else                           ctrl = { ST_MSG_LEN     , 3'b001 };
      else                             ctrl = { ST_MSG_SENDER  , 3'b000 };
    ST_MSG_LEN:
      if (rx_cnt == 11'd1 & eth_rxdv)  ctrl = { ST_MSG_PAYLOAD , 3'b001 }; 
      else                             ctrl = { ST_MSG_LEN     , 3'b000 };
    ST_MSG_PAYLOAD:
      if (rx_cnt + 1'b1 == rx_msg_len & eth_rxdv) ctrl = { ST_MSG_SENDER , 3'b001 };
      else                                 ctrl = { ST_MSG_PAYLOAD, 3'b000 };
    ST_UNUSED_BLOCK:
      if (eth_rxe)                      ctrl = { ST_TX_FCS   , 3'b101 };
      else                              ctrl = { ST_UNUSED_BLOCK, 3'b000 };
    ST_TX_FCS:
      if (flush_cnt == 8'd15)           ctrl = { ST_TX_FLUSH , 3'b100 };
      else                              ctrl = { ST_TX_FCS   , 3'b000 };
    ST_TX_FLUSH:
      if (flush_cnt == 8'd33)           ctrl = { ST_IDLE     , 3'b100 };
      else                              ctrl = { ST_TX_FLUSH , 3'b000 };
    default:                            ctrl = { ST_IDLE     , 3'b000 };
  endcase
end

//wire hop_downcount_rxtime = pkt_dibit_cnt == 16'hd0; //state == ST_PAYLOAD & rx_cnt == 11'd2 & rxdv;

udp_inbound_chain_writer chain_writer_inst
(.clk_100(clk_100),
 .outbox_txd(outbox_txd), .outbox_txdv(outbox_txdv), .outbox_txe(outbox_txe),
 .clk_50(clk_50), 
 .rxd(eth_rxd), .rxdv(eth_rxdv), .rxe(eth_rxe),
 .is_chain(is_chain),
 .hop_downcount(tx_hop_downcount),
 .in_unused_block(state == ST_UNUSED_BLOCK & is_chain),
 .phy_txd(phy_txd),
 .phy_txen(phy_txen));

//////////////////////////////////////////////////////////////////
// below here is the TX path

//wire txen = |pkt_dibit_cnt;

//wire increment_hop_count = is_chain & state == ST_HOP_COUNT & rx_cnt == 11'd0 & rxdv;
/*
wire [7:0] unused_block_start_shift;
r #(8) unused_block_start_shift_r
(.c(clk_50), .en(eth_rxdv), .rst(idle),
 .d({unused_block_start_shift[6:0], state == ST_UNUSED_BLOCK}),
 .q(unused_block_start_shift));

wire unused_block_txtime = unused_block_start_shift[5];

wire [7:0] msg_txd = 8'h42;

wire [7:0] chain_txd = decrement_hop_count_txtime ? tx_hop_downcount : 
                       unused_block_txtime ? msg_txd : 
                       rxd_shift[31:24];
wire [7:0] txd = state == ST_TX_FCS ? fcs[31:24] : 
                 (is_chain ? chain_txd : rxd_shift[31:24]);
//assign txdv = rxd_shift_valid & pkt_dibit_cnt[1:0] == 2'h0;

wire txdv = eth_rxdv | (~idle & pkt_dibit_cnt[1:0] == 2'h0);

localparam DELAY_BYTES = 8;
wire [8*DELAY_BYTES-1:0] txd_shift;
r #(8*DELAY_BYTES, 64'h5555_5555_5555_55d5) txd_shift_r
(.c(clk_50), .rst(idle), .en(txdv), 
 .d({txd_shift[8*(DELAY_BYTES-1)-1:0], txd}),
 .q(txd_shift));

wire [7:0] delayed_txd = txd_shift[8*DELAY_BYTES-1:8*(DELAY_BYTES-1)];
*/

endmodule

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

`ifdef test_udp_inbound_chain

module udp_inbound_chain_tb();

wire clk_50;
sim_clk #(50) clk_50_inst(clk_50);

wire clk_100;
sim_clk #(100) clk_100_inst(clk_100);


wire [1:0] dn_phy_rxd, up_phy_txd;
wire dn_phy_rxdv, up_phy_txen;

fake_rmii_phy #(.INPUT_FILE_NAME("inbound_chain_pkts.dat")) dn_phy
(.refclk(clk_50), .rst(1'b0), 
 .txd(2'b0), .txen(1'b0),
 .rxd(dn_phy_rxd), .rxdv(dn_phy_rxdv));

fake_rmii_phy up_phy
(.refclk(clk_50), .rst(1'b0),
 .txd(up_phy_txd), .txen(up_phy_txen));

wire [7:0] dne_rxd;
wire dne_rxdv, dne_rxe;
eth_rx dn_eth_rx // downstream ethernet PHY interface
(.c(clk_50), 
 .d(dne_rxd), .dv(dne_rxdv), .erx(dne_rxe),
 .phy_rxd(dn_phy_rxd), .phy_rxdv(dn_phy_rxdv));

reg [7:0] outbox_txd;
reg outbox_txdv, outbox_txe;

reg endpoint;

wire ute_start = endpoint ? outbox_txe : 1'b0; // start the chain each msg
wire [10:0] ute_len = 11'd64;
wire [15:0] ute_port = 16'd11300;
wire [31:0] ute_dst_ip = 32'h1234_5678;
wire [31:0] ute_src_ip = 32'habcd_ef01;
wire [47:0] ute_src_mac = 48'ha4f3c1_000011;
wire [47:0] ute_dst_mac = 48'h01005e_00007b;
wire [7:0] ute_txd;
wire ute_txdv, ute_txe;

wire [7:0] ute_hop_cnt = 8'h3;

udp_tx_empty udp_tx_empty_inst
(.clk_100(clk_100), .start(ute_start), .len(ute_len),
 .dst_port(ute_port), .dst_ip(ute_dst_ip), .src_ip(ute_src_ip),
 .src_mac(ute_src_mac), .dst_mac(ute_dst_mac), .hop_cnt(ute_hop_cnt),
 .clk_50(clk_50), .txd(ute_txd), .txdv(ute_txdv), .txe(ute_txe));

wire [7:0] uic_rxd  = endpoint ? ute_txd  : dne_rxd;
wire       uic_rxdv = endpoint ? ute_txdv : dne_rxdv;
wire       uic_rxe  = endpoint ? ute_txe  : dne_rxe;

udp_inbound_chain dut
(.clk_50(clk_50), .clk_100(clk_100),
 .eth_rxd(uic_rxd), .eth_rxdv(uic_rxdv), .eth_rxe(uic_rxe),
 .outbox_txd(outbox_txd), .outbox_txdv(outbox_txdv), .outbox_txe(outbox_txe),
 .phy_txd(up_phy_txd), .phy_txen(up_phy_txen));

initial begin
  $dumpfile("udp_inbound_chain.lxt");
  $dumpvars();
  outbox_txdv = 0;
  outbox_txe = 0;
  outbox_txd = 8'h0;
  endpoint = 0;
  #1000;
  // stuff an outbound message into the outbox
  @(posedge clk_100); #1;
  outbox_txdv = 1;
  outbox_txd = 8'h12;
  @(posedge clk_100); #1;
  outbox_txd = 8'h34;
  @(posedge clk_100); #1;
  outbox_txd = 8'h56;
  @(posedge clk_100); #1;
  outbox_txe = 1;
  outbox_txd = 8'h78;
  @(posedge clk_100); #1;
  outbox_txe = 0;
  outbox_txdv = 0;
  outbox_txd = 8'h0;
  #50000;
  // now, test if we can originate packets ourselves
  @(posedge clk_100);
  endpoint = 1;
  #1000;
  @(posedge clk_100); #1;
  outbox_txdv = 1;
  outbox_txd = 8'h11;
  @(posedge clk_100); #1;
  outbox_txd = 8'h22;
  @(posedge clk_100); #1;
  outbox_txd = 8'h33;
  @(posedge clk_100); #1;
  outbox_txd = 8'h44;
  @(posedge clk_100); #1;
  outbox_txd = 8'h55;
  outbox_txe = 1;
  @(posedge clk_100); #1;
  outbox_txe = 0;
  outbox_txdv = 0;
  outbox_txd = 8'h0;
  #50000;
  
  $finish();
end
endmodule

`endif
