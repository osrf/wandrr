`timescale 1ns/1ns
module eth_outbound_chain_tx
(input clk_50,
 input [7:0]  rxd,
 input        rxdv,
 input        rxe, // rx end
 output [1:0] phy_txd,
 output       phy_txen);

localparam SW = 4, CW = 3;
localparam ST_IDLE       = 4'd0;
localparam ST_MACS       = 4'd2;
localparam ST_ETHERTYPE  = 4'd3;
localparam ST_HEADERS    = 4'd4;
localparam ST_CHAIN_VER  = 4'd5;
localparam ST_HOP_COUNT  = 4'd6;
localparam ST_PAYLOAD    = 4'd7;
localparam ST_TX_FCS     = 4'd8;
localparam ST_TX_FLUSH   = 4'd9;


reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r(.c(clk_50), .rst(1'b0), .en(1'b1), .d(next_state), .q(state));

wire idle = state == ST_IDLE;

// follow behind the stream by 4 bytes, so we can swap out our new FCS
wire [31:0] rxd_shift;
r #(32) rxd_shift_r
(.c(clk_50), .en(rxdv), .rst(idle & ~rxdv),
 .d({rxd_shift[23:0], rxd}), .q(rxd_shift));

wire [3:0] rxdv_shift;
r #(4) rxdv_shift_r
(.c(clk_50), .en(1'b1), .rst(1'b0),
 .d({rxdv_shift[2:0], rxdv}), .q(rxdv_shift));
wire rxdv_idle = ~|rxdv_shift;

wire [10:0] rx_cnt;
r #(11) rx_cnt_r
(.c(clk_50), .en(rxdv), .rst(ctrl[0]), .d(rx_cnt + 1'b1), .q(rx_cnt));

wire drivethru_en;
wire drivethru; // this means to just let the packet drive thru, without mods
r drivethru_r
(.c(clk_50), .en(drivethru_en), .rst(idle),
 .d(1'b1), .q(drivethru));

wire [15:0] ethertype;
r #(16) ethertype_r
(.c(clk_50), .en(state == ST_HEADERS & rx_cnt == 11'h0), .rst(idle),
 .d(rxd_shift[15:0]), .q(ethertype));

wire [7:0] ip_proto;
r #(8) ip_proto_r
(.c(clk_50), .en(state == ST_HEADERS & rx_cnt == 11'ha), .rst(idle),
 .d(rxd_shift[7:0]), .q(ip_proto));

wire is_udp = ethertype == 16'h0800 & ip_proto == 8'h11;

wire [15:0] chain_ver;
r #(16) chain_ver_r
(.c(clk_50), .en(state == ST_HOP_COUNT & rx_cnt == 11'h0), .rst(idle),
 .d(rxd_shift[15:0]), .q(chain_ver));

wire [7:0] tx_hop_count; // todo: care about 16-bit hop counts.
r #(8) tx_hop_count_r
(.c(clk_50), .en(state == ST_HOP_COUNT & rx_cnt == 11'h1), .rst(idle),
 .d(rxd_shift[7:0] + 1'b1), .q(tx_hop_count));

wire is_chain = is_udp & chain_ver == 16'h2143;

wire increment_hop_count = is_chain & state == ST_HOP_COUNT & rx_cnt == 11'd0 & rxdv;
wire increment_hop_count_txtime = is_chain & state == ST_PAYLOAD & rx_cnt == 11'd2 & rxdv;

wire [7:0] clk_cnt;
r #(8) clk_cnt_r
(.c(clk_50), .en(1'b1), .rst(ctrl[2]), .d(clk_cnt+1'b1), .q(clk_cnt));

always @* begin
  case (state)
    ST_IDLE:
      if (rxdv)                     ctrl = { ST_MACS     , 3'b000 };
      else                          ctrl = { ST_IDLE     , 3'b011 };
    ST_MACS:
      if (rx_cnt == 11'd11 & rxdv)  ctrl = { ST_ETHERTYPE, 3'b001 };
      else if (rxdv_idle)           ctrl = { ST_IDLE     , 3'b000 };
      else                          ctrl = { ST_MACS     , 3'b000 };
    ST_ETHERTYPE:
      if (rx_cnt == 11'd1 & rxdv)   ctrl = { ST_HEADERS  , 3'b001 };
      else if (rxdv_idle)           ctrl = { ST_IDLE     , 3'b000 };
      else                          ctrl = { ST_ETHERTYPE, 3'b000 };
    ST_HEADERS:
      if (rx_cnt == 11'd27 & rxdv)  ctrl = { ST_CHAIN_VER, 3'b001 };
      else if (rxdv_idle)           ctrl = { ST_IDLE     , 3'b000 };
      else                          ctrl = { ST_HEADERS  , 3'b000 };
    ST_CHAIN_VER:
      if (rx_cnt == 11'd1 & rxdv)   ctrl = { ST_HOP_COUNT, 3'b001 };
      else if (rxdv_idle)           ctrl = { ST_IDLE     , 3'b000 };
      else                          ctrl = { ST_CHAIN_VER, 3'b000 };
    ST_HOP_COUNT:
      if (rx_cnt == 11'd1 & rxdv)   ctrl = { ST_PAYLOAD  , 3'b001 };
      else if (rxdv_idle)           ctrl = { ST_IDLE     , 3'b000 };
      else                          ctrl = { ST_HOP_COUNT, 3'b000 };
    ST_PAYLOAD:
      if (rxe)                      ctrl = { ST_TX_FCS   , 3'b101 };
      else if (rxdv_idle)           ctrl = { ST_IDLE     , 3'b000 };
      else                          ctrl = { ST_PAYLOAD  , 3'b000 };
    ST_TX_FCS:
      if (clk_cnt == 8'd15)         ctrl = { ST_TX_FLUSH , 3'b100 };
      else                          ctrl = { ST_TX_FCS   , 3'b000 };
    ST_TX_FLUSH:
      if (clk_cnt == 8'd33)         ctrl = { ST_IDLE     , 3'b100 };
      else                          ctrl = { ST_TX_FLUSH , 3'b000 };
    default:                        ctrl = { ST_IDLE     , 3'b000 };
  endcase
end

wire [1:0] dibit_cnt;
r #(2) dibit_cnt_r
(.c(clk_50), .en(1'b1), .rst(ctrl[1]), .d(dibit_cnt+1'b1), .q(dibit_cnt));

wire [1:0] rxd_shift_cnt;
r #(2) rxd_shift_cnt_r
(.c(clk_50), .en(rxdv), .rst(idle), 
 .d(~&rxd_shift_cnt ? rxd_shift_cnt+1'b1 : 2'h3), .q(rxd_shift_cnt));
wire rxd_shift_valid = &rxd_shift_cnt;

wire [7:0] txd;
wire txdv;
wire fcs_dv = txdv & state != ST_TX_FCS;
wire [31:0] fcs_live;
eth_crc32 fcs_inst
(.c(clk_50), .r(idle), .dv(fcs_dv), .d(txd), .crc(fcs_live));

wire [31:0] fcs;
r #(32) fcs_r
(.c(clk_50), .rst(idle), 
 .en(ctrl[2] | (state == ST_TX_FCS & clk_cnt[1:0] == 2'h3)), 
 .d(state == ST_TX_FCS ? { fcs[23:0], 8'h0 } : fcs_live), .q(fcs));

wire [7:0] chain_txd = increment_hop_count_txtime ? tx_hop_count : rxd_shift[31:24];
assign txd = state == ST_TX_FCS ? fcs[31:24] : (is_chain ? chain_txd : rxd_shift[31:24]);
assign txdv = rxd_shift_valid & dibit_cnt == 2'h0;

localparam DELAY_BYTES = 8;
wire [8*DELAY_BYTES-1:0] txd_shift;
r #(8*DELAY_BYTES, 64'h5555_5555_5555_55d5) txd_shift_r
(.c(clk_50), .rst(idle), .en(txdv), 
 .d({txd_shift[8*(DELAY_BYTES-1)-1:0], txd}),
 .q(txd_shift));

wire [7:0] delayed_txd = txd_shift[8*DELAY_BYTES-1:8*(DELAY_BYTES-1)];

wire [7:0] tx_dibit_shift;
r #(8) tx_dibit_shift_r
(.c(clk_50), .rst(1'b0), .en(1'b1),
 .d(txdv ? delayed_txd : { 2'h0, tx_dibit_shift[7:2] }),
 .q(tx_dibit_shift));
wire [1:0] phy_txd_posedge = tx_dibit_shift[1:0];

wire phy_txen_posedge;
r phy_txen_posedge_r
(.c(clk_50), .rst(idle), .en(txdv), .d(1'b1), .q(phy_txen_posedge));

wire [2:0] tx_d1;
d1 #(3) tx_d1_r
(.c(clk_50), .d({phy_txen_posedge, phy_txd_posedge}), .q(tx_d1));

r #(3) phy_negedge_r
(.c(clk_50 /*~clk_50*/), .rst(1'b0), .en(1'b1),
 .d(tx_d1),
 .q({phy_txen, phy_txd}));

endmodule

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
// BELOW HERE IS JUST DEBUGGING CODE
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

`ifdef test_eth_outbound_chain_tx

module eth_outbound_chain_tx_tb();

wire clk_50;
sim_clk #(50) clk_50_inst(clk_50);

wire clk_100;
sim_clk #(100) clk_100_inst(clk_100);


wire [1:0] rmii_txd, rmii_rxd;
wire rmii_rst, mdc, mdio;
wire rmii_txen, rmii_rxdv;

fake_rmii_phy #(.INPUT_FILE_NAME("tb_packets.dat")) phy
(.refclk(clk_50), .rst(rmii_rst), .mdc(mdc), .mdio(mdio), 
 .txd(rmii_txd), .txen(rmii_txen),
 .rxd(rmii_rxd), .rxdv(rmii_rxdv));

wire [7:0] rxd; // upstream ethernet RXD
wire rxdv, rxe; // upstream ethernet RXDV and end-RX
eth_rx upe // upstream ethernet PHY interface
(.c(clk_50),
 .d(rxd), .dv(rxdv), .erx(rxe),
 .phy_rxd(rmii_rxd), .phy_rxdv(rmii_rxdv));

wire [1:0] phy_txd;
wire phy_txen;

eth_outbound_chain_tx dut(.*);

fake_rmii_phy b0_sim_rmii_phy
(.refclk(clk_50), .mdc(enet_mdc), .mdio(enet_mdio), .rst(1'b1),
 .txd(rmii_txd), .txen(rmii_txen));

wire [7:0] b1_enet_rxd , b2_enet_rxd , b3_enet_rxd;
wire       b1_enet_rxdv, b2_enet_rxdv, b3_enet_rxdv;
wire       b1_enet_erx , b2_enet_erx , b3_enet_erx;
wire [1:0] b1_phy_txd  , b2_phy_txd  , b3_phy_txd;
wire       b1_phy_txen , b2_phy_txen , b3_phy_txen;


///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
// board 1

eth_rx b1_upe // rx interface on the next board in the chain
(.c(clk_50), 
 .d(b1_enet_rxd), .dv(b1_enet_rxdv), .erx(b1_enet_erx),
 .phy_rxd(phy_txd), .phy_rxdv(phy_txen));

eth_outbound_chain_tx b1_chain_tx
(.clk_50(clk_50),
 .rxd(b1_enet_rxd), .rxdv(b1_enet_rxdv), .rxe(b1_enet_erx),
 .phy_txd(b1_phy_txd), .phy_txen(b1_phy_txen));

wire [7:0] b1_upu_rxd; // upstream UDP RXD
wire b1_upu_rxdv, b1_upu_erx; // upstreadm UDP RXDV and end-RX
wire [15:0] b1_upu_port;
udp_rx b1_upu_rx
(.c(clk_50),
 .eth_d(b1_enet_rxd), .eth_dv(b1_enet_rxdv), .eth_stop(b1_enet_erx),
 .udp_d(b1_upu_rxd), .udp_dv(b1_upu_rxdv), .udp_last(b1_upu_erx),
 .udp_port(b1_upu_port));

wire [7:0] b1_submsg_rxd;
wire b1_submsg_rxdv, b1_submsg_rxlast;

udp_outbound_chain_rx b1_outbound_chain_rx_inst
(.clk_50(clk_50), .clk_100(clk_100),
 .rxd(b1_upu_rxd), .rxdv(b1_upu_rxdv & b1_upu_port == 16'd11300), 
 .rxlast(b1_upu_erx),
 .submsg_rxd(b1_submsg_rxd), 
 .submsg_rxdv(b1_submsg_rxdv), 
 .submsg_rxlast(b1_submsg_rxlast));

fake_rmii_phy b1_sim_rmii_phy
(.refclk(clk_50), .mdc(enet_mdc), .mdio(enet_mdio), .rst(1'b1),
 .txd(b1_phy_txd), .txen(b1_phy_txen));

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
// board 2

wire b2_clk_50, b2_clk_100;
assign b2_clk_50 = clk_50;
assign b2_clk_100 = clk_100;

eth_rx b2_upe // rx interface on the next board in the chain
(.c(b2_clk_50), 
 .d(b2_enet_rxd), .dv(b2_enet_rxdv), .erx(b2_enet_erx),
 .phy_rxd(b1_phy_txd), .phy_rxdv(b1_phy_txen));

eth_outbound_chain_tx b2_chain_tx
(.clk_50(b2_clk_50),
 .rxd(b2_enet_rxd), .rxdv(b2_enet_rxdv), .rxe(b2_enet_erx),
 .phy_txd(b2_phy_txd), .phy_txen(b2_phy_txen));

wire [7:0] b2_upu_rxd; // upstream UDP RXD
wire b2_upu_rxdv, b2_upu_erx; // upstreadm UDP RXDV and end-RX
wire [15:0] b2_upu_port;
udp_rx b2_upu_rx
(.c(b2_clk_50),
 .eth_d(b2_enet_rxd), .eth_dv(b2_enet_rxdv), .eth_stop(b2_enet_erx),
 .udp_d(b2_upu_rxd), .udp_dv(b2_upu_rxdv), .udp_last(b2_upu_erx),
 .udp_port(b2_upu_port));

wire [7:0] b2_submsg_rxd;
wire b2_submsg_rxdv, b2_submsg_rxlast;

udp_outbound_chain_rx b2_outbound_chain_rx_inst
(.clk_50(b2_clk_50), .clk_100(b2_clk_100),
 .rxd(b2_upu_rxd), .rxdv(b2_upu_rxdv & b2_upu_port == 16'd11300), 
 .rxlast(b2_upu_erx),
 .submsg_rxd(b2_submsg_rxd), 
 .submsg_rxdv(b2_submsg_rxdv), 
 .submsg_rxlast(b2_submsg_rxlast));

fake_rmii_phy b2_sim_rmii_phy
(.refclk(clk_50), .mdc(enet_mdc), .mdio(enet_mdio), .rst(1'b1),
 .txd(b2_phy_txd), .txen(b2_phy_txen));

///////////////////////////////////////////////////////////////
// board 3

eth_rx b3_upe // rx interface on the next board in the chain
(.c(clk_50), 
 .d(b3_enet_rxd), .dv(b3_enet_rxdv), .erx(b3_enet_erx),
 .phy_rxd(b2_phy_txd), .phy_rxdv(b2_phy_txen));

eth_outbound_chain_tx b3_chain_tx
(.clk_50(clk_50),
 .rxd(b3_enet_rxd), .rxdv(b3_enet_rxdv), .rxe(b3_enet_erx),
 .phy_txd(b3_phy_txd), .phy_txen(b3_phy_txen));

fake_rmii_phy b3_sim_rmii_phy
(.refclk(clk_50), .mdc(enet_mdc), .mdio(enet_mdio), .rst(1'b1),
 .txd(b3_phy_txd), .txen(b3_phy_txen));


initial begin
  $dumpfile("eth_outbound_chain_tx.lxt");
  $dumpvars();
  #50000;
  $finish();
end


endmodule

`endif
