`timescale 1ns/1ns
module udp_outbound_chain
(input        clk_50,
 input        clk_100,
 input  [7:0] rxd,
 input        rxdv,
 input        rxlast,
 output [7:0] txd,
 output       txdv,
 output [15:0] hop_count,
 output [7:0] submsg_rxd,
 output       submsg_rxdv,
 output       submsg_rxlast);

udp_outbound_chain_rx rx_inst
(.clk_50(clk_50), .clk_100(clk_100),
 .rxd(rxd), .rxdv(rxdv), .rxlast(rxlast),
 .hop_count(hop_count),
 .submsg_rxd(submsg_rxd), 
 .submsg_rxdv(submsg_rxdv), 
 .submsg_rxlast(submsg_rxlast));

udp_outbound_chain_tx tx_inst
(.c(clk_50), .rxd(rxd), .rxdv(rxdv), .txd(txd), .txdv(txdv));

endmodule 

`ifdef test_udp_outbound_chain

module udp_outbound_chain_tb();

wire c;
sim_clk #(125) clk_125(c);

reg [7:0] rxd;
reg rxdv, rxlast;

wire [7:0] submsg_rxd, txd;
wire submsg_rxdv, txdv;

udp_outbound_chain dut(.*);

wire [7:0] node2_txd, node2_submsg_rxd;
wire node2_txdv, node2_submsg_rxdv;
reg node2_rxlast;
udp_outbound_chain dut2
(.c(c), 
 .rxd(txd), .rxdv(txdv), .rxlast(node2_rxlast),
 .txd(node2_txd), .txdv(node2_txdv),
 .submsg_rxd(node2_submsg_rxd), .submsg_rxdv(node2_submsg_rxdv));

wire [7:0] node3_txd, node3_submsg_rxd;
wire node3_txdv, node3_submsg_rxdv;
reg node3_rxlast;
udp_outbound_chain dut3
(.c(c), 
 .rxd(node2_txd), .rxdv(node2_txdv), .rxlast(node3_rxlast),
 .txd(node3_txd), .txdv(node3_txdv),
 .submsg_rxd(node3_submsg_rxd), .submsg_rxdv(node3_submsg_rxdv));

localparam PKT_LEN = 24;
reg [7:0] pkt [PKT_LEN-1:0];
integer i;

initial begin
  $dumpfile("udp_outbound_chain.lxt");
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
  pkt[16] = 8'h2; // submsg addr low
  pkt[17] = 8'h0; // submsg addr high
  pkt[18] = 8'h4; // submsg len low
  pkt[19] = 8'h0; // submsg len high
  pkt[20] = 8'hab; // payload byte 0
  pkt[21] = 8'hcd; // payload byte 1
  pkt[22] = 8'hef; // payload byte 0
  pkt[23] = 8'h42; // payload byte 1
  ////

  rxd = 1'b0;
  rxdv = 8'h0;
  rxlast = 1'b0;
  #100;
  for (i = 0; i < PKT_LEN; i = i + 1) begin
    wait(~c);
    wait(c);
    rxd <= pkt[i];
    rxdv <= 1'b1;
  end
  wait(~c);
  wait(c);
  rxdv <= 1'b0;
  rxd <= 8'h0;

  for (i = 0; i < 20; i = i + 1) begin
    wait(~c);
    wait(c);
  end
  rxlast <= 1'b1;
  node2_rxlast <= 1'b1;
  node3_rxlast <= 1'b1;
  wait(~c);
  wait(c);
  rxlast <= 1'b0;
  node2_rxlast <= 1'b0;
  node3_rxlast <= 1'b0;

  #1000;
  $finish();
end
endmodule

`endif
