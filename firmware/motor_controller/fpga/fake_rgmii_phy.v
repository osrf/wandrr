`timescale 1ns / 1ns
module fake_rgmii_phy
#(parameter INPUT_FILE_NAME="rx_packets.dat")
(input  [3:0]     tx_d,
 input            tx_en,
 input            tx_clk,
 output [3:0]     rx_d,
 output           rx_clk,
 output           rx_dv,
 inout            mdc,
 inout            mdio,
 input            reset_n);

// all this is supposed to do is create a rgmii-gmii shim around fake_phy
// by extracting relevant bits from tap_phy.v

reg  [7:0] gmii_tx_d;
reg        gmii_tx_en;
reg        gmii_tx_er;
wire       gmii_tx_clk;
wire [7:0] gmii_rx_d;
wire       gmii_rx_dv;
wire       gmii_rx_er;
wire       gmii_rx_clk;

assign gmii_tx_clk = tx_clk;

wire [3:0] status = {1'b1, 1'b0, 1'b1, 1'b1}; 

always @(posedge gmii_tx_clk) begin
  gmii_tx_d[3:0] <= tx_d;
  gmii_tx_en <= tx_en;
end
always @(negedge gmii_tx_clk) begin
  gmii_tx_d[7:4] <= tx_d;
  gmii_tx_er <= 1'b0; // not sure here.
end

assign rx_clk = gmii_rx_clk;

/*
always @(posedge gmii_rx_clk) begin
  rx_d  <= gmii_rx_dv ? gmii_rx_d[3:0] : status;
  rx_dv <= gmii_rx_dv;
end
always @(negedge gmii_rx_clk) begin
  rx_d  <= gmii_rx_dv ? gmii_rx_d[7:4] : status;
  rx_dv <= gmii_rx_dv ^ gmii_rx_er;
end
*/

assign rx_d = gmii_rx_d[3:0];
assign rx_dv = gmii_rx_dv;

wire phy_txclk;

fake_eth_phy #(.input_file_name(INPUT_FILE_NAME)) fake_phy_0
  (.rgmii(1'b1), .gmii(1'b0), .rmii(1'b0), .REFCLK(1'b0),
   .TXD(gmii_tx_d), .TXEN(gmii_tx_en), .GTXCLK(gmii_tx_clk), 
   .TXER(gmii_tx_er), .TXCLK(phy_txclk),
   .RXCLK(gmii_rx_clk), .RXD(gmii_rx_d), .RXER(gmii_rx_er),
   .RXDV(gmii_rx_dv), .RESET_N(reset_n),
   .MDC(mdc), .MDIO(mdio), .force_rxer(1'b0));

endmodule
