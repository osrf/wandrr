`timescale 1ns / 1ns
module fake_rmii_phy
#(parameter INPUT_FILE_NAME="rx_packets.dat")
(input            refclk,
 input  [1:0]     txd,
 input            txen,
 output [1:0]     rxd,
 output           rxdv,
 inout            mdc,
 inout            mdio,
 input            rst);

wire [7:0] rmii_rxd;

assign rxd = rmii_rxd[1:0];

fake_eth_phy #(.input_file_name(INPUT_FILE_NAME)) fake_phy_inst
  (.rgmii(1'b0), .gmii(1'b0), .rmii(1'b1), .REFCLK(refclk),
   .TXD({6'b0, txd}), .TXEN(txen), .GTXCLK(1'b0), 
   .TXER(1'b0), .TXCLK(),
   .RXCLK(), .RXD(rmii_rxd), .RXER(),
   .RXDV(rxdv), .RESET_N(rst),
   .MDC(mdc), .MDIO(mdio), .force_rxer(1'b0));

endmodule
