`timescale 1ns/1ns
module chain_tb();

wire clk_20;
sim_clk #(20) sim_clk_20_inst(.clk(clk_20));

wire clk_25;
sim_clk #(25) sim_clk_25_inst(.clk(clk_25));

wire clk_50;
sim_clk #(50) sim_clk_50_inst(.clk(clk_50));

wire clk_48;
sim_clk #(48) sim_clk_48_inst(.clk(clk_48));

wire clk_100;
sim_clk #(100) sim_clk_100_inst(.clk(clk_100));

localparam CHAIN_LEN = 3;
wire [CHAIN_LEN-1:0] enet_rst, enet_mdc, enet_mdio;
wire [CHAIN_LEN*4-1:0] enet_leds;
wire [CHAIN_LEN*2-1:0] enet_txclk, enet_txen, enet_rxcclk, enet_rxdv;
wire [CHAIN_LEN*4-1:0] enet_txd, enet_rxd;

wire [CHAIN_LEN*16-1:0] outb;

wire [CHAIN_LEN*5-1:0] usb_oe, usb_rcv, usb_pwr, usb_vp, usb_vm;

wire [CHAIN_LEN*3-1:0] mosfet_hi, mosfet_lo, mclk;
reg  [CHAIN_LEN*3-1:0] mdata;
wire [CHAIN_LEN-1:0] mosfet_en;
wire [CHAIN_LEN*9-1:0] mcu_io;
wire [CHAIN_LEN-1:0] led;

//assign enet_mdc = {CHAIN_LEN{1'b0}};
//assign enet_mdc = {CHAIN_LEN{1'bz}};

meganode meganodes[CHAIN_LEN-1:0]
(.*);

fake_rmii_phy #(.INPUT_FILE_NAME("chain_tb.dat")) sim_rmii_phy_0_0
(.refclk(clk_50), .rst(1'b1),
 .txd(enet_txd[1:0]), .txen(enet_txen[0]),
 .rxd(enet_rxd[1:0]), .rxdv(enet_rxdv[0]));

fake_rmii_phy #(.INPUT_FILE_NAME("chain_tb.dat")) sim_rmii_phy_0_1
(.refclk(clk_50), .rst(1'b1),
 .txd(enet_txd[3:2]), .txen(enet_txen[1]),
 .rxd(enet_rxd[3:2]), .rxdv(enet_rxdv[1]));

fake_rmii_phy #(.INPUT_FILE_NAME("chain_tb.dat")) sim_rmii_phy_1_0
(.refclk(clk_50), .rst(1'b1),
 .txd(enet_txd[5:4]), .txen(enet_txen[2]),
 .rxd(enet_rxd[5:4]), .rxdv(enet_rxdv[2]));

fake_rmii_phy #(.INPUT_FILE_NAME("chain_tb.dat")) sim_rmii_phy_1_1
(.refclk(clk_50), .rst(1'b1),
 .txd(enet_txd[7:6]), .txen(enet_txen[3]),
 .rxd(enet_rxd[7:6]), .rxdv(enet_rxdv[3]));

fake_rmii_phy #(.INPUT_FILE_NAME("chain_tb.dat")) sim_rmii_phy_2_0
(.refclk(clk_50), .rst(1'b1),
 .txd(enet_txd[9:8]), .txen(enet_txen[4]),
 .rxd(enet_rxd[9:8]), .rxdv(enet_rxdv[4]));

fake_rmii_phy #(.INPUT_FILE_NAME("chain_tb.dat")) sim_rmii_phy_2_1
(.refclk(clk_50), .rst(1'b1),
 .txd(enet_txd[11:10]), .txen(enet_txen[5]),
 .rxd(enet_rxd[11:10]), .rxdv(enet_rxdv[5]));



initial begin
  $dumpfile("chain.lxt");
  $dumpvars();
  #200000 $finish();
end

endmodule
