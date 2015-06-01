`timescale 1ns / 1ns
module meganode_tb();

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

wire enet_rst, enet_mdc, enet_mdio;
wire [3:0] enet_leds;
wire [1:0] enet_txclk, enet_txen, enet_rxclk, enet_rxdv;
wire [3:0] enet_txd, enet_rxd;

wire [4:0] usb_oe, usb_rcv, usb_pwr, usb_vp, usb_vm;
wire [15:0] outb;
wire [2:0] mosfet_hi, mosfet_lo, mclk;
reg [2:0] mdata;
wire mosfet_en;
wire mcu_io;
wire mcu_spim_cs, mcu_spim_sclk, mcu_spim_mosi, mcu_spim_miso;
wire mcu_spis_cs, mcu_spis_sclk, mcu_spis_mosi, mcu_spis_miso;
wire led;

localparam W = 8;
reg [W-1:0] master_txd;
wire [W-1:0] master_rxd;
reg master_txdv;
wire master_rxdv;

wire master_done, master_busy;
spi_master #(.SCLK_DIV(50), .W(8)) spi_master_inst
  (.c(clk_100), .busy(master_busy), .done(master_done),
   .txd(master_txd), .txdv(master_txdv),
   .rxd(master_rxd), .rxdv(master_rxdv),
   .cs(mcu_spis_cs), .sclk(mcu_spis_sclk), 
   .mosi(mcu_spis_mosi), .miso(mcu_spis_miso));

initial begin
  master_txd = 8'ha5;
  master_txdv = 0;
  #100
  @(posedge clk_100);
  #1 master_txdv = 1;
  @(posedge clk_100);
  #1 master_txd = 8'h7;
  @(posedge clk_100);
  #1 master_txd = 8'h51;
  @(posedge clk_100);
  #1 master_txdv = 0;
  #50000
  @(posedge clk_100);
  #1 master_txdv = 1;
  @(posedge clk_100);
  #1 master_txd = 8'h8;
  @(posedge clk_100);
  #1 master_txd = 8'h52;
  @(posedge clk_100);
  #1 master_txdv = 0;
end

meganode meganode_inst(.*);

initial begin
  $dumpfile("meganode.lxt");
  $dumpvars();
  #1_000_000 $finish();
  //#150000 $finish();
end

fake_rmii_phy #(.INPUT_FILE_NAME("tb_packets.dat")) sim_rmii_phy_0
(.refclk(clk_50), .mdc(enet_mdc), .mdio(enet_mdio),
 .txd(enet_txd[1:0]), .txen(enet_txen[0]), 
 .rxd(enet_rxd[1:0]), .rxdv(enet_rxdv[0]), 
 .rst(1'b1));

fake_rmii_phy #(.INPUT_FILE_NAME("tb_packets.dat")) sim_rmii_phy_1
(.refclk(clk_50), .mdc(enet_mdc), .mdio(enet_mdio),
 .txd(enet_txd[3:2]), .txen(enet_txen[1]), 
 .rxd(enet_rxd[3:2]), .rxdv(enet_rxdv[1]), 
 .rst(1'b1));

wire [4:0] usb_dp, usb_dm;
sim_fsusb_phy sim_usb_phy[4:0]
(.vp(usb_vp), .vm(usb_vm), .oe_n(usb_oe), .pwr(usb_pwr), 
 .dp(usb_dp), .dm(usb_dm));

sim_fsusb_encoder sim_enc[2:0]
(.dp(usb_dp[2:0]), .dm(usb_dm[2:0]));

sim_fsusb_foot sim_foot
(.dp(usb_dp[3]), .dm(usb_dm[3]));

integer i;
reg [2:0] all_bits [999:0];
initial begin
  $readmemh("sigma_delta_test_data.txt", all_bits, 0, 999);
  mdata = 3'b0;
  for (i = 0; i < 1000; i = i + 1) begin
    wait(mclk[0]);
    wait(~mclk[0]);
    mdata = all_bits[i]; //{3{all_bits[i][0]}};
  end
end

endmodule
