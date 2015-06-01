module meganode_wrapper
( input clk, output led, output [3:0] enet_leds,
  output enet_rst, output enet_clk, output enet_mdc, inout enet_mdio, 
  output [1:0] enet_txen, output [3:0] enet_txd,
  input  [1:0] enet_rxdv, input  [3:0] enet_rxd,
  output [4:0] usb_oe, output [4:0] usb_pwr,
  inout [4:0] usb_vp, inout [4:0] usb_vm,
  output [2:0] mosfet_hi, output [2:0] mosfet_lo, output mosfet_en,
  output [2:0] mclk, input [2:0] mdata, 
  output [15:0] outb, 
  input mcu_io,
  output mcu_spim_cs, output mcu_spim_sclk, 
  output mcu_spim_mosi, input mcu_spim_miso,
  input mcu_spis_cs, input mcu_spis_sclk, 
  input mcu_spis_mosi, output mcu_spis_miso,
  output scl, inout sda, output mextra);

wire clk_20, clk_25, clk_48, clk_50, clk_100;
wire pll_locked, pll_reset;
altera_pll 
#(.fractional_vco_multiplier("false"),
  .reference_clock_frequency("25.0 MHz"),
  .operation_mode("direct"), .number_of_clocks(5),
  .output_clock_frequency0("50MHz"), .phase_shift0("0 ps"), .duty_cycle0(50),
  .output_clock_frequency1("25MHz"), .phase_shift1("0 ps"), .duty_cycle1(50),
  .output_clock_frequency2("20MHz"), .phase_shift2("0 ps"), .duty_cycle2(50),
  .output_clock_frequency3("48MHz"), .phase_shift3("0 ps"), .duty_cycle3(50),
  .output_clock_frequency4("100MHz"), .phase_shift4("0 ps"), .duty_cycle4(50),
  .pll_type("General"), .pll_subtype("General")
) altera_pll_inst (
  .refclk(clk), .rst(pll_reset), .locked(pll_locked),
  .outclk({clk_100, clk_48, clk_20, clk_25, clk_50}),
  .fboutclk(), .fbclk(1'b0)
);

assign enet_clk = ~clk_50;
assign pll_reset = 1'b0;

assign mextra = 1'b0;
assign scl = 1'b1;
assign sda = 1'bz;

meganode meganode_inst
( .clk_20(clk_20), .clk_25(clk_25), .clk_50(clk_50), .clk_48(clk_48), .clk_100(clk_100),
  .enet_rst(enet_rst), .enet_mdc(enet_mdc), .enet_mdio(enet_mdio),
  .enet_txen(enet_txen), .enet_txd(enet_txd),
  .enet_rxdv(enet_rxdv), .enet_rxd(enet_rxd),
  .enet_leds(enet_leds),
  .usb_oe(usb_oe), .usb_pwr(usb_pwr), .usb_vp(usb_vp), .usb_vm(usb_vm),
  .mosfet_hi(mosfet_hi), .mosfet_lo(mosfet_lo), .mclk(mclk), .mdata(mdata),
  .mosfet_en(mosfet_en),
  .mcu_io(mcu_io),
  .mcu_spim_cs(mcu_spim_cs),     .mcu_spim_sclk(mcu_spim_sclk),
  .mcu_spim_mosi(mcu_spim_mosi), .mcu_spim_miso(mcu_spim_miso),
  .mcu_spis_cs(mcu_spis_cs),     .mcu_spis_sclk(mcu_spis_sclk),
  .mcu_spis_mosi(mcu_spis_mosi), .mcu_spis_miso(mcu_spis_miso),
  .led(led));

assign outb = {13'h0, mcu_spis_mosi, mcu_spis_sclk, mcu_spis_cs};

endmodule

