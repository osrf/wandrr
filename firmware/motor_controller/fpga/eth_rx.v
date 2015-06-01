`timescale 1ns / 1ns
module eth_rx
(input c, // must be the 50 MHz RMII reference clock 
 input [1:0] phy_rxd,
 input phy_rxdv,
 output [7:0] d,
 output dv,
 output erx); // end of RX

wire [1:0] rmii_d;
sync #(2) rmii_d_sync_r
(.in(phy_rxd), .clk(c), .out(rmii_d));

wire rmii_dv;
sync rmii_dv_sync_r
(.in(phy_rxdv), .clk(c), .out(rmii_dv));

wire rmii_dv_d1;
d1 rxdv_d1_r(.d(rmii_dv), .c(c), .q(rmii_dv_d1));

wire [7:0] rxd_shift;
r #(8) rxd_shift_r
(.c(c), .d({rmii_d, rxd_shift[7:2]}), .rst(1'b0), .en(1'b1), .q(rxd_shift));

wire shift_cnt_rst;
wire [1:0] shift_cnt;
r #(2) shift_cnt_r
(.c(c), .d(shift_cnt+1'b1), .en(1'b1), .rst(shift_cnt_rst), .q(shift_cnt));

assign d = rxd_shift;

localparam SW = 3, CW = 3;
localparam ST_IDLE       = 3'd0;
localparam ST_PREAMBLE   = 3'd1; 
localparam ST_PAYLOAD    = 3'd2;
localparam ST_STOP       = 3'd3;

reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r(.c(c), .rst(1'b0), .en(1'b1), .d(next_state), .q(state));

always @* begin
  case (state)
    ST_IDLE: 
      if (rmii_dv && d == 8'h55)      ctrl = { ST_PREAMBLE, 3'b000 };
      else                            ctrl = { ST_IDLE    , 3'b000 };
    ST_PREAMBLE:                   
      if (rmii_dv && d == 8'h55)      ctrl = { ST_PREAMBLE, 3'b000 };
      else if (rmii_dv && d == 8'hd5) ctrl = { ST_PAYLOAD , 3'b100 };
      else                            ctrl = { ST_IDLE    , 3'b000 };
    ST_PAYLOAD:
      if (rmii_dv | rmii_dv_d1)       ctrl = { ST_PAYLOAD , 3'b001 };
      else                            ctrl = { ST_STOP    , 3'b010 };
    ST_STOP:                          ctrl = { ST_IDLE    , 3'b000 }; // fcs ?
    default:                          ctrl = { ST_IDLE    , 3'b000 };
  endcase
end

assign dv = state == ST_PAYLOAD & &shift_cnt;
assign erx = ctrl[1];
assign shift_cnt_rst = ctrl[2];

endmodule

`ifdef test_eth_rx

module eth_rx_tb();

wire c;
sim_clk #(50) clk_50(c);

wire [1:0] rmii_txd, rmii_rxd;
wire rmii_rst, mdc, mdio;
wire rmii_txen, rmii_rxdv;

fake_rmii_phy #(.INPUT_FILE_NAME("tb_packets.dat")) phy
(.refclk(c), .rst(rmii_rst), .mdc(mdc), .mdio(mdio), 
 .txd(rmii_txd), .txen(rmii_txen),
 .rxd(rmii_rxd), .rxdv(rmii_rxdv));

wire [7:0] rxd;
wire rxdv, erx;

eth_rx dut
(.c(c), .phy_rxd(rmii_rxd), .phy_rxdv(rmii_rxdv),
 .d(rxd), .dv(rxdv), .erx(erx));


initial begin
  $dumpfile("eth_rx.lxt");
  $dumpvars();
  #100000;
  $finish();
end


endmodule

`endif

