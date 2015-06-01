`timescale 1ns / 1ns
module udp_spi_tx
#(parameter SCLK_DIV=50)
(input       c,
 input [7:0] rxd,
 input       rxdv,
 output      cs,
 output      sclk,
 output      mosi,
 input       miso);

// by the time the data stream gets here, it is already known-valid.
// register it again to help timing
wire [7:0] rxd_i;
d1 #(8) rxd_d1_r(.c(c), .d(rxd), .q(rxd_i));

wire rxdv_i;
d1 rxdv_d1_r(.c(c), .d(rxdv), .q(rxdv_i));

wire [11:0] cnt;
r #(12) cnt_r(.c(c), .rst(~rxdv_i), .en(1'b1), .d(cnt + 1'b1), .q(cnt));

wire [7:0] reg_cmd;
r #(8) reg_cmd_r(.c(c), .rst(~rxdv_i), .en(cnt == 12'd0),
                 .d(rxd_i), .q(reg_cmd));
wire [7:0] reg_subcmd;
r #(8) reg_subcmd_r(.c(c), .rst(~rxdv_i), .en(cnt == 12'd1),
                    .d(rxd_i), .q(reg_subcmd));

wire cmd_valid = (reg_cmd == 8'h3) & (reg_subcmd == 8'h0) & (cnt > 11'h1);

/*
assign cs = rxdv_i; //cmd_valid;
assign sclk = cmd_valid; //1'b0;
assign mosi = 1'b0;
*/

spi_master #(.SCLK_DIV(SCLK_DIV)) spi_master_inst
(.c(c),
 .txd(rxd_i), .txdv(cmd_valid & rxdv_i),
 .sclk(sclk), .mosi(mosi), .miso(miso), .cs(cs));

endmodule
