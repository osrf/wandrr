`timescale 1ns / 1ns
module eth_grx
(input c,
 input [3:0] phy_rxd,
 input phy_rxc,
 input phy_rxdv,
 output [7:0] d,
 output dv,
 output erx); // end of RX

wire [7:0] int_d;
wire int_dv;
wire int_er;

altddio_in #(.width(5)) phy_rx_ddr
 (.inclock(~phy_rxc),
  .datain({phy_rxdv, phy_rxd}),
  .dataout_h({int_er, int_d[7:4]}),
  .dataout_l({int_dv, int_d[3:0]}),
  .aclr(1'b0), .aset(1'b0), .inclocken(1'b1), .sset(1'b0), .sclr(1'b0));

///////////////////////////////////////////////////////////////////////////
// this fifo takes us from the phy GRX clock domain to the on-chip 125 mhz 
wire [4:0] xclk_fifo_rdusedw;
wire [7:0] xclk_fifo_q;

wire draining;
r draining_r
 (.c(c), .en(|xclk_fifo_rdusedw[4:2]), .rst(xclk_fifo_rdusedw == 5'h2),
  .d(1'b1), .q(draining));

dcfifo #(.lpm_width(8),
         .lpm_numwords(32),
         .lpm_widthu(5),
         .lpm_showahead("ON"),
         .use_eab("ON"),
         .intended_device_family("CYCLONE V")) xclk_fifo
 (.wrclk(~phy_rxc), .rdclk(c),
  .wrreq(int_dv), .rdreq(draining),
  .data(int_d), .q(xclk_fifo_q),
  .rdusedw(xclk_fifo_rdusedw),
  .aclr(1'b0));

// synchronize to on-chip clk_125 domain
// todo: this doesn't always work, depending on how the PHY phase-locked 
// to the 25mhz clock. need to do a FIFO here.
/*
wire [29:0] rx_sync;
r #(30) rx_sync_r
 (.c(clk), .rst(1'b0), .en(1'b1),
  .d({rx_sync[19:0], int_dv, int_er, int_d}),
  .q(rx_sync));
wire raw_dv = rx_sync[29];
assign d = rx_sync[27:20];
*/
wire eth_dv = draining; //|xclk_fifo_rdusedw;
assign d = xclk_fifo_q;

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
      if (eth_dv && d == 8'h55)      ctrl = { ST_PREAMBLE, 3'b000 };
      else                           ctrl = { ST_IDLE    , 3'b000 };
    ST_PREAMBLE:                   
      if (eth_dv && d == 8'h55)      ctrl = { ST_PREAMBLE, 3'b000 };
      else if (eth_dv && d == 8'hd5) ctrl = { ST_PAYLOAD , 3'b000 };
      else                           ctrl = { ST_IDLE    , 3'b000 };
    ST_PAYLOAD:
      if (eth_dv)                    ctrl = { ST_PAYLOAD , 3'b001 };
      else                           ctrl = { ST_STOP    , 3'b010 };
    ST_STOP:                         ctrl = { ST_IDLE    , 3'b000 }; // fcs ?
    default:                         ctrl = { ST_IDLE    , 3'b000 };
  endcase
end

assign dv = ctrl[0];
assign erx = ctrl[1];

endmodule

