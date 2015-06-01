`timescale 1ns/1ns
module usb_poll
(input         c,           // bus clock: 125 mhz
 input         start,
 input         tx_sie_done,
 input         dont_respond, // for debugging stalling issues
 output [18:0] token_d,
 output        token_start,
 output  [7:0] payload_d,
 output        payload_dv,
 input   [7:0] sie_rxd,
 input         sie_rxdv,
 output        ack_start
);

`include "usb_pids.v"
`include "usb_defs.v"
wire [6:0] addr = USB_DEV_ADDR;
assign token_d = { 4'h1, addr, PID_IN }; // endpoint 1 IN

/*
wire [63:0] ctrl_tx_data;
wire ctrl_start, ctrl_done;
usb_ctrl usb_ctrl_inst
(.c(c), .tx_sie_done(tx_sie_done), .d(ctrl_tx_data), 
 .txd(data_d), .txdv(data_dv), .rxd(rxd), .rxdv(rxdv),
 .start(ctrl_start), .done(ctrl_done),
 .token_start(token_start), .ack_start(ack_start),
 .addr(addr), .token(token_d));
*/


localparam ST_IDLE            = 4'd0;
localparam ST_TX_IN           = 4'd1;
localparam ST_TX_IN_WAIT      = 4'd2;
localparam ST_RX_PID          = 4'd3;
localparam ST_RX_DATA         = 4'd4;
localparam ST_RX_NAK          = 4'd5;
localparam ST_TX_ACK          = 4'd6;
localparam ST_TX_ACK_WAIT     = 4'd7;
localparam ST_SUCCESS         = 4'd8;
localparam ST_ERROR           = 4'd9;

localparam SW=4, CW=7;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
wire rst = 1'b0; // TODO: allow remote reset/restart in case of bork pkt?
r #(SW) state_r
(.c(c), .rst(rst), .en(1'b1), .d(next_state), .q(state));

wire [15:0] timeout_cnt;
r #(16) timeout_cnt_r
(.c(c), .rst(state == ST_IDLE), .en(state == ST_RX_PID | state == ST_RX_DATA),
 .d(timeout_cnt + 1'b1), .q(timeout_cnt));

always @* begin
  case (state)
    ST_IDLE: 
      if (start)       ctrl = { ST_TX_IN      , 7'b0000000 };
      else             ctrl = { ST_IDLE       , 7'b0000000 };
    ST_TX_IN:          ctrl = { ST_TX_IN_WAIT , 7'b0000000 };
    ST_TX_IN_WAIT:
      if (tx_sie_done) ctrl = { ST_RX_PID     , 7'b0000000 };
      else             ctrl = { ST_TX_IN_WAIT , 7'b0000000 };
    ST_RX_PID:
      if (dont_respond) ctrl = { ST_ERROR     , 7'b0000000 };
      else if (sie_rxdv)
        if (sie_rxd == PID_DATA0 | sie_rxd == PID_DATA1)
                       ctrl = { ST_RX_DATA    , 7'b0000000 };
        else           ctrl = { ST_ERROR      , 7'b0000000 }; // maybe nak?
      else if (timeout_cnt >= 16'h1650)
                       ctrl = { ST_ERROR      , 7'b0000000 };
      else             ctrl = { ST_RX_PID     , 7'b0000000 };
    ST_RX_DATA:
      if (~sie_rxdv)   ctrl = { ST_TX_ACK     , 7'b0000000 };
      else             ctrl = { ST_RX_DATA    , 7'b0000000 };
    ST_TX_ACK:         ctrl = { ST_TX_ACK_WAIT, 7'b0000000 };
    ST_TX_ACK_WAIT:
      if (tx_sie_done) ctrl = { ST_SUCCESS    , 7'b0000000 };
      else             ctrl = { ST_TX_ACK_WAIT, 7'b0000000 };
    ST_SUCCESS:        ctrl = { ST_IDLE       , 7'b0000000 };
    ST_ERROR:          ctrl = { ST_IDLE       , 7'b0000000 };
    default:           ctrl = { ST_IDLE       , 7'b0000000 };
  endcase
end

assign payload_dv = sie_rxdv & state == ST_RX_DATA;
assign payload_d  = sie_rxd;

assign token_start = state == ST_TX_IN;
assign ack_start   = state == ST_TX_ACK;

endmodule
