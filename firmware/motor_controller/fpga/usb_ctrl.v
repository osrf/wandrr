`timescale 1ns/1ns
module usb_ctrl // usb control endpoint transfer
(input         c,          // bus clock
 input         rst,
 output        token_start,
 output        ack_start,
 input         tx_sie_done,
 input  [ 6:0] addr,
 input  [63:0] d,
 output [18:0] token,
 output  [7:0] txd,
 output        txdv,
 input   [7:0] rxd,
 input         rxdv,
 input         start,
 output        done);
localparam ST_IDLE             = 4'h0;
localparam ST_TX_SETUP         = 4'h1;
localparam ST_TX_SETUP_WAIT    = 4'h2;
localparam ST_TX_DATA_PID      = 4'h3;
localparam ST_TX_DATA          = 4'h4;
localparam ST_TX_DATA_WAIT     = 4'h5;
localparam ST_TX_DATA_ACK_WAIT = 4'h6;
localparam ST_TX_IN            = 4'h7;
localparam ST_TX_IN_WAIT       = 4'h8;
localparam ST_RX_PID           = 4'h9;
localparam ST_RX_DATA          = 4'ha;
localparam ST_RX_NAK           = 4'hb;
localparam ST_TX_ACK           = 4'hc;
localparam ST_TX_ACK_WAIT      = 4'hd;
localparam ST_SUCCESS          = 4'he;
localparam ST_ERROR            = 4'hf;

`include "usb_pids.v"

localparam SW=4, CW=5;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r
(.c(c), .rst(rst), .en(1'b1), .d(next_state), .q(state));

wire [18:0] token_setup = { 4'b0, addr, PID_SETUP };
wire [18:0] token_in    = { 4'b0, addr, PID_IN    };
wire [1:0] token_sel;
gmux #(.DWIDTH(19), .SELWIDTH(2)) token_gmux
(.d({19'hx, 19'hx, token_in, token_setup }),
 .sel(token_sel), .z(token));

wire [2:0] cnt;
r #(3) cnt_r
(.c(c), .rst(state == ST_TX_DATA_PID), .en(1'b1), .d(cnt + 1'b1), .q(cnt));

localparam NAK_TIMEOUT = 11'h3ff; // this seems to be what linux does
wire nak_timer_rst;
wire [10:0] nak_timer;
r #(11) nak_timer_r
(.c(c), .rst(nak_timer_rst), .en(1'b1), .d(nak_timer+1'b1), .q(nak_timer));

always @* begin
  case (state)
    ST_IDLE:
      if (start)                 ctrl = { ST_TX_SETUP         , 5'b00000 };
      else                       ctrl = { ST_IDLE             , 5'b00000 };
    ST_TX_SETUP:                 ctrl = { ST_TX_SETUP_WAIT    , 5'b00000 };
    ST_TX_SETUP_WAIT:
      if (tx_sie_done)           ctrl = { ST_TX_DATA_PID      , 5'b00000 };
      else                       ctrl = { ST_TX_SETUP_WAIT    , 5'b00000 };
    ST_TX_DATA_PID:              ctrl = { ST_TX_DATA          , 5'b00001 };
    ST_TX_DATA:
      if (cnt == 3'd7)           ctrl = { ST_TX_DATA_WAIT     , 5'b00001 };
      else                       ctrl = { ST_TX_DATA          , 5'b00001 };
    ST_TX_DATA_WAIT:
      if (tx_sie_done)           ctrl = { ST_TX_DATA_ACK_WAIT , 5'b00000 };
      else                       ctrl = { ST_TX_DATA_WAIT     , 5'b00000 };
    ST_TX_DATA_ACK_WAIT: // TODO: timeout
      if (rxdv & rxd == PID_ACK) ctrl = { ST_TX_IN            , 5'b00000 };
      else                       ctrl = { ST_TX_DATA_ACK_WAIT , 5'b00000 };
    ST_TX_IN:                    ctrl = { ST_TX_IN_WAIT       , 5'b00010 };
    ST_TX_IN_WAIT:
      if (tx_sie_done)           ctrl = { ST_RX_PID           , 5'b00010 };
      else                       ctrl = { ST_TX_IN_WAIT       , 5'b00010 };
    ST_RX_PID: // TODO: timeout
      if (rxdv)
        if (rxd == PID_DATA1)    ctrl = { ST_RX_DATA          , 5'b00000 };  
        else if (rxd == PID_NAK) ctrl = { ST_RX_NAK           , 5'b01000 };
        else                     ctrl = { ST_ERROR            , 5'b00000 };
      else                       ctrl = { ST_RX_PID           , 5'b00000 };
    ST_RX_NAK:
      if (nak_timer == NAK_TIMEOUT)  ctrl = { ST_TX_IN        , 5'b00000 };
      else                           ctrl = { ST_RX_NAK       , 5'b00000 };
    ST_RX_DATA:
      if (~rxdv)                 ctrl = { ST_TX_ACK           , 5'b00000 };
      else                       ctrl = { ST_RX_DATA          , 5'b00000 };
    ST_TX_ACK:                   ctrl = { ST_TX_ACK_WAIT      , 5'b00000 };
    ST_TX_ACK_WAIT:
      if (tx_sie_done)           ctrl = { ST_SUCCESS          , 5'b00000 };
      else                       ctrl = { ST_TX_ACK_WAIT      , 5'b00000 };
    ST_SUCCESS:                  ctrl = { ST_IDLE             , 5'b00000 };
    ST_ERROR:                    ctrl = { ST_IDLE             , 5'b00000 };
    default:                     ctrl = { ST_IDLE             , 5'b00000 };
  endcase
end

assign token_start = state == ST_TX_SETUP | 
                     state == ST_TX_IN    ;
assign ack_start = state == ST_TX_ACK;
assign token_sel = ctrl[2:1];
assign nak_timer_rst = ctrl[3];
assign done = state == ST_ERROR | state == ST_SUCCESS; // todo: distinguish...

wire [63:0] shift;
r #(64) shift_r
(.c(c), .rst(1'b0), .en(state == ST_IDLE | state == ST_TX_DATA),
 .d(state == ST_IDLE ? d : { 8'h0, shift[63:8] }), .q(shift));
assign txdv = ctrl[0];
assign txd  = state == ST_TX_DATA_PID ? PID_DATA0 : shift[7:0];

endmodule
