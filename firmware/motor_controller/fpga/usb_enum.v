`timescale 1ns/1ns
module usb_enum
(input         c,           // bus clock: 100 mhz
 input         rst,         // means we need to start enum over again
 input         en,          // means we're allowed to tx/rx
 output        done,        // means enumeration is complete
 input         tx_sie_done,
 output [18:0] token_d,
 output        token_start,
 output  [7:0] data_d,
 output        data_dv,
 input   [7:0] rxd,
 input         rxdv,
 output        ack_start
);

`include "usb_defs.v"

wire use_zero_addr;
wire [6:0] addr = use_zero_addr ? 7'b0 : USB_DEV_ADDR; 

wire [63:0] ctrl_tx_data;
wire ctrl_start, ctrl_done;
usb_ctrl usb_ctrl_inst
(.c(c), .rst(rst), .tx_sie_done(tx_sie_done), .d(ctrl_tx_data), 
 .txd(data_d), .txdv(data_dv), .rxd(rxd), .rxdv(rxdv),
 .start(ctrl_start), .done(ctrl_done),
 .token_start(token_start), .ack_start(ack_start),
 .addr(addr), .token(token_d));

`include "usb_pids.v"

wire [1:0] tx_data_sel;
gmux #(.DWIDTH(64), .SELWIDTH(2)) tx_data_gmux
(.d({64'h0, 64'h0, 
     16'h0, 16'h0, 16'h0,                8'h9, 8'h0,   // set configuration 0
     16'h0, 16'h0, {9'h0, USB_DEV_ADDR}, 8'h5, 8'h0}), // set address 1
 .sel(tx_data_sel), .z(ctrl_tx_data));

localparam ST_RESET           = 4'd0;
localparam ST_SET_ADDR        = 4'd1;
localparam ST_SET_ADDR_WAIT   = 4'd2;
localparam ST_CHILL           = 4'd3;
localparam ST_SET_CONFIG      = 4'd4;
localparam ST_SET_CONFIG_WAIT = 4'd5;
localparam ST_MORECHILL       = 4'd6;
localparam ST_DONE            = 4'd7;

localparam SW=4, CW=7;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r
(.c(c), .rst(rst), .en(1'b1), .d(next_state), .q(state));

wire en_d1;
d1 en_d1_r(.c(c), .d(en), .q(en_d1));

wire [7:0] frame_cnt;
r #(8) frame_cnt_r
(.c(c), .rst(state == ST_RESET | ctrl[4]), .en(en & ~en_d1),
 .d(frame_cnt+1'b1), .q(frame_cnt));

always @* begin
  case (state)
    ST_RESET:
      if (en)               ctrl = { ST_SET_ADDR       , 7'b01000 };
      else                  ctrl = { ST_RESET          , 7'b01000 };
    ST_SET_ADDR:            ctrl = { ST_SET_ADDR_WAIT  , 7'b01001 };
    ST_SET_ADDR_WAIT:
      if (ctrl_done)        ctrl = { ST_CHILL          , 7'b01000 };
      else                  ctrl = { ST_SET_ADDR_WAIT  , 7'b01000 };
    ST_CHILL: // wait for a bit, for usb device to change addrs
      if (frame_cnt > 8'h1) ctrl = { ST_SET_CONFIG     , 7'b00000 };
      else                  ctrl = { ST_CHILL          , 7'b00000 };
    ST_SET_CONFIG:          ctrl = { ST_SET_CONFIG_WAIT, 7'b00011 };
    ST_SET_CONFIG_WAIT:
      if (ctrl_done)        ctrl = { ST_MORECHILL      , 7'b10000 };
      else                  ctrl = { ST_SET_CONFIG_WAIT, 7'b00000 };
    ST_MORECHILL: // wait for a few more frames
      if (frame_cnt > 8'h1) ctrl = { ST_DONE           , 7'b00000 };
      else                  ctrl = { ST_MORECHILL      , 7'b00000 };
    ST_DONE:                ctrl = { ST_DONE           , 7'b00000 };
    default:                ctrl = { ST_RESET          , 7'b00000 };
  endcase
end

assign ctrl_start  = ctrl[0];
assign tx_data_sel = ctrl[2:1];
assign use_zero_addr = ctrl[3];

assign done = state == ST_DONE;

endmodule
