`timescale 1ns/1ns
module udp_outbound_chain_tx
(input        c,
 input  [7:0] rxd,
 input        rxdv,
 output [7:0] txd,
 output       txdv);

localparam SW = 5, CW = 5;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r
  (.c(c), .rst(1'b0), .en(1'b1), .d(next_state), .q(state));

localparam ST_IDLE           = 5'h0;
localparam ST_PROTO_VER      = 5'h1;
localparam ST_HOP_COUNT_LOW  = 5'h2;
localparam ST_HOP_COUNT_HIGH = 5'h3;
localparam ST_PAYLOAD        = 5'h4;
localparam ST_POSTLUDE       = 5'h5;
localparam ST_DISCARD        = 5'h6;

wire rx_cnt_rst;
wire [10:0] rx_cnt;
r #(11) rx_cnt_r
(.c(c), .rst(rx_cnt_rst), .en(1'b1), .d(rx_cnt+1'b1), .q(rx_cnt));
assign rx_cnt_rst = ctrl[0];

wire [7:0] rxd_i = state == ST_HOP_COUNT_LOW ? rxd + 1'b1 : rxd;

wire [7:0] prev_rxd;
r #(8) prev_rxd_r(.c(c), .rst(1'b0), .en(rxdv), .d(rxd), .q(prev_rxd));

wire [7:0] rxd_d1, rxd_d2;
d1 #(8) rxd_d1_r(.c(c), .d(rxd_i ), .q(rxd_d1));
d1 #(8) rxd_d2_r(.c(c), .d(rxd_d1), .q(rxd_d2));
wire [15:0] rx_16bit = { rxd, prev_rxd };

wire [7:0] rx_hop_count;
r #(8) rx_hop_count_r
(.c(c), .rst(1'b0), .en(state == ST_PROTO_VER), 
 .d(rxd), .q(rx_hop_count));

assign txdv = ctrl[1];
assign txd = rxd_d2;

always @* begin
  case (state)
    ST_IDLE:
      if (rxdv)                 ctrl = { ST_PROTO_VER     , 5'b00000 };
      else                      ctrl = { ST_IDLE          , 5'b00001 };
    ST_PROTO_VER: 
      if (rxdv)
        if (rx_16bit == 16'h4321)    ctrl = { ST_HOP_COUNT_LOW , 5'b00001 };
        else                         ctrl = { ST_DISCARD       , 5'b00000 };
      else                      ctrl = { ST_PROTO_VER     , 5'b00001 };
    ST_HOP_COUNT_LOW:           
      if (rxdv)                 ctrl = { ST_HOP_COUNT_HIGH, 5'b00011 };
      else                      ctrl = { ST_HOP_COUNT_LOW , 5'b00001 };
    ST_HOP_COUNT_HIGH:          ctrl = { ST_PAYLOAD       , 5'b00011 };
    ST_PAYLOAD:
      if (~rxdv)                ctrl = { ST_POSTLUDE      , 5'b00011 };
      else                      ctrl = { ST_PAYLOAD       , 5'b00010 };
    ST_POSTLUDE:
      if (rx_cnt == 16'h1)      ctrl = { ST_IDLE          , 5'b00000 };
      else                      ctrl = { ST_POSTLUDE      , 5'b00010 };
    ST_DISCARD:
      if (~rxdv)                ctrl = { ST_IDLE          , 5'b00000 };
      else                      ctrl = { ST_DISCARD       , 5'b00000 };
    default:                    ctrl = { ST_IDLE          , 5'b00000 };
  endcase
end

endmodule

