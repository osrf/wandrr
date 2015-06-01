`timescale 1ns/1ns
module spi_slave_rx
(input clk,
 input cs,
 input sclk, 
 input mosi,
 output miso,
 output [7:0] rxd,
 output rxdv,
 output rxe); // rx end

assign miso = 1'b0; // for now, this thing is just an RX machine

wire sclk_sync, mosi_sync, cs_sync;
sync #(3, 3) in_sync
  (.clk(clk), .in({sclk, mosi, ~cs}), .out({sclk_sync, mosi_sync, cs_sync}));

wire cs_sync_d1;
r cs_sync_d1_r(.c(clk), .rst(1'b0), .en(1'b1), .d(cs_sync), .q(cs_sync_d1));

wire sclk_sync_d1;
r sclk_sync_d1_r(.c(clk), .rst(1'b0), .en(1'b1), 
                 .d(sclk_sync), .q(sclk_sync_d1));

wire mosi_sync_d1;
r mosi_sync_d1_r(.c(clk), .rst(1'b0), .en(1'b1),
                 .d(mosi_sync), .q(mosi_sync_d1));

localparam SW = 3;
localparam ST_IDLE    = 3'd0;
localparam ST_CS_LO   = 3'd1;
localparam ST_SCLK_LO = 3'd2;
localparam ST_SCLK_HI = 3'd3;
localparam ST_CS_HI = 3'd4;

localparam CW = 4;
reg [SW+CW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r(clk, next_state, 1'b0, 1'b1, state);

wire bit_cnt_rst, bit_cnt_en;
wire [2:0] bit_cnt;
r #(3) bit_cnt_r(.c(clk), .rst(bit_cnt_rst), .en(bit_cnt_en), 
                 .d(bit_cnt+1'b1), .q(bit_cnt));

always @* begin
  case (state)
    ST_IDLE:
      if (cs_sync)          ctrl = { ST_CS_LO  , 4'b0001 };
      else                  ctrl = { ST_IDLE   , 4'b0000 };
    ST_CS_LO:
      if (~sclk_sync)       ctrl = { ST_SCLK_LO, 4'b0000 };
      else                  ctrl = { ST_CS_LO  , 4'b0000 };
    ST_SCLK_LO:
      if (sclk_sync)        ctrl = { ST_SCLK_HI, 4'b0010 };
      else                  ctrl = { ST_SCLK_LO, 4'b0000 };
    ST_SCLK_HI:
      if (~cs_sync)         ctrl = { ST_CS_HI  , 4'b0100 };
      else if (~sclk_sync)  ctrl = { ST_SCLK_LO, 4'b0100 };
      else                  ctrl = { ST_SCLK_HI, 4'b0000 };
    ST_CS_HI:               ctrl = { ST_IDLE   , 4'b0000 };
    default:                ctrl = { ST_IDLE   , 4'b0000 };
  endcase
end

assign bit_cnt_rst  = ctrl[0];
assign bit_cnt_en   = ctrl[1];
wire sclk_hi_done = ctrl[2];

r #(8) rxd_r(.c(clk), .rst(1'b0), .en(bit_cnt_en), 
             .d({rxd[6:0], mosi_sync_d1}), .q(rxd));
assign rxdv = bit_cnt == 3'b0 & sclk_hi_done; //state == ST_SCLK_HI;

assign rxe = state == ST_CS_HI;

endmodule

