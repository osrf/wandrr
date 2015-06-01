module mlvds_tx
(input c, input [4:0] mcb_addr,
 output [2:0] mlvds_de, output [2:0] mlvds_di,
 input [7:0] txd, input txdv, output tx_active,
 input rx_active);

// for now, this assumes that we do not accept incoming packets while the
// previous outgoing one was being transmitted. revisit if that ever matters.
wire txdv_d1;
d1 txdv_d1_r(.c(c), .d(txdv), .q(txdv_d1));
wire txe = ~txdv & txdv_d1;

// ensure we have waited a bit after the RX packet completion before we TX
wire [7:0] rx_inactive_count;
localparam RX_INACTIVE_TIME = 8'h80;
r #(8) rx_inactive_count_r
(.c(c), .rst(rx_active), .en(rx_inactive_count != RX_INACTIVE_TIME),
 .d(rx_inactive_count + 1'b1), .q(rx_inactive_count));
wire rx_released = rx_inactive_count == RX_INACTIVE_TIME;

localparam ST_IDLE           = 4'd0;
localparam ST_WAIT_FOR_CLEAR = 4'd1;
localparam ST_WARMUP         = 4'd2;
localparam ST_PREAMBLE       = 4'd3;
localparam ST_LEN_HI         = 4'd4;
localparam ST_LEN_LO         = 4'd5;
localparam ST_ADDR           = 4'd6;
localparam ST_FORMAT         = 4'd7;
localparam ST_FRAME          = 4'd8;
localparam ST_CRC_HI         = 4'd9;
localparam ST_CRC_LO         = 4'd10;
localparam ST_COOLDOWN       = 4'd11;

localparam SW = 4, CW = 6;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];

r #(SW) state_r
  (.c(c), .rst(1'b0), .en(1'b1), .d(next_state), .q(state));

wire [7:0] state_cnt;
r #(8) state_cnt_r(.c(c), .rst(next_state != state), .en(1'b1),
                    .d(state_cnt + 1'b1), .q(state_cnt));

wire fifo_rst = 1'b0;
wire [7:0] fifo_d, fifo_q;
wire fifo_dv = txdv & state == ST_IDLE;
wire fifo_rd;
wire fifo_empty;
wire [8:0] fifo_cnt;
fifo_512x8 fifo
(.c(c), .rst(fifo_rst), .d(txd), .dv(fifo_dv),
 .q(fifo_q), .read(fifo_rd), .count(fifo_cnt), .empty(fifo_empty));

always @* begin
  case (state)
    ST_IDLE:
      if (txe)                ctrl = { ST_WAIT_FOR_CLEAR, 6'b000000 };
      else                    ctrl = { ST_IDLE          , 6'b000000 };
    ST_WAIT_FOR_CLEAR:
      if (rx_released)        ctrl = { ST_WARMUP        , 6'b000001 };
      else                    ctrl = { ST_WAIT_FOR_CLEAR, 6'b000000 };
    ST_WARMUP:
      if (state_cnt == 8'h7f) ctrl = { ST_PREAMBLE , 6'b000001 };
      else                    ctrl = { ST_WARMUP   , 6'b000001 };
    ST_PREAMBLE:
      if (state_cnt == 8'hf)  ctrl = { ST_LEN_HI   , 6'b000011 };
      else                    ctrl = { ST_PREAMBLE , 6'b000011 };
    ST_LEN_HI:
      if (state_cnt == 8'hf)  ctrl = { ST_LEN_LO   , 6'b010001 };
      else                    ctrl = { ST_LEN_HI   , 6'b010001 };
    ST_LEN_LO:
      if (state_cnt == 8'hf)  ctrl = { ST_ADDR     , 6'b010101 };
      else                    ctrl = { ST_LEN_LO   , 6'b010101 };
    ST_ADDR:
      if (state_cnt == 8'hf)  ctrl = { ST_FRAME    , 6'b010111 };
      else                    ctrl = { ST_ADDR     , 6'b010111 };
    ST_FRAME:
      if (state_cnt[3:0] == 4'hf & fifo_empty)
                              ctrl = { ST_CRC_HI   , 6'b011011 };
      else                    ctrl = { ST_FRAME    , 6'b011011 };
    ST_CRC_HI:
      if (state_cnt == 8'hf)  ctrl = { ST_CRC_LO   , 6'b001101 };
      else                    ctrl = { ST_CRC_HI   , 6'b001101 };
    ST_CRC_LO:
      if (state_cnt == 8'hf)  ctrl = { ST_COOLDOWN , 6'b001111 };
      else                    ctrl = { ST_CRC_LO   , 6'b001111 };
    ST_COOLDOWN:
      if (state_cnt == 8'hf)  ctrl = { ST_IDLE     , 6'b000001 };
      else                    ctrl = { ST_COOLDOWN , 6'b000001 };
    default:                  ctrl = { ST_IDLE     , 6'b000000 };
  endcase
end


wire [2:0] tx_byte_mux_sel = ctrl[3:1];
wire [15:0] crc;

wire [7:0] tx_byte;
gmux #(.DWIDTH(8), .SELWIDTH(3)) tx_byte_mux
  (.d({crc[7:0], crc[15:8], fifo_q, 8'h0,
       {3'b0, mcb_addr}, fifo_cnt[7:0]+2'h1, 8'h16, 8'h0}),
   .sel(tx_byte_mux_sel),
   .z(tx_byte));

wire crc_dv = ctrl[4] & state_cnt[3:0] == 4'h0; //[0];
crc_ccitt crc_ccitt_inst
  (.clk(c), .rst(state == ST_IDLE), .d(tx_byte),
   .dv(crc_dv), .crc(crc));

`ifdef SIM
  reg [3:0] tx_squawk_nibble;
  reg tx_squawk_active;
  wire [3:0] tx_nibble = tx_squawk_active ? tx_squawk_nibble : (state_cnt[3] ? tx_byte[3:0] : tx_byte[7:4]);
`else
  wire [3:0] tx_nibble = state_cnt[3] ? tx_byte[3:0] : tx_byte[7:4];
`endif

// generate the outgoing data clock from the state clock

wire mlvds_di_clk_out = (state_cnt[2:0] == 3'h2) |
                        (state_cnt[2:0] == 3'h3) |
                        (state_cnt[2:0] == 3'h4) |
                        (state_cnt[2:0] == 3'h5);

wire [1:0] mlvds_di_d_out = state_cnt[2] ? tx_nibble[1:0] : tx_nibble[3:2];

r #(3) mlvds_out_r
(.c(c), .rst(1'b0), .en(1'b1), 
 .d({mlvds_di_d_out, mlvds_di_clk_out}), .q(mlvds_di));


assign fifo_rd = state == ST_FRAME & state_cnt[3:0] == 4'he;


/*
// need to delay the outbound negative-edge bits for one cycle
wire [3:0] tx_nibble_d1;
d1 #(4) tx_nibble_d1_r(.c(c), .d(tx_nibble), .q(tx_nibble_d1));

wire [3:0] tx_nibble_dout = { tx_nibble[3:2], tx_nibble_d1[1:0] };
*/

wire mlvds_de_d1;
d1 mlvds_de_d1_r
(.c(c), .d(ctrl[0]), .q(mlvds_de_d1));

assign mlvds_de = {3{mlvds_de_d1}};

assign tx_active = ctrl[0];

/*
SB_IO #(.PIN_TYPE(6'b110001)) mlvds_tx_ddr_d [1:0]
  (.PACKAGE_PIN(mlvds_di[2:1]),
   .OUTPUT_CLK(clk_50),
   .CLOCK_ENABLE(1'b1), .OUTPUT_ENABLE(1'b1),
   .INPUT_CLK(1'b0), .LATCH_INPUT_VALUE(1'b0),
   .D_OUT_0(tx_nibble_dout[3:2]),
   .D_OUT_1(tx_nibble_dout[1:0]));

SB_IO #(.PIN_TYPE(6'b110001)) mlvds_tx_ddr_c
  (.PACKAGE_PIN(mlvds_di[0]),
   .OUTPUT_CLK(clk_50_90),
   .CLOCK_ENABLE(1'b1), .OUTPUT_ENABLE(1'b1),
   .INPUT_CLK(1'b0), .LATCH_INPUT_VALUE(1'b0),
   .D_OUT_0(1'b1),
   .D_OUT_1(1'b0));
*/

`ifdef SIM
integer squawk_delay, seed;
initial begin
  seed = 0;
  tx_squawk_active = 1'b0;
  /*
  forever begin
    squawk_delay = $dist_uniform(seed, 1000, 2000);
    #squawk_delay;
    tx_squawk_nibble = $dist_uniform(seed, 0, 3);
    tx_squawk_active = 1'b1;
    #50;
    tx_squawk_active = 1'b0;
  end
  */
  //$display($time, " waiting %d ns...", delay);
end
`endif

endmodule

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// this is an unholy mess. it must be utterly rewritten at some point.
//////////////////////////////////////////////////////////////////////



/*
`ifdef SIM
reg mlvds_pll_reset;
initial begin
  mlvds_pll_reset = 0;
  #1000 mlvds_pll_reset = 1;
  $display("%g releasing simulated MLVDS PLL reset", $time);
end
`else
wire mlvds_pll_reset = 1'b1;
`endif

///////////////// MLVDS PLL
wire mlvds_pll_clk, mlvds_pll_clk_90, mlvds_pll_lock;
mlvds_pll mlvds_pll_inst
  (.REFCLK(mlvds_ro[0]), .RESET(mlvds_pll_reset), .LOCK(mlvds_pll_lock),
   .PLLOUTGLOBALA(mlvds_pll_clk), .PLLOUTGLOBALB(mlvds_pll_clk_90));
*/

/*
wire mlvds_clk, mlvds_clk_90;
//`ifdef SIM
  // add delays to simulate what the real on-chip routing does
//  assign #2.25 mlvds_clk = mlvds_pll_clk;
//  assign #2.25 mlvds_clk_90 = mlvds_pll_clk_90;
//`else
  assign mlvds_clk    = mlvds_pll_clk;
  assign mlvds_clk_90 = mlvds_pll_clk_90;
//`endif

wire c = mlvds_clk; // save typing

wire [1:0] mlvds_ro_dibit;
sync #(.W(2), .S(3)) ddr_data_sync
  (.in({mlvds_ro_posedge[1], mlvds_ro_negedge[1]}), 
   .out(mlvds_ro_dibit), .clk(c));
wire mlvds_ro_cs;
sync #(.S(3)) ddr_cs_sync
  (.in(mlvds_ro_posedge[0]), .out(mlvds_ro_cs), .clk(c));
wire mlvds_ro_cs_d1; // need this d1 to catch the last rx word
r mlvds_ro_cs_d1_r(.c(c), .rst(1'b0), .en(1'b1),
                   .d(mlvds_ro_cs), .q(mlvds_ro_cs_d1));
wire mlvds_ro_cs_posedge = mlvds_ro_cs & ~mlvds_ro_cs_d1;

wire [2:0] dibit_cnt;
wire dibit_cnt_rst; 
r #(3) dibit_cnt_r(.c(c), .rst(dibit_cnt_rst), .en(1'b1),
                   .d(dibit_cnt+1'b1), .q(dibit_cnt));

wire [7:0] rxd;
r #(8) rxd_r
  (.c(c), .rst(1'b0), .en(1'b1), 
   .d({rxd[5:0], mlvds_ro_dibit}), .q(rxd));

wire rxdv = dibit_cnt[1:0] == 2'b11 & mlvds_ro_cs_d1;

localparam ST_IDLE         = 4'd0;
localparam ST_RX_LEN_HI    = 4'd1;
localparam ST_RX_LEN_LO    = 4'd2;
localparam ST_RX_FRAME     = 4'd3;
localparam ST_RX_CSUM_HI   = 4'd4;
localparam ST_RX_CSUM_LO   = 4'd5;
localparam ST_RX_CHECK_PKT = 4'd6;

localparam ST_TX_

localparam ST_FRAME      = 4'd2;
localparam ST_CHECK_PKT  = 4'd3;
localparam ST_DRAIN_BUF  = 4'd4;
localparam ST_CRC_LO     = 4'd5; // save the low CRC byte after rxdv falls
localparam ST_ATX_TXWARM = 4'd6; // warm up the transmitter
localparam ST_ATX_PRE    = 4'd7; // async tx preamble
localparam ST_ATX_DRAIN  = 4'd8; // async tx frame format
localparam ST_ATX_TXCOOL = 4'd9; // async tx leave transmitter on one more byte

localparam SW = 4, CW = 6;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r
  (.c(c), .rst(1'b0), .en(1'b1), .d(next_state), .q(state));
wire idle = state == ST_IDLE; // save typing...

wire addr_match;
r addr_match_r
  (.c(c), .rst(idle), .d(1'b1), .q(addr_match),
   .en(state == ST_ADDR & 
       rxdv & 
       ((rxd[3:0] == mcb_addr) | (rxd[3:0] == 4'hf)))); // unicast or broadcast
wire reject_pkt;

wire rxfifo_almost_empty;

always @* begin
  case (state)
    ST_IDLE:
      if (rxdv & rxd == 8'h57)     ctrl = { ST_ADDR       , 6'b000000 };
      else if (|atxfifo_count)     ctrl = { ST_ATX_TXWARM , 6'b001010 };
      else                         ctrl = { ST_IDLE       , 6'b000000 };
    ST_ADDR:
      if (rxdv)                    ctrl = { ST_FRAME      , 6'b000000 };
      else                         ctrl = { ST_ADDR       , 6'b000000 };
    ST_FRAME:
      if (~mlvds_ro_cs)            ctrl = { ST_CHECK_PKT  , 6'b000000 };
      else                         ctrl = { ST_FRAME      , 6'b000000 };
    ST_CHECK_PKT:
      if (reject_pkt)              ctrl = { ST_IDLE       , 6'b000000 };
      else                         ctrl = { ST_DRAIN_BUF  , 6'b000001 };
    ST_DRAIN_BUF:               
      if (fifo_almost_empty)       ctrl = { ST_CRC_LO     , 6'b000001 };
      else                         ctrl = { ST_DRAIN_BUF  , 6'b000001 };
    ST_CRC_LO:                     ctrl = { ST_IDLE       , 6'b000000 };
    ST_ATX_TXWARM:
      if (dibit_cnt[1:0] == 2'b10) ctrl = { ST_ATX_PRE    , 6'b101110 };
      else                         ctrl = { ST_ATX_TXWARM , 6'b001100 };
    ST_ATX_PRE:
      if (dibit_cnt[1:0] == 2'b11) ctrl = { ST_ATX_DRAIN  , 6'b011100 };
      else                         ctrl = { ST_ATX_PRE    , 6'b001100 };
    ST_ATX_DRAIN:
      if (dibit_cnt[1:0] == 2'b11)
        if (atxfifo_count == 9'b1) ctrl = { ST_ATX_TXCOOL , 6'b111110 };
        else                       ctrl = { ST_ATX_DRAIN  , 6'b011100 };
      else                         ctrl = { ST_ATX_DRAIN  , 6'b001100 };
    ST_ATX_TXCOOL:
      if (dibit_cnt[2:0] == 3'd7)  ctrl = { ST_IDLE       , 6'b000100 };
      else                         ctrl = { ST_ATX_TXCOOL , 6'b001100 };
    default:                       ctrl = { ST_IDLE       , 6'b000000 };
  endcase
end

wire rxfifo_read = ctrl[0];
assign dibit_cnt_rst = ((state == ST_IDLE) & mlvds_ro_cs_posedge) | ctrl[1];

//wire mlvds_di_d;
//assign mlvds_di[2] = ((state == ST_ATX_PRE) | (state == ST_ATX_DRAIN)) ? 
//                     mlvds_di_d : 2'b0;
assign mlvds_de[2:1] = {ctrl[2], ctrl[2]};

// initialize the CRC calculator to start having already received a 0 (format)
wire [15:0] calc_crc;
crc_ccitt crc_ccitt_inst
  (.clk(c), .rst(idle), .d(rxd), .dv(rxdv & ~idle), .crc(calc_crc));

wire [15:0] calc_crc_d1;
r #(16) calc_crc_d1_r
  (.c(c), .rst(1'b0), .en(rxdv), .d(calc_crc), .q(calc_crc_d1));

wire [7:0] rxd_d1;
r #(8) rxd_d1_r(.c(c), .rst(1'b0), .en(rxdv), .d(rxd), .q(rxd_d1));

wire crc_match = state == ST_FRAME & { rxd_d1, rxd } == calc_crc_d1;
wire crc_match_d1;
r crc_match_d1_r(.c(c), .rst(1'b0), .en(1'b1), .d(crc_match), .q(crc_match_d1));

assign reject_pkt = ~addr_match | ~crc_match_d1;

wire [8:0] rxfifo_count;
fifo_512x8 rxfifo
(.c(c), .r(idle), .d(rxd), .dv(rxdv & state == ST_FRAME),
 .q(mlvds_rxd), .read(rxfifo_read), .count(rxfifo_count));

assign mlvds_rxdv = state == ST_DRAIN_BUF;
assign fifo_almost_empty = rxfifo_count == 9'd2;

////////////////////////////////////////////////////////////////////////
// TX stuff
// atxfifo = async tx fifo
wire [8:0] atxfifo_count;
wire [7:0] atxfifo_q;
wire atxfifo_read = ctrl[4];
wire atxfifo_rst = 1'b0;
fifo_512x8 atxfifo
(.c(c), .r(atxfifo_rst), .d(async_txd), .dv(async_txdv),
 .q(atxfifo_q), .read(atxfifo_read), .count(atxfifo_count));

wire [3:0] tx_octet_mux_state = (state - 4'd6);
wire [7:0] next_tx_octet;
gmux #(.DWIDTH(8), .SELWIDTH(2)) tx_octet_mux
  (.d({8'h0, atxfifo_q, atxfifo_q, 8'h57}),
   .sel(state >= ST_ATX_TXWARM ? tx_octet_mux_state[1:0] : 2'b11),
   .z(next_tx_octet));

wire [9:0] tx_shift;
r #(10) tx_shift_r
  (.c(mlvds_clk), .rst(1'b0), .en(1'b1), 
   .d(((dibit_cnt[1:0] == 2'b11) | ctrl[5]) ? 
      {tx_shift[7:6], next_tx_octet} : 
      {tx_shift[7:0], 2'b0}),
   .q(tx_shift));

wire etx_p = state == ST_ATX_TXCOOL & dibit_cnt >= 3'd4;
wire etx_n = state == ST_ATX_TXCOOL & dibit_cnt >= 3'd5;

SB_IO #(.PIN_TYPE(6'b110001)) mlvds_tx_ddr [1:0]
  (.PACKAGE_PIN(mlvds_di[2:1]),
   .OUTPUT_CLK(mlvds_clk),
   .CLOCK_ENABLE(1'b1), .OUTPUT_ENABLE(1'b1),
   .INPUT_CLK(1'b0), .LATCH_INPUT_VALUE(1'b0),
   .D_OUT_0({tx_shift[7], ctrl[3] & ~etx_p}),
   .D_OUT_1({tx_shift[8], ctrl[2] & ~etx_n}));

*/


