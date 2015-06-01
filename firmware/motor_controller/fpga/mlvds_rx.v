module mlvds_rx
#(parameter VENDOR="LATTICE",
  parameter FILTER_ON_ADDR="YES")
( input c,
  input  [4:0] mcb_addr,
  input  [2:0] mlvds_ro, input tx_active,
  output [15:0] rxlen, output [7:0] rxd, output rxdv,
  output rx_active);

wire [2:0] mlvds_ro_sync;
sync #(.W(3), .S(3)) mlvds_ro_sync_r
  (.clk(c), .in(mlvds_ro), .out(mlvds_ro_sync));

wire rxclk = mlvds_ro_sync[0];
wire rxclk_d1;
d1 rxclk_d1_r(.c(c), .d(rxclk)   , .q(rxclk_d1));

wire [7:0] rxclk_shift;
wire rxclk_active = |rxclk_shift & ~tx_active;
r #(8) rxclk_shift_r
 (.c(c), .rst(1'b0), .en(1'b1), 
  .d({rxclk_shift[6:0], rxclk}), .q(rxclk_shift));

wire rxclk_posedge =  rxclk & ~rxclk_d1 & ~tx_active;
wire rxclk_negedge = ~rxclk &  rxclk_d1 & ~tx_active;
wire rxclk_edge = (rxclk_posedge | rxclk_negedge) & ~tx_active;

wire [2:0] mlvds_ro_sync_d1;
d1 #(3) mlvds_ro_sync_d1_r
(.c(c), .d(mlvds_ro_sync), .q(mlvds_ro_sync_d1));

wire [1:0] rxd_posedge;
r #(2) rxd_posedge_r
 (.c(c), .rst(1'b0), .en(rxclk_posedge), 
  .d(mlvds_ro_sync_d1[2:1]), .q(rxd_posedge));

wire [1:0] rxd_negedge;
r #(2) rxd_negedge_r
 (.c(c), .rst(1'b0), .en(rxclk_negedge), 
  .d(mlvds_ro_sync_d1[2:1]), .q(rxd_negedge));

wire [3:0] nibble = { rxd_posedge, rxd_negedge };
wire nibble_dv;
d1 rx_negedge_d1_r(.c(c), .d(rxclk_negedge), .q(nibble_dv));

///////////////////////////////////////////////////////////////////////////
// below here, everything is in the on-chip clk domain

localparam ST_IDLE     = 3'd0;
localparam ST_PREAMBLE = 3'd1;
localparam ST_LEN      = 3'd2;
localparam ST_ADDR     = 3'd3;
localparam ST_FRAME    = 3'd4;
localparam ST_CRC      = 3'd5;
localparam ST_CHECK    = 3'd6;
localparam ST_DRAIN    = 3'd7;

localparam SW = 3, CW = 6;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
wire state_rst = (state <= ST_CRC & ~rxclk_active);
r #(SW) state_r
  (.c(c), .rst(state_rst), .en(1'b1), .d(next_state), .q(state));

wire nibble_cnt_rst = ctrl[3];
wire [15:0] nibble_cnt;
r #(16) nibble_cnt_r
(.c(c), .rst(nibble_cnt_rst), .en(nibble_dv),
 .d(nibble_cnt + 1'b1), .q(nibble_cnt));

// add another stage here to help with timing...
wire [15:0] nibble_cnt_plus4;
r #(16) nibble_cnt_plus4_r
 (.c(c), .rst(next_state != state), .en(1'b1),
  .d(nibble_cnt + 3'h4), .q(nibble_cnt_plus4));

wire [15:0] rx16;
wire [15:0] rx16_next = { rx16[12:0], nibble };
r #(16) rx16_r(.c(c), .rst(1'b0), .en(nibble_dv), .d(rx16_next), .q(rx16));

wire [14:0] len;
r #(15) len_r
( .c(c), .rst(1'b0), .en(state == ST_LEN & next_state == ST_ADDR),
  .d(rx16[14:0]), .q(len));
wire [15:0] len_nibbles = {len, 1'b0};
assign rxlen = {1'b0, len};

wire [15:0] rx_crc;
r #(16) rx_crc_r
( .c(c), .rst(1'b0), .en(state == ST_CRC & next_state == ST_CHECK),
  .d(rx16), .q(rx_crc));

wire [7:0] rx_addr;
r #(8) rx_addr_r
 (.c(c), .rst(1'b0), .en(state == ST_ADDR & next_state == ST_FRAME),
  .d(rx16[7:0]), .q(rx_addr));

// compute the crc byte-wise
wire nibble_dv_d1;
d1 nibble_dv_d1_r(.c(c), .d(nibble_dv), .q(nibble_dv_d1));
wire ctrl_0_d1;
d1 ctrl_0_d1_r(.c(c), .d(ctrl[0]), .q(ctrl_0_d1));
wire rx_byte_dv = ctrl_0_d1 & nibble_dv_d1 & ~nibble_cnt[0];
wire [7:0] rx_byte = rx16[7:0];
wire [15:0] calc_crc;
crc_ccitt crc_ccitt_inst
  (.clk(c), .rst(state == ST_IDLE), .d(rx_byte),
   .dv(rx_byte_dv), .crc(calc_crc));

wire rxfifo_almost_empty;
wire check_passed;

generate 
  if (FILTER_ON_ADDR == "YES") begin
    assign check_passed = (rx_crc == calc_crc) & 
                          (rx_addr == mcb_addr | rx_addr == 8'hff); // bcast
  end else begin
    assign check_passed = rx_crc == calc_crc;
  end
endgenerate

always @* begin
  case (state)
    ST_IDLE:
      if (nibble_dv)
                                           ctrl = { ST_PREAMBLE, 6'b001000 };
      else                                 ctrl = { ST_IDLE    , 6'b000000 };
    ST_PREAMBLE:
      if (rx16_next[7:0] == 8'h16)         ctrl = { ST_LEN     , 6'b001010 };
      else                                 ctrl = { ST_PREAMBLE, 6'b000000 };
    ST_LEN:
      if (nibble_cnt == 16'd4)             ctrl = { ST_ADDR    , 6'b001011 };
      else                                 ctrl = { ST_LEN     , 6'b000001 };
    ST_ADDR:
      if (nibble_cnt == 16'd2)             ctrl = { ST_FRAME   , 6'b001001 };
      else                                 ctrl = { ST_ADDR    , 6'b000001 };
    ST_FRAME:
      if (nibble_cnt + 16'd2 == len_nibbles) ctrl = { ST_CRC   , 6'b001001 };
      else                                 ctrl = { ST_FRAME   , 6'b000001 };
    ST_CRC:
      if (nibble_cnt == 16'd4)             ctrl = { ST_CHECK   , 6'b001010 };
      else                                 ctrl = { ST_CRC     , 6'b000000 };
    ST_CHECK:
      if (check_passed)                    ctrl = { ST_DRAIN   , 6'b001000 };
      else                                 ctrl = { ST_IDLE    , 6'b001000 };
    ST_DRAIN:
      if (rxfifo_almost_empty)             ctrl = { ST_IDLE    , 6'b001100 };
      else                                 ctrl = { ST_DRAIN   , 6'b000100 };
    default:                               ctrl = { ST_IDLE    , 6'b001000 };
  endcase
end

//assign rxfifo_rst = tx_active | ctrl[3];
assign rx_active = state != ST_IDLE;

generate
  if (VENDOR == "LATTICE") begin

    wire rxdv_early = state == ST_DRAIN;
    d1 rxdv_d1(.c(c), .d(rxdv_early), .q(rxdv));

    wire rxfifo_read = ctrl[2];
    wire [8:0] rxfifo_count;
    fifo_512x8 rxfifo
    (.c(c), .rst(state == ST_IDLE), 
     .d(rx_byte), .dv(rx_byte_dv & state == ST_FRAME),
     .q(rxd), .read(rxfifo_read), .count(rxfifo_count),
     .almost_empty(rxfifo_almost_empty));

  end else if (VENDOR == "ALTERA") begin

    assign rxdv = state == ST_DRAIN;
    //assign mlvds_rxfifo_rst = tx_active | ctrl[3];

    wire rxfifo_read = ctrl[2];
    wire [8:0] rxfifo_count;

    scfifo #(.lpm_width(8), .lpm_numwords(512), .lpm_widthu(9),
             .lpm_showahead("ON"), .intended_device_family("CYCLONE V"),
             .use_eab("ON")) rxfifo
     (.clock(c),
      .wrreq(rx_byte_dv & ((state == ST_ADDR) | (state == ST_FRAME))),
      .rdreq(rxfifo_read),
      .data(rx_byte), .q(rxd),
      .sclr(state == ST_IDLE),
      .usedw(rxfifo_count));

    assign rxfifo_almost_empty = rxfifo_count == 9'h1 | rxfifo_count == 9'h0;
  end
endgenerate

endmodule

