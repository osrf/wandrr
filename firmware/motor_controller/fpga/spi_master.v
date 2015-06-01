`timescale 1ns/1ns
module spi_master
#(parameter SCLK_DIV = 8'd8,
  parameter W = 8)
(input c, output busy, output done,
 input [W-1:0] txd, input txdv, 
 output [W-1:0] rxd, output rxdv,
 output sclk, output mosi, input miso, output cs);

assign rxd = 8'h0;
assign rxdv = 1'b0;

wire [W-1:0] txd_d1;
d1 #(8) txd_d1_r(.c(c), .d(txd), .q(txd_d1));

wire txdv_d1;
d1 txdv_d1_r(.c(c), .d(txdv), .q(txdv_d1));

wire txe = txdv_d1 & ~txdv;

wire [7:0] sclk_cnt;
wire sclk_match = sclk_cnt == SCLK_DIV-1;

r #(8) sclk_cnt_r
(.c(c), .en(1'b1), .rst(sclk_match), .d(sclk_cnt+1'b1), .q(sclk_cnt));

wire sclk_int;
r sclk_int_r(.c(c), .en(sclk_match), .rst(1'b0), 
             .d(~sclk_int), .q(sclk_int));

wire sclk_rising  = ~sclk_int & sclk_match;
wire sclk_falling =  sclk_int & sclk_match;

localparam SW = 3;
localparam ST_IDLE        = 3'd0;
localparam ST_ASSERT_CS   = 3'd1;
localparam ST_WAIT_BEFORE_DATA = 3'd2;
localparam ST_READ_DATA   = 3'd3;
localparam ST_TX_WORD     = 3'd4;
localparam ST_POST_DATA   = 3'd5;
localparam ST_DONE        = 3'd6;

localparam CW = 4;
reg [SW+CW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_reg
(.c(c), .d(next_state), .rst(1'b0), .en(1'b1), .q(state));

wire dfifo_rdreq, dfifo_empty;
wire [8:0] dfifo_q;
wire [7:0] dfifo_usedw;

scfifo #(.lpm_width(9),
         .lpm_numwords(256),
         .lpm_widthu(8),
         .lpm_showahead("ON"),
         .use_eab("ON"),
         .intended_device_family("CYCLONE V")) dfifo // data fifo
(.clock(c),
 .wrreq(txdv_d1), .data({txe, txd_d1}),
 .rdreq(dfifo_rdreq), .q(dfifo_q),
 .empty(dfifo_empty), .usedw(dfifo_usedw), .aclr(1'b0), .sclr(1'b0));

assign cs = ~ctrl[0];

localparam BIT_CNT_W = 8; 
wire [BIT_CNT_W-1:0] bit_cnt;
wire bit_cnt_rst, bit_cnt_en;
r #(BIT_CNT_W) bit_cnt_r
(.c(c), .rst(bit_cnt_rst), .en(bit_cnt_en), .d(bit_cnt + 1'b1), .q(bit_cnt));
assign bit_cnt_rst = state == ST_READ_DATA | state == ST_ASSERT_CS;
assign bit_cnt_en = (state == ST_TX_WORD & sclk_rising) |
                    (state == ST_WAIT_BEFORE_DATA & sclk_rising);

wire last_byte;
r last_byte_r
(.c(c), .rst(state == ST_IDLE), .en(state == ST_READ_DATA), 
 .d(dfifo_q[8]), .q(last_byte));

assign dfifo_rdreq = state == ST_READ_DATA;

always @* begin
  case (state)
    ST_IDLE:
      if (~dfifo_empty & sclk_rising) ctrl = { ST_ASSERT_CS , 4'b0001 };
      else                            ctrl = { ST_IDLE      , 4'b0000 };
    ST_ASSERT_CS:
      if (sclk_falling)               ctrl = { ST_WAIT_BEFORE_DATA, 4'b0001 };
      else                            ctrl = { ST_ASSERT_CS , 4'b0001 };
    ST_WAIT_BEFORE_DATA:
      if (sclk_falling & bit_cnt == 8'h7)
                                      ctrl = { ST_READ_DATA , 4'b0001 };
      else                            ctrl = { ST_WAIT_BEFORE_DATA, 4'b0001};
    ST_READ_DATA:                     ctrl = { ST_TX_WORD   , 4'b0011 };
    ST_TX_WORD:
      if (sclk_falling & bit_cnt == 8'h8)
        if (last_byte | dfifo_empty)  ctrl = { ST_DONE      , 4'b0011 };
        else                          ctrl = { ST_READ_DATA , 4'b0011 };
      else                            ctrl = { ST_TX_WORD   , 4'b0011 };
    ST_POST_DATA:
      if (sclk_falling)               ctrl = { ST_DONE      , 4'b0001 };
      else                            ctrl = { ST_POST_DATA , 4'b0001 };
    ST_DONE:                          ctrl = { ST_IDLE      , 4'b0001 };
    default:                          ctrl = { ST_IDLE, 4'b0000 };
  endcase
end

assign sclk = ctrl[1] ? sclk_int : 1'b1;
assign done = state == ST_DONE;
assign busy = (state != ST_IDLE) & (state != ST_DONE);

wire mosi_load = state == ST_READ_DATA;
wire mosi_shift_en = state == ST_TX_WORD & sclk_falling;
wire [W-1:0] mosi_shift;
r #(W) mosi_shift_r
  (.c(c), .rst(1'b0), .en(mosi_shift_en | mosi_load),
   .d(state == ST_READ_DATA ? dfifo_q[W-1:0] : {mosi_shift[W-2:0],1'b0}), 
   .q(mosi_shift));
assign mosi = ~cs & mosi_shift[W-1];

endmodule

/*

wire [WIDTH-1:0] miso_shift;
wire miso_shift_en;
wire miso_shift_reset = state == ST_ASSERT_CS;
r #(WIDTH) miso_shift_r(.c(clk), .rst(miso_shift_reset), .en(miso_shift_en),
                        .d({miso_shift[WIDTH-2:0], miso_sync}), 
                        .q(miso_shift));
// delay registering MISO until it works its way out of synchronization

// todo: this is probably not enough. probably needs to take CPOL into account
wire miso_sclk_capture_edge;
generate 
  if (CPHA == 1) begin
    assign miso_sclk_capture_edge = sclk_rising;
  end else begin
    assign miso_sclk_capture_edge = sclk_falling;
  end
endgenerate

wire hack_cpol_extra_sample; // abomination
wire [DEBOUNCE-1:0] miso_shift_en_delay;
r #(DEBOUNCE) miso_shift_en_delay_r
  (.c(clk), .en(1'b1), .rst(1'b0),
   .d({((state == ST_DATA) & miso_sclk_capture_edge) | hack_cpol_extra_sample, 
       miso_shift_en_delay[DEBOUNCE-1:1]}), 
   .q(miso_shift_en_delay));
assign miso_shift_en = miso_shift_en_delay[0];


wire sclk_gated = (state == ST_DATA ? sclk_int : 1'b1 );
wire sclk_pos_pol = SCLK_FREE_RUNNING ? sclk_int : sclk_gated;
assign sclk = CPOL ? sclk_pos_pol : ~sclk_pos_pol;
assign ss = ~ctrl[0];
assign done = (state == ST_DONE) & sclk_falling;
assign miso_data = miso_shift;

generate 
  if (CPOL == 0) begin
    assign hack_cpol_extra_sample = ctrl[3];
  end else begin
    assign hack_cpol_extra_sample = 0;
  end
endgenerate
*/

/////////////////////////////////////////////////////////////////////////

`ifdef test_spi_master
module spi_master_tb();
wire c;
sim_clk #(100) clk_inst(c);

localparam W = 8;
reg [W-1:0] txd;
wire [W-1:0] rxd;
reg txdv;
wire rxdv;

wire done, busy;
wire sclk, mosi, miso, cs;
spi_master #(.SCLK_DIV(50), .W(8)) spi_master_inst
  (.c(c), .busy(busy), .done(done),
   .txd(txd), .txdv(txdv),
   .rxd(rxd), .rxdv(rxdv),
   .sclk(sclk), .mosi(mosi), .miso(miso), .cs(cs));

wire [W-1:0] slave_rxd;
wire slave_rxdv, slave_rxe;
spi_slave_rx spi_slave_rx_inst
(.clk(c), 
 .rxd(slave_rxd), .rxdv(slave_rxdv), .rxe(slave_rxe),
 .cs(cs), .mosi(mosi), .miso(miso), .sclk(sclk));

initial begin
  $dumpfile("spi_master_test.lxt");
  $dumpvars();
  txd = 8'ha5;
  txdv = 0;
  #100
  @(posedge c);
  #1 txdv = 1;
  @(posedge c);
  #1 txd = 8'h7;
  @(posedge c);
  #1 txd = 8'h51;
  @(posedge c);
  #1 txdv = 0;
  #10000
  @(posedge c);
  #1 txdv = 1;
  @(posedge c);
  #1 txdv = 0;
  #20000
  $finish;
end
endmodule
`endif

