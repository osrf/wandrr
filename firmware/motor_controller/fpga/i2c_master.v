`timescale 1ns/1ns
// TODO: clean this up. it's pretty old.
module i2c_master
#(parameter LENWIDTH = 4)
( input         c, // clock

  // user interface
  input         req,
  input  [6:0]  addr,
  input  [LENWIDTH-1:0]  len, // # of bytes to xfer
  input         we,
  output        ack,
  output        err,

  input  [31:0] din,
  output        din_ack,

  output [31:0] dout,
  output        dout_dv,
  output        dout_eop,

  // i2c interface
  inout         sda,
  inout         scl
);

`include "clog2.inc"
localparam integer SCL_QUAD = 125000 / (4 * 400); // 78125
localparam integer SCL_WIDTH = 17; 

// generate scl
wire  [SCL_WIDTH-1:0] scl_cnt;
wire  scl_match = (scl_cnt == (SCL_QUAD-1));
wire  [SCL_WIDTH-1:0] scl_cnt_p1 = scl_match ? 0 : scl_cnt + 1;
wire pause_scl;
r #(SCL_WIDTH) scl_cnt_r
(.c(c), .rst(pause_scl), .d(scl_cnt_p1), .q(scl_cnt));
wire [1:0] scl_q; // quadrant of the SCL waveform that we're in
r #(2) scl_r
(.c(c), rst(1'b0), .en(scl_match & ~pause_scl), .d(scl_q + 2'h1), .q(scl_q));
// generate a pulse on the rising and falling edges of SCL

wire scl_0   = scl_match & scl_q == 2'b01;
wire scl_180 = scl_match & scl_q == 2'b11;
wire scl_270 = scl_match & scl_q == 2'b00;
wire scl_90_int  = scl_match & scl_q == 2'b10;
wire scl_90 = scl_90_int & scl;
assign pause_scl = scl_90_int & ~scl;

localparam SW            = 4;
localparam CTRL_BITS     = 9;
localparam ST_IDLE       = 4'd0;
localparam ST_START      = 4'd1;
localparam ST_ADDR       = 4'd2;
localparam ST_AACK       = 4'd3;
localparam ST_TXD        = 4'd4;
localparam ST_TXACK      = 4'd5;
localparam ST_RXD        = 4'd6;
localparam ST_RXACK      = 4'd7;
localparam ST_STOP       = 4'd8;
localparam ST_ERROR      = 4'd9;

// ctrl[0] = sda state, 0 = drive it low, 1 = let float high
// ctrl[1] = clear state counter
// ctrl[2] = increment state counter
// ctrl[3] = unused
// ctrl[4] = decrement word counter
// ctrl[5] = read sda into rx_data
// ctrl[7:6] = next_shift sel
// ctrl[8] = clear shift

reg [SW+CTRL_BITS-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CTRL_BITS-1:CTRL_BITS];
r #(SW) state_r(.c(c), rst(1'b0), .en(1'b1), .d(next_state), .q(state));
wire next_ack = (next_state == ST_STOP); // probably could be earlier?
r #(1) ack_r(.c(clk), .rst(1'b1), .en(1'b1), .d(next_ack), .q(ack));

assign sda = ctrl[0] ? 1'bz : 1'b0;

wire [3:0] cnt; // generic state counter, use for a bit count, status, etc.
r #(4) cnt_r(.c(c), .rst(ctrl[1]), .d(ctrl[2] ? cnt+4'h1 : cnt), .q(cnt));

// latch whether we are doing a read or write
wire we_reg_w;
r #(1) we_reg(.c(c), .rst(1'b0), .d(we), .en(req && !ack), .q(we_reg_w));

assign err = state == ST_ERROR;

wire [LENWIDTH-1:0] word, word_next;
assign word_next = word + 1;
r #(LENWIDTH) word_r
(.c(c), .d(word_next), .rst(state == ST_IDLE), .en(ctrl[4]), .q(word));
wire last_word = (word == len);

wire [0:31] tx_data;
wire new_tx_data = (state == ST_IDLE) | (word[1:0] == 2'h3);
wire [0:31] next_tx_data = new_tx_data ? din : {tx_data[8:31], 8'b0};
r #(32) tx_data_reg
(.c(c), .d(next_tx_data), .rst(1'b0), .en(ctrl[4]), .q(tx_data));
assign din_ack = new_tx_data & ctrl[4] & (we | we_reg_w);

wire [31:0] rx_data;
r #(32) rx_data_reg
(.c(c), .d({rx_data[30:0], sda}), .rst(1'b0), .en(ctrl[5]), .q(rx_data));

assign dout = rx_data;
wire next_dout_dv = (word[1:0] == 2'h3) & ctrl[4] & (state == ST_RXD);
wire dout_dv_int;
r #(1) dout_dv_reg
(.c(c), .d(next_dout_dv), .rst(1'b0), .en(1'b1), .q(dout_dv_int));

wire last_word_d1;
r #(1) lw_reg(.c(c), .d(last_word), .rst(1'b0), .en(1'b1), .q(last_word_d1));
wire last_dout = last_word & ~last_word_d1 & ~we_reg_w;

assign dout_dv = dout_dv_int | last_dout;
assign dout_eop = last_word;

wire [7:0] shift, shift_next;
gmux #(.DWIDTH(8), .SELWIDTH(2)) shift_sel
(.d({addr, ~we_reg_w, tx_data[0:7], shift[6:0], 1'b0, shift}),
 .sel(ctrl[7:6]), .z(shift_next));

r #(8) shift_r(.c(c), .d(shift_next), .rst(ctrl[8]), .en(1'b1), .q(shift));

always @* begin
case (state)
  ST_IDLE:
    if (req)                ctrl = {ST_START, 3'b100, 6'b010011};
    else                    ctrl = {ST_IDLE , 3'b100, 6'b000001};
  ST_ERROR:
    if (req)                ctrl = {ST_START, 3'b100, 6'b000011};
    else                    ctrl = {ST_ERROR, 3'b100, 6'b000001};
  ST_START:
    if (~cnt[0])  // haven't triggered the pulldown yet
      if (scl_90)           ctrl = {ST_START, 3'b000, 6'b000100}; // start hold down
      else                  ctrl = {ST_START, 3'b000, 6'b000001}; // wait for 90 degrees
    else
      if (scl_270)          ctrl = {ST_ADDR,  3'b011, 6'b000010}; // done. move along.
      else                  ctrl = {ST_START, 3'b000, 6'b000000}; // keep it down
  ST_ADDR:
    if (scl_270)
      if (&cnt[2:0])        ctrl = {ST_AACK,  3'b001, 6'b000001};
      else                  ctrl = {ST_ADDR,  3'b001, 5'b00010, shift[7]};
    else                    ctrl = {ST_ADDR,  3'b000, 5'b00000, shift[7]};
  ST_AACK: // in the address phase, we are the transmitter; check for rx ack
    if (scl_90)
      if (sda)              ctrl = {ST_ERROR, 3'b000, 6'b000001}; // rx should pull down
      else                  ctrl = {ST_AACK , 3'b010, 6'b000001};
    else if (scl_270)       ctrl = {(we_reg_w ? ST_TXD : ST_RXD), 3'b000, 6'b000011};
    else                    ctrl = {ST_AACK , 3'b000, 6'b000001};
  ST_TXD:
    if (scl_270)
      if (&cnt[2:0])        ctrl = {ST_TXACK, 3'b001, 6'b000001};
      else                  ctrl = {ST_TXD  , 3'b001, 5'b00010, shift[7]};
    else                    ctrl = {ST_TXD  , 3'b000, 5'b00000, shift[7]};
  ST_TXACK:
    if (scl_90)
      if (sda)              ctrl = {ST_ERROR, 3'b000, 6'b000001}; // rx should pull down
      else                  ctrl = {ST_TXACK, 3'b000, 6'b010001};
    else if (scl_270)       ctrl = {last_word ? ST_STOP : ST_TXD,  3'b010, 6'b000011};
    else                    ctrl = {ST_TXACK, 3'b000, 6'b000001};
  ST_RXD:
    if (scl_90) // sample midway through the clock high time
      if (&cnt[2:0])        ctrl = {ST_RXACK, 3'b000, 6'b110011};
      else                  ctrl = {ST_RXD  , 3'b000, 6'b100101};
    else                    ctrl = {ST_RXD  , 3'b000, 6'b000001};
  ST_RXACK:
    if (~cnt[0])
      if (scl_270)          ctrl = {ST_RXACK, 3'b000, 5'b00010, last_word};
      else                  ctrl = {ST_RXACK, 3'b000, 6'b000001}; // wait for clk phase
    else
      if (scl_270)          ctrl = {last_word ? ST_STOP : ST_RXD, 3'b000,   6'b000011};
      else                  ctrl = {ST_RXACK, 3'b000, 5'b00000, last_word}; // hold down
  ST_STOP:
    if (scl_90)             ctrl = {ST_IDLE , 3'b000, 6'b000001};
    else                    ctrl = {ST_STOP , 3'b000, 6'b000000};
  default:                  ctrl = {ST_IDLE , 3'b000, 6'b000001};
endcase
end

assign scl = ((state == ST_IDLE | (state == ST_START & ctrl[0])) ? 
             1'bz : (scl_q[1] ? 1'bz : 1'b0));

endmodule

