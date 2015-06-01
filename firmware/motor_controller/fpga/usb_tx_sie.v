`timescale 1ns/1ns
module usb_tx_sie
(input c,
 input c_48,
 input rst,
 input [7:0] d,
 input dv,
 output done,
 output oe_n,
 inout vp,
 inout vm);

wire txf_q, txf_read, txf_empty;
usb_tx_fifo usb_tx_fifo_inst
(.c(c), .c_48(c_48), 
 .d(d), .dv(dv), // at 125 mhz
 .q(txf_q), .read(txf_read), 
 .empty(txf_empty)); // at 48 mhz

/////////////////////////////////////////////////////////////////////////
// everything below here is in the 48 MHz clock domain 

wire stuff_q, stuff_q_empty; // bit-stuffed output data and output data-valid
wire stuff_q_req; // read request from bit-stuffed data stream
usb_tx_stuff usb_tx_stuff_inst
(.c(c_48), .d(txf_q), .d_empty(txf_empty), .d_req(txf_read),
 .q(stuff_q), .q_req(stuff_q_req), .q_empty(stuff_q_empty));

localparam ST_IDLE          = 4'd0;
localparam ST_OE            = 4'd1;
localparam ST_DATA_BIT      = 4'd2;
localparam ST_LAST_BIT      = 4'd3;
localparam ST_EOP_SE0_0     = 4'd4;
localparam ST_EOP_SE0_1     = 4'd5;
localparam ST_EOP_J         = 4'd6;
localparam ST_IPG           = 4'd7; // let's do inter-packet gap of 6 bit times
localparam ST_DONE          = 4'd8;

localparam SW=4, CW=5;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r
(.c(c_48), .rst(rst), .en(1'b1), .d(next_state), .q(state));

wire [2:0] ones_count;
wire ones_count_rst, ones_count_en;
r #(3) ones_count_r
(.c(c_48), .rst(ones_count_rst), .en(ones_count_en),
 .d(ones_count + 1'b1), .q(ones_count));

wire [1:0] bit_timer;
r #(2) bit_timer_r
(.c(c_48), .rst(1'b0), .en(1'b1), .d(bit_timer+1'b1), .q(bit_timer));

wire advance = bit_timer == 2'b00;
//wire t1 = bit_timer == 2'b01;

assign stuff_q_req = ctrl[0];

// inter-packet gap timer
wire ipg_cnt_rst;
wire [4:0] ipg_cnt;
r #(5) ipg_cnt_r
(.c(c_48), .rst(ipg_cnt_rst), .en(1'b1), .d(ipg_cnt+1'b1), .q(ipg_cnt));

always @* begin
  case (state)
    ST_IDLE:
      if (~stuff_q_empty)            ctrl = { ST_OE       , 5'b11110 };
      else                           ctrl = { ST_IDLE     , 5'b00000 };
    ST_OE:
      if (advance /*& ipg_cnt > 5'd20*/) ctrl = { ST_DATA_BIT , 5'b01110 };
      else                           ctrl = { ST_OE       , 5'b01110 };
    ST_DATA_BIT:
      if (advance & stuff_q_empty)   ctrl = { ST_EOP_SE0_0, 5'b00010 };
      else if (advance)              ctrl = { ST_DATA_BIT , 5'b00011 };
      else                           ctrl = { ST_DATA_BIT , 5'b00010 };
    /*
    ST_LAST_BIT:
      if (advance)                   ctrl = { ST_EOP_SE0_0, 5'b00010 };
      else                           ctrl = { ST_LAST_BIT , 5'b00010 };
    */
    ST_EOP_SE0_0: 
      if (advance)                   ctrl = { ST_EOP_SE0_1, 5'b00110 };
      else                           ctrl = { ST_EOP_SE0_0, 5'b00110 };
    ST_EOP_SE0_1: 
      if (advance)                   ctrl = { ST_EOP_J    , 5'b10110 };
      else                           ctrl = { ST_EOP_SE0_1, 5'b00110 };
    ST_EOP_J:
      if (advance /*& ipg_cnt > 5'd20*/) ctrl = { ST_IPG      , 5'b11110 };
      else                           ctrl = { ST_EOP_J    , 5'b01110 };
    ST_IPG:
      if (ipg_cnt == 5'd3)           ctrl = { ST_DONE     , 5'b00000 };
      else                           ctrl = { ST_IPG      , 5'b00000 };
    ST_DONE:                         ctrl = { ST_IDLE     , 5'b00000 };
    default:                         ctrl = { ST_IDLE     , 5'b00000 };
  endcase
end

wire bit_hardcoded_en = ctrl[2];
wire bit_hardcoded_vp = ctrl[3];
wire bit_hardcoded_vm = 1'b0;

assign ipg_cnt_rst = ctrl[4];

wire oe_n_i = ~(ctrl[1] | rst);
d1 oe_d1_r(.c(c_48), .d(oe_n_i), .q(oe_n));

wire vp_out = rst ? 1'b0 : (bit_hardcoded_en ? bit_hardcoded_vp :  stuff_q);
wire vm_out = rst ? 1'b0 : (bit_hardcoded_en ? bit_hardcoded_vm : ~stuff_q);

wire vp_out_d1, vm_out_d1;
d1 vp_out_d1_r(.c(c_48), .d(vp_out), .q(vp_out_d1));
d1 vm_out_d1_r(.c(c_48), .d(vm_out), .q(vm_out_d1));

assign vp = ~oe_n ? vp_out_d1 : 1'bz;
assign vm = ~oe_n ? vm_out_d1 : 1'bz;

wire done_c48 = state == ST_DONE; 
sync done_s(.in(done_c48), .clk(c), .out(done));

endmodule
