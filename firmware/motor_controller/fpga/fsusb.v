`timescale 1ns/1ns
module fsusb
(input c,      // bus clock = 100 mhz
 input c_48,   // usb clock
 input en,     // enable input: otherwise it just sits in idle
 inout vp,
 inout vm,
 input dont_respond, // for debugging poll-path stalls
 output oe_n,
 output [7:0] ep1_rxd,
 output ep1_rxdv,
 output pwr);

`include "usb_pids.v"

localparam ST_IDLE       = 4'd0;
localparam ST_PWR_OFF    = 4'd1;
localparam ST_PWR_ON     = 4'd2;
localparam ST_DETECT     = 4'd3;
localparam ST_RESET      = 4'd4;
localparam ST_POST_RESET = 4'd5;
localparam ST_ENABLED    = 4'd6;
localparam ST_SOF_START  = 4'd7;
localparam ST_SOF_WAIT   = 4'd8;
localparam ST_ENUM       = 4'd9;
localparam ST_POLL       = 4'ha;

localparam SW=4, CW=7;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r
(.c(c), .rst(~en), .en(1'b1), .d(next_state), .q(state));

wire cnt_rst;
wire cnt_en = 1'b1;
wire [31:0] cnt; // counter used by many states
r #(32) cnt_r(.c(c), .rst(cnt_rst), .en(cnt_en), .d(cnt + 1'b1), .q(cnt));

wire vp_i48, vm_i48, vp_i, vm_i;
sync vp_sync48(.in(vp), .clk(c_48), .out(vp_i48));
sync vm_sync48(.in(vm), .clk(c_48), .out(vm_i48));
sync vp_sync  (.in(vp), .clk(c), .out(vp_i));
sync vm_sync  (.in(vm), .clk(c), .out(vm_i));


`ifndef SIM
localparam PWR_OFF_CNT    = 32'd050_000_000; // 0.5-second power-off delay
localparam PWR_ON_CNT     = 32'd100_000_000; // 1-second power-on delay for BL
localparam RESET_CNT      = 32'd002_000_000; // 20ms reset period
localparam POST_RESET_CNT = 32'd001_000_000; // 10ms reset-recovery period
`else
// these timeouts take forever to simulate. let's make it lots faster...
localparam PWR_OFF_CNT    = 32'd125; 
localparam PWR_ON_CNT     = 32'd062; 
localparam RESET_CNT      = 32'd050; 
localparam POST_RESET_CNT = 32'd080; 
`endif

`ifndef SIM
localparam FRAME_TRAFFIC_START = 17'h200;
localparam FRAME_LENGTH        = 17'd100_000; // every millisecond
localparam FRAME_END_QUIET     = 17'd005_000; // don't clobber SOF
localparam NUM_WAKEUP_FRAMES   = 11'd68; // sort of what linux does
`else
localparam FRAME_TRAFFIC_START = 17'h200; 
localparam FRAME_LENGTH        = 17'd010_000; // every 100 us, to speed up sim.
localparam FRAME_END_QUIET     = 17'd001_000; // don't clobber SOF
localparam NUM_WAKEUP_FRAMES   = 11'd2; // save time in simulation
`endif

wire [16:0] frame_time;
r #(17) frame_time_r
(.c(c), .rst(frame_time == FRAME_LENGTH), .en(1'b1), 
 .d(frame_time+1'b1), .q(frame_time));
wire sof = frame_time == 17'h0 & state >= ST_ENABLED;
wire sof_d1 = frame_time == 17'h1 & state >= ST_ENABLED;

wire [10:0] frame_number;
r #(11, 11'd0) frame_number_r
(.c(c), .en(sof), .rst(state == ST_RESET), 
 .d(frame_number+1'b1), .q(frame_number));

wire [18:0] sof_token = { frame_number, PID_SOF };
wire [18:0] enum_token, poll_token;
wire [1:0]  token_sel;
wire [18:0] token_d;
gmux #(.DWIDTH(19), .SELWIDTH(2)) token_gmux
(.d({19'h0, poll_token, enum_token, sof_token}), 
 .sel(token_sel), .z(token_d));

wire tx_token_start;
wire [7:0] tx_token_sie_d;
wire tx_token_sie_dv;
usb_tx_token usb_tx_token_inst
(.c(c), .d(token_d), .start(tx_token_start),
 .sie_d(tx_token_sie_d), .sie_dv(tx_token_sie_dv));

wire enum_en, enum_done;
wire [7:0] enum_data_d;
wire enum_data_dv, enum_token_start;

wire [7:0] tx_data_sie_d;
wire tx_data_sie_dv;
usb_tx_data usb_tx_data_inst
(.c(c), 
 .d(enum_data_d), 
 .dv(enum_data_dv),
 .sie_d(tx_data_sie_d), .sie_dv(tx_data_sie_dv));

wire [7:0] tx_ack_sie_d;
wire tx_ack_sie_dv;
wire enum_ack_start, poll_ack_start;
usb_tx_ack usb_tx_ack_inst
(.c(c),
 .start(enum_ack_start | poll_ack_start),
 .sie_d(tx_ack_sie_d), .sie_dv(tx_ack_sie_dv));

wire reset_48;
sync reset_sync(.in(state == ST_RESET), .clk(c_48), .out(reset_48));

wire tx_sie_done;
usb_tx_sie tx_sie_inst
(.c(c), .c_48(c_48), 
 .d(tx_token_sie_d   | tx_data_sie_d  | tx_ack_sie_d ),
 .dv(tx_token_sie_dv | tx_data_sie_dv | tx_ack_sie_dv),
 .oe_n(oe_n), .done(tx_sie_done), 
 .vp(vp), .vm(vm),
 .rst(reset_48));

wire [7:0] rx_sie_d;
wire rx_sie_dv;
usb_rx_sie rx_sie_inst
(.c(c), .c_48(c_48), .oe(~oe_n), .vp(vp_i48), .vm(vm_i48),
 .d(rx_sie_d), .dv(rx_sie_dv));

/*
assign oe_n = state == ST_RESET ? 1'b0 : tx_sie_oe_n; // assert OE during reset
assign   vp = state == ST_RESET ? 1'b0 : tx_sie_vp; // assert SE0 during reset
assign   vm = state == ST_RESET ? 1'b0 : tx_sie_vm;
*/

usb_enum enum_inst
(.c(c), .rst(state == ST_RESET), .en(enum_en), .done(enum_done),
 .token_d(enum_token), .token_start(enum_token_start), 
 .ack_start(enum_ack_start),
 .data_d(enum_data_d), .data_dv(enum_data_dv),
 .tx_sie_done(tx_sie_done),
 .rxd(rx_sie_d), .rxdv(rx_sie_dv));

wire poll_start, poll_token_start;
wire [7:0] poll_rxd;
wire poll_rxdv;
usb_poll poll_inst
(.c(c), .start(poll_start), .ack_start(poll_ack_start),
 .token_d(poll_token), .token_start(poll_token_start),
 .payload_d(poll_rxd), .payload_dv(poll_rxdv),
 .sie_rxd(rx_sie_d), .sie_rxdv(rx_sie_dv),
 .dont_respond(dont_respond),
 .tx_sie_done(tx_sie_done));

assign ep1_rxd  = poll_rxd;
assign ep1_rxdv = poll_rxdv;

always @* begin
  case (state)
    ST_IDLE:                     ctrl = { ST_PWR_OFF   , 2'b00, 5'b00001 };
    ST_PWR_OFF:
      if (cnt == PWR_OFF_CNT)    ctrl = { ST_PWR_ON    , 2'b00, 5'b00001 };
      else                       ctrl = { ST_PWR_OFF   , 2'b00, 5'b00000 };
    ST_PWR_ON:
      if (cnt == PWR_ON_CNT)     ctrl = { ST_DETECT    , 2'b00, 5'b00001 };
      else                       ctrl = { ST_PWR_ON    , 2'b00, 5'b00000 };
    ST_DETECT:                   ctrl = { ST_RESET     , 2'b00, 5'b00001 };
    /*
    ST_DETECT: // add timeout?
      if (vp_i & ~vm_i)          ctrl = { ST_RESET     , 2'b00, 5'b00001 };
      else                       ctrl = { ST_DETECT    , 2'b00, 5'b00000 };
    */
    ST_RESET:
      if (cnt == RESET_CNT)      ctrl = { ST_ENABLED   , 2'b00, 5'b00001 };
      else                       ctrl = { ST_RESET     , 2'b00, 5'b00000 };
    ST_ENABLED:
      if (sof_d1)                ctrl = { ST_SOF_START , 2'b00, 5'b00101 };
      else                       ctrl = { ST_ENABLED   , 2'b00, 5'b00000 };
    ST_SOF_START:                ctrl = { ST_SOF_WAIT  , 2'b00, 5'b01000 };
    ST_SOF_WAIT:
      if (frame_time > FRAME_TRAFFIC_START)
        if (enum_done)           ctrl = { ST_POLL      , 2'b00, 5'b00000 };
        else if (frame_number < NUM_WAKEUP_FRAMES) 
                                 ctrl = { ST_ENABLED   , 2'b00, 5'b00000 };
        else                     ctrl = { ST_ENUM      , 2'b00, 5'b00000 };
      else                       ctrl = { ST_SOF_WAIT  , 2'b00, 5'b00000 };
    ST_ENUM:
      if (frame_time > FRAME_LENGTH - FRAME_END_QUIET)
                                 ctrl = { ST_ENABLED   , 2'b01, 5'b00000 };
      else                       ctrl = { ST_ENUM      , 2'b01, 5'b00000 };
    ST_POLL:
      if (frame_time > FRAME_LENGTH - FRAME_END_QUIET)
                                 ctrl = { ST_ENABLED   , 2'b00, 5'b00000 };
      else if (frame_time[12:0] == 11'h500) // todo: smarter regular schedule
                                 ctrl = { ST_POLL      , 2'b10, 5'b00010 };
      else                       ctrl = { ST_POLL      , 2'b10, 5'b00000 };
    default:                     ctrl = { ST_IDLE      , 2'b00, 5'b00000 };
  endcase
end

// register the power signal to help timing
wire pwr_next = (state != ST_IDLE) & (state != ST_PWR_OFF);
wire pwr_d1, pwr_d2;
d1 pwr_d1_r(.c(c), .d(pwr_next), .q(pwr_d1));
d1 pwr_d2_r(.c(c), .d(pwr_d1), .q(pwr_d2));
d1 pwr_d3_r(.c(c), .d(pwr_d2), .q(pwr));

assign cnt_rst = ctrl[0];
assign tx_token_start = ctrl[2] | enum_token_start | poll_token_start;
assign token_sel = ctrl[6:5];
assign enum_en = state == ST_ENUM;
assign poll_start = ctrl[1];

endmodule

`ifdef TEST_FSUSB
module tb();

wire c, c_48;
sim_clk #(100) clk_100_inst(c);
sim_clk #( 48) clk_48_inst (c_48);
wire dp, dm, vp, vm, oe_n, pwr;
reg dont_respond;

sim_fsusb_phy sim_phy(.*);

sim_fsusb_encoder sim_enc(.*);

fsusb fsusb_inst
(.c(c), .c_48(c_48), .en(1'b1), .dont_respond(dont_respond),
 .vp(vp), .vm(vm), .oe_n(oe_n), .pwr(pwr));

initial begin
  $dumpfile("fsusb.lxt");
  $dumpvars();
  dont_respond = 1'b1;
  #2_000_000;
  //#10_400_000;
  $finish();
end

endmodule
`endif

