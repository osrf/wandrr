`timescale 1ns/1ns
module usb_sof
(input c,
 input c_12,
 input start,
 output done,
 inout vp,
 inout vm);

wire [4:0] crc5;

wire [31:0] sof_pkt = { crc5,
                        frame_number,
                        8'b10100101,   // SOF PID
                        8'b11010101 }; 

wire [10:0] crc5_data_in = 
{ frame_number[0], frame_number[1], frame_number[2], frame_number[3],
  frame_number[4], frame_number[5], frame_number[6], frame_number[7],
  frame_number[8], frame_number[9], frame_number[10] }; // abomination

wire crc5_en, crc5_rst;
usb_crc5 usb_crc5_inst
(.clk(c), .rst(crc5_rst), 
 .data_in(crc5_data_in), .crc_en(crc5_en), .crc_out(crc5));

localparam ST_IDLE     = 4'd0;
localparam ST_CALC_CRC = 4'd1;
localparam ST_STX      = 4'd2;
localparam ST_TX       = 4'd3;

localparam SW=4, CW=5;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r
(.c(c), .rst(1'b0), .en(1'b1), .d(next_state), .q(state));

wire cnt_rst;
wire cnt_en = 1'b1;
wire [7:0] cnt; // counter used by many states
r #(8) cnt_r(.c(c), .rst(cnt_rst), .en(cnt_en), .d(cnt + 1'b1), .q(cnt));

wire tx_load; // in c12 domain
wire tx_load_flag = state == ST_STX;
sync tx_load_r(.in(tx_load_flag), .clk(c_12), .out(tx_load));

wire [7:0] tx_cnt;
r #(8) tx_cnt_r 
(.c(c_12), .rst(tx_load), .en(tx_cnt < 6'd32), .d(tx_cnt+1'b1), .q(tx_cnt));

wire [31:0] pkt_xclk;
sync #(32) pkt_xclk_r(.in(sof_pkt), .clk(c_12), .out(pkt_xclk));

wire [31:0] shift;
r #(32) shift_r
(.c(c_12), .rst(1'b0), .en(1'b1), 
 .d(tx_load ? pkt_xclk : {1'b0, shift[31:1]}), .q(shift));

assign vp = |tx_cnt & tx_cnt < 6'd32 ?  shift[0] : 1'bz;
assign vm = |tx_cnt & tx_cnt < 6'd32 ? ~shift[0] : 1'bz;

always @* begin
  case (state)
    ST_IDLE:
      if (start)        ctrl = { ST_CALC_CRC, 5'b00000 };
      else              ctrl = { ST_IDLE    , 5'b00000 };
    ST_CALC_CRC:        ctrl = { ST_STX     , 5'b00000 };
    ST_STX:             
      if (cnt == 8'd15) ctrl = { ST_TX      , 5'b00000 };
      else              ctrl = { ST_STX     , 5'b00000 };
    ST_TX:
      if (cnt == 8'd32) ctrl = { ST_IDLE    , 5'b00010 };
      else              ctrl = { ST_TX      , 5'b00000 };
    default:            ctrl = { ST_IDLE    , 5'b00000 };
  endcase
end

assign crc5_rst = state == ST_IDLE;
assign crc5_en = state == ST_CALC_CRC;
assign done = ctrl[1];
wire tx_en = ctrl[0];

endmodule

`ifdef TEST_USB_SOF
module tb();

wire c, c_12;
sim_clk #(125) clk_125(c);
sim_clk #(12) clk_12(c_12);
wire vp, vm;
reg start;
wire done;

usb_sof sof_inst(.*);

initial begin
  start <= 1'b0;
  $dumpfile("sof.lxt");
  $dumpvars();
  wait(~c);
  wait(c);
  wait(~c);
  start <= 1'b1;
  wait(c);
  wait(~c);
  start <= 1'b0;
  wait(c);
  wait(~c);
  #10000;
  wait(c);
  wait(~c);
  start <= 1'b1;
  wait(c);
  wait(~c);
  start <= 1'b0;
  wait(c);
  wait(~c);
  #10000;
  $finish();
end

endmodule
`endif

