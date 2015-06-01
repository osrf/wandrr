`timescale 1ns/1ns
module usb_tx_token
(input c,
 input [18:0] d,
 input start,
 output [7:0] sie_d,
 output sie_dv);

localparam ST_IDLE     = 3'd0;
localparam ST_CALC_CRC = 3'd1;
localparam ST_SYNC     = 3'd2;
localparam ST_DATA     = 3'd3;

localparam SW=4, CW=5;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r
(.c(c), .rst(1'b0), .en(1'b1), .d(next_state), .q(state));

wire cnt_rst;
wire cnt_en = 1'b1;
wire [7:0] cnt; // counter used by many states
r #(8) cnt_r
(.c(c), .rst(cnt_rst | state == ST_IDLE), .en(cnt_en), 
 .d(cnt + 1'b1), .q(cnt));

// calculate the crc5 serially 
wire [10:0] payload_shifter;
wire payload_shifter_load;
r #(11) payload_shifter_r
(.c(c), .rst(1'b0), .en(1'b1), 
 .d(payload_shifter_load ? d[18:8] : {1'b0, payload_shifter[10:1]}),
 .q(payload_shifter));

wire crc_dv;
wire [4:0] crc5;
usb_crc5_serial crc5_inst
(.c(c), .r(state == ST_IDLE), .d(payload_shifter[0]), .dv(crc_dv), .crc(crc5));

wire [31:0] shift;
wire shift_load;
r #(32) shift_r
(.c(c), .rst(1'b0), .en(1'b1), 
 .d(shift_load ? {~crc5, d, 8'b10000000} : { 8'h0, shift[31:8] }),
 .q(shift));

assign sie_d = shift[7:0];

always @* begin
  case (state)
    ST_IDLE:
      if (start)                   ctrl = { ST_CALC_CRC, 5'b00011 };
      else                         ctrl = { ST_IDLE    , 5'b00000 };
    ST_CALC_CRC:
      if (cnt == 8'd10)            ctrl = { ST_SYNC    , 5'b00000 };
      else                         ctrl = { ST_CALC_CRC, 5'b00000 };
    ST_SYNC:                       ctrl = { ST_DATA    , 5'b00101 };
    ST_DATA: 
      if (cnt == 8'd3)             ctrl = { ST_IDLE    , 5'b01000 };
      else                         ctrl = { ST_DATA    , 5'b01000 };
    default:                       ctrl = { ST_IDLE, 5'b00000 };
  endcase
end

assign payload_shifter_load = ctrl[1];
assign cnt_rst = ctrl[0];
assign crc_dv  = state == ST_CALC_CRC;
assign shift_load = ctrl[2];
assign sie_dv = ctrl[3];

endmodule

