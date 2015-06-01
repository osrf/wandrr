`timescale 1ns/1ns
module usb_rx_nrzi
(input c_48,
 input vp,
 input vm,
 input oe,
 output d,
 output dv,
 output eop);

localparam J   = 2'b10;
localparam K   = 2'b01;
localparam SE0 = 2'b00;

// sharpen the inbound edges. synchronize them to c48
wire [1:0] rx;
sync #(2) input_sync(.in({vp, vm}), .clk(c_48), .out(rx));

// first task: when our driver is not enabled, watch for falling VP which
// implies start-of-packet

localparam ST_IDLE = 4'd0;
localparam ST_INIT = 4'd1;
localparam ST_K    = 4'd2;
localparam ST_J    = 4'd3;
localparam ST_SE0  = 4'd4;
localparam ST_EOP  = 4'd5;
localparam ST_DONE = 4'd6;
localparam ST_ERROR = 4'd7;

localparam SW=4, CW=5;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r
(.c(c_48), .rst(oe), .en(1'b1), .d(next_state), .q(state));

wire [1:0] cnt;
wire cnt_rst;
r #(2) cnt_r
(.c(c_48), .en(1'b1), .rst(cnt_rst), .d(cnt+1'b1), .q(cnt));

wire sample = cnt == 2'd1; // todo: shift this as needed to track tx clock?

wire num_ones_en, num_ones_rst;
wire [2:0] num_ones;
r #(3) num_ones_r
(.c(c_48), .en(num_ones_en), .rst(state == ST_IDLE | (num_ones_rst & sample)), 
 .d(num_ones+1'b1), .q(num_ones));

always @* begin
  case (state)
    ST_IDLE:
      if (~oe & rx == K)      ctrl = { ST_INIT , 5'b00100 };
      else                    ctrl = { ST_IDLE , 5'b00000 };
    ST_INIT:
      if (sample)
        if (rx == K)          ctrl = { ST_K    , 5'b00001 };
        else                  ctrl = { ST_ERROR, 5'b00000 };
      else                    ctrl = { ST_INIT , 5'b00000 };
    ST_K:                     
      if (sample) begin
        if (rx == SE0)        ctrl = { ST_SE0  , 5'b00000 };
        else if (rx == K)     ctrl = { ST_K    , 5'b01011 }; // same = 1
        else if (rx == J)     ctrl = { ST_J    , 5'b10001 }; // flip = 0
        else                  ctrl = { ST_ERROR, 5'b00000 };
      end else                ctrl = { ST_K    , 5'b00000 };
    ST_J:
      if (sample) begin
        if (rx == SE0)        ctrl = { ST_SE0  , 5'b00000 };
        else if (rx == J)     ctrl = { ST_J    , 5'b01011 }; // same = 1
        else if (rx == K)     ctrl = { ST_K    , 5'b10001 }; // flip = 0
        else                  ctrl = { ST_ERROR, 5'b00000 };
      end else                ctrl = { ST_J    , 5'b00000 };
    ST_SE0:
      if (sample & rx == J)   ctrl = { ST_EOP  , 5'b00000 };
      else                    ctrl = { ST_SE0  , 5'b00000 };
    ST_EOP:
      if (sample)             ctrl = { ST_DONE , 5'b00000 };
      else                    ctrl = { ST_EOP  , 5'b00000 };
    ST_DONE:                  ctrl = { ST_IDLE , 5'b00000 };
    ST_ERROR:                 ctrl = { ST_ERROR, 5'b00000 };
    default:                  ctrl = { ST_IDLE , 5'b00000 };
  endcase
end

assign cnt_rst = ctrl[2];
assign dv = ctrl[0] & num_ones != 3'd6;
assign d  = ctrl[1];
assign num_ones_en = ctrl[3];
assign num_ones_rst = ctrl[4] | num_ones == 3'd6;
assign eop = state == ST_SE0;

endmodule
