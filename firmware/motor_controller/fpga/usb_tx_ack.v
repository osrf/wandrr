`timescale 1ns/1ns
module usb_tx_ack
(input c,
 input start,
 output [7:0] sie_d,
 output sie_dv);

`include "usb_pids.v"

localparam ST_IDLE   = 3'd0;
localparam ST_SYNC   = 3'd1;
localparam ST_PID    = 3'd2;

localparam SW=4, CW=2;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r
(.c(c), .rst(1'b0), .en(1'b1), .d(next_state), .q(state));

wire sie_d_sel;
wire [7:0] sie_mux_z;
gmux #(.DWIDTH(8), .SELWIDTH(1)) sie_d_gmux
(.d({PID_ACK, 8'b10000000}), .sel(sie_d_sel), .z(sie_mux_z));
wire [7:0] sie_d_i = sie_dv_i ? sie_mux_z : 8'h0;

always @* begin
  case (state)
    ST_IDLE:
      if (start)  ctrl = { ST_SYNC  , 2'b01 };
      else        ctrl = { ST_IDLE  , 2'b00 };
    ST_SYNC:      ctrl = { ST_PID   , 2'b11 };
    ST_PID:       ctrl = { ST_IDLE  , 2'b00 };
    default:      ctrl = { ST_IDLE  , 2'b00 };
  endcase
end

wire   sie_dv_i  = ctrl[0];
assign sie_d_sel = ctrl[1];

// help timing a bit
d1 #(8) sie_d1_d_r (.c(c), .d(sie_d_i ), .q(sie_d ));
d1      sie_d1_dv_r(.c(c), .d(sie_dv_i), .q(sie_dv));

endmodule
