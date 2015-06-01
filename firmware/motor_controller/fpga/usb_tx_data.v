`timescale 1ns/1ns
module usb_tx_data
(input c,
 input [7:0] d,
 input dv,
 output [7:0] sie_d,
 output sie_dv);

// no need for FIFO: just delay it one clock (after SYNC) and compute CRC16
// on the fly

localparam ST_IDLE   = 4'd0;
localparam ST_SYNC   = 4'd1;
localparam ST_DATA   = 4'd2;
localparam ST_CRC_HI = 4'd3;
localparam ST_CRC_LO = 4'd4;
localparam ST_DONE   = 4'd5;

localparam SW=4, CW=5;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r
(.c(c), .rst(1'b0), .en(1'b1), .d(next_state), .q(state));

wire [7:0] d_d1;
d1 #(8) d_d1_r(.c(c), .d(d), .q(d_d1));

//wire [15:0] crc = 16'h1234;

wire [15:0] crc;
wire crc_dv;
usb_crc16 usb_crc16_inst
(.c(c), .d(d_d1), .dv(crc_dv), .rst(state == ST_IDLE), .crc(crc));

wire [1:0] sie_d_sel;
wire [7:0] sie_mux_z;
gmux #(.DWIDTH(8), .SELWIDTH(2)) sie_d_gmux
(.d({crc, d_d1, 8'b10000000}),
 .sel(sie_d_sel),
 .z(sie_mux_z));
wire [7:0] sie_d_i = sie_dv_i ? sie_mux_z : 8'h0;

always @* begin
  case (state)
    ST_IDLE:
      if (dv)  ctrl = { ST_SYNC  , 5'b00_00_1 };
      else     ctrl = { ST_IDLE  , 5'b00_00_0 };
    ST_SYNC:   ctrl = { ST_DATA  , 5'b00_01_1 };
    ST_DATA:
      if (~dv) ctrl = { ST_CRC_LO, 5'b01_01_1 };
      else     ctrl = { ST_DATA  , 5'b01_01_1 };
    ST_CRC_LO: ctrl = { ST_CRC_HI, 5'b00_10_1 };
    ST_CRC_HI: ctrl = { ST_DONE  , 5'b00_11_1 };
    ST_DONE:   ctrl = { ST_IDLE  , 5'b00_11_0 };
    default:   ctrl = { ST_IDLE  , 5'b00_00_0 };
  endcase
end

wire   sie_dv_i  = ctrl[0];
assign sie_d_sel = ctrl[2:1];
assign crc_dv    = ctrl[3];

// help timing a bit
d1 #(8) sie_d1_d_r (.c(c), .d(sie_d_i ), .q(sie_d ));
d1      sie_d1_dv_r(.c(c), .d(sie_dv_i), .q(sie_dv));

endmodule
