`timescale 1ns/1ns
module usb_rx_sie
(input c,
 input c_48,
 input oe,
 input vp,
 input vm,
 output [7:0] d,
 output dv);

wire bit_d, bit_dv;
wire eop;

usb_rx_nrzi rx_nrzi_inst
(.c_48(c_48), .vp(vp), .vm(vm), .oe(oe), .d(bit_d), .dv(bit_dv), .eop(eop));

// todo: state machine to parse out the bytes and figure out when to 
// expect crc16, etc., based on length of the inbound data stream

localparam ST_IDLE      = 4'h0;
localparam ST_SYNC      = 4'h1;
localparam ST_DATA      = 4'h2;
localparam ST_EOP       = 4'h3;
localparam ST_CHECK_CRC = 4'h4;
localparam ST_DRAIN     = 4'h5; // drains the RX fifo out to busclk domain
localparam ST_DONE      = 4'h6;
localparam ST_ERROR     = 4'hf;

localparam SW=4, CW=5;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r
(.c(c_48), .rst(oe), .en(1'b1), .d(next_state), .q(state));

wire [3:0] bit_cnt;
wire bit_cnt_rst;
r #(4) bit_cnt_r
(.c(c_48), .en(bit_dv), .rst(bit_cnt_rst), .d(bit_cnt+1'b1), .q(bit_cnt));

wire rx_byte_en = bit_dv;
wire [7:0] rx_byte;
r #(8) rx_byte_r
(.c(c_48), .en(rx_byte_en), .rst(1'b0), 
 .d({bit_d, rx_byte[7:1]}), .q(rx_byte));

wire [6:0] byte_cnt;
wire byte_cnt_en, byte_cnt_rst;
r #(7) byte_cnt_r
(.c(c_48), .en(byte_cnt_en), .rst(byte_cnt_rst),
 .d(byte_cnt + 1'b1), .q(byte_cnt));

always @* begin
  case (state)
    ST_IDLE:
      if (bit_dv)                ctrl = { ST_SYNC     , 5'b00000 };
      else                       ctrl = { ST_IDLE     , 5'b00001 };
    ST_SYNC:
      if (bit_cnt == 4'd8)
        if (rx_byte == 8'h80 )   ctrl = { ST_DATA     , 5'b00001 };
        else                     ctrl = { ST_ERROR    , 5'b00001 };
      else                       ctrl = { ST_SYNC     , 5'b00000 };
    ST_DATA:
      if (eop)                   ctrl = { ST_EOP      , 5'b00001 };
      else if (bit_cnt == 4'd8)  ctrl = { ST_DATA     , 5'b00011 };
      else                       ctrl = { ST_DATA     , 5'b00000 };
    ST_EOP:
      if (byte_cnt == 7'd0)      ctrl = { ST_DONE     , 5'b00000 }; // bad
      else if (byte_cnt == 7'd1) ctrl = { ST_DRAIN    , 5'b00100 }; // no crc
      else                       ctrl = { ST_CHECK_CRC, 5'b00000 };
    ST_CHECK_CRC: // TODO: actually check CRC
                                 ctrl = { ST_DRAIN    , 5'b00100 };
    ST_DRAIN:                    ctrl = { ST_DONE     , 5'b00000 };
    ST_DONE:                     ctrl = { ST_IDLE     , 5'b00000 };
    ST_ERROR:                    ctrl = { ST_ERROR    , 5'b00000 };
    default:                     ctrl = { ST_IDLE     , 5'b00000 };
  endcase
end

assign bit_cnt_rst  = ctrl[0];
wire   fifo_wrreq   = ctrl[1];
assign byte_cnt_en  = ctrl[1];
assign byte_cnt_rst = state == ST_IDLE;
wire   fifo_drain_start_48 = state == ST_DRAIN;

wire fifo_drain_start;
sync fifo_drain_start_sync_r
(.in(fifo_drain_start_48), .clk(c), .out(fifo_drain_start));

wire fifo_draining, fifo_rdempty;
r fifo_draining_r
(.c(c), .en(fifo_drain_start), .rst(fifo_rdempty),
 .d(1'b1), .q(fifo_draining));

wire pkt_valid;
r pkt_valid_r
(.c(c_48), .en(ctrl[2]), .rst(state == ST_IDLE), .d(1'b1), .q(pkt_valid));

wire pkt_valid_s;
sync pkt_valid_sync_r(.in(pkt_valid), .clk(c), .out(pkt_valid_s));
wire pkt_valid_s_sticky;
r pkt_valid_s_sticky_r
(.c(c), .rst(fifo_rdempty), .en(pkt_valid_s), 
 .d(1'b1), .q(pkt_valid_s_sticky));

wire xclk_fifo_aclr;
sync xclk_fifo_aclr_sync_r
(.in(oe), .clk(c), .out(xclk_fifo_aclr));

wire [7:0] d_i;
dcfifo 
#(.lpm_width(8), .lpm_widthu(7), .lpm_numwords(128), .lpm_showahead("ON"),
  .use_eab("ON"), .intended_device_family("CYCLONE V")) xclk_fifo
(.wrclk(c_48), .data(rx_byte), .wrreq(fifo_wrreq),
 .rdclk(c), .rdreq(fifo_draining & ~fifo_rdempty), 
 .q(d_i), .rdempty(fifo_rdempty),
 .aclr(xclk_fifo_aclr)); //1'b0));

wire dv_i = pkt_valid_s_sticky & fifo_draining & ~fifo_rdempty;

// delay the output stream one clock to help timing
d1 dv_d1_r(.c(c), .d(dv_i), .q(dv));
d1 #(8) d_d1_r(.c(c), .d(d_i), .q(d));

endmodule
