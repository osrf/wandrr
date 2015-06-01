`timescale 1ns / 1ns
module udp_rx
(input c,
 input [7:0] eth_d,
 input eth_dv,
 input eth_stop,
 output [7:0] udp_d,
 output udp_dv,
 output udp_last, // only signaled for valid UDP packets
 output [15:0] udp_port
);

localparam SW = 3, CW = 1;
localparam ST_IDLE        = 3'd0;
localparam ST_ETH_HEADER  = 3'd1;
localparam ST_IP_HEADER   = 3'd2;
localparam ST_UDP_HEADER  = 3'd3;
localparam ST_UDP_PAYLOAD = 3'd4;
localparam ST_FCS_CHECK   = 3'd5;

reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r(.c(c), .rst(1'b0), .en(1'b1), .d(next_state), .q(state));

wire [11:0] rx_cnt;
wire rx_cnt_rst;
r #(12) rx_cnt_r(.c(c), .rst(rx_cnt_rst), .en(eth_dv),
                 .d(rx_cnt + 1'b1), .q(rx_cnt));

wire ignore_pkt, ignore_pkt_en;
r ignore_pkt_r(.c(c), .rst(state == ST_IDLE), .en(ignore_pkt_en),
               .d(1'b1), .q(ignore_pkt));

wire eth_stop_d3, eth_stop_d2, eth_stop_d1;
d1 eth_stop_d1_r(.c(c), .d(eth_stop   ), .q(eth_stop_d1));
d1 eth_stop_d2_r(.c(c), .d(eth_stop_d1), .q(eth_stop_d2));
d1 eth_stop_d3_r(.c(c), .d(eth_stop_d2), .q(eth_stop_d3));

always @* begin
  case (state)
    ST_IDLE:
      if (eth_dv)                  ctrl = { ST_ETH_HEADER , 1'b0};
      else                         ctrl = { ST_IDLE       , 1'b1};
    ST_ETH_HEADER:
      if (eth_stop)                ctrl = { ST_IDLE       , 1'b0};
      else if (rx_cnt == 12'd13 & eth_dv)   ctrl = { ST_IP_HEADER  , 1'b1};
      else                         ctrl = { ST_ETH_HEADER , 1'b0};
    ST_IP_HEADER:
      if (eth_stop)                ctrl = { ST_IDLE       , 1'b0};
      else if (rx_cnt == 12'd19 & eth_dv)   ctrl = { ST_UDP_HEADER , 1'b1};
      else                         ctrl = { ST_IP_HEADER  , 1'b0};
    ST_UDP_HEADER:
      if (eth_stop)                ctrl = { ST_IDLE       , 1'b0};
      else if (rx_cnt == 12'd7 & eth_dv)    ctrl = { ST_UDP_PAYLOAD, 1'b1};
      else                         ctrl = { ST_UDP_HEADER , 1'b0};
    ST_UDP_PAYLOAD:
      if (eth_stop_d3)             ctrl = { ST_FCS_CHECK  , 1'b0};
      else                         ctrl = { ST_UDP_PAYLOAD, 1'b0};
    ST_FCS_CHECK:                  ctrl = { ST_IDLE       , 1'b1};
    default:                       ctrl = { ST_IDLE       , 1'b1};
  endcase
end

assign rx_cnt_rst = ctrl[0];

wire [47:0] rx_shifter;
r #(48) rx_shifter_r(.c(c), .en(eth_dv), .rst(1'b0),
                     .d({rx_shifter[39:0], eth_d}), .q(rx_shifter));

wire [47:0] dmac = rx_shifter;
wire dmac_valid = (state == ST_ETH_HEADER) & (rx_cnt == 12'd6);
wire dmac_ignore =  dmac_valid & 
                   (dmac[47:24] != 24'h01005e) &     // multicast
                   (dmac[47:0] != 48'hffffffffffff); // broadcast

wire [47:0] smac = rx_shifter;
wire smac_valid = (state == ST_ETH_HEADER) & (rx_cnt == 12'd12);

wire [15:0] ethertype = rx_shifter[15:0];
wire ethertype_valid = (state == ST_IP_HEADER) & (rx_cnt == 12'd0);
wire ethertype_ignore = ethertype_valid & (ethertype != 16'h0800); // IPv4

wire [3:0] ip_ver = rx_shifter[7:4];
wire ip_ver_valid = state == ST_IP_HEADER & rx_cnt == 12'd1;
wire [3:0] ihl = rx_shifter[3:0];
wire ihl_ver_valid = state == ST_IP_HEADER & rx_cnt == 12'd1;
wire ip_fmt_ignore = ip_ver_valid & (ip_ver != 4'd4 | ihl != 4'd5);

wire [15:0] ip_len;
wire ip_len_valid = state == ST_IP_HEADER & (rx_cnt == 12'd4);
r #(16) ip_len_r(.c(c), .rst(state == ST_IDLE), .en(ip_len_valid),
                 .d(rx_shifter[15:0]), .q(ip_len));

wire [2:0] ip_flags = rx_shifter[7:5];
wire ip_flags_valid = state == ST_IP_HEADER & rx_cnt == 12'd7;
wire ip_flags_ignore = ip_flags_valid & ip_flags[0]; // more fragment flag

wire [7:0] ip_proto = rx_shifter[7:0];
wire ip_proto_valid = state == ST_IP_HEADER & rx_cnt == 12'ha;
wire ip_proto_ignore = ip_proto_valid & ip_proto != 8'h11;

wire [31:0] source_ip;
r #(32) source_ip_r(.c(c), .rst(state == ST_IDLE),
                    .en(state == ST_IP_HEADER & rx_cnt == 12'h10),
                    .d(rx_shifter[31:0]), .q(source_ip));

wire [31:0] dest_ip;
r #(32) dest_ip_r(.c(c), .rst(state == ST_IDLE),
                  .en(state == ST_UDP_HEADER & rx_cnt == 12'd0),
                  .d(rx_shifter[31:0]), .q(dest_ip));

wire [19:0] local_ip_header_sum;
r #(20) local_ip_header_sum_r
  (.c(c), .rst(state == ST_IDLE),
   .en(state == ST_IP_HEADER & rx_cnt[0] & eth_dv),
   .d(local_ip_header_sum + {4'h0, rx_shifter[7:0], eth_d}), 
   .q(local_ip_header_sum));
wire [15:0] local_ip_header_csum = local_ip_header_sum[15:0] + 
                                   {12'h0, local_ip_header_sum[19:16]};
wire local_ip_header_csum_valid = state == ST_UDP_HEADER;
wire ip_header_bad_csum = local_ip_header_csum_valid & ~&local_ip_header_csum;

wire [15:0] source_port, dest_port, udp_len;
r #(16) source_port_r(.c(c), .rst(state == ST_IDLE), 
                      .en(state == ST_UDP_HEADER & rx_cnt == 12'd2),
                      .d(rx_shifter[15:0]), .q(source_port));
r #(16) dest_port_r(.c(c), .rst(state == ST_IDLE), 
                    .en(state == ST_UDP_HEADER & rx_cnt == 12'd4),
                    .d(rx_shifter[15:0]), .q(dest_port));
r #(16) udp_len_r(.c(c), .rst(state == ST_IDLE), 
                  .en(state == ST_UDP_HEADER & rx_cnt == 12'd6),
                  .d(rx_shifter[15:0]), .q(udp_len));
wire udp_len_ignore = state == ST_UDP_PAYLOAD & udp_len > 16'd1500;

wire [31:0] fcs;
eth_crc32 crc32_calc
  (.c(c), .r(state == ST_IDLE), .dv(eth_dv), .d(eth_d), .crc(fcs));

wire eth_dv_d1;
d1 eth_dv_d1_r(.c(c), .d(eth_dv), .q(eth_dv_d1));

wire [159:0] fcs_shift;
r #(160) fcs_shift_r
  (.c(c), .rst(1'b0), .en(eth_dv_d1),
   .d({fcs_shift[127:0], fcs}), .q(fcs_shift));
wire [31:0] current_fcs = fcs_shift[159:128];

assign ignore_pkt_en = dmac_ignore | ethertype_ignore | ip_fmt_ignore |
                       ip_flags_ignore | ip_proto_ignore | ip_header_bad_csum |
                       udp_len_ignore;

wire [31:0] rx_fcs = rx_shifter[31:0];
wire fcs_match = eth_stop_d3 & (current_fcs == rx_fcs);

// register this, since it can't seem to meet timing otherwise...
wire [15:0] expected_udp_payload_len;
r #(16) expected_udp_payload_len_r
 (.c(c), .rst(state == ST_IDLE), .en(1'b1),
  .d(udp_len - 16'd8), .q(expected_udp_payload_len));

// deal with short packets. linux seems to pad 4 fewer bytes than windows ?
wire normal_len_match = rx_cnt == expected_udp_payload_len + 16'h4; // FCS = 4
wire short_len_match_unix = expected_udp_payload_len < 16'h12 & 
                            rx_cnt == 16'h16;
wire short_len_match_windows = expected_udp_payload_len < 16'h16 & 
                               rx_cnt == 16'h1a;
wire len_match = eth_stop_d3 & (normal_len_match       |
                                short_len_match_unix   |
                                short_len_match_windows);

wire udp_payload_end   = ~ignore_pkt & eth_stop_d3;
wire udp_payload_valid = udp_payload_end & fcs_match & len_match;

// register this; it can't meet timing otherwise...
wire udp_in_padding;
r udp_in_padding_r
 (.c(c), .rst(state == ST_IDLE), .en(eth_dv),
  .d(rx_cnt >= expected_udp_payload_len + 3'd4), .q(udp_in_padding));

assign udp_port = dest_port;
assign udp_d = rx_shifter[39:32];
assign udp_dv = ~ignore_pkt & 
                (state == ST_UDP_PAYLOAD & rx_cnt > 4) & 
                ~udp_in_padding & (eth_dv | eth_stop_d3);
assign udp_last = udp_payload_valid;

endmodule

