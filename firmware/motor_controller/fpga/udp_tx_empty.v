`timescale 1ns/1ns
module udp_tx_empty
(input        clk_100,
 input        start,
 input [10:0] len,
 input [15:0] dst_port,
 input [31:0] dst_ip,
 input [31:0] src_ip,
 input [47:0] src_mac,
 input [47:0] dst_mac,
 // below here is the 50 mhz (ethernet) clock domain
 input [ 7:0] hop_cnt,
 input        clk_50,
 output [7:0] txd,
 output       txdv,
 output       txe);

// register inputs here to cross to the 50 mhz domain
wire [10:0] len_i;
s #(11) len_i_r(.c(clk_50), .d(len), .q(len_i));

wire [15:0] dst_port_i;
s #(16) dst_port_i_r(.c(clk_50), .d(dst_port), .q(dst_port_i));

wire [31:0] dst_ip_i, src_ip_i;
s #(32) dst_ip_i_r(.c(clk_50), .d(dst_ip), .q(dst_ip_i));
s #(32) src_ip_i_r(.c(clk_50), .d(src_ip), .q(src_ip_i));

wire [47:0] src_mac_i, dst_mac_i;
s #(48) src_mac_i_r(.c(clk_50), .d(src_mac), .q(src_mac_i));
s #(48) dst_mac_i_r(.c(clk_50), .d(dst_mac), .q(dst_mac_i));

// make sure the start flag is long enough for clk_50 to see it
wire [4:0] start_shift;
r #(5) start_shift_r
(.c(clk_100), .rst(1'b0), .en(1'b1), 
 .d({start_shift[3:0], start}), .q(start_shift));

wire start_i;
s start_i_r(.c(clk_50), .d(|start_shift), .q(start_i));

/////////////////////////////////////////////////////////////////////////////
// state machine to craft UDP packets and push out bytes every 4th clock
localparam SW = 4, CW = 6;
localparam ST_IDLE    = 4'd0;
localparam ST_HEADER  = 4'd1;
localparam ST_VER_LO  = 4'd2;
localparam ST_VER_HI  = 4'd3;
localparam ST_HOP_LO  = 4'd4;
localparam ST_HOP_HI  = 4'd5;
localparam ST_PAYLOAD = 4'd6;
localparam ST_FCS     = 4'd7;
localparam ST_TXE     = 4'd8; // send single-clock "end TX" signal
localparam ST_IFG     = 4'd9;

reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r
(.c(clk_50), .rst(1'b0), .en(1'b1), .d(next_state), .q(state));

wire cnt_rst;
wire [13:0] cnt; // dibit count
r #(14) cnt_r
(.c(clk_50), .rst(cnt_rst), .en(1'b1), .d(cnt + 1'b1), .q(cnt));
wire [11:0] byte_cnt = cnt[13:2];

wire [15:0] ipv4_len; // register it to meet timing...
r #(16) ipv4_len_r
(.c(clk_50), .rst(1'b0), .en(1'b1),
 .d(16'd28 + len_i), .q(ipv4_len)); // ipv4 + udp headers = 28 

wire [15:0] udp_len = 16'h8 + len_i;

// header checksum calculation got a bit gross in order to chop it up
// to meet timing... not very pretty but it seems to work now.
wire [19:0] ips_sum;
r #(20) ips_sum_r
(.c(clk_50), .rst(1'b0), .en(1'b1),
 .d({4'h0, dst_ip_i[15:0]} +
    {4'h0, dst_ip_i[31:16]} +
    {4'h0, src_ip_i[15:0]} +
    {4'h0, src_ip_i[31:16]}),
 .q(ips_sum));

wire [19:0] ips_payload_sum;
r #(20) ips_payload_sum_r
(.c(clk_50), .rst(1'b0), .en(1'b1),
 .d(ips_sum + ipv4_len),
 .q(ips_payload_sum));

wire [19:0] ipv4_csum_20;
r #(20) ipv4_csum_20_r
(.c(clk_50), .rst(1'b0), .en(1'b1),
 .d(20'h04500 + 
    ips_payload_sum +
    20'h04000 +
    20'h00111), // ttl and flags(?)
 .q(ipv4_csum_20));

wire [15:0] ipv4_csum;
r #(16) ipv4_csum_r
(.c(clk_50), .rst(1'b0), .en(1'b1),
 .d(~(ipv4_csum_20[15:0] + ipv4_csum_20[19:16])), .q(ipv4_csum));

wire [511:0] header = { 176'h0, // leftovers from power of 2 rounding
                        16'h0, // UDP checksum is optional. let's not.
                        udp_len[7:0], udp_len[15:8],
                        dst_port[7:0], dst_port[15:8],
                        dst_port[7:0], dst_port[15:8],
                        dst_ip_i[7:0],   dst_ip_i[15:8], 
                        dst_ip_i[23:16], dst_ip_i[31:24],
                        src_ip_i[7:0],   src_ip_i[15:8], 
                        src_ip_i[23:16], src_ip_i[31:24],
                        ipv4_csum[7:0], ipv4_csum[15:8],
                        8'h11, 8'h01, 8'h00, 8'h40, 16'h0,
                        ipv4_len[7:0], ipv4_len[15:8],
                        32'h0045_0008, // ipv4
                        src_mac_i[ 7: 0], src_mac_i[15: 8], src_mac_i[23:16],
                        src_mac_i[31:24], src_mac_i[39:32], src_mac_i[47:40],
                        dst_mac_i[ 7: 0], dst_mac_i[15: 8], dst_mac_i[23:16],
                        dst_mac_i[31:24], dst_mac_i[39:32], dst_mac_i[47:40] };

wire [7:0] header_octet;
gmux #(.DWIDTH(8), .SELWIDTH(6)) header_gmux
(.d(header), .sel(cnt[7:2]), .z(header_octet));

wire payload_cnt_rst;
wire [13:0] payload_cnt;
r #(14) payload_cnt_r
(.c(clk_50), .rst(payload_cnt_rst), .en(1'b1),
 .d(payload_cnt+1'b1), .q(payload_cnt));

wire next_byte = cnt[1:0] == 2'h3;

always @* begin
  case (state)
    ST_IDLE:
      if (start_i)                         ctrl = { ST_HEADER , 6'b010001};
      else                                 ctrl = { ST_IDLE   , 6'b000001};
    ST_HEADER:
      if (byte_cnt == 12'd41 & next_byte)  ctrl = { ST_VER_LO , 6'b001011};
      else                                 ctrl = { ST_HEADER , 6'b001000};
    ST_VER_LO:
      if (next_byte)                       ctrl = { ST_VER_HI , 6'b001000};
      else                                 ctrl = { ST_VER_LO , 6'b001000};
    ST_VER_HI:
      if (next_byte)                       ctrl = { ST_HOP_LO , 6'b001000};
      else                                 ctrl = { ST_VER_HI , 6'b001000};
    ST_HOP_LO:
      if (next_byte)                       ctrl = { ST_HOP_HI , 6'b001000};
      else                                 ctrl = { ST_HOP_LO , 6'b001000};
    ST_HOP_HI:
      if (next_byte)                       ctrl = { ST_PAYLOAD, 6'b001011};
      else                                 ctrl = { ST_HOP_HI , 6'b001000};
    ST_PAYLOAD:
      if (payload_cnt == {len_i, 2'b11})   ctrl = { ST_FCS    , 6'b101001};
      else                                 ctrl = { ST_PAYLOAD, 6'b101000};
    ST_FCS:
      if (cnt == 12'd15)                   ctrl = { ST_TXE    , 6'b001001};
      else                                 ctrl = { ST_FCS    , 6'b001000};
    ST_TXE:                                ctrl = { ST_IFG    , 6'b000001};
    ST_IFG:
      if (cnt == 12'd47)                   ctrl = { ST_IDLE   , 6'b000001};
      else                                 ctrl = { ST_IFG    , 6'b000000};
    default:                               ctrl = { ST_IDLE   , 6'b000001};
  endcase
end

assign cnt_rst         = ctrl[0];
assign payload_cnt_rst = ctrl[1];
wire   eth_txdv        = ctrl[3];
wire   fcs_latch       = state == ST_FCS & cnt == 12'd0;
wire   fcs_dv          = (ctrl[5] | (state == ST_HEADER)) & cnt[1:0] == 2'b11;

wire [31:0] fcs_latched, fcs_live;
r #(32) fcs_r(.c(clk_50), .rst(1'b0), .en(fcs_latch), 
              .d(fcs_live), .q(fcs_latched));
wire [7:0] fcs_octet;
gmux #(.DWIDTH(8), .SELWIDTH(2)) fcs_mux
(.d({fcs_latched[ 7: 0],
     fcs_latched[15: 8],
     fcs_latched[23:16], 
     fcs_latched[31:24]}), 
 .sel(cnt[3:2]), .z(fcs_octet));

wire [7:0] eth_txd;
wire [SW-1:0] state_m1 = state - 1'b1;
gmux #(.DWIDTH(8), .SELWIDTH(3)) txd_mux
(.d({8'h0, fcs_octet, 8'hff,
     8'h00, hop_cnt, 8'h43, 8'h21, header_octet}),
 .sel(state_m1[2:0]), .z(eth_txd));

eth_crc32 fcs_inst
(.c(clk_50), .r(state == ST_IDLE),
 .dv(fcs_dv), .d(eth_txd), .crc(fcs_live));

// register the outgoing byte here to help timing 
wire [7:0] eth_txd_d1;
d1 #(8) eth_txd_d1_r(.c(clk_50), .d(eth_txd), .q(eth_txd_d1));

// and register the outgoing tx_en signal for the same reason
wire eth_txdv_d1;
d1 eth_txdv_d1_r(.c(clk_50), .d(eth_txdv), .q(eth_txdv_d1));

assign txd  = eth_txd_d1;
assign txdv = eth_txdv_d1 & cnt[1:0] == 2'b11;
assign txe  = state == ST_TXE;

endmodule

/////////////////////////////////////////////////////////////////////////////

`ifdef test_udp_tx_empty

module udp_tx_empty_tb();

wire clk_50, clk_100;
sim_clk #(50)  clk_50_inst (clk_50);
sim_clk #(100) clk_100_inst(clk_100);

wire [7:0] txd;
wire txdv, txe;

wire [47:0] smac = 48'ha4f3c1_000011;
wire [47:0] dmac = 48'h01005e_00007b;

reg start;

udp_tx_empty dut
(.clk_50(clk_50),
 .clk_100(clk_100),
 .len(11'd64), .start(start),
 .src_mac(smac),
 .dst_mac(dmac),
 .dst_port(16'd11300), 
 .dst_ip(32'h1234_5678), .src_ip(32'habcd_ef01),
 .txd(txd), .txdv(txdv), .txe(txe));

integer i;
initial begin
  $dumpfile("udp_tx_empty.lxt");
  $dumpvars();
  start = 0;
  #1000;
  @(posedge clk_100);
  #1 start = 1;
  @(posedge clk_100);
  #1 start = 0;
  #20000;
  @(posedge clk_100);
  #1 start = 1;
  @(posedge clk_100);
  #1 start = 0;
  #20000;
  $finish();
end

endmodule

`endif
