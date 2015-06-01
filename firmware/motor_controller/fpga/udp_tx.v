`timescale 1ns/1ns
module udp_tx
( input c,
  input [7:0] d,
  input dv,
  input [15:0] dst_port,
  input [31:0] dst_ip,
  input [31:0] src_ip,
  output [7:0] txd,
  output       txdv,
  output       txe);

// register inputs here to help timing and cross to the clk_50 domain
wire [31:0] dst_ip_i, src_ip_i;
sync #(32) dst_ip_i_r(.clk(clk_50), .in(dst_ip), .out(dst_ip_i));
sync #(32) src_ip_i_r(.clk(clk_50), .in(src_ip), .out(src_ip_i));
wire [15:0] dst_port_i;
sync #(16) dst_port_i_r(.clk(clk_50), .in(dst_port), .out(dst_port_i));

// we have to buffer the stream to know how long it is, to build the header
wire dfifo_rdreq, dfifo_empty;
wire [8:0] dfifo_q;
wire [10:0] dfifo_usedw;

wire [7:0] d_d1;
d1 #(8) d_d1_r(.c(c), .d(d), .q(d_d1));

wire dv_d1;
d1 dv_d1_r(.c(c), .d(dv), .q(dv_d1));

wire last = dv_d1 & ~dv;

dcfifo #(.lpm_width(9),
         .lpm_numwords(2048),
         .lpm_widthu(11),
         .lpm_showahead("ON"),
         .use_eab("ON"),
         .intended_device_family("CYCLONE V")) dfifo // data fifo
 (.wrclk(c),  .wrreq(dv_d1), .data({last, d_d1}),
  .rdclk(clk_50), .rdreq(dfifo_rdreq), .q(dfifo_q),
  .rdempty(dfifo_empty),
  .wrusedw(dfifo_usedw),
  .aclr(1'b0));

wire [31:0] sfifo_q;
wire sfifo_rdreq, sfifo_empty;
dcfifo #(.lpm_width(32),
         .lpm_numwords(30),
         .lpm_widthu(5),
         .lpm_showahead("ON"),
         .use_eab("ON"),
         .intended_device_family("CYCLONE V")) sfifo // size and port fifo
 (.wrclk(c), .wrreq(last), 
  .data({dst_port_i, 5'h0, dfifo_usedw + 1'b1}), // TODO: separate tx counter
  .rdclk(clk_50), .rdreq(sfifo_rdreq), .q(sfifo_q),
  .rdempty(sfifo_empty),
  .aclr(1'b0));


/////////////////////////////////////////////////////////////////////////////
// as data comes in, calculate sums of the packets for use in UDP checksum 
// generation as they are read out
/*
wire [31:0] data_sum;
wire [7:0] d_d2;
d1 #(8) d_d2_r(.c(c), .d(d_d1), .q(d_d2));
wire odd_data;
r odd_data_r(.c(c), .rst(~dv_d1), .en(dv_d1), .d(~odd_data), .q(odd_data));

r #(32) data_sum_r
 (.c(c), .rst(sumfifo_rdreq), .en(dv_d1 & odd_data),
  .d(data_sum + {16'h0, d_d2, d_d1}), .q(data_sum));
*/

/*
wire [15:0] sumfifo_q, sumfifo_d;
wire [4:0] sumfifo_usedw;
wire sumfifo_rdreq, sumfifo_empty;
scfifo #(.lpm_width(16),
         .lpm_numwords(30),
         .lpm_widthu(5),
         .lpm_showahead("ON"),
         .use_eab("ON"),
         .intended_device_family("CYCLONE V")) sumfifo // sum fifo
 (.clock(clk),
  .wrreq(last),
  .rdreq(sumfifo_rdreq),
  .data(sumfifo_d),
  .q(sumfifo_q),
  .empty(sumfifo_empty),
  .usedw(sumfifo_usedw));
*/

/////////////////////////////////////////////////////////////////////////////
// now, a state machine to drain the data and size fifos, crafting UDP pkts 
localparam SW = 3, CW = 6;
localparam ST_IDLE    = 3'd0;
localparam ST_HEADER  = 3'd1;
localparam ST_PAYLOAD = 3'd2;
localparam ST_PAD     = 3'd3;
localparam ST_FCS     = 3'd4;
localparam ST_TXE     = 3'd5; // send single-clock "end TX" signal
localparam ST_IFG     = 3'd6;

reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r
  (.c(clk_50), .rst(1'b0), .en(1'b1), .d(next_state), .q(state));

wire cnt_rst;
wire [13:0] cnt; // byte count
r #(14) cnt_r
  (.c(clk_50), .rst(cnt_rst), .en(1'b1), .d(cnt + 1'b1), .q(cnt));

// add registers to help timing...
wire [15:0] payload_len;
d1 #(16) payload_len_r(.c(clk_50), .d(sfifo_q[15:0]), .q(payload_len));
//wire [15:0] payload_len = sfifo_q[15: 0];
wire [15:0] packet_port = sfifo_q[31:16];
wire [9:0] pad_len; 
r #(10) pad_len_r
  (.c(clk_50), .rst(1'b0), .en(1'b1),
   .d(payload_len > 16'd18 ? 10'h0 : 10'd18 - payload_len[7:0]),
   .q(pad_len));

wire [47:0] smac = 48'ha4f3c1_000011;
wire [47:0] dmac = 48'h01005e_00007b;

wire [15:0] ipv4_len; // register it to meet timing...
r #(16) ipv4_len_r
  (.c(clk_50), .rst(1'b0), .en(1'b1),
   .d(16'd28 + payload_len), .q(ipv4_len)); // ipv4 + udp headers = 28 

wire [15:0] udp_len = 16'h8 + payload_len;
//wire [31:0] source_ip = 32'h0a42ab30;  //32'hc0a80102; // TODO

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
      //{4'h0, ipv4_len} + 
      20'h04000 +
      20'h00111 //+ // ttl and flags(?)
      /*ips_sum*/),
   .q(ipv4_csum_20));

wire [15:0] ipv4_csum;
r #(16) ipv4_csum_r
  (.c(clk_50), .rst(1'b0), .en(1'b1),
   .d(~(ipv4_csum_20[15:0] + ipv4_csum_20[19:16])), .q(ipv4_csum));

wire [511:0] header = { 176'h0, // leftovers from power of 2 rounding
                        16'h0, // UDP checksum is optional. let's not.
                        udp_len[7:0], udp_len[15:8],
                        packet_port[7:0], packet_port[15:8],
                        packet_port[7:0], packet_port[15:8],
                        dst_ip_i[7:0],   dst_ip_i[15:8], 
                        dst_ip_i[23:16], dst_ip_i[31:24],
                        src_ip_i[7:0],   src_ip_i[15:8], 
                        src_ip_i[23:16], src_ip_i[31:24],
                        ipv4_csum[7:0], ipv4_csum[15:8],
                        8'h11, 8'h01, 8'h00, 8'h40, 16'h0,
                        ipv4_len[7:0], ipv4_len[15:8],
                        32'h0045_0008, // ipv4
                        smac[ 7: 0], smac[15: 8], smac[23:16],
                        smac[31:24], smac[39:32], smac[47:40],
                        dmac[ 7: 0], dmac[15: 8], dmac[23:16],
                        dmac[31:24], dmac[39:32], dmac[47:40] };

wire [7:0] header_octet;
gmux #(.DWIDTH(8), .SELWIDTH(6)) header_gmux
  (.d(header), .sel(cnt[7:2]), .z(header_octet));

always @* begin
  case (state)
    ST_IDLE:
      if (~sfifo_empty)        ctrl = { ST_HEADER , 6'b010001};
      else                     ctrl = { ST_IDLE   , 6'b000001};
    ST_HEADER:
      if (cnt == 12'd41)       ctrl = { ST_PAYLOAD, 6'b001001};
      else                     ctrl = { ST_HEADER , 6'b001000};
    ST_PAYLOAD:
      if (dfifo_q[8])       
        if (|pad_len)          ctrl = { ST_PAD    , 6'b101011};
        else                   ctrl = { ST_FCS    , 6'b101011};
      else                     ctrl = { ST_PAYLOAD, 6'b101010};
    ST_PAD:
      if (cnt == pad_len-1'b1) ctrl = { ST_FCS    , 6'b101001};
      else                     ctrl = { ST_PAD    , 6'b101000};
    ST_FCS:
      if (cnt == 12'd3)        ctrl = { ST_TXE    , 6'b001101};
      else                     ctrl = { ST_FCS    , 6'b001000};
    ST_TXE:                    ctrl = { ST_IFG    , 6'b000001};
    ST_IFG:
      if (cnt == 12'd11)       ctrl = { ST_IDLE   , 6'b000001};
      else                     ctrl = { ST_IFG    , 6'b000000};
    default:                   ctrl = { ST_IDLE   , 6'b000001};
  endcase
end

assign cnt_rst     = ctrl[0];
assign dfifo_rdreq = ctrl[1];
assign sfifo_rdreq = ctrl[2];
wire   eth_txdv    = ctrl[3];
wire   fcs_latch   = state == ST_FCS & cnt == 12'd0;
wire   fcs_dv      = ctrl[5] | (state == ST_HEADER);

wire [31:0] fcs_latched, fcs_live;
r #(32) fcs_r(.c(clk_50), .rst(1'b0), .en(fcs_latch), 
              .d(fcs_live), .q(fcs_latched));
wire [7:0] fcs_octet;
gmux #(.DWIDTH(8), .SELWIDTH(2)) fcs_mux
  (.d({fcs_latched[ 7: 0],
       fcs_latched[15: 8],
       fcs_latched[23:16], 
       fcs_live   [31:24]}), 
   .sel(cnt[3:2]), .z(fcs_octet));

wire [7:0] eth_txd;
wire [SW-1:0] state_m1 = state - 1'b1;
gmux #(.DWIDTH(8), .SELWIDTH(3)) txd_mux
  (.d({32'h0, fcs_octet, 8'h0, dfifo_q[7:0], header_octet}),
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
assign txdv = eth_txdv_d1;
assign txe  = state == ST_TXE;

endmodule

/////////////////////////////////////////////////////////////////////////////

`ifdef test_udp_tx

module udp_tx_tb();

wire c;
sim_clk #(100) clk_100_inst(c);

wire clk_50;
sim_clk #(50) clk_50_inst(clk_50);

reg udp_txdv;
reg [7:0] udp_txd;

wire [7:0] txd;
wire txdv, txe;

udp_tx dut
(.c(c), .clk_50(clk_50), .d(udp_txd), .dv(udp_txdv),
 .dst_port(16'd12345), 
 .dst_ip(32'h1234_5678), .src_ip(32'habcd_ef01),
 .txd(txd), .txdv(txdv), .txe(txe));

integer i;
initial begin
  $dumpfile("eth_tx.lxt");
  $dumpvars();
  udp_txd = 8'h0;
  udp_txdv = 1'b0;
  #1000;
  wait(~c);
  wait(c);
  #1 udp_txdv = 1'b1;
  udp_txd = 8'hca;
  wait(~c);
  wait(c);
  #1 udp_txd = 8'hfe;
  wait(~c);
  wait(c);
  #1 udp_txd = 8'h42;
  for (i = 0; i < 99; i = i + 1) begin
    wait(~c);
    wait(c);
  end
  #1 udp_txdv = 1'b0;
  #50000;
  $finish();
end

endmodule

`endif
