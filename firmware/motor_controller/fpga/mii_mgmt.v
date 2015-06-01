`timescale 1ns/1ns
module mii_mgmt
#(parameter CLK_MHZ=0,
  parameter MDC_KHZ=2500)
(input clk,

 input [0:4]   phyad,
 input [0:4]   addr,
 input [15:0]  wdata,
 output [15:0] rdata,
 input         req,
 input         we,
 output        ack,

 output mdc,
 inout mdio);

// IEEE 802.3-2008 section 22.2.2.11 says mdc must have
// a period > 400ns, so we need to divide the user clock
//`include "clog2.inc"
localparam MDC_DIV_WIDTH = 8; //`clog2(CLK_MHZ*1000/MDC_KHZ);

// generate mdc
wire [MDC_DIV_WIDTH-1:0] div_cnt;
wire [MDC_DIV_WIDTH-1:0] div_cnt_p1 = div_cnt + 1'b1;
r #(MDC_DIV_WIDTH) div_reg
(.c(clk), .d(div_cnt_p1), .rst(1'b0), .en(1'b1), .q(div_cnt));

assign mdc = div_cnt[MDC_DIV_WIDTH-1];

wire mdc_re = ~mdc & &div_cnt[MDC_DIV_WIDTH-2:0]; // high one clk before rising edge
wire mdc_fe = &div_cnt; // high one clk before falling edge

// state machine
localparam SW = 4;
localparam ST_IDLE     = 4'd0;
localparam ST_PREAMBLE = 4'd1;
localparam ST_START    = 4'd2;
localparam ST_OP1      = 4'd3;
localparam ST_OP2      = 4'd4;
localparam ST_PHYAD    = 4'd5;
localparam ST_REGAD    = 4'd6;
localparam ST_RD_TA    = 4'd7;
localparam ST_RD_RECV  = 4'd8;
localparam ST_WR_TA    = 4'd9;
localparam ST_WR_SEND  = 4'd10;

// ctrl[0] = mdio output
// ctrl[1] = mdio output enable
// ctrl[2] = receiving: 1 = use mdc_re, 0 = use mdc_fe
// ctrl[3] = clear count
// ctrl[SW+4-1:4] = next state
reg [SW+4-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+4-1:4];
wire step = ctrl[2] ? mdc_re : mdc_fe;
r #(SW) state_reg
(.c(clk), .d(next_state), .rst(1'b0), .en(step), .q(state));

// counter
wire [3:0] count;
wire [3:0] count_p1 = count + 1'b1;
r #(4) count_reg
(.c(clk), .d(count_p1), .rst(ctrl[3] & step), .en(step), .q(count));

// want to send bit 15 first, so swap wdata
wire [0:15] wdata_swap = wdata;

always @* begin
   case(state)
     ST_IDLE:                  // nxt_state clr_cnt rcving, mdo_oe, mdo
       if (req)       ctrl = {ST_PREAMBLE,  1'b1,    1'b0,   1'b0, 1'b0};
       else           ctrl = {ST_IDLE,      1'b1,    1'b0,   1'b0, 1'b0};
     ST_PREAMBLE:
       if (&count)    ctrl = {ST_START,     1'b1,    1'b0,   1'b1, 1'b1};
       else           ctrl = {ST_PREAMBLE,  1'b0,    1'b0,   1'b1, 1'b1};
     ST_START:
       if (~count[0]) ctrl = {ST_START,     1'b0,    1'b0,   1'b1, 1'b0};
       else           ctrl = {ST_OP1,       1'b1,    1'b0,   1'b1, 1'b1};
     ST_OP1:          ctrl = {ST_OP2,       1'b1,    1'b0,   1'b1, ~we};
     ST_OP2:          ctrl = {ST_PHYAD,     1'b1,    1'b0,   1'b1, we};
     ST_PHYAD:
       if (count[2])  ctrl = {ST_REGAD,     1'b1,    1'b0,   1'b1, phyad[count[2:0]]};
       else           ctrl = {ST_PHYAD,     1'b0,    1'b0,   1'b1, phyad[count[2:0]]};
     ST_REGAD:
       if (count[2])
         if (we)      ctrl = {ST_WR_TA,     1'b1,    1'b0,   1'b1, addr[count[2:0]]};
         else         ctrl = {ST_RD_TA,     1'b1,    1'b0,   1'b1, addr[count[2:0]]};
       else           ctrl = {ST_REGAD,     1'b0,    1'b0,   1'b1, addr[count[2:0]]};
     ST_RD_TA:
       if (~count[0]) ctrl = {ST_RD_TA,     1'b0,    1'b1,   1'b0, 1'b0};
       else           ctrl = {ST_RD_RECV,   1'b1,    1'b1,   1'b0, 1'b0};
     ST_RD_RECV:
       if (&count)    ctrl = {ST_IDLE,      1'b1,    1'b1,   1'b0, 1'b0};
       else           ctrl = {ST_RD_RECV,   1'b0,    1'b1,   1'b0, 1'b0};
     ST_WR_TA:
       if (~count[0]) ctrl = {ST_WR_TA,     1'b0,    1'b0,   1'b1, 1'b1};
       else           ctrl = {ST_WR_SEND,   1'b1,    1'b0,   1'b1, 1'b0};
     ST_WR_SEND:
       if (&count)    ctrl = {ST_IDLE,      1'b1,    1'b0,   1'b1, wdata_swap[count[3:0]]};
       else           ctrl = {ST_WR_SEND,   1'b0,    1'b0,   1'b1, wdata_swap[count[3:0]]};
     default:         ctrl = {ST_IDLE,      1'b1,    1'b0,   1'b0, 1'b0};
   endcase
end

wire [15:0] next_rdata = {rdata[14:0], mdio};
wire latch_mdio = (state == ST_RD_RECV) & mdc_re;
r #(16) rdata_reg
(.c(clk), .d(next_rdata), .rst(1'b0), .en(latch_mdio), .q(rdata));

// only want to ack for one user clk cycle
wire done = (state == ST_IDLE);
wire ackd;
r #(1) ackd_reg
(.c(clk), .d(done), .rst(1'b0), .en(1'b1), .q(ackd));
assign ack = done & ~ackd;

assign mdio = ctrl[1] ? ctrl[0] : 1'bz;

endmodule

