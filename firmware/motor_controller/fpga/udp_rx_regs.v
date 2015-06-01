`timescale 1ns / 1ns
module udp_rx_regs
#(parameter NUM_REGS=4,
  parameter DEFAULTS=0,
  parameter CMD_IDX=0)
(input       c,
 input [7:0] rxd,
 input       rxdv,
 input       rxlast,
 output      cmd_dv, // pulses high when the CMD_IDX reg is written
 output [NUM_REGS*32-1:0] regs);

// by the time the data stream gets here, it is already known-valid.

wire [11:0] cnt;
r #(12) cnt_r(.c(c), .rst(~rxdv), .en(1'b1), .d(cnt + 1'b1), .q(cnt));

wire [7:0] reg_cmd;
r #(8) reg_cmd_r(.c(c), .rst(~rxdv), .en(cnt == 12'd0),
                 .d(rxd), .q(reg_cmd));
wire [7:0] reg_subcmd;
r #(8) reg_subcmr_r(.c(c), .rst(~rxdv), .en(cnt == 12'd1),
                    .d(rxd), .q(reg_subcmd));
wire [7:0] pkt_reg_ofs;
r #(8) pkt_reg_ofs_r(.c(c), .rst(~rxdv), .en(cnt == 12'd2),
                     .d(rxd), .q(pkt_reg_ofs));
// ignore byte 3 of the packet. could use it in the future for 16-bit offsets

wire cmd_rx_i;
r cmd_rx_i_r
(.c(c), .rst(~rxdv), .d(1'b1), .q(cmd_rx_i),
 .en(CMD_IDX == cnt[11:2] + pkt_reg_ofs));

wire [NUM_REGS*32-1:0] int_regs;
genvar i, j;
generate
for (i = 0; i < NUM_REGS; i = i + 1) begin: gen_regs
  wire [31:0] int_reg_q;
  for (j = 0; j < 4; j = j + 1) begin: gen_reg_bytes
    wire en = rxdv & reg_cmd == 8'h1 & reg_subcmd == 8'h0 & cnt >= 4 & 
              (cnt + {2'b0, pkt_reg_ofs, 2'b0} == 4 + 4*i + j); 
    localparam [7:0] DEFAULT = DEFAULTS[i*32+(j+1)*8-1:i*32+j*8];
    r #(8, DEFAULT) byte_r(.c(c), .rst(1'b0), .en(en),
                           .d(rxd), .q(int_reg_q[j*8+7:j*8]));
  end
  assign int_regs[(i+1)*32-1:i*32] = int_reg_q;
end
endgenerate

// sanity-check and update all of them at once. 
// enforce that register cmd and subcmd are correct, and packet length is
// aligned on a 32-bit boundary
wire save_regs = reg_cmd == 8'h1 && reg_subcmd == 8'h0 && 
                 rxdv & rxlast & cnt[1:0] == 2'h3; 
wire save_regs_d1;
d1 save_regs_d1_r(.c(c), .d(save_regs), .q(save_regs_d1));

r #(NUM_REGS*32, DEFAULTS) regs_r
  (.c(c), .rst(1'b0), .en(save_regs_d1), .d(int_regs), .q(regs));

assign cmd_dv = cmd_rx_i & save_regs;

endmodule

