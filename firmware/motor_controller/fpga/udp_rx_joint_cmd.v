`timescale 1ns/1ns
module udp_rx_joint_cmd
(input         c,
 input  [7:0]  rxd,
 input         rxdv,
 output [7:0]  mode,
 output [31:0] tgt_0,
 output [31:0] tgt_1,
 output [31:0] tgt_2,
 output [31:0] control_id,
 output        cmd_valid);

// by the time the data stream gets here, it is already known-valid.
// register it again to help timing
wire [7:0] rxd_i;
d1 #(8) rxd_d1_r(.c(c), .d(rxd), .q(rxd_i));

wire rxdv_i;
d1 rxdv_d1_r(.c(c), .d(rxdv), .q(rxdv_i));

wire rxe;
d1 rxe_r(.c(c), .d(rxdv_i & ~rxdv), .q(rxe));

wire [11:0] cnt;
r #(12) cnt_r(.c(c), .rst(~rxdv_i), .en(1'b1), .d(cnt + 1'b1), .q(cnt));

wire [7:0] reg_cmd;
r #(8) reg_cmd_r(.c(c), .rst(~rxdv_i), .en(cnt == 12'd0),
                 .d(rxd_i), .q(reg_cmd));
wire [7:0] reg_subcmd;
r #(8) reg_subcmd_r(.c(c), .rst(~rxdv_i), .en(cnt == 12'd1),
                    .d(rxd_i), .q(reg_subcmd));

wire rx_valid = (reg_cmd == 8'h4) & (reg_subcmd == 8'h0) & (cnt == 11'd19);

wire [31:0] rx_shift;
r #(32) rx_shift_r
(.c(c), .rst(1'b0), .en(1'b1),
 .d({rx_shift[23:0], rxd}), .q(rx_shift));

wire [7:0] mode;
r #(8) mode_r(.c(c), .rst(1'b0), .en(cnt == 11'd2), .d(rxd), .q(mode));

r #(32) tgt_0_r(.c(c), .rst(1'b0), .en(cnt == 11'd07), .d(rx_shift), .q(tgt_0));
r #(32) tgt_1_r(.c(c), .rst(1'b0), .en(cnt == 11'd11), .d(rx_shift), .q(tgt_1));
r #(32) tgt_2_r(.c(c), .rst(1'b0), .en(cnt == 11'd15), .d(rx_shift), .q(tgt_2));

r #(32) control_id_r(.c(c), .rst(1'b0), .en(cnt == 11'd19), 
                     .d(rx_shift), .q(control_id));

endmodule
