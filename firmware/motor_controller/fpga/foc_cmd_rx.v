`timescale 1ns/1ns
module foc_cmd_rx
(input c,
 input [7:0] submsg_rxd,
 input submsg_rxdv,
 input submsg_rxlast,
 output [ 7:0] control_mode,
 output [31:0] control_id,
 output [31:0] target,
 output [31:0] damping,
 output cmd_rx);

// register again to help timing
wire [7:0] rxd;
d1 #(8) rxd_r(.c(c), .d(submsg_rxd), .q(rxd));

wire rxdv;
d1 rxdv_r(.c(c), .d(submsg_rxdv), .q(rxdv));

wire rxe;
d1 rxe_r(.c(c), .d(submsg_rxlast), .q(rxe));


wire [11:0] cnt;
r #(12) cnt_r(.c(c), .rst(~rxdv), .en(1'b1), .d(cnt + 1'b1), .q(cnt));

wire [7:0] reg_cmd;
r #(8) reg_cmd_r(.c(c), .rst(~rxdv), .en(cnt == 12'd0),
                 .d(rxd), .q(reg_cmd));
wire [7:0] reg_subcmd;
r #(8) reg_subcmd_r(.c(c), .rst(~rxdv), .en(cnt == 12'd1),
                    .d(rxd), .q(reg_subcmd));

wire rx_valid = (reg_cmd == 8'h4) & (reg_subcmd == 8'h0) & (cnt == 11'd15);

wire [31:0] rx_shift;
r #(32) rx_shift_r
(.c(c), .rst(1'b0), .en(1'b1),
 .d({rxd, rx_shift[31:8]}), .q(rx_shift));

wire [7:0] mode_i;
r #(8) mode_i_r(.c(c), .rst(1'b0), .en(cnt == 11'd2), .d(rxd), .q(mode_i));

wire [31:0] tgt_0_i, tgt_1_i, control_id_i;
r #(32) tgt_0_i_r(.c(c), .rst(1'b0), .en(cnt == 11'd07), 
                  .d(rx_shift), .q(tgt_0_i));
r #(32) tgt_1_i_r(.c(c), .rst(1'b0), .en(cnt == 11'd11), 
                  .d(rx_shift), .q(tgt_1_i));
r #(32) control_id_i_r(.c(c), .rst(1'b0), .en(cnt == 11'd15), 
                       .d(rx_shift), .q(control_id_i));

wire rx_valid_d1;
d1 rx_valid_d1_r(.c(c), .d(rx_valid), .q(rx_valid_d1));

r #(32) target_r
(.c(c), .rst(1'b0), .en(rx_valid_d1), .d(tgt_0_i), .q(target));

r #(8) mode_r
(.c(c), .rst(1'b0), .en(rx_valid_d1), .d(mode_i), .q(control_mode));

r #(32) damping_r
(.c(c), .rst(1'b0), .en(rx_valid_d1), .d(tgt_1_i), .q(damping));

r #(32) control_id_r
(.c(c), .rst(1'b0), .en(rx_valid_d1), .d(control_id_i), .q(control_id));

assign cmd_rx = rx_valid_d1;

endmodule
