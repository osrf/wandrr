`timescale 1ns/1ns
module park
#(parameter INVERSE=0)
(input c,
 input in_dv,
 input signed [15:0] in_0,
 input signed [15:0] in_1,
 input signed [15:0] sin,
 input signed [15:0] cos,
 output signed [15:0] out_0,
 output signed [15:0] out_1,
 output out_dv);

wire [2:0] pipe_cnt;
r #(3) pipe_cnt_r
(.c(c), .en(|pipe_cnt | in_dv), .rst(1'b0), 
 .d(pipe_cnt + 1'b1), .q(pipe_cnt));

wire signed [15:0] input_z;
gmux #(.DWIDTH(16), .SELWIDTH(1)) input_mux
(.sel(pipe_cnt[0]), .z(input_z), .d({ in_1, in_0 }));

wire [31:0] x_z_mux_d = INVERSE ? { sin, cos } : { -sin, cos };

wire signed [15:0] mult_x_z;
gmux #(.DWIDTH(16), .SELWIDTH(1)) mult_x_z_mux
(.sel(pipe_cnt[0]), .z(mult_x_z), .d(x_z_mux_d)); //{ -sin, cos }));

wire [31:0] y_z_mux_d = INVERSE ? { cos, -sin } : { cos, sin };

wire signed [15:0] mult_y_z;
gmux #(.DWIDTH(16), .SELWIDTH(1)) mult_y_z_mux
(.sel(pipe_cnt[0]), .z(mult_y_z), .d(y_z_mux_d)); //{ cos, sin }));

wire signed [31:0] out_0_prod;
lpm_mult #(.lpm_widtha(16), .lpm_widthb(16), .lpm_widthp(32),
           .lpm_representation("SIGNED"),
           .lpm_pipeline(2)) out_x_mult
(.clock(c), .clken(1'b1), .aclr(1'b0), .sum(1'b0),
 .dataa(input_z), .datab(mult_x_z), .result(out_0_prod));

r #(16) out_0_prod_r
(.c(c), .en(pipe_cnt == 3'h2 || pipe_cnt == 3'h3),
 .rst(pipe_cnt == 3'h1), .d(out_0 + out_0_prod[30:15]), .q(out_0));

wire signed [31:0] out_1_prod;
lpm_mult #(.lpm_widtha(16), .lpm_widthb(16), .lpm_widthp(32),
           .lpm_representation("SIGNED"),
           .lpm_pipeline(2)) out_1_mult
(.clock(c), .clken(1'b1), .aclr(1'b0), .sum(1'b0),
 .dataa(input_z), .datab(mult_y_z), .result(out_1_prod));

r #(16) out_1_prod_r
(.c(c), .en(pipe_cnt == 3'h2 || pipe_cnt == 3'h3),
 .rst(pipe_cnt == 3'h1), .d(out_1 + out_1_prod[30:15]), .q(out_1));

assign out_dv = pipe_cnt == 3'h4;

endmodule
