`timescale 1ns/1ns
module foc
(input c,
 input [47:0] i_d,
 input i_dv,
 output [15:0] clark_x,
 output [15:0] clark_y,
 input [10:0] menc_raw,
 input [10:0] stator_offset,
 input [15:0] current_target,
 input signed [31:0] max_integrator_windup,
 input [15:0] p_gain,
 input [15:0] i_gain,
 input [15:0] max_v,
 output signed [15:0] park_x,
 output signed [15:0] park_y,
 output signed [31:0] integrator_d,
 output signed [31:0] integrator_q,
 output signed [15:0] error_d,
 output signed [15:0] error_q,
 output signed [15:0] v_a,
 output signed [15:0] v_b,
 output signed [15:0] v_c,
 output [15:0] pwm_a,
 output [15:0] pwm_b,
 output [15:0] pwm_c,
 input active);

//////////////////////////////////////////////////////////////////////////
// register the inbound parameters coming across the chip, to help timing
wire [31:0] max_integrator_windup_i, neg_max_integrator_windup_i;
d1 #(32) max_integrator_windup_i_r
(.c(c), .d(max_integrator_windup), .q(max_integrator_windup_i));
d1 #(32) neg_max_integrator_windup_i_r
(.c(c), .d(-max_integrator_windup), .q(neg_max_integrator_windup_i));

wire [4:0] pipe_cnt;
r #(5) pipe_cnt_r
(.c(c), .en(|pipe_cnt | i_dv), .rst(1'b0), .d(pipe_cnt + 1'b1), .q(pipe_cnt));

// step 1: scale and bias the current command. TODO: non-ideal bias
wire signed [15:0] i_a, i_b, i_c;
r #(16) i_a_r
(.c(c), .rst(1'b0), .en(i_dv), .d(16'h8000 - {i_d[14:0], 1'b0} /*- 16'h8000*/), .q(i_a));
r #(16) i_b_r
(.c(c), .rst(1'b0), .en(i_dv), .d(16'h8000 - {i_d[30:16], 1'b0} /*- 16'h8000*/), .q(i_b));
r #(16) i_c_r
(.c(c), .rst(1'b0), .en(i_dv), .d(16'h8000 - {i_d[46:32], 1'b0} /*- 16'h8000*/), .q(i_c));

// step 2: calculate the clark transform
wire [15:0] mult_x_a;
gmux #(.DWIDTH(16), .SELWIDTH(2)) mult_x_a_mux
(.sel(pipe_cnt[1:0]), .z(mult_x_a),
 .d({ 16'hx, i_c, i_b, i_a }));

wire [15:0] mult_x_b;
gmux #(.DWIDTH(16), .SELWIDTH(2)) mult_x_b_mux
(.sel(pipe_cnt[1:0]), .z(mult_x_b),
 .d({ 16'hx, 16'hd555, 16'hd555, 16'h5555 }));

wire [31:0] clark_x_prod;
lpm_mult #(.lpm_widtha(16), .lpm_widthb(16), .lpm_widthp(32),
           .lpm_representation("SIGNED"),
           .lpm_pipeline(2)) clark_x_mult
(.clock(c), .clken(1'b1), .aclr(1'b0), .sum(1'b0),
 .dataa(mult_x_a), .datab(mult_x_b), .result(clark_x_prod));

r #(16) clark_x_r
(.c(c), .en(pipe_cnt == 5'h2 || pipe_cnt == 5'h3 || pipe_cnt == 5'h4),
 .rst(pipe_cnt == 5'h1), .d(clark_x + clark_x_prod[30:15]), .q(clark_x));

/////////////////////////////////////////////////

wire [15:0] mult_y_a;
gmux #(.DWIDTH(16), .SELWIDTH(1)) mult_y_a_mux
(.sel(pipe_cnt[0]), .z(mult_y_a),
 .d({ i_c, i_b }));

wire [15:0] mult_y_b;
gmux #(.DWIDTH(16), .SELWIDTH(1)) mult_y_b_mux
(.sel(pipe_cnt[0]), .z(mult_y_b),
 .d({ 16'hb619, 16'h49e7 }));

wire [31:0] clark_y_prod;
lpm_mult #(.lpm_widtha(16), .lpm_widthb(16), .lpm_widthp(32),
           .lpm_representation("SIGNED"),
           .lpm_pipeline(2)) clark_y_mult
(.clock(c), .clken(1'b1), .aclr(1'b0), .sum(1'b0),
 .dataa(mult_y_a), .datab(mult_y_b), .result(clark_y_prod));

r #(16) clark_y_r
(.c(c), .en(pipe_cnt == 5'h2 || pipe_cnt == 5'h3),
 .rst(pipe_cnt == 5'h1), .d(clark_y + clark_y_prod[30:15]), .q(clark_y));

//  ENCODER SINE/COSINE LOOKUP  ///////////////

wire [10:0] elec_angle, elec_angle_90;

d1 #(11) elec_angle_r(.c(c), .d(menc_raw + stator_offset), .q(elec_angle));

d1 #(11) elec_angle_90_r
(.c(c), .d(menc_raw+stator_offset+11'h200), .q(elec_angle_90));

wire [10:0] lookup_z;
gmux #(.DWIDTH(11), .SELWIDTH(1)) lookup_mux
(.d({ elec_angle_90, elec_angle }), .sel(pipe_cnt[0]), .z(lookup_z));

wire signed [15:0] lookup_val;
sine_table_11x16 sine_inst
(.c(c), .angle(lookup_z), .sine(lookup_val));

wire signed [15:0] sin;
r #(16) sin_r
(.c(c), .rst(1'b0), .en(pipe_cnt == 5'h3), .d(lookup_val), .q(sin));
// the nonsense with the double inverter seems to be a simulator bug for 
// reasons i don't understand. without the double inverter, it picks up the
// wrong clock cycle's value. i don't know why.
wire signed [15:0] cos;
r #(16) cos_r
(.c(c), .rst(1'b0), .en(pipe_cnt == 5'h4), .d(~(~lookup_val)), .q(cos));

wire clark_dv = pipe_cnt == 5'h5; // maybe earlier?
wire park_dv;
park park_inst
(.c(c), .in_dv(clark_dv), .in_0(clark_x), .in_1(clark_y),
 .sin(sin), .cos(cos), .out_0(park_x), .out_1(park_y), .out_dv(park_dv));

////////////////////////////////////////////////////////////////////////
// now we have the park transform, so we can calculate the error currents
wire [4:0] pi_cnt;
r #(5) pi_cnt_r
(.c(c), .rst(1'b0), .en(|pi_cnt | park_dv), .d(pi_cnt+1'b1), .q(pi_cnt));

r #(16) error_d_r(.c(c), .rst(1'b0), .en(park_dv), .d(16'h0/*-park_x*/), .q(error_d));

r #(16) error_q_r
(.c(c), .rst(1'b0), .en(park_dv), .d(current_target - park_y), .q(error_q));

wire active_d1;
d1 active_d1_r(.c(c), .d(active), .q(active_d1));
wire int_rst = active & ~active_d1; // wipe out the integrators at start

////////////////////////////////////////////////////////////////////////
// wind up the integrators and limit them
wire signed [31:0] error_d_extended = { {16{error_d[15]}}, error_d };
wire signed [31:0] error_q_extended = { {16{error_q[15]}}, error_q };

wire signed [31:0] int_d, int_q;

wire int_d_over, int_d_under;
d1 int_d_over_r
(.c(c), .d(int_d > max_integrator_windup_i), .q(int_d_over));

d1 int_d_under_r
(.c(c), .d(int_d < neg_max_integrator_windup_i), .q(int_d_under));

wire signed [31:0] int_d_z;
gmux #(.DWIDTH(32), .SELWIDTH(2)) int_d_mux
(.sel(pi_cnt[1:0]-1'b1), .z(int_d_z),
 .d({32'hx,
     int_d_over ? max_integrator_windup_i : 
     (int_d_under ? neg_max_integrator_windup_i : int_d),
     32'hx,
     int_d + error_d_extended } ));

r #(32) integrator_d_r
(.c(c), .rst(int_rst), 
 .en(pi_cnt == 5'd1 | pi_cnt == 5'd3), 
 .d(int_d_z), .q(int_d));

wire signed [31:0] int_q_z;
gmux #(.DWIDTH(32), .SELWIDTH(2)) int_q_mux
(.sel(pi_cnt[1:0]-1'b1), .z(int_q_z),
 .d({32'hx,
     int_q < neg_max_integrator_windup_i ? neg_max_integrator_windup_i : int_q,
     int_q > max_integrator_windup_i ? max_integrator_windup_i : int_q,
     int_q + error_q_extended } ));

r #(32) integrator_q_r
(.c(c), .rst(int_rst), 
 .en(pi_cnt == 5'd1 | pi_cnt == 5'd2 | pi_cnt == 5'd3), 
 .d(int_q_z), .q(int_q));

//////////////////////////////////////////////////////////////////////////
// apply gains
wire signed [31:0] pi_mul_a;
gmux #(.DWIDTH(32), .SELWIDTH(2)) pi_mul_a_mux
(.sel(pi_cnt[1:0]), .z(pi_mul_a),
 .d( { int_q, error_q_extended, 
       int_d, error_d_extended } ));

wire signed [15:0] pi_mul_b;
gmux #(.DWIDTH(16), .SELWIDTH(2)) pi_mul_b_mux
(.sel(pi_cnt[1:0]), .z(pi_mul_b),
 .d( { i_gain, p_gain, i_gain, p_gain } ));

wire signed [47:0] pi_prod;
lpm_mult #(.lpm_widtha(32), .lpm_widthb(16), .lpm_widthp(48),
           .lpm_representation("SIGNED"),
           .lpm_pipeline(2)) pi_prod_mult
(.clock(c), .clken(1'b1), .aclr(1'b0), .sum(1'b0),
 .dataa(pi_mul_a), .datab(pi_mul_b), .result(pi_prod));

wire [47:0] pi_prod_d1;
d1 #(48) pi_prod_r(.c(c), .d(pi_prod), .q(pi_prod_d1));

wire signed [47:0] v_d48;
r #(48) v_d48_r
(.c(c), .en(pi_cnt == 5'h7 | pi_cnt == 5'h8), .rst(pi_cnt == 5'h5),
 .d(v_d48 + pi_prod_d1), .q(v_d48));

wire signed [47:0] v_q48;
r #(48) v_q48_r
(.c(c), .en(pi_cnt == 5'h9 | pi_cnt == 5'ha), .rst(pi_cnt == 5'h5),
 .d(v_q48 + pi_prod_d1), .q(v_q48));

wire [15:0] v_d, v_q;
sat sat_d(.c(c), .d(v_d48), .q(v_d));
sat sat_q(.c(c), .d(v_q48), .q(v_q));

wire dq_dv = pi_cnt == 5'hc;

////////////////////////////////////
// inverse park transform
wire signed [15:0] v_alpha, v_beta;
wire v_alpha_beta_dv;
park #(.INVERSE(1)) park_inv_inst
(.c(c), .in_dv(dq_dv), .in_0(v_d), .in_1(v_q), 
 .sin(sin), .cos(cos), .out_0(v_alpha), .out_1(v_beta), 
 .out_dv(v_alpha_beta_dv));

///////////////////////////////////
// inverse clark transform
wire signed [31:0] inv_clark_prod;
lpm_mult #(.lpm_widtha(16), .lpm_widthb(16), .lpm_widthp(32),
           .lpm_representation("SIGNED"), .lpm_pipeline(2)) inv_clark_mult
(.clock(c), .clken(1'b1), .aclr(1'b0), .sum(1'b0),
 .dataa(v_beta), .datab($signed(16'd28399)), .result(inv_clark_prod));

wire [15:0] v_a_unclamped, v_b_unclamped, v_c_unclamped;
r #(16) v_a_r
(.c(c), .en(pi_cnt == 5'h10), .rst(1'b0), 
 .d(v_alpha), .q(v_a_unclamped));

wire signed [15:0] v_alpha_div2 = { {1{v_alpha[15]}}, v_alpha[15:1] };
r #(16) v_b_r
(.c(c), .en(pi_cnt == 5'h12), .rst(1'b0), 
 .d(-v_alpha_div2 + inv_clark_prod[30:15]), .q(v_b_unclamped));

r #(16) v_c_r
(.c(c), .en(pi_cnt == 5'h12), .rst(1'b0), 
 .d(-v_alpha_div2 - inv_clark_prod[30:15]), .q(v_c_unclamped));

/////////////////////////////////////////
// clamp voltages
wire clamp_a_qv, clamp_b_qv, clamp_c_qv;
clamp_signed v_a_clamp(.c(c), .clamp(max_v), .d(v_a_unclamped), .dv(pi_cnt == 5'h13), .q(v_a), .qv(clamp_a_qv));
clamp_signed v_b_clamp(.c(c), .clamp(max_v), .d(v_b_unclamped), .dv(pi_cnt == 5'h13), .q(v_b), .qv(clamp_b_qv));
clamp_signed v_c_clamp(.c(c), .clamp(max_v), .d(v_c_unclamped), .dv(pi_cnt == 5'h13), .q(v_c), .qv(clamp_c_qv));

wire out_v_valid = clamp_a_qv;

wire [15:0] v_a_pos = v_a + 16'h8000;
wire [15:0] v_a_ext = { 4'b0, v_a_pos[15:4] };

wire [15:0] v_b_pos = v_b + 16'h8000;
wire [15:0] v_b_ext = { 4'b0, v_b_pos[15:4] };

wire [15:0] v_c_pos = v_c + 16'h8000;
wire [15:0] v_c_ext = { 4'b0, v_c_pos[15:4] };

r #(16) pwm_a_r
(.c(c), .rst(1'b0), .en(out_v_valid), .d(v_a_ext), .q(pwm_a));

r #(16) pwm_b_r
(.c(c), .rst(1'b0), .en(out_v_valid), .d(v_b_ext), .q(pwm_b));

r #(16) pwm_c_r
(.c(c), .rst(1'b0), .en(out_v_valid), .d(v_c_ext), .q(pwm_c));

endmodule

//////////////////////////////////////////////////////////////////////////

`ifdef TEST_FOC
module tb();
wire c;
sim_clk #(125) clk_125(c);
reg [47:0] i_d;
reg i_dv;
wire [15:0] clark_x, clark_y, park_x, park_y, error_d, error_q;
wire [15:0] v_a, v_b, v_c, pwm_a, pwm_b, pwm_c;
wire [31:0] integrator_d, integrator_q;
reg [10:0] menc_raw, stator_offset;
reg [15:0] current_target;
reg [31:0] max_integrator_windup;
reg [15:0] p_gain;
reg [15:0] i_gain;
reg [15:0] max_v;
reg active;
foc foc_inst(.*);

initial begin
  $dumpfile("foc.lxt");
  $dumpvars();
  i_dv = 1'b0;
  //i_d = { 16'h3800, 16'h4000, 16'h4800 };
  i_d = { 16'd16360, 16'd16430, 16'd16424 };
  stator_offset = 10'h0;
  menc_raw = 10'h035;
  current_target = 16'h1234;
  max_integrator_windup = 16'h100;
  p_gain = 8000;
  i_gain = 0;
  active = 0;
  max_v = 16'h7fff;
  #1000;
  wait(~c);
  wait(c);
  active = 1;
  i_dv = 1'b1;
  wait(~c);
  wait(c);
  i_dv = 1'b0;
  #1000;
  $finish();
end
  
endmodule
`endif
