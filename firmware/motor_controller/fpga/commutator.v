`timescale 1ns/1ns
module commutator
(input c,
 input  [15:0] menc_raw,
 input  [15:0] stator_offset,    
 input  [31:0] amps_float,        // float32 magnitude to modulate
 output [47:0] current_targets);

wire [10:0] elec_angle_a, elec_angle_b, elec_angle_c;

d1 #(11) elec_angle_a_r
(.c(c), .d(menc_raw[10:0] + stator_offset[10:0]           ), .q(elec_angle_a));

d1 #(11) elec_angle_b_r
(.c(c), .d(menc_raw[10:0] + stator_offset[10:0] + 11'd683 ), .q(elec_angle_b));

d1 #(11) elec_angle_c_r
(.c(c), .d(menc_raw[10:0] + stator_offset[10:0] + 11'd1365), .q(elec_angle_c));

// continually look these up and modulate them
wire [1:0] phase_idx;
d1 #(2) phase_idx_r
(.c(c), .d(phase_idx == 2'd2 ? 2'b0 : phase_idx+1'b1), .q(phase_idx));

wire [10:0] elec_angle_z;
gmux #(.DWIDTH(11), .SELWIDTH(2)) angle_mux
(.d({11'h0, elec_angle_c, elec_angle_b, elec_angle_a}),
 .sel(phase_idx), .z(elec_angle_z));

wire [31:0] elec_angle_sine;
sine_table_11bit sine_inst
(.c(c), .angle(elec_angle_z), .sine(elec_angle_sine));

wire [31:0] target;
mult mult_inst
(.clock(c), 
 .dataa(elec_angle_sine), 
 .datab(amps_float),
 .result(target));

wire [31:0] tgt_int32;
wire fpconv_nan;
fpconv fpconv_inst
(.clock(c), .dataa(target),
 .nan(fpconv_nan),
 .result(tgt_int32));

wire [15:0] tgt = tgt_int32[15:0] + 16'h4000;

wire [15:0] a_tgt, b_tgt, c_tgt;
r #(16) a_tgt_r
(.c(c), .rst(1'b0), .en(phase_idx == 2'h0), .d(tgt), .q(a_tgt));
r #(16) b_tgt_r
(.c(c), .rst(1'b0), .en(phase_idx == 2'h1), .d(tgt), .q(b_tgt));
r #(16) c_tgt_r
(.c(c), .rst(1'b0), .en(phase_idx == 2'h2), .d(tgt), .q(c_tgt));

assign current_targets = { c_tgt, b_tgt, a_tgt };


/*
wire [10:0] elec_angle;
r #(11) elec_angle_r
(.c(c), .rst(1'b0), .en(1'b1),
 .d(menc_int32
*/

endmodule

`ifdef TEST_COMMUTATOR

module tb();

reg [15:0] menc_raw;
reg [15:0] stator_offset;
wire [47:0] current_targets;
wire [15:0] a_tgt = current_targets[15:0];
wire [15:0] b_tgt = current_targets[31:16];
wire [15:0] c_tgt = current_targets[47:32];
reg [31:0] amps_float;

wire c;
sim_clk #(125) clk_125(c);

commutator dut(.*);

initial begin
  $dumpfile("commutator.lxt");
  $dumpvars();
  menc_raw = 16'd0;
  stator_offset = 16'h0;
  wait(~c);
  wait(c);
  wait(~c);
  wait(c);
  menc_raw = 16'd300;
  amps_float = 32'h3f800000;
  #200;
  $finish();
end

endmodule

`endif
