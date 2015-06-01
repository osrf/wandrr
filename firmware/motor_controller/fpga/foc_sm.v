`timescale 1ns/1ns
module foc_sm
(input c,
 input [47:0] i_d, // current readings 
 input i_dv,       // current reading data-valid flag
 input active,
 input [15:0] menc_raw,
 input [15:0] stator_offset,
 input [ 7:0] pole_pairs,
 input [31:0] current_target,
 input [31:0] integrator_limit,
 input [31:0] kp,
 input [31:0] ki,
 input [31:0] bus_voltage, // used to scale efforts, etc.
 input [31:0] max_effort,
 input [31:0] resistance,
 input [31:0] damping,
 input [31:0] menc_vel,
 output [31:0] current_a_fp,
 output [31:0] current_b_fp,
 output [31:0] current_c_fp,
 output [31:0] park_d,
 output [31:0] park_q,
 output [31:0] effort_d,
 output [31:0] effort_q,
 output [15:0] pwm_a,
 output [15:0] pwm_b,
 output [15:0] pwm_c,
 output done);

`include "ops.v"

//////////////////////////////////////////////////////////
wire active_d1;
d1 active_d1_r(.c(c), .d(active), .q(active_d1));
wire run_zero_integrators = active & ~active_d1;
wire run_program = (i_dv_i & active) | run_zero_integrators;
wire [6:0] pc_entry_point, pc_entry_point_d1;
assign pc_entry_point = run_zero_integrators ? 7'h7c : 7'h00;
d1 #(7) pc_entry_point_d1_r(.c(c), .d(pc_entry_point), .q(pc_entry_point_d1));

//////////////////////////////////////////////////////////
// sin/cos lookup
//wire [31:0] menc_sin, menc_cos;
//menc_sincos menc_sincos_inst
//(.c(c), .menc(menc_raw), .offset(stator_offset),
// .sin(menc_sin), .cos(menc_cos));

// register the incoming current estimates to help timing
wire [47:0] i_d_i;
d1 #(48) i_d_i_r(.c(c), .d(i_d), .q(i_d_i));
wire i_dv_i;
d1 i_dv_i_r(.c(c), .d(i_dv), .q(i_dv_i));
// register the (offset) motor-encoder
wire [15:0] menc_raw_i;
d1 #(16) menc_raw_i_r(.c(c), .d(menc_raw + stator_offset), .q(menc_raw_i));
// register the pole-pairs
wire [7:0] pole_pairs_i;
d1 #(8) pole_pairs_i_r(.c(c), .d(pole_pairs), .q(pole_pairs_i));

localparam SW=8, CW=24;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r(.c(c), .rst(1'b0), .en(1'b1), .d(next_state), .q(state));

wire [31:0] reg_a, reg_b;

wire mul_start, mul_done;
wire [31:0] mul_q;
mul mul_inst
(.c(c), .a(reg_a), .b(reg_b), .q(mul_q),
 .start(mul_start), .done(mul_done));

wire addsub_start, addsub_done, addsub_add_sub;
wire [31:0] addsub_q;
addsub addsub_inst
(.c(c), .add_sub(addsub_add_sub),
 .a(reg_a), .b(reg_b), .q(addsub_q),
 .start(addsub_start), .done(addsub_done));

wire s2f_start, s2f_done;
wire [31:0] s2f_q;
s2f s2f_inst
(.c(c), .d(reg_a[15:0]), .q(s2f_q), .start(s2f_start), .done(s2f_done));

wire minmax_start, minmax_done, minmax_minmax;
wire [31:0] minmax_q;
minmax minmax_inst
(.c(c), .minmax(minmax_minmax),
 .a(reg_a), .b(reg_b), .q(minmax_q),
 .start(minmax_start), .done(minmax_done));

wire f2s_start, f2s_done;
wire [15:0] f2s_q;
f2s f2s_inst
(.c(c), .d(reg_a), .q(f2s_q), .start(f2s_start), .done(f2s_done));

wire sqrt_start, sqrt_done;
wire [31:0] sqrt_q;
sqrt sqrt_inst
(.c(c), .d(reg_a), .q(sqrt_q), .start(sqrt_start), .done(sqrt_done));

wire div_start, div_done;
wire [31:0] div_q;
div div_inst
(.c(c), .a(reg_a), .b(reg_b), .q(div_q), .start(div_start), .done(div_done));

wire sin_start, sin_done;
wire [31:0] sin_q;
op_sin op_sin_inst
(.c(c), .d(reg_a), .q(sin_q), .start(sin_start), .done(sin_done));

wire usmod_start, usmod_done;
wire [15:0] usmod_q;
op_usmod op_usmod_inst
(.c(c), .a(reg_a[15:0]), .b(reg_b[15:0]), .q(usmod_q), .start(usmod_start), .done(usmod_done));

/////////////////

wire [31:0] drom_q;
wire [4:0] drom_addr;
foc_consts_32x32 drom // data ROM
(.c(c), .addr(drom_addr), .q(drom_q));

wire [6:0] prom_addr;
wire [31:0] prom_q;
foc_sw foc_sw_inst
(.clock(c), .address({1'b0, prom_addr}), .q(prom_q));

localparam ST_IDLE     = 8'h0;
localparam ST_RESET    = 8'h1;
localparam ST_FETCH    = 8'h2;
localparam ST_DECODE   = 8'h3;
localparam ST_READ_A   = 8'h4;
localparam ST_READ_B   = 8'h5;
localparam ST_EX_START = 8'h6;
localparam ST_EX_WAIT  = 8'h7;
localparam ST_WRITE    = 8'h8;
localparam ST_IMAGE_LOAD_WORD  = 8'h9;
localparam ST_IMAGE_SHIFT_WORD_1 = 8'ha;
localparam ST_IMAGE_SHIFT_WORD_2 = 8'hb;
localparam ST_IMAGE_SHIFT_WORD_3 = 8'hc;

wire dram_we;
wire [31:0] dram_d, dram_q;
wire [5:0] dram_waddr, dram_raddr;
wire imaging = ctrl[0];
ram_64x32 dram // data RAM
(.c(c), .we(dram_we), .d(dram_d), .q(dram_q), 
 .waddr(dram_waddr), .raddr(dram_raddr));

wire [6:0] pc;
r #(7) pc_r
(.c(c), .rst(1'b0), .en(state == ST_RESET | state == ST_FETCH), 
 .d(state == ST_RESET ? pc_entry_point_d1 : pc+1'b1), .q(pc));
assign prom_addr = pc;

wire [31:0] instr;
r #(32) instr_r
(.c(c), .rst(1'b0), .en(state == ST_FETCH), .d(prom_q), .q(instr));

wire [7:0] instr_op = instr[31:24];
wire [7:0] instr_a  = instr[23:16];
wire [7:0] instr_b  = instr[15:8];
wire [7:0] instr_q  = instr[7:0];

assign dram_waddr   = instr_q[5:0];
assign dram_raddr   = state == ST_READ_A ? instr_b[5:0] : instr_a[5:0];
assign drom_addr    = state == ST_READ_A ? instr_b[4:0] : instr_a[4:0];

r #(32) reg_a_r
(.c(c), .rst(1'b0), .en(state == ST_READ_A), // read delayed one clock
 .d(instr_a[7] ? drom_q : dram_q), .q(reg_a));

r #(32) reg_b_r
(.c(c), .rst(1'b0), .en(state == ST_READ_B), // read delayed one clock
 .d(instr_b[7] ? drom_q : dram_q), .q(reg_b));

////////////////////////////////////////////////////////////////////
wire op_in_we       = instr_op == OP_IN;
wire op_in_done     = instr_op == OP_IN;
////////////////////////////////////////////////////////////////////
wire op_out_done    = instr_op == OP_OUT;
////////////////////////////////////////////////////////////////////
wire port_write     = instr_op == OP_OUT & state == ST_WRITE;
wire [7:0] port_num = instr_a;
wire [31:0] port_in;

gmux #(.DWIDTH(32), .SELWIDTH(4)) port_mux
(.sel(port_num[3:0]), .z(port_in),
 .d({
     32'h0,
     32'h0,
     menc_vel,            // 13 = 0xd
     resistance,          // 12 = 0xc
     damping,             // 11 = 0xb
     bus_voltage,         // 10 = 0xa
     max_effort,          // 9
     ki,                  // 8
     kp,                  // 7
     integrator_limit,    // 6
     current_target,      // 5
     24'h0, pole_pairs,   // 4
     16'h0, menc_raw_i,   // 3
     16'h0, i_d_i[47:32],
     16'h0, i_d_i[31:16],
     16'h0, i_d_i[15: 0] }));

r #(32) park_d_r
(.c(c), .rst(1'b0), .en(port_write & port_num == 8'h00),
 .d(reg_b), .q(park_d));

r #(32) park_q_r
(.c(c), .rst(1'b0), .en(port_write & port_num == 8'h01),
 .d(reg_b), .q(park_q));

r #(32) effort_d_r
(.c(c), .rst(1'b0), .en(port_write & port_num == 8'h02),
 .d(reg_b), .q(effort_d));

r #(32) effort_q_r
(.c(c), .rst(1'b0), .en(port_write & port_num == 8'h03),
 .d(reg_b), .q(effort_q));

///////////////////////////////////////////////////////////////////////////////
r #(16) pwm_a_r
(.c(c), .rst(1'b0), .en(port_write & port_num == 8'h04), .d(reg_b[15:0]), .q(pwm_a));

r #(16) pwm_b_r
(.c(c), .rst(1'b0), .en(port_write & port_num == 8'h05), .d(reg_b[15:0]), .q(pwm_b));

r #(16) pwm_c_r
(.c(c), .rst(1'b0), .en(port_write & port_num == 8'h06), .d(reg_b[15:0]), .q(pwm_c));

///////////////////////////////////////////////////////////////////////////////

r #(32) current_a_fp_r
(.c(c), .rst(1'b0), .en(port_write & port_num == 8'h07),
 .d(reg_b), .q(current_a_fp));

r #(32) current_b_fp_r
(.c(c), .rst(1'b0), .en(port_write & port_num == 8'h08),
 .d(reg_b), .q(current_b_fp));

r #(32) current_c_fp_r
(.c(c), .rst(1'b0), .en(port_write & port_num == 8'h09),
 .d(reg_b), .q(current_c_fp));

///////////////////////////////////////////////////////////////////////////////

wire done_port;
r done_port_r
(.c(c), .rst(1'b0), .en(port_write & port_num == 8'h0a),
 .d(|reg_b), .q(done_port));

wire done_port_d1;
d1 done_port_d1_r(.c(c), .d(done_port), .q(done_port_d1));
assign done = done_port & ~done_port_d1;

///////////////////////////////////////////////////////////////////////////////

gmux #(.DWIDTH(32), .SELWIDTH(4)) dram_d_mux
(.sel(instr_op[3:0]), .z(dram_d),
 .d({
     16'h0, usmod_q, // OP_USMOD
     32'hdeadbeef,   // unused
     sin_q,          // OP_SIN
     div_q,          // OP_DIV
     sqrt_q,         // OP_SQRT
     {16'h0, f2s_q}, // OP_F2S
     minmax_q,       // OP_MAX
     minmax_q,       // OP_MIN
     32'h0,          // OP_HALT
     mul_q,          // OP_MUL
     s2f_q,          // OP_S2F
     addsub_q,       // OP_SUB
     addsub_q,       // OP_ADD
     32'h0,          // OP_OUT
     port_in,        // OP_IN
     32'h0}));       // OP_NOP

///////////////////////////////////////////////////////////
// execute ADD and SUB
//assign add_start = instr_op == OP_ADD & state == ST_EX_START;
wire op_add_or_sub     = instr_op == OP_ADD | instr_op == OP_SUB;
assign addsub_start    = op_add_or_sub & state == ST_EX_START;
assign addsub_add_sub  = instr_op == OP_ADD;
////////////////////////////////////////////////////////////
// execute S2F
wire   op_s2f      = instr_op == OP_S2F;
assign s2f_start   = op_s2f & state == ST_EX_START;
////////////////////////////////////////////////////////////
// execute F2S
wire   op_f2s      = instr_op == OP_F2S;
assign f2s_start   = op_f2s & state == ST_EX_START;
////////////////////////////////////////////////////////////
// execute MUL
wire   op_mul      = instr_op == OP_MUL;
assign mul_start   = op_mul & state == ST_EX_START;
////////////////////////////////////////////////////////////
// execute MIN and MAX
wire   op_minmax     = instr_op == OP_MIN | instr_op == OP_MAX;
assign minmax_start  = op_minmax & state == ST_EX_START;
assign minmax_minmax = instr_op == OP_MIN;
////////////////////////////////////////////////////////////
// execute SQRT
wire   op_sqrt    = instr_op == OP_SQRT;
assign sqrt_start = op_sqrt & state == ST_EX_START;
////////////////////////////////////////////////////////////
// execute DIV
wire   op_div     = instr_op == OP_DIV;
assign div_start  = op_div & state == ST_EX_START;
////////////////////////////////////////////////////////////
// execute SIN
wire   op_sin     = instr_op == OP_SIN;
assign sin_start  = op_sin & state == ST_EX_START;
////////////////////////////////////////////////////////////
// execute USMOD
wire   op_usmod    = instr_op == OP_USMOD;
assign usmod_start = op_usmod & state == ST_EX_START;

////////////////////////////////////////////////////////////
wire op_nop  = instr_op == OP_NOP;
wire op_halt = instr_op == OP_HALT;
wire ex_done = op_nop | op_halt |
               op_out_done | op_in_done | 
               s2f_done | f2s_done |
               addsub_done | mul_done | minmax_done | sqrt_done | div_done |
               sin_done | 
               usmod_done;

assign dram_we = state == ST_WRITE & (op_in_we  | 
                                      op_add_or_sub | 
                                      op_s2f    | 
                                      op_f2s    |
                                      op_mul    | 
                                      op_minmax |
                                      op_sqrt   |
                                      op_div    |
                                      op_sin    |
                                      op_usmod  );
////////////////////////////////////////////////////////////

always @* begin
  case (state)
    ST_IDLE:
      if (run_program)      ctrl = { ST_RESET   , 24'h0 };
      else                  ctrl = { ST_IDLE    , 24'h0 };
    ST_RESET:               ctrl = { ST_FETCH   , 24'h0 };
    ST_FETCH:               ctrl = { ST_DECODE  , 24'h0 };
    ST_DECODE:              ctrl = { ST_READ_A  , 24'h0 };
    ST_READ_A:              ctrl = { ST_READ_B  , 24'h0 };
    ST_READ_B:              ctrl = { ST_EX_START, 24'h0 };
    ST_EX_START:            ctrl = { ST_EX_WAIT , 24'h0 };
    ST_EX_WAIT:
      if (ex_done)          ctrl = { ST_WRITE   , 24'h0 };
      else                  ctrl = { ST_EX_WAIT , 24'h0 };
    ST_WRITE:
      if (op_halt)          ctrl = { ST_IDLE    , 24'h0 };
      else                  ctrl = { ST_FETCH   , 24'h0 };
    default:                ctrl = { ST_IDLE    , 24'h0 };
  endcase
end

endmodule

//////////////////////////////////////////////////////////////////////////

`ifdef test_foc_sm

module foc_sm_tb();

wire c;
sim_clk #(125) clk_125(c);
reg [47:0] i_d;
reg i_dv, active;
reg [15:0] stator_offset;
reg [15:0] menc_raw;
reg [ 7:0] pole_pairs;
reg [31:0] current_target;
reg [31:0] bus_voltage;
reg [31:0] integrator_limit;
reg [31:0] kp, ki;
reg [31:0] max_effort;
reg [31:0] resistance, damping, menc_vel;
wire [31:0] current_a_fp, current_b_fp, current_c_fp;
wire [31:0] park_d, park_q;
wire [15:0] pwm_a, pwm_b, pwm_c; 
wire [31:0] effort_d, effort_q;
wire done;
wire [7:0] image_d;
wire image_dv;
reg image_start;
foc_sm foc_inst(.*);

initial begin
  $dumpfile("foc_sm.lxt");
  $dumpvars();
  i_dv = 1'b0;
  i_d = { 16'd16000, 16'd15000, 16'd17000 };
  stator_offset = 11'h0;
  menc_raw = 16'd4242;
  current_target = 32'h3f80_0000; // 1.0 in fp
  active = 1'b0;
  integrator_limit = 32'h4120_0000; // 10.0 in fp
  kp = 32'h3e4c_cccd; // 0.2 in fp
  ki = 32'h3dcc_cccd; // 0.1 in fp
  max_effort = 32'h3f80_0000; // 1.0 in fp 32'h4080_0000; // 4.0 in fp
  image_start = 1'b0;
  pole_pairs = 8'd12;
  resistance = 32'h0;
  damping = 32'h0;
  menc_vel = 32'h0;
  #500;
  wait(~c);
  wait(c);
  active <= 1'b1;
  #1000;
  wait(~c);
  wait(c);
  i_dv = 1'b1;
  wait(~c);
  wait(c);
  i_dv = 1'b0;
  #13000;
  wait(~c);
  wait(c);
  image_start = 1'b1;
  wait(~c);
  wait(c);
  image_start = 1'b0;
  #3000;
  $finish();
end
  
endmodule

`endif
