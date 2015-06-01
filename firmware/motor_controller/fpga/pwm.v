`timescale 1ns/1ns
module pwm
#(parameter MIN_DEAD=16'd63) // intended to be overridden at instantiation
(input c,   // clock input
 input [15:0] w_top,
 input [47:0] duty,
 input [15:0] dead,     // intended to be a register
 output [2:0] hi,
 output [2:0] lo);

wire [15:0] w, w_next;

wire w_start = w == 16'h0;
wire [15:0] w_top_i; // w_top internal signal (synchronized to pwm cycle)
r #(16) w_top_r(.c(c), .rst(1'b0), .en(w_start), .d(w_top), .q(w_top_i));

r #(16) w_r(.c(c), .rst(1'b0), .en(1'b1), .d(w_next), .q(w));
wire w_dir, w_dir_en;
r w_dir_r(.c(c), .rst(1'b0), .en(w_dir_en), .d(~w_dir), .q(w_dir));
assign w_next = w_dir == 1'b0 ? w + 1'b1 : w - 1'b1;
assign w_dir_en = w_next == 16'h0 | w_next == w_top_i;

wire [47:0] duty_i;
wire [15:0] a_i = duty_i[15:0];
wire [15:0] b_i = duty_i[31:16];
wire [15:0] c_i = duty_i[47:32];
r #(48) duty_r(.c(c), .rst(1'b0), .en(w_start), .d(duty), .q(duty_i));

wire [15:0] dead_i; 
r #(16) dead_i_r(.c(c), .rst(1'b0), .en(1'b1), .q(dead_i),
                   .d(dead > MIN_DEAD ? dead : MIN_DEAD));

wire ah, al, bh, bl, ch, cl;
r ah_r(.c(c), .rst(1'b0), .en(1'b1), .d((a_i>dead_i) & (a_i-dead_i)>w), .q(ah));
r bh_r(.c(c), .rst(1'b0), .en(1'b1), .d((b_i>dead_i) & (b_i-dead_i)>w), .q(bh));
r ch_r(.c(c), .rst(1'b0), .en(1'b1), .d((c_i>dead_i) & (c_i-dead_i)>w), .q(ch));
r al_r(.c(c), .rst(1'b0), .en(1'b1), .d(a_i<=w), .q(al));
r bl_r(.c(c), .rst(1'b0), .en(1'b1), .d(b_i<=w), .q(bl));
r cl_r(.c(c), .rst(1'b0), .en(1'b1), .d(c_i<=w), .q(cl));

assign hi = { ah & ~al, bh & ~bl, ch & ~cl };
assign lo = { al,       bl,       cl       };

endmodule

`ifdef TEST_PWM
module tb();
wire c;
sim_clk #(125) clk_125(c);
wire [2:0] hi, lo;
pwm #(.MIN_DEAD(16'd1)) pwm_inst
(.c(c), .w_top(16'd16), .duty({ 16'd5, 16'd4, 16'd3 }),
 .dead(16'd1), .hi(hi), .lo(lo));
initial begin
  $dumpfile("pwm.lxt");
  $dumpvars();
  #10000; 
  $finish();
end
endmodule
`endif
