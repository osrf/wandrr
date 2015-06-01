`timescale 1ns/1ns
module sigma_delta
(input c_mod,     // modulation clock 
 input mod_bit,   // data bit from the modulator
 input c,         // the clock for the rest of the chip
 input invert,
 output [15:0] raw_d, // raw ADC word
 output raw_dv,
 output [15:0] d, // filtered ADC word (dumb for now)
 output dv);

wire mod_bit_s; // synchronized modulator bit
sync mod_sync(.in(mod_bit), .clk(c_mod), .out(mod_bit_s));

wire invert_modclk;
sync invert_sync(.in(invert), .clk(c_mod), .out(invert_modclk));

// let's use a decimation ratio of 32 for now. This will give 11.4 ENOB
// and a settling time of 2.4 usec.

localparam W = 16; 
localparam DECIM = 32;

wire [W-1:0] i0, i1, i2; // integrators 
r #(W) int_0(.c(c_mod), .rst(1'b0), .en(1'b1), .d(i0 + mod_bit_s), .q(i0));
r #(W) int_1(.c(c_mod), .rst(1'b0), .en(1'b1), .d(i0 + i1), .q(i1));
r #(W) int_2(.c(c_mod), .rst(1'b0), .en(1'b1), .d(i1 + i2), .q(i2));

wire dec_match;
wire [7:0] dec_cnt;
r #(8) dec_cnt_r
(.c(c_mod), .rst(dec_match), .en(1'b1), .d(dec_cnt + 1'b1), .q(dec_cnt));
assign dec_match = dec_cnt == DECIM-1; //8'd3; //8'd31; // decimate by 32

wire [W-1:0] decim; // the simplest decimator possible
r #(W) decim_r(.c(c_mod), .rst(1'b0), .en(dec_match), .d(i2), .q(decim));

// now the differentiators
wire [W-1:0] d0, d1, d2;

wire [W-1:0] diff_0 = decim - d0;
wire [W-1:0] diff_1 = diff_0 - d1;
wire [W-1:0] diff_2 = diff_1 - d2;

r #(W) d0_r(.c(c_mod), .rst(1'b0), .en(dec_match), .d(decim), .q(d0));
r #(W) d1_r(.c(c_mod), .rst(1'b0), .en(dec_match), .d(diff_0), .q(d1));
r #(W) d2_r(.c(c_mod), .rst(1'b0), .en(dec_match), .d(diff_1), .q(d2));

wire raw_dv_d1;
d1 raw_dv_d1_r(.c(c_mod), .d(dec_match), .q(raw_dv_d1));

// delay things one clock cycle in order to meet timing
wire [15:0] raw_d_modclk; // data word, in the modulation-clock domain
wire raw_dv_modclk;
d1 #(16) raw_d_d1_r(.c(c_mod), 
                .d(invert_modclk ? 16'd32767 - diff_2 : diff_2), 
                .q(raw_d_modclk));
d1 raw_dv_d2_r(.c(c_mod), .d(raw_dv_d1), .q(raw_dv_modclk));

// clock it up to the rest of the logic speed
sync #(16) raw_d_s(.in(raw_d_modclk), .clk(c), .out(raw_d));

wire raw_dv_slow; // this signal will be synchronized to "c" but is high too long
sync #(.W(1), .S(3)) raw_dv_s(.in(raw_dv_modclk), .clk(c), .out(raw_dv_slow));

wire raw_dv_slow_d1;
d1 raw_dv_slow_d1_r(.c(c), .d(raw_dv_slow), .q(raw_dv_slow_d1));
assign raw_dv = raw_dv_slow & ~raw_dv_slow_d1; // high for only one cycle

//////////////////////////////////////
// now, implement a dumb placeholder filter. maybe in the future do something
// more sophisticated, but for now just average x32 to make a 19.531 kHz 
// stream for feeding PWM
wire [4:0] sample_cnt;
r #(5) sample_cnt_r
(.c(c_mod), .en(raw_dv_modclk), .rst(1'b0), 
 .d(sample_cnt + 1'b1), .q(sample_cnt));

wire raw_dv_modclk_d1;
d1 raw_dv_modclk_d1_r(.c(c_mod), .d(raw_dv_modclk), .q(raw_dv_modclk_d1));

wire [21:0] accum_d, accum;
wire accum_en, accum_rst;
r #(22) accum_r
(.c(c_mod), .d(accum_d), .rst(accum_rst), .en(raw_dv_modclk), .q(accum));
assign accum_d = accum + {5'h0, raw_d};
assign accum_rst = sample_cnt == 5'h00 & raw_dv_modclk_d1;

wire [21:0] last_accum;
r #(22) last_accum_r
(.c(c_mod), .rst(1'b0), .en(accum_rst), .d(accum), .q(last_accum));

wire [15:0] filtered_modclk = last_accum[20:5]; // in the c_mod domain
wire filtered_dv_modclk;
d1 filtered_dv_d1_r(.c(c_mod), .d(accum_rst), .q(filtered_dv_modclk));

// clock it up to the rest of the logic speed
sync #(16) d_s(.in(filtered_modclk), .clk(c), .out(d));

wire dv_slow; // this signal will be synchronized to "c" but is high too long
sync #(.W(1), .S(3)) dv_s(.in(filtered_dv_modclk), .clk(c), .out(dv_slow));

wire dv_slow_d1;
d1 dv_slow_d1_r(.c(c), .d(dv_slow), .q(dv_slow_d1));
assign dv = dv_slow & ~dv_slow_d1; // high for only one cycle

endmodule

`ifdef test_sigma_delta
module sigma_delta_tb();
wire c, c_mod;
sim_clk #(20) clk_20(c_mod);
sim_clk #(125) clk_125(c);
wire [2:0] hi, lo;
wire [15:0] sd_d;
reg [1:0] all_bits [3999:0];
wire sd_dv;
reg mod_bit;
reg invert;
sigma_delta sd_inst
(.c_mod(c_mod), .c(c), .invert(invert), .mod_bit(mod_bit), .d(sd_d), .dv(sd_dv));

integer i;
initial begin
  $readmemh("sigma_delta_test_data.txt", all_bits, 0, 3999);
  $dumpfile("sigma_delta.lxt");
  $dumpvars();
  mod_bit = 1'b0;
  invert = 1'b0;
  for (i = 0; i < 4000; i = i + 1) begin
    wait(c_mod);
    wait(~c_mod);
    mod_bit = all_bits[i][0];
  end
  $finish();
end
endmodule
`endif
