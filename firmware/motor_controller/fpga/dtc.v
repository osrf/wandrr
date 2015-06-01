`timescale 1ns/1ns
module dtc
#(parameter INVERT_CURRENT_DIRECTION = 1'b1,
  parameter MIN_PULSE = 16'd2500,  // this is 10 usec => 25 kHz
  parameter MAX_PULSE = 16'd5000, // this is 40 usec
  parameter MIN_DEAD = 16'd62)    // this is 500 nsec
(input c, // clock input
 input [15:0] i_tgt,   // target current
 input [15:0] i_est,   // estimated current
 input i_est_valid,    // flag indicating that i_est is valid
 input [15:0] max_t,   // maximum state pulse time, in 125 MHz ticks
 input [15:0] min_t,   // minimum state pulse time, in 125 MHz ticks
 input [15:0] dead,    // dead time, in 125 MHz ticks
 output hi,    // high-side MOSFET
 output lo);   // low-side MOSFET

// we have to re-register the input signals, since they are taking
// too long to go cross-chip from the UDP RX registers

wire [15:0] i_tgt_i;
d1 #(16) i_tgt_i_r(.c(c), .d(i_tgt), .q(i_tgt_i));

localparam ST_HI     = 2'd0;
localparam ST_DEAD_0 = 2'd1;
localparam ST_LO     = 2'd2;
localparam ST_DEAD_1 = 2'd3;

localparam SW=2, CW=3;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r(.c(c), .rst(1'b0), .en(1'b1), .d(next_state), .q(state));

wire [15:0] min_i, max_i, dead_i;
d1 #(16) min_i_r (.c(c), .d(min_t>MIN_PULSE ? min_t : MIN_PULSE), .q(min_i));
d1 #(16) max_i_r (.c(c), .d(max_t>MAX_PULSE ? max_t : MAX_PULSE), .q(max_i));
d1 #(16) dead_i_r(.c(c), .d(dead >MIN_DEAD  ? dead  : MIN_DEAD ), .q(dead_i));

wire cnt_rst;
wire [15:0] cnt;
r #(16) cnt_r(.c(c), .rst(cnt_rst), .en(1'b1), .d(cnt+1'b1), .q(cnt));

wire overcurrent  = (i_est_valid & (i_est > i_tgt_i)) & (cnt >= min_i);
wire undercurrent = (i_est_valid & (i_est < i_tgt_i)) & (cnt >= min_i);
wire timeout      = cnt > max_i;

always @* begin
  case (state)
    ST_HI:
      if (overcurrent | timeout)  ctrl = { ST_DEAD_0, 3'b100 };
      else                        ctrl = { ST_HI    , 3'b010 };
    ST_DEAD_0:
      if (cnt > dead_i)           ctrl = { ST_LO    , 3'b101 };
      else                        ctrl = { ST_DEAD_0, 3'b000 };
    ST_LO:
      if (undercurrent | timeout) ctrl = { ST_DEAD_1, 3'b100 };
      else                        ctrl = { ST_LO    , 3'b001 };
    ST_DEAD_1:
      if (cnt > dead_i)           ctrl = { ST_HI    , 3'b110 };
      else                        ctrl = { ST_DEAD_1, 3'b000 };
  endcase
end

assign cnt_rst = ctrl[2];
assign hi = INVERT_CURRENT_DIRECTION ? ctrl[0] : ctrl[1];
assign lo = INVERT_CURRENT_DIRECTION ? ctrl[1] : ctrl[0];

endmodule

`ifdef TEST_DTC
module tb();
wire c;
sim_clk #(125) clk_125(c);
reg [15:0] i_est;
reg i_est_valid;
wire mosfet_hi, mosfet_lo;
dtc #(.MIN_PULSE(50),
      .MAX_PULSE(200),
      .MIN_DEAD(10)) dtc_inst
(.c(c), .i_tgt(16'h4300), .i_est(i_est), .i_est_valid(i_est_valid),
 .max_t(16'd0), .min_t(16'd0), .dead(16'd0),
 .hi(mosfet_hi), .lo(mosfet_lo));

initial begin
  $dumpfile("dtc.lxt");
  $dumpvars();
  #100000; 
  $finish();
end

initial begin
  i_est = 16'h4000;
  forever begin
    #100;
    if (mosfet_hi)
      i_est = i_est - 4;
    else if (mosfet_lo)
      i_est = i_est + 4;
    wait(~c);
    wait(c);
    i_est_valid = 1;
    wait(~c);
    wait(c);
    i_est_valid = 0;
  end
end

endmodule
`endif
