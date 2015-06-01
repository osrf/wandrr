`timescale 1ns/1ns
module state_tx
(input c, input [15:0] period_adc, input [15:0] status,
 input [47:0]  current, input current_dv,
 input [31:0]  menc_usecs, input [31:0] menc_angle, input [31:0] menc_vel,
 input [15:0]  menc_raw, input [2:0]  menc_halls, input [31:0] menc_celsius,
 input [31:0]  jenc0_angle, input [31:0] jenc0_vel,
 input [31:0]  jenc1_angle, input [31:0] jenc1_vel,
 input [31:0]  foc_d, input [31:0]  foc_q,
 input [31:0]  effort_d, input [31:0]  effort_q,
 input [31:0]  foc_target, input [31:0] control_id,
 input foot_en, input [31:0]  foot_usecs, input [255:0] foot_pressures,
 output [7:0]  udp_txd, output udp_txdv, output udp_txe);

// re-register things here to help timing
wire [15:0] period_adc_i;
d1 #(16) period_adc_i_r(.c(c), .d(period_adc), .q(period_adc_i));

// adc tick counter, matching against the period given to us
wire [15:0] adc_ticks;
wire period_match = current_dv & 
                    (adc_ticks >= period_adc_i) & 
                    (period_adc_i != 16'hffff);
r #(16) adc_ticks_r
(.c(c), .rst(period_match),
 .en(current_dv), .d(adc_ticks + 1'b1), .q(adc_ticks));

localparam ST_IDLE         = 4'd0;
localparam ST_TX           = 4'd1;
//localparam ST_TX_FPU_START = 4'd2;
//localparam ST_TX_FPU       = 4'd3;

localparam SW=4, CW=3;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r(.c(c), .rst(1'b0), .en(1'b1), .d(next_state), .q(state));

localparam [7:0] MAX_TXLEN = 1 +  // submsg ID
                             8 +  // system time
                             6 +  // raw currents
                             2 +  // status (?)
                             12 + // motor encoder time, angle, vel
                             2 +  // raw motor encoder word
                             1 +  // hall states
                             4 +  // motor temperature in C
                             8 +  // FOC D and Q
                             8 +  // FOC effort D and Q
                             8 +  // joint encoder 0 position and velocity
                             8 +  // joint encoder 1 position and velocity
                             4 +  // FOC control target
                             4 +  // control ID
                             36;  // foot timestamp and pressure sensors

wire [7:0] txlen;
r #(8) txlen_r
(.c(c), .rst(1'b0), .en(current_dv),
 .d(foot_en ? MAX_TXLEN : MAX_TXLEN - 8'd36),
 .q(txlen));

wire [7:0] txcnt;
r #(8) txcnt_r(.c(c), .rst(state == ST_IDLE), .en(state == ST_TX), 
               .d(txcnt + 1'b1), .q(txcnt));

always @* begin
  case (state)
    ST_IDLE:
      if (period_match)      ctrl = { ST_TX          , 3'b001 };
      else                   ctrl = { ST_IDLE        , 3'b000 };
    ST_TX:
      if (txcnt == txlen)    ctrl = { ST_IDLE        , 3'b110 };
      else                   ctrl = { ST_TX          , 3'b000 };
    default:                 ctrl = { ST_IDLE        , 3'b000 };
  endcase
end

assign udp_txdv = ~(state == ST_IDLE); //state != ST_IDLE:
assign udp_txe = ctrl[2];

// start by dividng 100 mhz clock down to 1 mhz
wire [7:0] div_cnt;
wire div_match = div_cnt == 8'd99;
r #(8) div_cnt_r
(.c(c), .rst(div_match), .en(1'b1), .d(div_cnt+1'b1), .q(div_cnt));

wire [63:0] t_us;
r #(64) t_us_r(.c(c), .rst(1'b0), .en(div_match), .d(t_us+1'b1), .q(t_us));

wire t_load = ctrl[0];

localparam MAX_TXBITS = MAX_TXLEN * 8;
wire [MAX_TXBITS-1:0] msg = 
{ foot_pressures, foot_usecs,
  control_id, foc_target,
  jenc1_vel, jenc1_angle,
  jenc0_vel, jenc0_angle,
  effort_q, effort_d, foc_q, foc_d, 
  menc_celsius, {5'b0, menc_halls}, menc_raw, menc_vel, menc_angle, menc_usecs, 
  status, current, t_us, foot_en ? 8'h43 : 8'h42 };

wire [MAX_TXBITS-1:0] shifter;

r #(MAX_TXBITS) shifter_r(.c(c), .rst(1'b0), .en(1'b1), .q(shifter),
                      .d(t_load ? msg : { 8'h0, shifter[MAX_TXBITS-1:8] }));
assign udp_txd = shifter[7:0];

endmodule

