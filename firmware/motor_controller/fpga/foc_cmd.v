`timescale 1ns/1ns
module foc_cmd
(input c,
 input udp_cmd_dv,
 input [31:0]  udp_target_reg,
 input [31:0]  udp_damping_reg,
 input         udp_foc_active_reg,
 input [31:0]  temperature,
 input [31:0]  temperature_limit,
 input         ignore_temperature,
 input         overtemp_rst,
 input  [7:0]  submsg_rxd,
 input         submsg_rxdv,
 input         submsg_rxlast,
 output [31:0] foc_target,
 output [31:0] foc_damping,
 output [31:0] control_id,
 output        foc_active,
 output        float);

wire [7:0]  foc_cmd_rx_control_mode;
wire [31:0] foc_cmd_rx_control_id;
wire [31:0] foc_cmd_rx_target;
wire [31:0] foc_cmd_rx_damping; // = 32'hbf80_0000;
wire foc_cmd_rx_rx;

foc_cmd_rx foc_cmd_rx_inst
(.c(c), 
 .submsg_rxd(submsg_rxd), .submsg_rxdv(submsg_rxdv),
 .submsg_rxlast(submsg_rxlast),
 .control_mode(foc_cmd_rx_control_mode),
 .control_id(control_id),
 .target(foc_cmd_rx_target),
 .damping(foc_cmd_rx_damping),
 .cmd_rx(foc_cmd_rx_rx));

wire last_cmd_from_foc_cmd;
r last_cmd_from_foc_cmd_r
(.c(c), .d(1'b1), .q(last_cmd_from_foc_cmd),
 .rst(udp_cmd_dv), .en(foc_cmd_rx_rx));

wire udp_cmd_dv_d1, udp_cmd_dv_d2;
d1 udp_cmd_dv_d1_r(.c(c), .d(udp_cmd_dv),    .q(udp_cmd_dv_d1));
d1 udp_cmd_dv_d2_r(.c(c), .d(udp_cmd_dv_d1), .q(udp_cmd_dv_d2));

localparam [31:0] HARD_OVERTEMP_LIMIT = 32'h42d2_0000; // 105.0 celsius

wire hard_overtemp;
fp_compare hard_overtemp_compare_r
(.clock(c), .ageb(hard_overtemp),
 .dataa(temperature),
 .datab(HARD_OVERTEMP_LIMIT)); 

wire soft_overtemp_ageb;
wire soft_overtemp = soft_overtemp_ageb & |temperature_limit;
fp_compare soft_overtemp_compare_r
(.clock(c), .ageb(soft_overtemp_ageb),
 .dataa(temperature),
 .datab(temperature_limit));

wire [23:0] overtemp_cnt;
r #(24) overtemp_cnt_r
(.c(c), .rst(ignore_temperature), .en(1'b1),
 .d((hard_overtemp | soft_overtemp) ? overtemp_cnt + 1'b1 :
    (|overtemp_cnt ? overtemp_cnt - 1'b1 : overtemp_cnt)),
 .q(overtemp_cnt));

`ifdef SIM
localparam TARGET_TIMEOUT   = 24'd100_000; // 1 ms
localparam ZERO_TIMEOUT     = 24'd001_000;
localparam OVERTEMP_TIMEOUT = 24'd000_100;
`else
localparam TARGET_TIMEOUT   = 24'd10_000_000; // 100ms timeout
localparam ZERO_TIMEOUT     = 24'd10_000_000; // command zero amps for 100ms
localparam OVERTEMP_TIMEOUT = 24'd01_000_000; // 10ms overtemp timeout
`endif

localparam ST_IDLE       = 4'd0;
localparam ST_RUNNING    = 4'd1;
localparam ST_DRIVE_ZERO = 4'd2;
localparam ST_OVERTEMP   = 4'd3;

localparam SW=4, CW=4;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r
(.c(c), .rst(1'b0), .en(1'b1), .d(next_state), .q(state));

wire state_cnt_rst = ctrl[0];
wire [17:0] state_cnt;
r #(18) state_cnt_r
(.c(c), .rst(state_cnt_rst), .en(1'b1), .d(state_cnt+1'b1), .q(state_cnt));

always @* begin
  case (state)
    ST_IDLE:
      if (udp_cmd_dv_d2 | foc_cmd_rx_rx)   ctrl = { ST_RUNNING   , 4'b0000 };
      else                                 ctrl = { ST_IDLE      , 4'b0000 };
    ST_RUNNING:
      if (overtemp_cnt > OVERTEMP_TIMEOUT) ctrl = { ST_DRIVE_ZERO, 4'b0010 };
      else if (timeout)                    ctrl = { ST_DRIVE_ZERO, 4'b0001 };
      else                                 ctrl = { ST_RUNNING   , 4'b0000 };
    ST_DRIVE_ZERO:
      if ((udp_cmd_dv_d2 | foc_cmd_rx_rx) &
          ~overtemp_latch)                 ctrl = { ST_RUNNING   , 4'b0000 };
      else if (state_cnt == ZERO_TIMEOUT)  
        if (overtemp_latch)                ctrl = { ST_OVERTEMP  , 4'b0000 };
        else                               ctrl = { ST_IDLE      , 4'b0000 };
      else                                 ctrl = { ST_DRIVE_ZERO, 4'b0000 };
    ST_OVERTEMP:
      if (~overtemp_latch)                 ctrl = { ST_IDLE      , 4'b0000 };
      else                                 ctrl = { ST_OVERTEMP  , 4'b0000 };
    default:                               ctrl = { ST_IDLE      , 4'b0000 };
  endcase
end

wire overtemp_latch;
r overtemp_latch_r
(.c(c), .rst(overtemp_rst), .en(ctrl[1]), .d(1'b1), .q(overtemp_latch));


wire [23:0] timeout_cnt;
r #(24) timeout_cnt_r
(.c(c), .rst(udp_cmd_dv | foc_cmd_rx_rx), .en(1'b1), 
 .d(timeout_cnt+1'b1), .q(timeout_cnt));

wire timeout = timeout_cnt == TARGET_TIMEOUT;

wire [31:0] next_foc_target;
d1 #(32) next_foc_target_r
(.c(c), .d(last_cmd_from_foc_cmd ? foc_cmd_rx_target : udp_target_reg),
 .q(next_foc_target));

wire [31:0] next_foc_damping;
d1 #(32) next_foc_damping_r
(.c(c), .d(last_cmd_from_foc_cmd ? foc_cmd_rx_damping : udp_damping_reg),
 .q(next_foc_damping));

r #(32) foc_target_r
(.c(c), .rst(1'b0), .en(1'b1),
 .d(state == ST_RUNNING ? next_foc_target : 32'h0), .q(foc_target));

r #(32) foc_damping_r
(.c(c), .rst(1'b0), .en(1'b1),
 .d(state == ST_RUNNING ? next_foc_damping : 32'h0), .q(foc_damping));
//assign foc_damping = foc_cmd_rx_damping;

assign float = state == ST_IDLE | state == ST_OVERTEMP;

assign foc_active = udp_foc_active_reg & 
                    (state == ST_RUNNING |
                     state == ST_DRIVE_ZERO);

endmodule
