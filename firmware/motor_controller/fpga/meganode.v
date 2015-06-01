`timescale 1 ns / 1 ns
module meganode
( input clk_20, input clk_25, input clk_50, input clk_48, input clk_100,
  output enet_rst, output enet_mdc, inout enet_mdio, output [3:0] enet_leds,
  output [1:0] enet_txen, output [3:0] enet_txd,
  input  [1:0] enet_rxdv, input  [3:0] enet_rxd,
  output [4:0] usb_oe, output [4:0] usb_pwr, 
  inout [4:0] usb_vp, inout [4:0] usb_vm,
  input mcu_io,
  output mcu_spim_cs, output mcu_spim_sclk,
  output mcu_spim_mosi, input mcu_spim_miso,
  input mcu_spis_cs, input mcu_spis_sclk,
  input mcu_spis_mosi, output mcu_spis_miso,
  output [2:0] mosfet_hi, output [2:0] mosfet_lo, output mosfet_en,
  output [2:0] mclk, input [2:0] mdata,
  output led);


//assign enet_mdc = 1'b0;
//assign enet_mdio = 1'b1;

// give a 1-second reset pulse to the PHYs to allow all power rails to settle
`ifdef SIM
localparam ENET_RST_CNT_TOP = 24'h00_00ff;
`else
localparam ENET_RST_CNT_TOP = 24'hff_ffff;
`endif
wire [23:0] enet_rst_cnt;
r #(24, ENET_RST_CNT_TOP) enet_rst_cnt_r
(.c(clk_25), .rst(1'b0), .en(|enet_rst_cnt), 
 .d(enet_rst_cnt - 1'b1), .q(enet_rst_cnt));
assign enet_rst = ~|enet_rst_cnt;


`ifdef SIM
localparam MII_SEQ_START_CNT = 29'h0000_01ff;
`else
localparam MII_SEQ_START_CNT = 29'h0fff_ffff;
`endif
wire [28:0] mii_seq_start_cnt;
r #(29, ENET_RST_CNT_TOP) mii_seq_start_cnt_r
(.c(clk_50), .rst(1'b0), .en(|mii_seq_start_cnt), 
 .d(mii_seq_start_cnt - 1'b1), .q(mii_seq_start_cnt));
wire mii_seq_start = mii_seq_start_cnt == 29'h1;

assign enet_mdc = 1'b0;
assign enet_mdio = 1'bz;

/*
mii_seq mii_seq_inst
(.c(clk_125), .start(mii_seq_start), .mdc(enet_mdc), .mdio(enet_mdio));
*/

// ETH RX ///////////////////////////////////////////////////////////////////
wire [7:0] upe_rxd; // upstream ethernet RXD
wire upe_rxdv, upe_erx; // upstream ethernet RXDV and end-RX
eth_rx upe // upstream ethernet PHY interface
(.c(clk_50),
 .d(upe_rxd), .dv(upe_rxdv), .erx(upe_erx),
 .phy_rxd(enet_rxd[1:0]), .phy_rxdv(enet_rxdv[0]));

eth_outbound_chain_tx outbound_chain
(.clk_50(clk_50),
 .rxd(upe_rxd), .rxdv(upe_rxdv), .rxe(upe_erx),
 .phy_txd(enet_txd[3:2]), .phy_txen(enet_txen[1]));

// UDP RX ///////////////////////////////////////////////////////////////////
wire [7:0] upu_rxd; // upstream UDP RXD
wire upu_rxdv, upu_erx; // upstreadm UDP RXDV and end-RX
wire [15:0] upu_port;
wire [15:0] upu_hop_count;
udp_rx upu_rx
(.c(clk_50),
 .eth_d(upe_rxd), .eth_dv(upe_rxdv), .eth_stop(upe_erx),
 .udp_d(upu_rxd), .udp_dv(upu_rxdv), .udp_last(upu_erx),
 .udp_port(upu_port));

wire [7:0] submsg_rxd;
wire submsg_rxdv, submsg_rxlast;

udp_outbound_chain_rx outbound_chain_rx_inst
(.clk_50(clk_50), .clk_100(clk_100),
 .rxd(upu_rxd), .rxdv(upu_rxdv & upu_port == 16'd11300), .rxlast(upu_erx),
 .hop_count(upu_hop_count),
 .submsg_rxd(submsg_rxd), 
 .submsg_rxdv(submsg_rxdv), 
 .submsg_rxlast(submsg_rxlast));

// UDP REGISTERS ////////////////////////////////////////////////////////////
`ifdef SIM
localparam TX_PERIOD_ADC = 16'd1;
localparam USB_EN_INIT   = 5'h0f;
localparam FOC_INIT_TGT  = 16'h1234;
localparam FOOT_REG_INIT = 32'h0000_0301; // foot present, on port 3
`else
localparam TX_PERIOD_ADC = 16'd4883; // this will be around 4 Hz
localparam USB_EN_INIT   = 5'h0;
localparam FOC_INIT_TGT  = 16'h0;
localparam FOOT_REG_INIT = 32'h0;
`endif

localparam UDP_REGS = 13+12+1; 
wire [UDP_REGS*32-1:0] uregs; // all UDP-accessible registers are 32-bit
wire [31:0] ureg_aux, ureg_master;
wire [31:0] ureg_pwm0, ureg_pwm1, ureg_pwm2;
wire [31:0] ureg_foc0, ureg_foc1, ureg_foc2,  ureg_foc3, 
            ureg_foc4, ureg_foc5, ureg_foc6,  ureg_foc7,
            ureg_foc8, ureg_foc9, ureg_foc10, ureg_foc11;
wire [31:0] ureg_net0, ureg_net1, ureg_net2, ureg_net3,
            ureg_net4, ureg_net5, ureg_net6, ureg_net7;
wire [31:0] ureg_foot;
assign { ureg_foot,
         ureg_foc11,  ureg_foc10, ureg_foc9, ureg_foc8,
         ureg_net7 ,   ureg_net6, ureg_net5, ureg_net4,
         ureg_net3 ,   ureg_net2, ureg_net1, ureg_net0,
         ureg_foc7 ,   ureg_foc6, ureg_foc5, ureg_foc4, 
         ureg_foc3 ,   ureg_foc2, ureg_foc1, ureg_foc0,
         ureg_pwm2 ,   ureg_pwm1, ureg_pwm0, 
         ureg_aux  , ureg_master } = uregs;
localparam [UDP_REGS*32-1:0] uregs_defaults = 
{ FOOT_REG_INIT, // 25 ureg_foot
  32'h0,         // 24 foc 11
  32'h0,         // 23 foc 10 = soft overtemp limit. must be nonzero to work.
  32'h0,         // 22 foc  9 = resistance
  32'h0,         // 21 foc  8 = damping
  32'h0,         // 20 net  7
  32'h0,         // 19 net  6
  32'ha4f3_c100, // 18 net  5: MSBs of src MAC addr
  32'h0011_0100, // 17 net  4: LSBs of src MAC addr, MSBs of dst MAC addr
  32'h5e00_007b, // 16 net  3: LSBs of dst MAC addr
  32'he000_007b, // 15 net  2: destination IP
  32'h0a42_ab30, // 14 net  1: source IP
  32'h2ff_01,    // 13 net  0: payload size = 767, and is end-of-chain
  32'h0,         // 12 foc  7
  32'h42c8_0000, // 11 foc  6 = bus voltage = 100.0 (floating point)
  32'h0008_0000, // 10 foc  5: LSB's = stator offset, MSB's = pole count
  32'h0,         //  9 foc  4
  32'h0,         //  8 foc  3
  32'h0,         //  7 foc  2
  32'h0,         //  6 foc  1
  32'h0,         //  5 foc  0
  32'h0,         //  4 duty for phase C (low 16) and dead time (high 16)
  32'h0,         //  3 duty for phases A and B
  32'h00ff0000,  //  2 pwm duty_top (high 16) and mode/control flags (low 16)
  16'h21_00, 3'h0, USB_EN_INIT, 8'h0, // 1 usb crossbar, usb power
  TX_PERIOD_ADC, 16'h0 };         // 0 master (tx period, MOSFET enable)

wire [15:0] pwm_dead     = ureg_pwm2[31:16];
wire [15:0] pwm_duty_c   = ureg_pwm2[15:0];
wire [15:0] pwm_duty_b   = ureg_pwm1[31:16];
wire [15:0] pwm_duty_a   = ureg_pwm1[15:0];
wire [15:0] pwm_duty_top = ureg_pwm0[31:16];
wire [15:0] pwm_ctrl     = ureg_pwm0[15:0];

assign led = ureg_aux[0];
wire usb_dont_respond = ureg_aux[1];

wire master_en = ureg_master[0];
wire [15:0] master_tx_period_adc = ureg_master[31:16];

wire [4:0] usb_en = ureg_aux[12:8];

wire udp_rx_regs_cmd_dv;

udp_rx_regs #(.NUM_REGS(UDP_REGS),
              .DEFAULTS(uregs_defaults),
              .CMD_IDX(5)) udp_rx_regs_inst
(.c(clk_100), .regs(uregs), .cmd_dv(udp_rx_regs_cmd_dv),
 .rxd(submsg_rxd), .rxdv(submsg_rxdv), .rxlast(submsg_rxlast));

udp_spi_tx #(.SCLK_DIV(25)) udp_spi_tx_inst
(.c(clk_100), 
 .rxd(submsg_rxd), .rxdv(submsg_rxdv), 
 .cs(mcu_spim_cs), .sclk(mcu_spim_sclk), 
 .mosi(mcu_spim_mosi), .miso(mcu_spim_miso));

// catch inbound SPI messages from the microcontroller
wire [7:0] sspi_rxd;
wire sspi_rxdv, sspi_rxe;
spi_slave_rxq spi_slave_rxq_inst
(.c(clk_100),
 .cs(mcu_spis_cs), .sclk(mcu_spis_sclk), .mosi(mcu_spis_mosi),
 .rxd(sspi_rxd), .rxdv(sspi_rxdv), .rxe(sspi_rxe));


assign mcu_spis_miso = 1'b1; // whoops, tied to NJTRST on mcu. needs to be high.

///////////////////////////////////////////////////////////////
// INBOUND NETWORKING

wire [7:0] outbox_txd;
wire outbox_txdv, outbox_txe;
wire chain_endpoint = ureg_net0[0];
wire [10:0] chain_inbound_payload_size = ureg_net0[18:8];

wire [31:0] src_ip = ureg_net1; 
wire [31:0] dst_ip = ureg_net2; 
wire [47:0] dst_mac = {ureg_net4[15:0], ureg_net3};
wire [47:0] src_mac = {ureg_net5, ureg_net4[31:16]};

// todo: someday multiplex this intelligently. for now, just give priority to 
// the out-of-band traffic, since it will (should) only happen when the
// inbound traffic is very light.
wire [7:0] state_outbox_txd;
wire state_outbox_txdv, state_outbox_txe;

wire [7:0] muxed_outbox_txd = sspi_rxdv ? sspi_rxd  : state_outbox_txd;
wire muxed_outbox_txdv      = sspi_rxdv ? sspi_rxdv : state_outbox_txdv;
wire muxed_outbox_txe       = sspi_rxdv ? sspi_rxe  : state_outbox_txe;

wire [7:0] blank_inbound_udp_d;
wire blank_inbound_udp_dv, blank_inbound_udp_e;
wire udp_tx_empty_start = chain_endpoint ? muxed_outbox_txe : sspi_rxe; //1'b0;
udp_tx_empty udp_tx_empty_inst
(.start(udp_tx_empty_start),
 .clk_100(clk_100), .clk_50(clk_50), .len(chain_inbound_payload_size),
 .dst_ip(dst_ip), .src_ip(src_ip), .dst_port(16'd11300), 
 .dst_mac(dst_mac), .src_mac(src_mac),
 .hop_cnt(upu_hop_count[7:0]+1'b1),  // in clk_50 domain
 .txd(blank_inbound_udp_d), 
 .txdv(blank_inbound_udp_dv), 
 .txe(blank_inbound_udp_e));

wire [7:0] dne_rxd; // downstream ethernet RXD
wire dne_rxdv, dne_erx; // downstream ethernet RXDV and end-RX
eth_rx dne // downstream ethernet PHY interface
(.c(clk_50),
 .d(dne_rxd), .dv(dne_rxdv), .erx(dne_erx),
 .phy_rxd(enet_rxd[3:2]), .phy_rxdv(enet_rxdv[1]));

wire chain_endpoint_clk50;
sync chain_endpoint_clk50_r
(.clk(clk_50), .in(chain_endpoint), .out(chain_endpoint_clk50));

wire inbound_stream_sel;
r inbound_stream_sel_r
(.c(clk_50), .rst(1'b0), .en(~dne_rxdv & ~blank_inbound_udp_dv),
 .d(chain_endpoint_clk50), .q(inbound_stream_sel));

udp_inbound_chain inbound_chain
(.clk_50(clk_50), .clk_100(clk_100), 
 .outbox_txd(muxed_outbox_txd), 
 .outbox_txdv(muxed_outbox_txdv), 
 .outbox_txe(muxed_outbox_txe),
 .eth_rxd (inbound_stream_sel ? blank_inbound_udp_d  : dne_rxd ), 
 .eth_rxdv(inbound_stream_sel ? blank_inbound_udp_dv : dne_rxdv), 
 .eth_rxe (inbound_stream_sel ? blank_inbound_udp_e  : dne_erx ),
 .phy_txd(enet_txd[1:0]), .phy_txen(enet_txen[0]));

// CURRENT SENSORS //////////////////////////////////////////////////////////

wire [47:0] current;
wire [ 2:0] current_dv;

assign mclk = {3{clk_20}};
wire sigma_delta_invert;
sigma_delta sigma_delta_inst[2:0]
(.c_mod(mclk), .c(clk_100), .mod_bit(mdata), .invert(sigma_delta_invert),
 .d(current), .dv(current_dv));

`ifdef SIM
localparam SMALLEST_MIN_TIME  = 16'd50;
localparam SMALLEST_MAX_TIME  = 16'd200;
localparam SMALLEST_DEAD_TIME = 16'd10;
`else
localparam SMALLEST_MIN_TIME  = 16'd625;  // 5 usec
localparam SMALLEST_MAX_TIME  = 16'd1250; // 10 usec
localparam SMALLEST_DEAD_TIME = 16'd62;   // 500 nsec
`endif

// VOLTAGE-MODE MOSFET OUTPUT CONTROL ///////////////////////////////////////
`ifdef SIM
localparam PWM_MIN_DEAD = 16'd3;
`else
localparam PWM_MIN_DEAD = 16'd63;
`endif
wire [2:0] pwm_hi, pwm_lo;
wire [15:0] foc_pwm_a, foc_pwm_b, foc_pwm_c;
wire pwm_duty_sel = pwm_ctrl[0];
wire [47:0] pwm_duty_muxed;
gmux #(.DWIDTH(48), .SELWIDTH(1)) pwm_duty_mux
(.sel(pwm_duty_sel), .z(pwm_duty_muxed),
 .d({foc_pwm_c ,  foc_pwm_b, foc_pwm_a,
     pwm_duty_c, pwm_duty_b, pwm_duty_a}));
pwm #(.MIN_DEAD(PWM_MIN_DEAD)) pwm_inst
(.c(clk_100), .w_top(pwm_duty_top),
 .duty(pwm_duty_muxed),
 .dead(pwm_dead),
 .hi(pwm_hi), .lo(pwm_lo));

wire foc_cmd_float; // triggered by timeout or overheating
assign mosfet_en = ~(master_en & ~foc_cmd_float);

// MASTER MOSFET CONTROL ////////////////////////////////////////////////////

assign mosfet_hi = pwm_hi;
assign mosfet_lo = pwm_lo;

// USB //////////////////////////////////////////////////////////////////////

wire [39:0] ep1_rxd;
wire [ 4:0] ep1_rxdv;
fsusb fsusb_inst[4:0]
(.c(clk_100), .c_48(clk_48), .en(usb_en), .dont_respond(usb_dont_respond),
 .vp(usb_vp), .vm(usb_vm), 
 .oe_n(usb_oe), .pwr(usb_pwr),
 .ep1_rxd(ep1_rxd), .ep1_rxdv(ep1_rxdv));

wire [31:0] usb_crossbar_q;
wire [ 3:0] usb_crossbar_qv;
wire [15:0] usb_crossbar_sel = {ureg_foot[11:8], ureg_aux[31:20]};

usb_crossbar usb_crossbar_inst
(.c(clk_100), .sel(usb_crossbar_sel),
 .d(ep1_rxd), .dv(ep1_rxdv),
 .q(usb_crossbar_q), .qv(usb_crossbar_qv));

wire [31:0] menc_usecs, menc_angle, menc_vel, menc_celsius;
wire [15:0] menc_raw;
wire [ 2:0] menc_halls;
wire        menc_direction = ureg_aux[16];
assign      sigma_delta_invert = ureg_aux[17];
menc_parser menc_parser_inst
(.c(clk_100), .rxd(usb_crossbar_q[7:0]), .rxdv(usb_crossbar_qv[0]),
 .enc_usecs(menc_usecs), .enc_angle(menc_angle), .enc_vel(menc_vel),
 .enc_raw(menc_raw), .direction(menc_direction),
 .halls(menc_halls), .temp_celsius(menc_celsius));

wire [31:0] jenc0_usecs, jenc0_angle, jenc0_vel;
wire [15:0] jenc0_raw;
wire        jenc0_direction = ureg_aux[18];
jenc_parser jenc_parser_inst_0
(.c(clk_100), .rxd(usb_crossbar_q[15:8]), .rxdv(usb_crossbar_qv[1]),
 .enc_usecs(jenc0_usecs), .enc_angle(jenc0_angle), .enc_vel(jenc0_vel),
 .enc_raw(jenc0_raw), .direction(jenc0_direction));

wire [31:0] jenc1_usecs, jenc1_angle, jenc1_vel;
wire [15:0] jenc1_raw;
wire        jenc1_direction = ureg_aux[19];
jenc_parser jenc_parser_inst_1
(.c(clk_100), .rxd(usb_crossbar_q[23:16]), .rxdv(usb_crossbar_qv[2]),
 .enc_usecs(jenc1_usecs), .enc_angle(jenc1_angle), .enc_vel(jenc1_vel),
 .enc_raw(jenc1_raw), .direction(jenc1_direction));

wire [31:0] foot_usecs;
wire [255:0] foot_pressures;
wire foot_en = ureg_foot[0];
foot_parser foot_parser_inst
(.c(clk_100), 
 .rxd(usb_crossbar_q[31:24]), .rxdv(foot_en & usb_crossbar_qv[3]),
 .usecs(foot_usecs), .pressures(foot_pressures));

// FOC //////////////////////////////////////////////////////////////////////

// ureg_foc0 is the target register, which is multiplexed with the inbound
// submessages that contain control_id and the inline damping command
wire [31:0] foc_target, foc_damping;
wire [31:0] foc_kp               = ureg_foc1;
wire [31:0] foc_ki               = ureg_foc2;
wire [31:0] foc_max_effort       = ureg_foc3;
wire [31:0] foc_integrator_limit = ureg_foc4;
wire [15:0] foc_stator_offset    = ureg_foc5[15:0]; // integer! 
wire [ 7:0] foc_pole_pairs       = ureg_foc5[23:16]; // integer!
wire [31:0] foc_bus_voltage      = ureg_foc6; // float, in volts
//wire foc_active = ureg_master[2];
wire foc_active;
//wire [31:0] foc_damping          = ureg_foc8; // float, in amps per rad/sec
wire [31:0] foc_resistance       = ureg_foc9; // float, in ohms
wire [31:0] foc_d, foc_q;
wire [31:0] foc_current_a_fp, foc_current_b_fp, foc_current_c_fp;
wire [31:0] foc_effort_d, foc_effort_q;
wire [7:0] foc_fpu_image_d;
wire foc_fpu_image_dv;
wire foc_fpu_image_start;
wire foc_done;

wire [31:0] control_id;

foc_cmd foc_cmd_inst
(.c(clk_100),
 .udp_cmd_dv(udp_rx_regs_cmd_dv),
 .udp_target_reg(ureg_foc0),
 .udp_damping_reg(ureg_foc8),
 .udp_foc_active_reg(ureg_master[2]),
 .submsg_rxd(submsg_rxd),
 .submsg_rxdv(submsg_rxdv),
 .submsg_rxlast(submsg_rxlast),
 .foc_target(foc_target),
 .foc_damping(foc_damping),
 .foc_active(foc_active),
 .temperature(menc_celsius),
 .temperature_limit(ureg_foc10),
 .overtemp_rst(ureg_master[3]),
 .ignore_temperature(ureg_master[4]),
 .control_id(control_id),
 .float(foc_cmd_float));

foc_sm foc_inst
(.c(clk_100), .i_d(current), .i_dv(current_dv[0]), 
 .current_target(foc_target), .active(foc_active), .done(foc_done),
 .menc_raw(menc_raw), .stator_offset(foc_stator_offset),
 .menc_vel(menc_vel),
 .pole_pairs(foc_pole_pairs),
 .resistance(foc_resistance), .damping(foc_damping),
 .kp(foc_kp), .ki(foc_ki), .bus_voltage(foc_bus_voltage),
 .max_effort(foc_max_effort), .integrator_limit(foc_integrator_limit),
 .current_a_fp(foc_current_a_fp),
 .current_b_fp(foc_current_b_fp),
 .current_c_fp(foc_current_c_fp),
 .park_d(foc_d), .park_q(foc_q), 
 .effort_d(foc_effort_d), .effort_q(foc_effort_q),
 .pwm_a(foc_pwm_a), .pwm_b(foc_pwm_b), .pwm_c(foc_pwm_c));

// UDP TX ///////////////////////////////////////////////////////////////////

state_tx state_tx_inst
(.c(clk_100), .period_adc(master_tx_period_adc),
 .current(current), .current_dv(current_dv[0]),
 .status({12'b0, 
          pwm_duty_sel, foc_cmd_float, foc_active, mosfet_en}),
 .udp_txd(state_outbox_txd), 
 .udp_txdv(state_outbox_txdv), 
 .udp_txe(state_outbox_txe),
 .menc_usecs(menc_usecs), .menc_angle(menc_angle), .menc_vel(menc_vel),
 .menc_raw(menc_raw), .menc_halls(menc_halls), .menc_celsius(menc_celsius),
 .jenc0_angle(jenc0_angle), .jenc0_vel(jenc0_vel),
 .jenc1_angle(jenc1_angle), .jenc1_vel(jenc1_vel),
 .foc_d(foc_d), .foc_q(foc_q),
 .effort_d(foc_effort_d), .effort_q(foc_effort_q),
 .foc_target(foc_target), 
 .control_id(control_id),
 .foot_en(foot_en), .foot_usecs(foot_usecs), .foot_pressures(foot_pressures));

// enforce "quiet time" for a few seconds after boot, to allow PHY negotiation
// before we start blasting data. TODO: reset this counter whenever the link
// goes down/up
`ifdef SIM
localparam ENET_QUIET_CNT_TOP = 28'h000_01ff;
`else
localparam ENET_QUIET_CNT_TOP = 28'hfff_ffff;
`endif
wire [27:0] enet_quiet_cnt;
r #(28, ENET_QUIET_CNT_TOP) enet_quiet_cnt_r
(.c(clk_100), .rst(1'b0), .en(|enet_quiet_cnt), 
 .d(enet_quiet_cnt - 1'b1), .q(enet_quiet_cnt));
wire enet_tx_quiet = |enet_quiet_cnt;

/*
wire [7:0] upu_txd_d1;
d1 #(8) upu_txd_d1_r(.c(clk_100), .d(upu_txd), .q(upu_txd_d1));
wire upu_txdv_d1;
d1 upu_txdv_d1_r(.c(clk_100), .d(upu_txdv), .q(upu_txdv_d1));
*/

/////////////////////////////////////////////////////////////////////////////
assign enet_leds = {3'b0, upe_rxdv};
/////////////////////////////////////////////////////////////////////////////

endmodule

