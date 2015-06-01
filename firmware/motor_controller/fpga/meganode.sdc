# Clock constraints

create_clock -name board_clk -period 40 [get_ports {clk}]
#create_clock -name virtual_phy0_rxclk -period 8
#create_clock -name virtual_phy1_rxclk -period 8
#create_clock -name phy0_rxclk -period 8 -waveform {1.2 5.2} [get_ports enet_rxclk[0]]
#create_clock -name phy1_rxclk -period 8 -waveform {1.2 5.2} [get_ports enet_rxclk[1]]

# Automatically constrain PLL and other generated clocks
#derive_pll_clocks -use_net_name
#-create_base_clocks
derive_pll_clocks

set clk_50 altera_pll_inst|general[0].gpll~PLL_OUTPUT_COUNTER|divclk

#create_generated_clock -name clk_48 -source [get_pins {altera_pll_inst|general[3].gpll~PLL_OUTPUT_COUNTER|divclk}]
set clk_100 altera_pll_inst|general[4].gpll~PLL_OUTPUT_COUNTER|divclk

#create_generated_clock -name clk_100 -source [get_pins {altera_pll_inst|general[4].gpll~PLL_OUTPUT_COUNTER|divclk}]

##create_generated_clock -name clk_vco -source [get_pins {altera_pll_inst|general[0].gpll~FRACTIONAL_PLL|refclkin}] -multiply_by 48 -duty_cycle 50.00 [get_pins {altera_pll_inst|general[0].gpll~FRACTIONAL_PLL|vcoph[0]}]

##create_generated_clock -name clk_50 -source [get_pins {altera_pll_inst|general[0].gpll~PLL_OUTPUT_COUNTER|vco0ph[0]}] -divide_by 24 -duty_cycle 50.00 [get_pins {altera_pll_inst|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}]

#create_generated_clock -name clk_50_90 -source [get_pins {altera_pll_inst|general[2].gpll~PLL_OUTPUT_COUNTER|vco0ph[0]}] -divide_by 6 -phase 90.00 -duty_cycle 50.00 [get_pins {altera_pll_inst|general[2].gpll~PLL_OUTPUT_COUNTER|divclk}]

##create_generated_clock -name clk_25 -source [get_pins {altera_pll_inst|general[0].gpll~PLL_OUTPUT_COUNTER|vco0ph[0]}] -divide_by 48 -duty_cycle 50.00 [get_pins {altera_pll_inst|general[1].gpll~PLL_OUTPUT_COUNTER|divclk}]

##create_generated_clock -name clk_20 -source [get_pins {altera_pll_inst|general[0].gpll~PLL_OUTPUT_COUNTER|vco0ph[0]}] -divide_by 60 -duty_cycle 50.00 [get_pins {altera_pll_inst|general[2].gpll~PLL_OUTPUT_COUNTER|divclk}]

##create_generated_clock -name clk_48 -source [get_pins {altera_pll_inst|general[0].gpll~PLL_OUTPUT_COUNTER|vco0ph[0]}] -divide_by 25 -duty_cycle 50.00 [get_pins {altera_pll_inst|general[3].gpll~PLL_OUTPUT_COUNTER|divclk}]

##create_generated_clock -name clk_100 -source [get_pins {altera_pll_inst|general[0].gpll~PLL_OUTPUT_COUNTER|vco0ph[0]}] -divide_by 12 -duty_cycle 50.00 [get_pins {altera_pll_inst|general[4].gpll~PLL_OUTPUT_COUNTER|divclk}]

#create_generated_clock -source clk_50 -name clk_50_virt
#create_generated_clock -name clk_50_virt -source [get_pins {altera_pll_inst|general[0].gpll~PLL_OUTPUT_COUNTER|vco0ph[0]}] -divide_by 24 -duty_cycle 50.00 [get_pins {altera_pll_inst|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}]

# Automatically calculate clock uncertainty to jitter and other effects.
derive_clock_uncertainty

#create_generated_clock -source $clk_50 -name rmii_clk_virt
#create_clock -source $clk_50 -name rmii_clk_virt
create_clock -period 20 -name rmii_clk_virt

# setup: 4ns
# hold: -4ns
set_input_delay -clock rmii_clk_virt -max  4.0 [get_ports {enet_rxd[*] enet_rxdv[*]}]
set_input_delay -clock rmii_clk_virt -min -4.0 [get_ports {enet_rxd[*] enet_rxdv[*]}]

####set_false_path -to *:upe|r:rx_sync_r|q[*]

#set phy_tx_max_delay  0.5
#set phy_tx_min_delay -0.5

# setup:  4ns
# hold:  -4ns
set_output_delay -clock rmii_clk_virt -max  4.0 [get_ports {enet_txd[*] enet_txen[*]}]
set_output_delay -clock rmii_clk_virt -min -4.0 [get_ports {enet_txd[*] enet_txen[*]}]

# not sure what this one should be
#set_output_delay -clock rmii_clk_virt 0 [get_ports {enet_clk}]
#create_clock -name rmii_clk -period 20 -waveform { 10 20 } [get_ports enet_clk]

##############################################################################################3

set_false_path -to *sigma_delta_inst[*]|sync*|q[*]

# these fifo false-path assignments were supposed to be added automatically,
# but it seems like they still fail unless added here:
set_false_path -from [get_registers {*dcfifo*delayed_wrptr_g[*]}] -to [get_registers {*dcfifo*rs_dgwp*}]
set_false_path -from [get_registers {*dcfifo*rdptr_g[*]}] -to [get_registers {*dcfifo*ws_dgrp*}]
set_false_path -to   *tx_sie_inst|sync:*|*|q[*]
set_false_path -to   *fsusb_inst[*]|sync:*|*|q[*]
set_false_path -to   *rx_sie_inst|sync:*|*|q[*]
set_false_path -from enet_rxd[*] -to meganode:meganode_inst|eth_rx:dne|sync:rmii_d_sync_r|r:shift_r|q[*]
set_false_path -from enet_rxd[*] -to meganode:meganode_inst|eth_rx:upe|sync:rmii_d_sync_r|r:shift_r|q[*]
set_false_path -from enet_rxdv[*] -to meganode:meganode_inst|eth_rx:dne|sync:rmii_dv_sync_r|r:shift_r|q[*]

set_false_path -from meganode:meganode_inst|fsusb:fsusb_inst[*]|usb_rx_sie:rx_sie_inst|sync:xclk_fifo_aclr_sync_r|r:shift_r|q[1] 

set usb_in_min    0.0
set usb_in_max   10.0
set usb_out_min   0.0
set usb_out_max  10.0

create_clock -period 20.833 -name usb_0_clk
create_clock -period 20.833 -name usb_1_clk
create_clock -period 20.833 -name usb_2_clk
create_clock -period 20.833 -name usb_3_clk
create_clock -period 20.833 -name usb_4_clk

set_input_delay  -clock usb_0_clk -min $usb_in_min [get_ports {usb_vp[0] usb_vm[0]}]
set_input_delay  -clock usb_1_clk -min $usb_in_min [get_ports {usb_vp[1] usb_vm[1]}]
set_input_delay  -clock usb_2_clk -min $usb_in_min [get_ports {usb_vp[2] usb_vm[2]}]
set_input_delay  -clock usb_3_clk -min $usb_in_min [get_ports {usb_vp[3] usb_vm[3]}]
set_input_delay  -clock usb_4_clk -min $usb_in_min [get_ports {usb_vp[4] usb_vm[4]}]

set_input_delay  -clock usb_0_clk -max $usb_in_max [get_ports {usb_vp[0] usb_vm[0]}]
set_input_delay  -clock usb_1_clk -max $usb_in_max [get_ports {usb_vp[1] usb_vm[1]}]
set_input_delay  -clock usb_2_clk -max $usb_in_max [get_ports {usb_vp[2] usb_vm[2]}]
set_input_delay  -clock usb_3_clk -max $usb_in_max [get_ports {usb_vp[3] usb_vm[3]}]
set_input_delay  -clock usb_4_clk -max $usb_in_max [get_ports {usb_vp[4] usb_vm[4]}]

set_output_delay -clock usb_0_clk -min $usb_out_min [get_ports {usb_vp[0] usb_vm[0] usb_oe[0]}]
set_output_delay -clock usb_1_clk -min $usb_out_min [get_ports {usb_vp[1] usb_vm[1] usb_oe[1]}]
set_output_delay -clock usb_2_clk -min $usb_out_min [get_ports {usb_vp[2] usb_vm[2] usb_oe[2]}]
set_output_delay -clock usb_3_clk -min $usb_out_min [get_ports {usb_vp[3] usb_vm[3] usb_oe[3]}]
set_output_delay -clock usb_4_clk -min $usb_out_min [get_ports {usb_vp[4] usb_vm[4] usb_oe[4]}]

set_output_delay -clock usb_0_clk -max $usb_out_max [get_ports {usb_vp[0] usb_vm[0] usb_oe[0]}]
set_output_delay -clock usb_1_clk -max $usb_out_max [get_ports {usb_vp[1] usb_vm[1] usb_oe[1]}]
set_output_delay -clock usb_2_clk -max $usb_out_max [get_ports {usb_vp[2] usb_vm[2] usb_oe[2]}]
set_output_delay -clock usb_3_clk -max $usb_out_max [get_ports {usb_vp[3] usb_vm[3] usb_oe[3]}]
set_output_delay -clock usb_4_clk -max $usb_out_max [get_ports {usb_vp[4] usb_vm[4] usb_oe[4]}]

set_output_delay -min 0.0 -clock $clk_100 [get_ports {usb_pwr[*]}]
set_output_delay -max 0.0 -clock $clk_100 [get_ports {usb_pwr[*]}]

# allow the usb power signals to take forever. we don't care.
set_multicycle_path -setup -to [get_ports {usb_pwr[*]}] 4
set_multicycle_path -hold  -to [get_ports {usb_pwr[*]}] 3

#set_false_path -from meganode:meganode_inst|fsusb:fsusb_inst[*]|usb_tx_sie:tx_sie_inst|r:state_r|q[0] -to meganode:meganode_inst|fsusb:fsusb_inst[*]|usb_rx_sie:rx_sie_inst|dcfifo:xclk_fifo|dcfifo_96d1:*
#auto_generated|altsyncram_3g71:fifo_ram|ram_block11a0~PORT_B_ADDRESS_CLEAR
#set_false_path -from [get_clocks clk_20] -to [get_clocks clk_125]
#set_false_path -from [get_clocks clk_125] -to [get_clocks clk_20]

#set_output_delay -clock 

##############################################################################
# internal JTAG nonsense. but perhaps this isn't needed anymore (?)
#create_clock -name altera_reserved_tclk -period "100.000 ns" [get_ports altera_reserved_tck]
#set_clock_groups -exclusive -group [get_clocks altera_reserved_tck]
#set_input_delay -clock altera_reserved_tck 20 [get_ports altera_reserved_tdi]
#set_input_delay -clock altera_reserved_tck 20 [get_ports altera_reserved_tms]
#set_input_delay -clock altera_reserved_tck 20 [get_ports altera_reserved_tdo]

