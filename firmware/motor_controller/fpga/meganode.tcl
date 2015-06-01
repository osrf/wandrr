set_global_assignment -name FAMILY "Cyclone V"
set_global_assignment -name DEVICE 5CEBA4F17C8
set_global_assignment -name ROUTER_TIMING_OPTIMIZATION_LEVEL MAXIMUM
set_global_assignment -name ENABLE_BENEFICIAL_SKEW_OPTIMIZATION ON
set_global_assignment -name FITTER_EFFORT "STANDARD FIT"
set_global_assignment -name NUM_PARALLEL_PROCESSORS all
#set_global_assignment -name "optimize_hold_timing" "io paths and minimum tpd paths"
#set_global_assignment -name "router_effort_multiplier" 2
#set_global_assignment -name "placement_effort_multiplier" 2
#set_global_assignment -name "final_placement_optimization" "always"

#set_instance_assignment -name SLEW_RATE 0 -to as5132_*
#set_instance_assignment -name CURRENT_STRENGTH_NEW 4MA -to ad7091_*

set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to clk
set_location_assignment PIN_F12 -to clk

set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to led
set_instance_assignment -name SLEW_RATE 0 -to led 
set_instance_assignment -name CLAMPING_DIODE ON -to led
set_location_assignment PIN_A12 -to led

set_instance_assignment -name CLAMPING_DIODE ON -to mcu_*
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to mcu_*
set_instance_assignment -name SLEW_RATE 0 -to mcu_*

set_location_assignment PIN_R2  -to mcu_spim_cs
set_location_assignment PIN_K4  -to mcu_spim_sclk
set_location_assignment PIN_F3  -to mcu_spim_miso
set_location_assignment PIN_K5  -to mcu_spim_mosi

set_location_assignment PIN_R1  -to mcu_spis_cs
set_location_assignment PIN_F4  -to mcu_spis_sclk
set_location_assignment PIN_R4  -to mcu_spis_miso
set_location_assignment PIN_T3  -to mcu_spis_mosi

set_location_assignment PIN_T2  -to mcu_io

set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to sda
set_instance_assignment -name CLAMPING_DIODE ON -to sda
set_instance_assignment -name SLEW_RATE 0 -to sda
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to scl
set_instance_assignment -name CLAMPING_DIODE ON -to scl
set_instance_assignment -name SLEW_RATE 0 -to scl

set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to mosfet_hi[*]
set_instance_assignment -name CLAMPING_DIODE ON -to mosfet_hi[*]
set_instance_assignment -name SLEW_RATE 0 -to mosfet_hi[*]

set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to mosfet_lo[*]
set_instance_assignment -name CLAMPING_DIODE ON -to mosfet_lo[*]
set_instance_assignment -name SLEW_RATE 0 -to mosfet_lo[*]

set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to mdata[*]
set_instance_assignment -name CLAMPING_DIODE ON -to mdata[*]
set_instance_assignment -name SLEW_RATE 0 -to mdata[*]

set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to mclk[*]
set_instance_assignment -name CLAMPING_DIODE ON -to mclk[*]
set_instance_assignment -name SLEW_RATE 0 -to mclk[*]

set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to mextra
set_instance_assignment -name CLAMPING_DIODE ON -to mextra
set_instance_assignment -name SLEW_RATE 0 -to mextra

set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to mosfet_en
set_instance_assignment -name CLAMPING_DIODE ON -to mosfet_en
set_instance_assignment -name SLEW_RATE 0 -to mosfet_en

set_location_assignment PIN_P2  -to sda
set_location_assignment PIN_P1  -to mosfet_lo[0]
set_location_assignment PIN_J3  -to scl
set_location_assignment PIN_N1  -to mdata[0]
set_location_assignment PIN_J2  -to mclk[0]
set_location_assignment PIN_J1  -to mosfet_hi[0]
set_location_assignment PIN_H1  -to mextra 
set_location_assignment PIN_G1  -to mosfet_lo[1]
set_location_assignment PIN_G2  -to mclk[1] 
set_location_assignment PIN_F2  -to mdata[1] 
set_location_assignment PIN_E2  -to mosfet_hi[1]
set_location_assignment PIN_C3  -to mosfet_en 
set_location_assignment PIN_B3  -to mclk[2] 
set_location_assignment PIN_A3  -to mosfet_lo[2] 
set_location_assignment PIN_A4  -to mosfet_hi[2] 
set_location_assignment PIN_A5  -to mdata[2] 

#set_location_assignment PIN_P2  -to outa[0]
#set_location_assignment PIN_P1  -to outa[1]
#set_location_assignment PIN_J3  -to outa[2]
#set_location_assignment PIN_N1  -to outa[3]
#set_location_assignment PIN_J2  -to outa[4]
#set_location_assignment PIN_J1  -to outa[5]
#set_location_assignment PIN_H1  -to outa[6]
#set_location_assignment PIN_G1  -to outa[7]
#set_location_assignment PIN_G2  -to outa[8]
#set_location_assignment PIN_F2  -to outa[9]
#set_location_assignment PIN_E2  -to outa[10]
#set_location_assignment PIN_C3  -to outa[11]
#set_location_assignment PIN_B3  -to outa[12]
#set_location_assignment PIN_A3  -to outa[13]
#set_location_assignment PIN_A4  -to outa[14]
#set_location_assignment PIN_A5  -to outa[15]

set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to outb[*]
set_instance_assignment -name CLAMPING_DIODE ON -to outb[*]
set_instance_assignment -name SLEW_RATE 0 -to outb[*]
set_location_assignment PIN_E10 -to outb[0]
set_location_assignment PIN_F11 -to outb[1]
set_location_assignment PIN_D11 -to outb[2]
set_location_assignment PIN_F10 -to outb[3]
set_location_assignment PIN_E9  -to outb[4]
set_location_assignment PIN_C9  -to outb[5] 
set_location_assignment PIN_D7  -to outb[6]
set_location_assignment PIN_D8  -to outb[7]
set_location_assignment PIN_B6  -to outb[8]
set_location_assignment PIN_F7  -to outb[9]
set_location_assignment PIN_A9  -to outb[10]
set_location_assignment PIN_C4  -to outb[11]
set_location_assignment PIN_B8  -to outb[12]
set_location_assignment PIN_A8  -to outb[13]
set_location_assignment PIN_B7  -to outb[14]
set_location_assignment PIN_A7  -to outb[15]


set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to enet_clk
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to enet_r*
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to enet_m*
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to enet_t*
set_instance_assignment -name CLAMPING_DIODE ON -to enet_*

set_location_assignment PIN_P16 -to enet_clk
set_location_assignment PIN_P11 -to enet_rst
set_instance_assignment -name SLEW_RATE 0 -to enet_rst
set_instance_assignment -name SLEW_RATE 1 -to enet_clk

set_location_assignment PIN_R15 -to enet_mdc
set_location_assignment PIN_R16 -to enet_mdio
set_instance_assignment -name SLEW_RATE 0 -to enet_mdio

set_location_assignment PIN_T8  -to enet_rxdv[0]
set_location_assignment PIN_T14 -to enet_rxdv[1]

set_location_assignment PIN_R10 -to enet_rxd[0]
set_location_assignment PIN_T10 -to enet_rxd[1]
set_location_assignment PIN_R14 -to enet_rxd[2]
set_location_assignment PIN_T15 -to enet_rxd[3]

set_location_assignment PIN_R7  -to enet_txen[0]
set_location_assignment PIN_R12 -to enet_txen[1]

set_location_assignment PIN_T7  -to enet_txd[0]
set_location_assignment PIN_T5  -to enet_txd[1]
set_location_assignment PIN_T12 -to enet_txd[2]
set_location_assignment PIN_R11 -to enet_txd[3]

set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to enet_leds[*]
set_instance_assignment -name SLEW_RATE 0 -to enet_leds[*]
set_location_assignment PIN_N15 -to enet_leds[0]
set_location_assignment PIN_N16 -to enet_leds[1]
set_location_assignment PIN_L15 -to enet_leds[2]
set_location_assignment PIN_K16 -to enet_leds[3]

##########################################################################

set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to usb_*
set_instance_assignment -name CLAMPING_DIODE ON -to usb_*

set_location_assignment PIN_H16 -to usb_oe[0]
set_location_assignment PIN_D16 -to usb_oe[1]
set_location_assignment PIN_B16 -to usb_oe[2]
set_location_assignment PIN_A13 -to usb_oe[3]
set_location_assignment PIN_B10 -to usb_oe[4]

set_location_assignment PIN_H15 -to usb_vp[0]
set_location_assignment PIN_E16 -to usb_vp[1]
set_location_assignment PIN_C16 -to usb_vp[2]
set_location_assignment PIN_A14 -to usb_vp[3]
set_location_assignment PIN_B11 -to usb_vp[4]

set_location_assignment PIN_J16 -to usb_vm[0]
set_location_assignment PIN_E15 -to usb_vm[1]
set_location_assignment PIN_D14 -to usb_vm[2]
set_location_assignment PIN_A15 -to usb_vm[3]
set_location_assignment PIN_B12 -to usb_vm[4]

set_location_assignment PIN_K15 -to usb_pwr[0]
set_location_assignment PIN_G16 -to usb_pwr[1]
set_location_assignment PIN_F15 -to usb_pwr[2]
set_location_assignment PIN_C11 -to usb_pwr[3]
set_location_assignment PIN_C10 -to usb_pwr[4]
