BIN=bin
NAME=meganode
OUT=$(BIN)/$(NAME)
default: $(OUT).vvp
include common.mk
MODULES=foc_sm minmax fp_compare f2s \
        float32_int16 sine_table_11bit_float32 \
				sqrt fp_sqrt div fp_div ram_64x32 \
				sin op_sin op_usmod \
				sigma_delta foc_sw \
				udp_outbound_chain_rx eth_outbound_chain_tx \
				mii_seq mii_mgmt eth_rx udp_inbound_chain udp_inbound_chain_writer \
				udp_tx_empty jenc_parser spi_master udp_spi_tx spi_slave_rxq \
				usb_crossbar foot_parser foc_cmd foc_cmd_rx
SRCS=meganode.v eth_grx.v eth_crc32.v udp_rx.v udp_rx_regs.v udp_rxq.v pwm.v state_tx.v fsusb.v usb_crc5.v usb_crc5_serial.v usb_tx_token.v usb_tx_sie.v usb_tx_fifo.v usb_tx_stuff.v usb_ctrl.v usb_enum.v usb_tx_data.v usb_crc16.v usb_rx_sie.v usb_rx_nrzi.v usb_tx_ack.v usb_poll.v menc_parser.v commutator.v fpconv.v sine_table_11bit.v mult.v foc.v sine_table_11x16.v park.v sat.v clamp_signed.v sat32.v mul.v addsub.v prom.v foc_consts_32x32.v int16_float32.v s2f.v fpadd.v $(MODULES:%=%.v)
SYNTH_SRCS=meganode_wrapper.v
SIM_SRCS=meganode_tb.v fake_rgmii_phy.v fake_eth_phy.v fake_mii.v sim_fsusb_encoder.v sim_fsusb_phy.v encoder_tb.v fake_rmii_phy.v chain_tb.v sim_fsusb_foot.v
COMMON_SRCS=$(COMMON_SRCS_NAMES:%=%)
MCB_SIM_SRCS=$(MCB_SIM_SRCS_NAMES:%=../../mcb/fpga/%)
ALTERA_PATH ?= bogus____please_set_ALTERA_PATH______have_a_nice_day
ALTERA_SIM_FLAGS = $(ALTERA_PATH)/quartus/eda/sim_lib/altera_mf.v $(ALTERA_PATH)/quartus/eda/sim_lib/220model.v -I $(ALTERA_PATH)/quartus/eda/sim_lib -y $(ALTERA_PATH)/quartus/libraries/megafunctions
QUARTUS_PATH=$(ALTERA_PATH)/quartus/bin

UNIT_TESTS=$(MODULES:%=test_%)

default: $(OUT).lxt 

foc_sw.mif: foc_sw.rb
	ruby foc_sw.rb > foc_sw.mif

bin_symlinks: foc_sw.mif
	cd $(BIN) && ln -fs ../fp_div.hex .
	cd $(BIN) && ln -fs ../foc_sw.mif .
	cd $(BIN) && ln -fs ../sigma_delta_test_data.txt .

#view: $(OUT).lxt
view: 
	gtkwave $(OUT).lxt

$(OUT).vvp: $(BIN) $(SRCS) $(SIM_SRCS) bin_symlinks
	iverilog -Wall -DSIM -s $(NAME)_tb -o $@ -I. $(ALTERA_SIM_FLAGS) $(SRCS) $(SIM_SRCS)

$(OUT).lxt: $(OUT).vvp 
	-ln -fs ../tb_packets.dat $(BIN)/tb_packets.dat
	-ln -fs ../inbound_chain_pkts.dat $(BIN)/inbound_chain_pkts.dat
	cd $(BIN) && vvp -n ../$(OUT).vvp -lxt2

$(BIN):
	mkdir -p $(BIN)
	touch $@

clean:
	rm -rf bin

$(OUT).sdc: $(BIN) $(NAME).sdc
	cp $(NAME).sdc $@

$(OUT).tcl: $(BIN) $(NAME).tcl
	@echo "creating $@ ..."
	@echo "load_package flow" > $@
	@echo "project_new $(NAME) -overwrite" >> $@
	@echo "source \"../$(NAME).tcl\"" >> $@
	@echo "set_global_assignment -name TOP_LEVEL_ENTITY $(NAME)_wrapper" >> $@
	@echo "set_global_assignment -name SEARCH_PATH $(PWD)" >> $@
	@for src in $(SRCS) $(COMMON_SRCS) $(SYNTH_SRCS); do \
		echo "set_global_assignment -name VERILOG_FILE $$src" >> $@; \
	done
	@echo "execute_flow -compile" >> $@
	@echo "project_close" >> $@
	@echo "done creating $@"

synth: $(BIN) $(OUT).tcl $(OUT).sdc bin_symlinks
	cd $(BIN) && $(QUARTUS_PATH)/quartus_sh -t $(NAME).tcl
	$(QUARTUS_PATH)/quartus_cpf -c $(BIN)/$(NAME).sof $(BIN)/$(NAME).rbf

update_mif: foc_sw.mif
	cd $(BIN) && $(QUARTUS_PATH)/quartus_cdb --update_mif meganode
	cd $(BIN) && $(QUARTUS_PATH)/quartus_asm meganode

$(OUT).timing.tcl: $(BIN) $(NAME).timing.tcl $(OUT).sdc
	cp $(NAME).timing.tcl $@

timing: $(OUT).timing.tcl
	cd $(BIN) && $(QUARTUS_PATH)/quartus_sta -t $(NAME).timing.tcl

# todo: decent dependencies
dl: 
	$(QUARTUS_PATH)/quartus_pgm -z -m JTAG -o "p;$(BIN)/$(NAME).sof"

test_chain:
	iverilog -Wall -DSIM -s chain_tb -o $(BIN)/test_chain.vvp -I. -y. $(ALTERA_SIM_FLAGS) $(SRCS) $(SIM_SRCS)
	cd $(BIN) && vvp -n test_chain.vvp -lxt2

test_pwm:
	iverilog -Wall -DTEST_PWM -DSIM -s tb -o $(BIN)/test_pwm.vvp -I. -y. $(ALTERA_SIM_FLAGS) $(SRCS) $(SIM_SRCS)
	cd $(BIN) && vvp -n test_pwm.vvp -lxt2

test_dtc:
	iverilog -Wall -DTEST_DTC -DSIM -s tb -o $(BIN)/test_dtc.vvp -I. -y. $(ALTERA_SIM_FLAGS) $(SRCS) $(SIM_SRCS)
	cd $(BIN) && vvp -n test_dtc.vvp -lxt2

test_fsusb:
	iverilog -Wall -DTEST_FSUSB -DSIM -s tb -o $(BIN)/test_fsusb.vvp -I. -y. $(ALTERA_SIM_FLAGS) $(SRCS) $(SIM_SRCS)
	cd $(BIN) && vvp -n test_fsusb.vvp -lxt2

test_usb_crc5:
	iverilog -Wall -DTEST_USB_CRC5 -DSIM -s tb -o $(BIN)/test_usb_crc5.vvp -I. -y. $(ALTERA_SIM_FLAGS) $(SRCS) $(SIM_SRCS)
	cd $(BIN) && vvp -n test_usb_crc5.vvp -lxt2

test_usb_crc16:
	iverilog -Wall -DTEST_USB_CRC16 -DSIM -s tb -o $(BIN)/test_usb_crc16.vvp -I. -y. $(ALTERA_SIM_FLAGS) $(SRCS) $(SIM_SRCS)
	cd $(BIN) && vvp -n test_usb_crc16.vvp -lxt2

test_sof:
	iverilog -Wall -DTEST_USB_SOF  -DSIM -s tb -o $(BIN)/test_sof.vvp -I. -y. $(ALTERA_SIM_FLAGS) $(SRCS) $(SIM_SRCS)
	cd $(BIN) && vvp -n test_sof.vvp -lxt2

test_encoder:
	iverilog -Wall -DTEST_ENCODER -DSIM -s encoder_tb -o $(BIN)/test_encoder.vvp -I. -y. $(ALTERA_SIM_FLAGS) $(SRCS) $(SIM_SRCS)
	cd $(BIN) && vvp -n test_encoder.vvp -lxt2

test_commutator:
	iverilog -Wall -DTEST_COMMUTATOR -DSIM -s tb -o $(BIN)/test_commutator.vvp -I. -y. $(ALTERA_SIM_FLAGS) $(SRCS) $(SIM_SRCS)
	cd $(BIN) && vvp -n test_commutator.vvp -lxt2

test_foc:
	iverilog -Wall -DTEST_FOC -DSIM -s tb -o $(BIN)/test_foc.vvp -I. -y. $(ALTERA_SIM_FLAGS) $(SRCS) $(SIM_SRCS)
	cd $(BIN) && vvp -n test_foc.vvp -lxt2

$(UNIT_TESTS): test_%: %.v bin_symlinks
	iverilog -Wall -Dtest_$* -DSIM -s $*_tb -o $(BIN)/$@.vvp -I . -y. $(ALTERA_SIM_FLAGS) -y . $<
	cd $(BIN) && vvp -n $@.vvp -lxt2
