BIN=bin
OUT=$(BIN)/$(NAME)
OPENOCD_SCRIPTS=../../openocd
HARD_FLOAT_FLAGS=-mfloat-abi=hard -mfpu=fpv4-sp-d16
TC=arm-none-eabi-
ROOT=../..
OPENOCD=openocd -f $(ROOT)/common/stm32/olimex-arm-usb-tiny-h.cfg -f $(ROOT)/common/stm32/stm32f427.cfg -f ../common/serial_router.cfg
COMMON_SRCS=startup stubs leds pin console delay systime enet power serial dmxl xsens state
COMMON_OBJS=$(COMMON_SRCS:%=$(BIN)/%.o)
OBJS=$(SRCS:%.c=$(BIN)/%.o)
LDSCRIPT ?= app
IMAGE_START ?= 0x08020000

CFLAGS=-g $(HARD_FLOAT_FLAGS) -mthumb -mcpu=cortex-m4 -ffunction-sections -fdata-sections -MD -std=gnu99 -Wall -I$(ROOT)/common/stm32 -Werror -I. -I../common -DHSE_VALUE=25000000
LDFLAGS=-g $(HARD_FLOAT_FLAGS) -mthumb -mcpu=cortex-m4

default: $(BIN) $(OUT).bin

$(BIN):
	mkdir -p $(BIN)

$(COMMON_OBJS): $(BIN)/%.o: ../common/%.c
	$(TC)gcc $(CFLAGS) -c $< -o $@

$(BIN)/%.o: %.c
	$(TC)gcc $(CFLAGS) -c $< -o $@

$(OUT): $(OBJS) $(COMMON_OBJS)
	$(TC)gcc $(OBJS) $(COMMON_OBJS) -lc -lgcc -lm -T $(ROOT)/common/stm32/stm32f411_$(LDSCRIPT).ld -Wl,--no-gc-sections $(LDFLAGS) -o $(OUT) -Wl,-Map=$(BIN)/$(NAME).map,--cref
	$(TC)objdump -S -d $(OUT) > $(OUT).objdump

$(OUT).bin: $(OUT)
	$(TC)objcopy -O binary $(OUT) $(OUT).bin
	$(TC)size $(OUT)

clean:
	-rm -rf $(BIN)

program: $(OUT).bin
	$(OPENOCD) -c "init; sleep 100; halt; sleep 100; flash write_image erase $(OUT).bin $(IMAGE_START); verify_image $(OUT).bin $(IMAGE_START); sleep 100; reset run; sleep 100; shutdown"

dump_flash: $(BIN)
	$(OPENOCD) -c "init; halt; flash banks; dump_image $(OUT).bin.dump $(IMAGE_START) 0x1000; reset run; shutdown"

gdb_server: $(OUT).bin
	$(OPENOCD) -c "init; halt"

gdb: $(OUT).bin
	$(TC)gdb $(OUT) -x ../../../common/stm32/gdb_init_commands

reset:
	$(OPENOCD) -c "init; sleep 100; halt; sleep 100; reset run; sleep 100; shutdown"

bl_dfu_download: $(OUT).bin
	sudo dfu-util -d 0483:df11 --dfuse-address 0x08000000 -a 0 -D bin/bl.bin

app_dfu_download: $(OUT).bin
	sudo dfu-util -d 0483:df11 --dfuse-address 0x08020000 -a 0 -D $(OUT).bin
