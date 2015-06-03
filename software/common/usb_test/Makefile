BIN=bin
NAME=usb_test
OUT=$(BIN)/$(NAME)

LDFLAGS=-g -lusb-1.0
CFLAGS=-O2 -g

SRCS=main.cpp
OBJS=$(SRCS:%.cpp=$(BIN)/%.o)

default: $(BIN) $(OUT)

$(BIN):
	mkdir -p $(BIN)

$(BIN)/%.o: %.cpp
	g++ $(CFLAGS) -c $< -o $@

$(OUT): $(OBJS)
	g++ $(OBJS) $(LDFLAGS) -o $(OUT)

run: $(OUT)
	@echo "================================================="
	@$(OUT)
	@echo "================================================="
