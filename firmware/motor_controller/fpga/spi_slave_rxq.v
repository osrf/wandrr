`timescale 1ns/1ns
module spi_slave_rxq
(input c,
 input cs, input sclk, input mosi, 
 output [7:0] rxd, output rxdv, output rxe);

wire [7:0] spi_rxd;
wire spi_rxdv, spi_rxe;

spi_slave_rx spi_slave_rx_inst
(.clk(c), .cs(cs), .sclk(sclk), .mosi(mosi),
 .rxd(spi_rxd), .rxdv(spi_rxdv), .rxe(spi_rxe));

wire dfifo_rdreq, dfifo_empty;
wire [7:0] dfifo_q;
wire [7:0] dfifo_usedw;

scfifo #(.lpm_width(8),
         .lpm_numwords(256),
         .lpm_widthu(8),
         .lpm_showahead("ON"),
         .use_eab("ON"),
         .intended_device_family("CYCLONE V")) dfifo
(.clock(c),
 .wrreq(spi_rxdv), .data(spi_rxd),
 .rdreq(dfifo_rdreq), .q(dfifo_q),
 .empty(dfifo_empty), .usedw(dfifo_usedw), .aclr(1'b0), .sclr(1'b0));

localparam SW = 3;
localparam ST_IDLE        = 3'd0;
localparam ST_TX_HEADER   = 3'd1;
localparam ST_DRAIN_FIFO  = 3'd2;
localparam ST_DONE        = 3'd3;

localparam CW = 4;
reg [SW+CW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_reg
(.c(c), .d(next_state), .rst(1'b0), .en(1'b1), .q(state));

assign rxdv = ctrl[0];
assign rxd = state == ST_TX_HEADER ? 8'h99 : dfifo_q;
assign rxe = rxdv & dfifo_usedw == 8'h1;

assign dfifo_rdreq = ctrl[1];

always @* begin
  case (state)
    ST_IDLE:
      if (spi_rxe)             ctrl = { ST_TX_HEADER , 4'b0000 };
      else                     ctrl = { ST_IDLE      , 4'b0000 };
    ST_TX_HEADER:              ctrl = { ST_DRAIN_FIFO, 4'b0001 };
    ST_DRAIN_FIFO:
      if (dfifo_usedw == 8'h1) ctrl = { ST_IDLE      , 4'b0011 };
      else                     ctrl = { ST_DRAIN_FIFO, 4'b0011 };
    default:                   ctrl = { ST_IDLE      , 4'b0000 };
  endcase
end

endmodule

`ifdef test_spi_slave_rxq

module spi_slave_rxq_tb();

wire c;
sim_clk #(100) clk_inst(c);

localparam W = 8;
reg [W-1:0] master_txd;
wire [W-1:0] master_rxd;
reg master_txdv;
wire master_rxdv;

wire master_done, master_busy;
wire sclk, mosi, miso, cs;
spi_master #(.SCLK_DIV(50), .W(8)) spi_master_inst
  (.c(c), .busy(master_busy), .done(master_done),
   .txd(master_txd), .txdv(master_txdv),
   .rxd(master_rxd), .rxdv(master_rxdv),
   .sclk(sclk), .mosi(mosi), .miso(miso), .cs(cs));

wire [7:0] qrxd;
wire qrxdv, qrxe;
spi_slave_rxq dut
(.c(c), .cs(cs), .sclk(sclk), .mosi(mosi),
 .rxd(qrxd), .rxdv(qrxdv), .rxe(qrxe));

initial begin
  $dumpfile("spi_slave_rxq.lxt");
  $dumpvars();
  master_txd = 8'ha5;
  master_txdv = 0;
  #100
  @(posedge c);
  #1 master_txdv = 1;
  @(posedge c);
  #1 master_txd = 8'h7;
  @(posedge c);
  #1 master_txd = 8'h51;
  @(posedge c);
  #1 master_txdv = 0;
  #40000
  @(posedge c);
  #1 master_txdv = 1;
  @(posedge c);
  #1 master_txdv = 0;
  #40000
  $finish;
end

endmodule

`endif
