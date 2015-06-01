module uart_tx
#(parameter BAUD_DIV=2,
  parameter BAUD_DIV_WIDTH=8,
  parameter W=8) // data width
(input c,
 input [W-1:0] in,
 input in_en,
 output in_ack,
 output out,
 output busy);
// as implemented below, BAUD_DIV must be >= 2. Lower values are ignored.
wire tx_r_en, tx_sel;
wire [W+1:0] tx;
r #(W+2) tx_r
 (.c(c), .en(tx_r_en), .rst(1'b0), .q(tx),
  .d(tx_sel ? {1'b1, in, 1'b0} : {1'b0, tx[W+1:1]}));
wire [BAUD_DIV_WIDTH-1:0] baud_div_count;
r #(BAUD_DIV_WIDTH) baud_div_count_r
 (.c(c), .en(1'b1), .rst(baud_div_count == BAUD_DIV-1),
  .d(baud_div_count + 1'b1), .q(baud_div_count));
wire tx_edge = ~|baud_div_count;
localparam CW = 4;
localparam SW = 3;
localparam ST_IDLE = 3'd0;
localparam ST_SYNC = 3'd1; // line up with baud clock
localparam ST_TX   = 3'd2; // this includes the start and stop bits in tx_r
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[CW+SW-1:CW];
r #(SW) state_r(.c(c), .d(next_state), .rst(1'b0), .en(1'b1), .q(state));
always @* begin
  case (state)
    ST_IDLE:
      if (in_en)   ctrl = {ST_SYNC    , 4'b0111};
      else         ctrl = {ST_IDLE    , 4'b0000};
    ST_SYNC:
      if (tx_edge) ctrl = {ST_TX      , 4'b0000};
      else         ctrl = {ST_SYNC    , 4'b0000};
    ST_TX:
      if (tx_edge) 
        if (~|tx[W+1:1])  ctrl = {ST_IDLE, 4'b1001};
        else       ctrl = {ST_TX, 4'b1001};
      else         ctrl = {ST_TX, 4'b1000};
    default:       ctrl = {ST_IDLE    , 4'b0000};
  endcase
end
assign tx_r_en = ctrl[0];
assign in_ack = ctrl[1];
assign tx_sel = ctrl[2];
assign out = ctrl[3] ? tx[0] : 1'b1;
assign busy = (state != ST_IDLE);

endmodule
