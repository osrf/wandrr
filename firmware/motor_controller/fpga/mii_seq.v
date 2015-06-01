`timescale 1ns/1ns
module mii_seq
(input c,
 input start,
 output mdc,
 inout mdio);

wire [4:0] mii_phyad;
wire [4:0] mii_addr;
wire [15:0] mii_wdata;
wire [15:0] mii_rdata;
wire mii_req, mii_we, mii_ack;

mii_mgmt #(.CLK_MHZ(125), .MDC_KHZ(500)) mii
(.clk(c), .phyad(mii_phyad), .addr(mii_addr),
 .wdata(mii_wdata), .rdata(mii_rdata),
 .req(mii_req), .we(mii_we), .ack(mii_ack),
 .mdc(mdc), .mdio(mdio));

// ROM for the MII sequence
wire [3:0] seq_addr;
reg [31:0] seq_q;
always @(posedge c) begin
  case (seq_addr)
    4'h0: seq_q = 32'h8009_1c00; // disable 1g speed
    4'h1: seq_q = 32'h8109_1c00; // disable 1g speed
    4'h2: seq_q = 32'h8000_0340; // force 100M speed
    4'h3: seq_q = 32'h8100_0340; // force 100M speed
    //4'h4: seq_q = 32'h8000_8140; // software reset
    //4'h5: seq_q = 32'h8100_8140; // software reset
    default: seq_q = 32'h0;
  endcase
end

localparam ST_IDLE  = 3'h0;
localparam ST_TXRX  = 3'h1;
localparam ST_CHILL = 3'h2;

localparam SW=3, CW=4;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r(.c(c), .rst(1'b0), .en(1'b1), .d(next_state), .q(state));

wire [15:0] chill_cnt;

always @* begin
  case (state)
    ST_IDLE:
      if (start)   ctrl = { ST_TXRX , 4'b0000 };
      else         ctrl = { ST_IDLE , 4'b0000 };
    ST_TXRX:
      if (mii_ack) ctrl = { ST_CHILL, 4'b0000 };
      else         ctrl = { ST_TXRX , 4'b0000 };
    ST_CHILL:
      if (chill_cnt == 16'h1000)
        if (seq_addr == 4'h3) ctrl = { ST_IDLE , 4'b0000 };
        else                  ctrl = { ST_TXRX , 4'b0001 };
      else                    ctrl = { ST_CHILL, 4'b0000 };
    default:      ctrl = { ST_IDLE, 4'b0000 };
  endcase
end

r #(16) chill_cnt_r
(.c(c), .rst(state == ST_TXRX), .d(chill_cnt + 1'b1), .en(1'b1), .q(chill_cnt));

r #(4) seq_addr_r
(.c(c), .rst(state == ST_IDLE), .en(ctrl[0]), 
 .d(seq_addr + 1'b1), .q(seq_addr));

assign mii_req   = state == ST_TXRX;
assign mii_we    = seq_q[31];
assign mii_phyad = seq_q[28:24];
assign mii_wdata = seq_q[15:0];
assign mii_addr  = seq_q[20:16];

endmodule

`ifdef test_mii_seq

module mii_seq_tb();

wire c;
sim_clk #(125) clk_125(c);

reg start;
wire mdc, mdio;

mii_seq dut(.*);

fake_mii mii_inst(.*);

initial begin
  $dumpfile("mii_seq.lxt");
  $dumpvars();
  start = 0;
  #100;
  wait(~c);
  wait(c);
  start <= 1;
  wait(~c);
  wait(c);
  start <= 0;
  #1000000;
  $finish();
end
 
endmodule

`endif

