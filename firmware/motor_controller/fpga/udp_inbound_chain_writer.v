`timescale 1ns/1ns
module udp_inbound_chain_writer
(input        clk_100,
 input [7:0]  outbox_txd,
 input        outbox_txdv,
 input        outbox_txe,  // end of TX burst to outbox. send the message.
 // below here happens at 50 MHz, on the ethernet clock domain
 input        clk_50,
 input [7:0]  rxd,
 input        rxdv,
 input        rxe,
 input        is_chain,
 input [7:0]  hop_downcount,
 input        in_unused_block,
 output       phy_txen,
 output [1:0] phy_txd);

// dfifo holds the messages we have that are waiting to be sent
wire [4:0] sfifo_wrusedw;
wire outbox_full;
r outbox_full_r
(.c(clk_100), .rst(~outbox_txdv & ~outbox_txe & sfifo_wrusedw < 5'h2),
 .en(outbox_txe & sfifo_wrusedw >= 5'h2), .d(1'b1), .q(outbox_full));

wire dfifo_rdreq, dfifo_rdempty;
wire [8:0] dfifo_q;
dcfifo #(.lpm_width(9),
         .lpm_numwords(4096),
         .lpm_widthu(12),
         .lpm_showahead("ON"),
         .use_eab("ON"),
         .intended_device_family("CYCLONE V")) dfifo // data fifo
(.wrclk(clk_100), .wrreq(~outbox_full & outbox_txdv), 
 .data({outbox_txe, outbox_txd}),
 .rdclk(clk_50), .rdreq(dfifo_rdreq), .q(dfifo_q),
 .rdempty(dfifo_rdempty), .aclr(1'b0));

// sfifo holds the sizes of the packets waiting in the dfifo
wire outbox_txe_d1;
d1 outbox_txe_d1_r(.c(clk_100), .d(outbox_txe), .q(outbox_txe_d1));

wire [15:0] sfifo_q, sfifo_d;
wire sfifo_rdreq, sfifo_rdempty;
r #(16) sfifo_d_r
(.c(clk_100), .rst(outbox_txe_d1), .en(outbox_txdv),
 .d(sfifo_d+1'b1), .q(sfifo_d));

dcfifo #(.lpm_width(16),
         .lpm_numwords(30),
         .lpm_widthu(5),
         .lpm_showahead("ON"),
         .use_eab("ON"),
         .intended_device_family("CYCLONE V")) sfifo
(.wrclk(clk_100), .wrreq(~outbox_full & outbox_txe), .data(sfifo_d+1'b1), 
 .wrusedw(sfifo_wrusedw),
 .rdclk(clk_50), .rdreq(sfifo_rdreq), .q(sfifo_q),
 .rdempty(sfifo_rdempty), .aclr(1'b0));

////////////////////////////////////////////////////////////////
// everything below here is in the clk50 (ethernet) domain

localparam SW = 4, CW = 4;
localparam ST_IDLE               = 4'h0;
localparam ST_PREAMBLE           = 4'h1;
localparam ST_DRIVETHRU          = 4'h2;
localparam ST_WAIT_FOR_HOP_COUNT = 4'h3;
localparam ST_HOP_COUNT          = 4'h4;
localparam ST_WAIT_FOR_UNUSED    = 4'h5;
localparam ST_TX_MSG_SENDER_LO   = 4'h6;
localparam ST_TX_MSG_SENDER_HI   = 4'h7;
localparam ST_TX_MSG_LEN_LO      = 4'h8;
localparam ST_TX_MSG_LEN_HI      = 4'h9;
localparam ST_TX_MSG_PAYLOAD     = 4'ha;
localparam ST_FLUSH              = 4'hb;
localparam ST_WAIT_RXE           = 4'hc;
localparam ST_LAST_4_BYTES       = 4'hd;
localparam ST_FCS                = 4'he;
localparam ST_DONE               = 4'hf;

wire [23:0] eth_rxdv_cnt;
r #(24) eth_rxdv_cnt_r
(.c(clk_50), .en(1'b1), .rst(rxdv),
 .d(eth_rxdv_cnt+1'b1), .q(eth_rxdv_cnt));
wire idle_reset = eth_rxdv_cnt == 24'hff_ffff; // sanity check state machine

reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r
(.c(clk_50), .rst(idle_reset), .en(1'b1), .d(next_state), .q(state));

wire idle = state == ST_IDLE;

wire [15:0] tx_dibit_cnt;
r #(16) tx_dibit_cnt_r
(.c(clk_50), .en(1'b1), .rst(ctrl[0]),
 .d(tx_dibit_cnt+1'b1), .q(tx_dibit_cnt));

wire txdv = (~idle & tx_dibit_cnt[1:0] == 2'h0);
//wire rxd_shift_en = rxdv | txdv;

localparam DELAY_BYTES = 8;
wire [8*DELAY_BYTES-1:0] rxd_shift;
r #(8*DELAY_BYTES, 64'h5555_5555_5555_55d5) rxd_shift_r
(.c(clk_50), .rst(idle & ~rxdv), .en(rxdv), 
 .d({rxd_shift[8*(DELAY_BYTES-1)-1:0], rxd}),
 .q(rxd_shift));

wire [7:0] rxd_d8 = rxd_shift[8*DELAY_BYTES-1:8*(DELAY_BYTES-1)];

wire [7:0] cnt;
r #(8) cnt_r(.c(clk_50), .rst(ctrl[1]), .en(1'b1), .d(cnt+1'b1), .q(cnt));

wire [5:0] in_unused_block_shift;
r #(6) in_unused_block_shift_r
(.c(clk_50), .rst(idle), .en(rxdv),
 .d({in_unused_block_shift[4:0], in_unused_block}),
 .q(in_unused_block_shift));

always @* begin
  case (state)
    ST_IDLE:
      if (rxdv)                   ctrl = { ST_PREAMBLE          , 4'b0000 };
      else                        ctrl = { ST_IDLE              , 4'b0001 };
    ST_PREAMBLE:
      if (tx_dibit_cnt == 16'd31) ctrl = { ST_DRIVETHRU         , 4'b0000 };
      else                        ctrl = { ST_PREAMBLE          , 4'b0000 };
    ST_DRIVETHRU:
      if (is_chain)               ctrl = { ST_WAIT_FOR_HOP_COUNT, 4'b0000 };
      else if (rxe)               ctrl = { ST_DONE              , 4'b0000 };
      else                        ctrl = { ST_DRIVETHRU         , 4'b0000 };
    ST_WAIT_FOR_HOP_COUNT:
      if (tx_dibit_cnt == 16'hcc) ctrl = { ST_HOP_COUNT         , 4'b0000 };
      else                        ctrl = { ST_WAIT_FOR_HOP_COUNT, 4'b0000 };
    ST_HOP_COUNT:
      if (tx_dibit_cnt == 16'hd0) ctrl = { ST_WAIT_FOR_UNUSED   , 4'b0000 };
      else                        ctrl = { ST_HOP_COUNT         , 4'b0000 };
    ST_WAIT_FOR_UNUSED:
      if (in_unused_block_shift[5])
        if (sfifo_rdempty)        ctrl = { ST_WAIT_RXE          , 4'b0000 };
        else                      ctrl = { ST_TX_MSG_SENDER_LO  , 4'b0000 };
      else                        ctrl = { ST_WAIT_FOR_UNUSED   , 4'b0000 };
    ST_TX_MSG_SENDER_LO:
      if (txdv)                   ctrl = { ST_TX_MSG_SENDER_HI  , 4'b0000 };
      else                        ctrl = { ST_TX_MSG_SENDER_LO  , 4'b0000 };
    ST_TX_MSG_SENDER_HI:
      if (txdv)                   ctrl = { ST_TX_MSG_LEN_LO     , 4'b0000 };
      else                        ctrl = { ST_TX_MSG_SENDER_HI  , 4'b0000 };
    ST_TX_MSG_LEN_LO:             
      if (txdv)                   ctrl = { ST_TX_MSG_LEN_HI     , 4'b0000 };
      else                        ctrl = { ST_TX_MSG_LEN_LO     , 4'b0000 };
    ST_TX_MSG_LEN_HI:
      if (txdv)                   ctrl = { ST_TX_MSG_PAYLOAD    , 4'b0100 };
      else                        ctrl = { ST_TX_MSG_LEN_HI     , 4'b0000 };
    ST_TX_MSG_PAYLOAD:
      if (txdv)
        if (dfifo_q[8])
          if (sfifo_rdempty)      ctrl = { ST_WAIT_RXE          , 4'b1000 };
          else                    ctrl = { ST_TX_MSG_SENDER_LO  , 4'b1000 };
        else                      ctrl = { ST_TX_MSG_PAYLOAD    , 4'b1000 };
      else                        ctrl = { ST_TX_MSG_PAYLOAD    , 4'b0000 };
    ST_WAIT_RXE:
      if (rxe)                    ctrl = { ST_LAST_4_BYTES      , 4'b0010 };
      else                        ctrl = { ST_WAIT_RXE          , 4'b0000 };
    ST_LAST_4_BYTES:
      if (cnt == 16'hf)           ctrl = { ST_FCS               , 4'b0010 };
      else                        ctrl = { ST_LAST_4_BYTES      , 4'b0000 };
    ST_FCS:
      if (cnt == 16'h12)          ctrl = { ST_IDLE              , 4'b0000 };
      else                        ctrl = { ST_FCS               , 4'b0000 };
    default:                      ctrl = { ST_IDLE              , 4'b0000 };
  endcase
end

assign sfifo_rdreq = ctrl[2];
assign dfifo_rdreq = ctrl[3];

reg [7:0] tx_dibit_shift_d;

wire fcs_dv = txdv & state > ST_PREAMBLE & state != ST_FCS;
wire [31:0] fcs_live;
eth_crc32 fcs_inst
(.c(clk_50), .r(idle), .dv(fcs_dv), .d(tx_dibit_shift_d), .crc(fcs_live));

wire [31:0] fcs;
r #(32) fcs_r
(.c(clk_50), .rst(idle), 
 .en(txdv),
 .d(state == ST_FCS ? { fcs[23:0], 8'h0 } : fcs_live), .q(fcs));

always @* begin
  case (state)
    ST_HOP_COUNT:        tx_dibit_shift_d = hop_downcount;
    ST_TX_MSG_SENDER_LO: tx_dibit_shift_d = hop_downcount;
    ST_TX_MSG_SENDER_HI: tx_dibit_shift_d = 8'h0;
    ST_TX_MSG_LEN_LO:    tx_dibit_shift_d = sfifo_q[7:0];
    ST_TX_MSG_LEN_HI:    tx_dibit_shift_d = sfifo_q[15:8];
    ST_TX_MSG_PAYLOAD:   tx_dibit_shift_d = dfifo_q[7:0];
    ST_FCS:              tx_dibit_shift_d = fcs[31:24];
    default:             tx_dibit_shift_d = rxd_d8;
  endcase
end
  
wire phy_txen_posedge = ~idle;
/*
wire [7:0] tx_dibit_shift_d = state == ST_HOP_COUNT ? hop_downcount :
                              rxd_d8;
*/

wire [7:0] tx_dibit_shift;
r #(8) tx_dibit_shift_r
(.c(clk_50), .rst(1'b0), .en(1'b1),
 .d(idle | txdv ? tx_dibit_shift_d : { 2'h0, tx_dibit_shift[7:2] }),
 .q(tx_dibit_shift));
wire [1:0] phy_txd_posedge = tx_dibit_shift[1:0];

r #(3) phy_negedge_r
(.c(clk_50), .rst(1'b0), .en(1'b1),
 .d({phy_txen_posedge, phy_txd_posedge}),
 .q({phy_txen, phy_txd}));

/*
r #(3) phy_negedge_r
(.c(~clk_50), .rst(1'b0), .en(1'b1),
 .d({phy_txen_posedge, phy_txd_posedge}),
 .q({phy_txen, phy_txd}));
*/

endmodule
