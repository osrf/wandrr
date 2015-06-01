/* This file is a modified version of a file included in the NetFPGA
distribution, which has the following license notification:

Copyright (c) 2006 The Board of Trustees of The Leland Stanford
Junior University

We are making the NetFPGA tools and associated documentation (Software)
available for public use and benefit with the expectation that others will
use, modify and enhance the Software and contribute those enhancements back
to the community. However, since we would like to make the Software
available for broadest use, with as few restrictions as possible permission
is hereby granted, free of charge, to any person obtaining a copy of this
Software) to deal in the Software under the copyrights without restriction,
including without limitation the rights to use, copy, modify, merge,
publish, distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to the
following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.

The name and trademarks of copyright holder(s) may NOT be used in
advertising or publicity pertaining to the Software or any derivatives
without specific, written prior permission.
*/

`timescale 1ns / 1ns

// `define FAKE_PHY_TEST

module fake_eth_phy
(
 input       rgmii,
 input       gmii,
 input       rmii,
 input [7:0] TXD,
 input       TXEN,
 input       GTXCLK,
 input       REFCLK,
 input       TXER,
 output      TXCLK,
 output      RXCLK,
 output reg  [7:0] RXD,
 output      RXER,
 output reg  RXDV,
 input       RESET_N,
 input       MDC,
 inout       MDIO,
 input       force_rxer, // test interface
 output reg  rmii_crs_dv_toggle // asserted when near end of packet
);

parameter input_file_name="packets_in.dat";
parameter input_file_maxsize=10000;
parameter input_pkt_seperator=32'heeeeffff;
parameter max_pkt_size=9100;

integer seed = 10;

reg [7:0] rx_packet_buffer [0:max_pkt_size]; // data of receive packet.
reg [7:0] tx_packet_buffer [0:max_pkt_size];
reg [31:0] input_file[0:input_file_maxsize];
reg [31:0] crc_table[0:255];

fake_mii mii(MDC, MDIO);

wire rx_clk_125;
wire rx_clk_25;
sim_clk #(125) rxc_125(rx_clk_125);
sim_clk #(25)  rxc_25(rx_clk_25);

/*
// derive internal 25mhz clock from GTXCLK if doing rmii
wire clk50_div;
r clk50_div_r(.c(GTXCLK), .rst(1'b0), .en(1'b1), 
              .d(~clk50_div), .q(clk50_div));
assign rx_clk_25 = (rmii ? GTXCLK : rx_sim_clk_25);
*/

assign RXCLK = (rgmii | gmii) ? rx_clk_125 : (rmii ? REFCLK : rx_clk_25);

wire tx_clk_25;
sim_clk #(25) txc(tx_clk_25);
assign TXCLK = (rgmii | gmii) ? 1'b0 : (rmii ? REFCLK : tx_clk_25);
assign RXER = force_rxer;

initial begin
  RXD = 8'hx;
  RXDV = 0;
  rmii_crs_dv_toggle = 0;
  // read input
  $display($time, " reading input file %s.", input_file_name);
  $readmemh(input_file_name, input_file);
  check_integrity;
  gen_crc_table;
  fork
    handle_inputs;
    handle_outputs;
  join
  $finish;
end
//////////////////////////////////////////////////////////////////////////////
task handle_inputs;
integer packet_index, packet_start;  // pointer to next word in input memory
integer words, i, j;
reg [31:0] len, tmp, crc;
reg [63:0] tmp64;
reg [15:0] rep;
time repdelay;
time time2send;
begin
  packet_index = 0;
  // send while there are any packets left to send!
  while ((packet_index < max_pkt_size) && 
         (input_file[packet_index] !== 32'hxxxxxxxx)) begin
	  // get next packet and put in rx_packet_buffer
	  len = input_file[packet_index];
    tmp64 = {input_file[packet_index+1],input_file[packet_index+2]};
    rep = tmp64[63:48];
    repdelay = {48'h0, tmp64[47:32]};
    if (rep == 0) rep = 1;
    // time2send is EARLIEST we can send this packet.
    if (rep > 1) time2send = {32'h0, tmp64[31:0]};
    else         time2send = {16'h0, tmp64[47:0]};
	  if (time2send > $time) begin
	    //$display($time, " Info: Waiting to time %t to send pkt (length %0d)",
      //         time2send, len);
	    #(time2send - $time);
	  end
    j = 0;
    packet_start = packet_index + 3;
    while (j < rep) begin
	    //$display($time, " Sending next input packet (len %0d) to MAC.", len);
	    // Build the packet in rx_packet_buffer.
	    packet_index = packet_start;	//now points at DA
	    words = ((len-1)>>2)+1;           // number of 32 bit words in pkt
	    i = 0;                            // index into rx_packet_buffer
	    while (words) begin
	      tmp = input_file[packet_index];
	      rx_packet_buffer[i]   = tmp[31:24];
	      rx_packet_buffer[i+1] = tmp[23:16];
	      rx_packet_buffer[i+2] = tmp[15:8];
	      rx_packet_buffer[i+3] = tmp[7:0];
	      words = words - 1;
	      i = i + 4;
	      packet_index = packet_index + 1;
	    end
	    // might have gone too far so set byte index to correct position,
	    i = len;
	    // clear out buffer ready for CRC
	    rx_packet_buffer[i]   = 8'h0;
	    rx_packet_buffer[i+1] = 8'h0;
	    rx_packet_buffer[i+2] = 8'h0;
	    rx_packet_buffer[i+3] = 8'h0;
	    crc = update_crc(32'hffffffff,len)^32'hffffffff;
	    //$display("%t %m Info: CRC is %x", $time, crc);
	    rx_packet_buffer[i+3] = crc[31:24];
	    rx_packet_buffer[i+2] = crc[23:16];
	    rx_packet_buffer[i+1] = crc[15:8];
	    rx_packet_buffer[i]   = crc[7:0];
	    send_rx_pkt(len+4);  // data + CRC
	    if (input_file[packet_index] !== input_pkt_seperator) begin
	      $display($time,
                 " %m Error: expected to point at packet sep %x but saw %x",
		             input_pkt_seperator, input_file[packet_index]);
	      $fflush; $finish;
	    end
      j = j + 1;
      if (j < rep) #repdelay;
    end
	  packet_index = packet_index + 1;
  end
  //$display($time," rx packets finished at index %d.", packet_index);
  //#500 $finish;
end
endtask

task send_rx_pkt;
input [31:0] pkt_len_w_crc;
   begin
      if (rgmii)     send_rgmii_rx_pkt(pkt_len_w_crc);
      else if (gmii) send_gmii_rx_pkt(pkt_len_w_crc);
      else if (rmii) send_rmii_rx_pkt(pkt_len_w_crc);
      else           send_mii_rx_pkt(pkt_len_w_crc);
   end
endtask

task send_rgmii_rx_pkt;
input [31:0] pkt_len_w_crc;
integer i;
   begin
      @(negedge RXCLK) 
      #2 RXDV = 1;
         RXD = 8'hx5;
      for (i=0; i<7; i=i+1) @(negedge RXCLK) begin end
      @(posedge RXCLK) begin end
      #2 RXD = 8'hxd;
      @(negedge RXCLK) begin end

      for (i=0; i<pkt_len_w_crc; i=i+1) begin
         #2 RXD = {4'hx, rx_packet_buffer[i][3:0]};
         @(posedge RXCLK) begin end
         #2 RXD = {4'hx, rx_packet_buffer[i][7:4]};
         @(negedge RXCLK) begin end
      end
      #1 RXDV = 0;
      RXD = 8'hx;
      // IFG
      for (i=0; i<12; i=i+1) begin
         @(posedge RXCLK) begin end
      end
   end
endtask


task send_gmii_rx_pkt;
input [31:0] pkt_len_w_crc;
integer i;
   begin
      @(posedge RXCLK) #1 RXDV = 1;
      RXD = 8'h55;
      for (i=0; i<7; i=i+1) @(posedge RXCLK) begin end
      #1 RXD = 8'hd5;
      @(posedge RXCLK) begin end

      for (i=0; i<pkt_len_w_crc; i=i+1) begin
         #1;
         RXD = rx_packet_buffer[i];
         @(posedge RXCLK) begin end
      end
      #1 RXDV = 0;
      RXD = 8'hx;
      // IFG
      for (i=0; i<12; i=i+1) begin
         @(posedge RXCLK) begin end
      end
   end
endtask

task send_mii_rx_pkt;
input [31:0] pkt_len_w_crc;
integer i;
   begin
      @(posedge RXCLK) #1 RXDV = 1;
      RXD = 8'hx5;
      for (i=0;i<15;i=i+1) @(posedge RXCLK) begin end
      #1 RXD = 8'hxd;
      @(posedge RXCLK) begin end
      for (i=0; i<pkt_len_w_crc; i=i+1) begin
         #1;
         //$display($time, " rx byte %04x", rx_packet_buffer[i]);
         //$display($time," rx packets finished at index %d.", packet_index);
         RXD[3:0] = rx_packet_buffer[i][3:0];
         @(posedge RXCLK) begin end
         #1;
         RXD[3:0] = rx_packet_buffer[i][7:4];
         @(posedge RXCLK) begin end
      end
      rmii_crs_dv_toggle = 0;
      #1 RXDV = 0;
      RXD = 8'hx;
      // IFG
      for (i=0; i<12; i=i+1) begin
         @(posedge RXCLK) begin end
      end
   end
endtask

task send_rmii_rx_pkt;
input [31:0] pkt_len_w_crc;
integer i, crsdv_toggling;
   begin
      crsdv_toggling = 0;
      @(posedge RXCLK) #1 RXDV = 1;
      RXD = 8'bxxxxxx01;
      for (i=0;i<31;i=i+1) @(posedge RXCLK) begin end
      #1 RXD = 8'bxxxxxx11;
      @(posedge RXCLK) begin end
      for (i=0; i<pkt_len_w_crc; i=i+1) begin
         if (i > pkt_len_w_crc - 8)
           crsdv_toggling = 1;
         //$display($time, " rx byte %04x", rx_packet_buffer[i]);
         //$display($time," rx packets finished at index %d.", packet_index);
         #1; 
         RXDV = crsdv_toggling ? 0 : 1;
         RXD[1:0] = rx_packet_buffer[i][1:0]; 
         @(posedge RXCLK) begin end
         #1; 
         RXDV = 1;
         RXD[1:0] = rx_packet_buffer[i][3:2]; 
         @(posedge RXCLK) begin end
         #1; 
         RXDV = crsdv_toggling ? 0 : 1;
         RXD[1:0] = rx_packet_buffer[i][5:4]; 
         @(posedge RXCLK) begin end
         #1; 
         RXD[1:0] = rx_packet_buffer[i][7:6]; 
         RXDV = 1;
         @(posedge RXCLK) begin end
         if (i > pkt_len_w_crc - 8)
           rmii_crs_dv_toggle = 1;
         else
           rmii_crs_dv_toggle = 0;
      end
      rmii_crs_dv_toggle = 0;
      #1 RXDV = 0;
      RXD = 8'hx;
      // IFG
      for (i=0; i<12; i=i+1) begin
         @(posedge RXCLK) begin end
      end
   end
endtask

task handle_outputs;
   begin
      while (1) begin
         wait (TXEN == 1'b1);
         if (rgmii | gmii) recv_gmii_tx_pkt;
         else if (rmii) recv_rmii_tx_pkt;
         else recv_mii_tx_pkt;
      end
   end

endtask // handle_egress

task recv_gmii_tx_pkt;
reg [7:0] data;
integer   i;
reg       seeing_data;  // as opposed to preamble
reg error;
   begin
      seeing_data = 0;
      error = 0;
      i = 0;
      //$display($time, " Starting packet transmission.");
      while (TXEN)
        begin
           @(posedge GTXCLK)
             data[7:0] = TXD[7:0];
           if (TXER) begin
              error = 1;
              //$display($time, " MAC cancelled transmission via TXER.");
           end
           if (!error) begin
              if (seeing_data)
                begin
                   if (TXEN) begin
                      tx_packet_buffer[i] = data;
                      i = i + 1;
                   end
                end
              else begin
                 if (data == 8'hd5) seeing_data = 1;
                 else if (data != 8'h55)
                   $display($time, " ERROR %m : expected preamble but saw %2x", data);
              end
           end
        end
      if (!error) handle_tx_packet(i);
   end
endtask

task recv_mii_tx_pkt;
reg [7:0] data;
integer   i;
reg       seeing_data;  // as opposed to preamble
   begin
      seeing_data = 0;
      i = 0;
      //$display($time, " Starting packet transmission.");
      while (TXEN)
        begin
           @(posedge TXCLK)
             data[3:0] = TXD[3:0];
           @(posedge TXCLK)
             data[7:4] = TXD[3:0];
           if (seeing_data)
             begin
                if (TXEN) begin
                   tx_packet_buffer[i] = data;
                   i = i + 1;
                end
             end
           else begin
              if (data == 8'hd5) seeing_data = 1;
              else if (data != 8'h55)
                $display("%t ERROR %m : expected preamble but saw %2x", $time,data);
           end
        end

      handle_tx_packet(i);
   end
endtask

task recv_rmii_tx_pkt;
reg [7:0] data;
integer   i;
integer   preamble_length;
reg       seeing_data;  // as opposed to preamble
  begin
    seeing_data = 0;
    preamble_length = 0;
    i = 0;
    //$display($time, " Starting packet transmission.");
    while (TXEN) begin
      @(posedge TXCLK) data[1:0] = TXD[1:0];
      @(posedge TXCLK) data[3:2] = TXD[1:0];
      @(posedge TXCLK) data[5:4] = TXD[1:0];
      @(posedge TXCLK) data[7:6] = TXD[1:0];
      if (seeing_data) begin
        if (TXEN) begin
          tx_packet_buffer[i] = data;
          i = i + 1;
        end
      end else begin
        if (data == 8'hd5) begin
          seeing_data = 1;
          preamble_length = preamble_length + 1;
        end else if (data != 8'h55)
          $display("%t ERROR %m : expected preamble but saw %2x", $time,data);
        else
          preamble_length = preamble_length + 1;
      end
    end
    handle_tx_packet(i);
  end
endtask

///////////////////////////////////////////////////////////////////
// Board just transmitted a packet - we need to write it out to the file.
// Egress packet is in nibbles in tx_packet_buffer[]
// Number of nibbles is in counter.

task handle_tx_packet;

input [31:0] byte_len;  // includes CRC
integer i,j;
integer byte_len_no_crc;
reg [31:0] tmp, calc_crc,pkt_crc;


   begin

      // We're not going to put the transmitted CRC in the packet, but tell the user what
      // it was in case they need to know. (So put it in an XML comment)

      pkt_crc = {tx_packet_buffer[byte_len-4],
                 tx_packet_buffer[byte_len-3],
                 tx_packet_buffer[byte_len-2],
                 tx_packet_buffer[byte_len-1]};

      $display($time, " Transmitted packet: Full length = %0d. CRC was %2x %2x %2x %2x",
               byte_len,
               tx_packet_buffer[byte_len-4],
               tx_packet_buffer[byte_len-3],
               tx_packet_buffer[byte_len-2],
               tx_packet_buffer[byte_len-1]);

      // Now drop the CRC
      byte_len_no_crc = byte_len - 4;

      // Now check that the CRC was correct
      for (i=byte_len_no_crc; i<byte_len; i=i+1) tx_packet_buffer[i] = 'h0;

      tmp = tx_update_crc(32'hffffffff,byte_len_no_crc)^32'hffffffff;
      calc_crc = {tmp[7:0],tmp[15:8],tmp[23:16],tmp[31:24]};

      if (calc_crc !== pkt_crc)
        $display("%t ERROR: Observed CRC was %8x but calculated CRC was %8x",
                 $time, pkt_crc, calc_crc);

      // OK write the packet out.
      $write("\t\t\t");
      for (i=0;i<byte_len_no_crc;i=i+1) begin
         $write("%2x ", tx_packet_buffer[i]);
         if ((i % 16 == 15)) begin
            $write("\n");
            if (i != byte_len_no_crc-1) $write("\t\t\t");
         end
      end
      if (byte_len_no_crc % 16 != 0) $write("\n");
   end
endtask // handle_tx_packet


////////////////////////////////////////////////////////////////
// Check integrity of input file read
//    Format of memory data is:
//   0000003c // len = 60 (not including CRC)
//   00000000 // earliest send time MSB (ns)
//   00000001 // earliest send time LSB
//   aa000000 // packet data starting at DA
//   ...
//   24252627 // end of data
//   eeeeffff // token indicates end of packet
////////////////////////////////////////////////////////////////
task check_integrity;
integer pkt_count, i, words;
reg [31:0] len;
time time2send;
reg [63:0] tmp64;
reg [15:0] rep;
time repdelay;
begin
  #1 pkt_count = 0; // #1 is done so that time format is set.
  i = 0;
  while ((input_file[i] !== 32'hxxxxxxxx) && (input_file[i] !== 32'h0)) begin
    len = input_file[i];
    if (len <14) $display("%m Warning: packet length %0d is < 14", len);
    if (len >1518) $display("%m Warning: packet length %0d is > 1518", len);
    tmp64 = {input_file[i+1],input_file[i+2]};
    rep = tmp64[63:48];
    repdelay = {48'h0, tmp64[47:32]};
    if (rep == 0) rep = 1;
    pkt_count = pkt_count + rep;
    words = (len-1)/4 + 1;
    i = i + words + 3;
    if (input_file[i] !== input_pkt_seperator) begin
      $display("%m Error : expected to see %x at word %0d but saw %x",
               input_pkt_seperator, i, input_file[i]);
    end
    i=i+1;
  end
  $display($time," There will be %0d input packets.", pkt_count);
end
endtask

task gen_crc_table;
reg [31:0] c;
integer n, k;
   begin
      for (n = 0; n < 256; n = n + 1) begin
         c = n;
         for (k = 0; k < 8; k = k + 1) begin
            if (c & 1)
	      c = 32'hedb88320 ^ (c >> 1);
            else
              c = c >> 1;
         end
         crc_table[n] = c;
      end
   end
endtask

function [31:0] update_crc;
input [31:0]crc;
input [31:0] len;
reg [31:0] c;
integer i, n;
   begin
      c = crc;
      for (n = 0; n < len; n = n + 1) begin
         i = ( c ^ rx_packet_buffer[n] ) & 8'hff;
         c = crc_table[i] ^ (c >> 8);
      end
      update_crc = c;

   end
endfunction
////////////////////////////////////////////////////////////
// CRC generation function. Invert CRC when finished.
// REPEATED HERE FOR USE ON THE TX BUFFER!!! Gee pointers would be nice....
function [31:0]  tx_update_crc;
input [31:0] crc;
input [31:0] len;
reg [31:0] c;
integer i, n;
   begin

      c = crc;
      for (n = 0; n < len; n = n + 1) begin
         i = ( c ^ tx_packet_buffer[n] ) & 8'hff;
         c = crc_table[i] ^ (c >> 8);
      end
      tx_update_crc = c;

   end
endfunction // tx_update_crc

endmodule

`ifdef FAKE_PHY_TEST
module fake_phy_tb;

// PHY interface
reg [7:0] txd;
reg tx_en;
reg gtx_clk;
reg tx_er;
wire tx_clk;
wire rx_clk;
wire [7:0] rxd;
wire rx_er;
wire rx_dv;

// we don't need to generate a clock here because the phy does
// but we do need a reset
reg reset_n;
initial begin
   $dumpfile("fake_phy_tb.vcd");
   $dumpvars;
   reset_n = 1'b0;
   #6 reset_n = 1'b1;
end

integer i;
initial begin
   txd = 0;
   tx_en = 0;
   tx_er = 0;
   #100;
   @(posedge tx_clk) begin end
   tx_en = 1;
   for(i=0; i<15; i=i+1) begin
      txd[3:0] = 4'h5;
      @(posedge tx_clk) begin #1; end
   end
   txd[3:0] = 4'hd;
   @(posedge tx_clk) begin #1; end
   for(i=0; i<6; i=i+1) begin
      txd[3:0] = i;
      @(posedge tx_clk) begin #1; end
      txd[3:0] = i + 10;
      @(posedge tx_clk) begin #1; end
   end
   for(i=5; i>=0; i=i-1) begin
      txd[3:0] = i;
      @(posedge tx_clk) begin #1; end
      txd[3:0] = i + 10;
      @(posedge tx_clk) begin #1; end
   end
   txd[3:0] = 4'h8;
   @(posedge tx_clk) begin #1; end
   txd[3:0] = 4'h0;
   @(posedge tx_clk) begin #1; end
   txd[3:0] = 4'h0;
   @(posedge tx_clk) begin #1; end
   txd[3:0] = 4'h0;
   @(posedge tx_clk) begin #1; end
   for(i=0; i<46; i=i+1) begin
      txd[3:0] = i[3:0];
      @(posedge tx_clk) begin #1; end
      txd[3:0] = i[7:4];
      @(posedge tx_clk) begin #1; end
   end
   txd[3:0] = 4'h1;
   @(posedge tx_clk) begin #1; end
   txd[3:0] = 4'hb;
   @(posedge tx_clk) begin #1; end
   txd[3:0] = 4'hb;
   @(posedge tx_clk) begin #1; end
   txd[3:0] = 4'he;
   @(posedge tx_clk) begin #1; end
   txd[3:0] = 4'h2;
   @(posedge tx_clk) begin #1; end
   txd[3:0] = 4'hf;
   @(posedge tx_clk) begin #1; end
   txd[3:0] = 4'h5;
   @(posedge tx_clk) begin #1; end
   txd[3:0] = 4'h7;
   @(posedge tx_clk) begin #1; end
   tx_en = 0;
end

fake_phy
  #(.gmii(0))
U_fake_phy
  (.TXD(txd), .TXEN(tx_en), .GTXCLK(gtx_clk), .TXER(tx_er), .TXCLK(tx_clk),
   .RXCLK(rx_clk), .RXD(rxd), .RXER(rx_er), .RXDV(rx_dv),
   .RESET_N(reset_n));

endmodule
`endif
