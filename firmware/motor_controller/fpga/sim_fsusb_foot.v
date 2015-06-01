`timescale 1ps/1ps
module sim_fsusb_foot
(inout dp,
 inout dm);

localparam HALFBIT = 41667; 
localparam BIT = 83333;

reg vp_noise_inject;
reg vm_noise_inject;
reg vm_noise, vp_noise;

integer delay_amount;

initial begin
  vp_noise = 1'b0;
  vp_noise_inject = 1'b0;
  #1_200_000_000;
  forever begin
    vp_noise = ~vp_noise;
    vp_noise_inject = 1'b0;
    #232238251;
    vp_noise_inject = 1'b1;
    #1895231;
  end
end

initial begin
  vm_noise = 1'b0;
  vm_noise_inject = 1'b0;
  #1_300_000_000;
  forever begin
    vm_noise = ~vm_noise;
    vm_noise_inject = 1'b0;
    #158983124;
    vm_noise_inject = 1'b1;
    #832389;
    vm_noise_inject = 1'b0;
  end
end

reg oe = 1'b0;
reg vp = 1'b1;
reg vm = 1'b0;
assign dp = vp_noise_inject ? vp_noise : (oe ? vp : 1'bz);
assign dm = vm_noise_inject ? vm_noise : (oe ? vm : 1'bz);

integer i;
wire [7:0] sync = 8'b1101_0101;

reg decoded, prev_state, save_bit;
integer byte_count, bit_count, num_rx_ones;
reg [7:0] rx_byte;
reg [7:0] rx_pkt[63:0];

`include "usb_pids.v"
`include "usb_defs.v"

integer tx_num_ones = 0;

/*
task tx_sync;
  integer bit_cnt;
  begin
    vp = 1'b1;
    vm = 1'b0;
    oe = 1'b1;    
    #BIT;
    for (bit_cnt = 0; bit_cnt < 8; bit_cnt = bit_cnt + 1) begin
      vp =  sync[bit_cnt];
      vm = ~sync[bit_cnt];
      #BIT;
    end
    tx_nrzi = 1'b1;
    tx_prev_bit = 1'b1;
    tx_num_ones = 1'b1;
  end
endtask
*/

task tx_byte;
  input [7:0] byte;
  integer bit_cnt;
  reg bit;
  begin
    for (bit_cnt = 0; bit_cnt < 8; bit_cnt = bit_cnt + 1) begin
      bit = byte[bit_cnt]; // save some typing
      if (bit) begin
        tx_num_ones = tx_num_ones + 1'b1;
        if (tx_num_ones >= 7) begin
          // bit stuffing... throw a bit-flip in there
          //nrzi = ~nrzi;
          vp = ~vp;
          vm = ~vm;
          #BIT;
          tx_num_ones = 0;
        end
        #BIT; // to send a "1" we just leave the lines the same
      end else begin
        // to send a "0" we toggle the lines
        vp = ~vp;
        vm = ~vm;
        #BIT;
        tx_num_ones = 0;
      end
    end
  end
endtask

task tx_32bits_be;
  input [31:0] bits;
  begin
    tx_byte(bits[31:24]);
    tx_byte(bits[23:16]);
    tx_byte(bits[15:8]);
    tx_byte(bits[7:0]);
  end
endtask

task tx_eop;
  begin
    vp = 1'b0;
    vm = 1'b0;
    #BIT;
    #BIT;
    vp = 1'b1;
    #BIT;
    oe = 1'b0;
  end
endtask

task tx_warmup;
  begin
    oe = 1'b1;
    vp = 1'b1;
    vm = 1'b0;
    #BIT;
  end
endtask

task rx_data0; 
  integer len;
  begin
    len = byte_count - 3;
    $display("%t rx data0 len %d ", $time, len);
    #(2*BIT);
    tx_warmup();
    tx_byte(USB_SYNC);
    tx_byte(PID_ACK);
    tx_eop();
  end
endtask

integer rx_in_cnt = 0;
task rx_in;
  begin
    $display("%t rx IN", $time);
    #(2*BIT);
    tx_warmup();
    tx_byte(USB_SYNC);
    if (rx_in_cnt == 0) begin
      $display("%t sending NAK", $time);
      tx_byte(PID_NAK);
    end else if (rx_in_cnt == 5) begin
      $display("not transmitting anything in response to this IN request...");
    end else begin
      if (rx_pkt[1] == 8'h91) begin
        $display("%t received IN pkt on EP1", $time);
        tx_byte(PID_DATA1); // TODO: toggle data0 / data1
        tx_foot_pkt();
        /*
        for (i = 0; i < 64; i = i + 1) begin
          //$display("%t tx 0x%02h", $time, 64-i);
          tx_byte(64-i);
        end
        */
      end else if (rx_pkt[1] == 8'h00 | rx_pkt[1] == 8'h01 | rx_pkt[1] == USB_DEV_ADDR) begin
        $display("%t responding to IN pkt on EP0", $time);
        tx_byte(PID_DATA1);
        // for now, always just a zero-length packet
        tx_byte(0);
        tx_byte(0);
      end else begin
        $display("%t unhandled IN request!", $time);
        for (i = 0; i < 64; i = i + 1) begin
          $display("rx %d = 0x%02h", $time, rx_pkt[i]);
          //$display("%t tx 0x%02h", $time, 64-i);
          //tx_byte(64-i);
        end

      end
    end
    tx_eop();
    rx_in_cnt = rx_in_cnt + 1;
  end
endtask

task tx_foot_pkt;
  begin
    tx_32bits_be(32'h0000abcd);
    tx_32bits_be(32'h12345678);
    tx_32bits_be(32'h01000200); // 0
    tx_32bits_be(32'h03000400); // 2
    tx_32bits_be(32'h05000600); // 4
    tx_32bits_be(32'h07000800); // 6
    tx_32bits_be(32'h09000a00); // 8
    tx_32bits_be(32'h0b000c00); // 10
    tx_32bits_be(32'h0d000e00); // 12
    tx_32bits_be(32'h0f001000); // 14
    tx_32bits_be(32'h00000000);
    tx_32bits_be(32'h0);
    tx_32bits_be(32'h0);
    tx_32bits_be(32'h0);
    tx_32bits_be(32'h0);
    tx_32bits_be(32'h0);
 end
endtask

initial begin
  oe = 1'b0;
  vp = 1'b0;
  vm = 1'b0;
  //$printtimescale;
  // wait for USB reset
  wait(~dp && ~dm);
  $display("%t usb reset start", $time);
  wait(dp && ~dm);
  $display("%t usb reset end", $time); // todo: time reset pulse length

  wait(~dp && ~dm);
  $display("%t usb reset2 start", $time);
  wait(dp && ~dm);
  $display("%t usb reset2 end", $time); // todo: time reset pulse length


  forever begin
    num_rx_ones = 0;
    bit_count = 0;
    byte_count = 0;
    rx_byte = 8'h0;
    prev_state = 1;
    decoded = 0;
    save_bit = 1; // this is set to 0 when we get a stuffed bit
    wait(~dp);
    //$display("%t fsusb pkt start", $time);
    #41667; // shift to the middle of a bit period
    for (i = 0; i < 8; i = i + 1) begin
      //$display("%t sync bit", $time);
      if (dp != ~dm) begin
        $display("illegal usb state at %t", $time);
        //#1000 $finish();
      end
      if (sync[i] != dm) begin
        $display("sync fail at %t", $time);
        //#1000 $finish();
      end
      #83333; // skip over a full bit
    end 
    //$display("%t sync OK", $time);
    // decode the NRZI data
    while (~(dp == 0 & dm == 0)) begin
      if (dp != ~dm) begin
        $display("%t illegal usb state", $time);
        //#1000 $finish();
      end
      if (prev_state != dm) begin
        decoded = 0;
        prev_state = dm;
        if (num_rx_ones == 6) begin
          $display("%t stuffed bit detected", $time);
          save_bit = 0; // it's a stuffed bit; ignore it
        end
        else
          save_bit = 1;
        num_rx_ones = 0;
        //$display("%t rx flip @ %d", $time, bit_count);
      end else begin
        decoded = 1;
        num_rx_ones = num_rx_ones + 1;
        save_bit = 1; // ones are never stuffed
        if (num_rx_ones > 6) begin
          $display("%t received more than 6 ones in a row.", $time);
          //#100000 $finish();
        end
        //$display("%t rx same @ %d", $time, bit_count);
      end

      if (save_bit) begin
        rx_byte = { decoded, rx_byte[7:1] };
        bit_count = bit_count + 1;
      end

      if (bit_count == 8) begin
        rx_pkt[byte_count] = rx_byte;
        if (byte_count == 0) begin
          case (rx_byte)
            PID_SOF:   $display("%t SOF",   $time);
            PID_SETUP: $display("%t SETUP", $time);
            PID_DATA0: $display("%t DATA0", $time);
            PID_IN   : $display("%t IN", $time);
            PID_ACK  : $display("%t ACK", $time);
            default: begin
              $display("%t ERROR: rx unknown PID (0x%02h)", $time, rx_byte);
              //$finish();
            end
          endcase
        end
        else
          $display("%t rx     0x%02h", $time, rx_byte);
        byte_count = byte_count + 1;
        bit_count = 0;
      end
      #83333; // skip over a full bit
    end

    if (num_rx_ones == 6) begin
      $display("%t expected to see bit-stuffing right before SE0", $time);
      //#100000 $finish();
    end

    if (bit_count != 0) begin
      $display("%t SE0 seen at non-byte boundary", $time);
      //#5000 $finish();
    end
    //$display("%t found SE0", $time);
    #83333;
    if (dp != 0 | dm != 0) begin
      $display("%t SE0 state wasn't two bits long", $time);
      //#1000 $finish();
    end
    #83333;
    if (dp != 1 | dm != 0) begin
      $display("%t didn't finish EOP with J state", $time);
      //#1000 $finish();
    end
    #83.333;
    $display("%t packet RX complete", $time);
    case (rx_pkt[0])
      PID_SOF:   ;
      PID_SETUP: ;
      PID_DATA0: rx_data0();
      PID_IN:    rx_in();
      PID_ACK:   ;
      default: begin
        $display("%t unknown rx PID (0x%02h)", $time, rx_pkt[0]);
        //$finish();
      end
    endcase
    //if (rx_pkt[0] == PID_DATA0)
    //  rx_data0();
  end
end

endmodule
