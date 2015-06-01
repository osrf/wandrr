`timescale 1ns/1ns
module usb_crc5_serial
(input c, // clock
 input r, // reset
 input d, // data bit input
 input dv, // data-valid input
 output [4:0] crc);

wire [4:0] next_crc = // polynomial: x^5 + x^2 + 1
{
  /*
  crc[3],
  crc[2],
  crc[1] ^ crc[4] ^ d,
  crc[0],
  crc[4] ^ d
  */

  crc[0] ^ d,
  crc[4],
  crc[3] ^ crc[0] ^ d,
  crc[2],
  crc[1]
};
r #(5, 5'h1f) lfsr(.c(c), .en(dv), .rst(r), .d(next_crc), .q(crc));

endmodule

/*
  // this gives a big-endian ordering, which isn't what we want.
  */

