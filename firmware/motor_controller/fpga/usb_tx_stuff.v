`timescale 1ns/1ns
module usb_tx_stuff
(input  c,
 input  d,
 input  d_empty,
 output d_req,
 output q,
 input  q_req,
 output q_empty);

localparam ST_IDLE     = 4'd0;
localparam ST_READ     = 4'd1;
localparam ST_STUFF    = 4'd2;
localparam ST_DONE     = 4'd3;

localparam SW=4, CW=5;
reg [CW+SW-1:0] ctrl;
wire [SW-1:0] state;
wire [SW-1:0] next_state = ctrl[SW+CW-1:CW];
r #(SW) state_r
(.c(c), .rst(1'b0), .en(1'b1), .d(next_state), .q(state));

wire [2:0] ones_count;

wire [2:0] done_cnt; // hold the DONE state for a full two bits for downstream
wire done_cnt_rst;
r #(3) done_cnt_r
(.c(c), .rst(done_cnt_rst), .en(1'b1), .d(done_cnt+1'b1), .q(done_cnt));


// todo: clean this up. it got ugly.
always @* begin
  case (state)
    ST_IDLE:
      if (~d_empty)              ctrl = { ST_READ , 5'b00000 };
      else                       ctrl = { ST_IDLE , 5'b00000 };
    ST_READ: 
      if (d_empty)               ctrl = { ST_DONE , 5'b00010 };
      else if (q_req)
        if (ones_count == 3'd5 & d == 1'b1)  
                                 ctrl = { ST_STUFF, 5'b00110 };
        else                     ctrl = { ST_READ , 5'b00011 };
      else                       ctrl = { ST_READ , 5'b00000 };
    ST_STUFF:
      if (d_empty)               ctrl = { ST_DONE , 5'b00011 };
      else if (q_req)            ctrl = { ST_READ , 5'b00001 };
      else                       ctrl = { ST_STUFF, 5'b00100 };
    ST_DONE:
      if (done_cnt == 3'h7)      ctrl = { ST_IDLE , 5'b00000 };
      else                       ctrl = { ST_DONE , 5'b00000 };
    default:                     ctrl = { ST_IDLE , 5'b00000 };
  endcase
end

assign d_req = ctrl[0];
assign done_cnt_rst = ctrl[1];
wire supress_empty = ctrl[2];

r #(3, 3'd0) ones_count_r
(.c(c), .rst(state == ST_IDLE | (q_req & d == 1'b0) | state == ST_STUFF), 
 .en(q_req & d == 1'b1),
 .d(ones_count + 1'b1), .q(ones_count));

wire nrzi;
wire next_nrzi = (state == ST_STUFF) ? ~nrzi : (d ? nrzi : ~nrzi);

r #(1, 1'b1) nrzi_r
(.c(c), .rst(state == ST_IDLE), .en(state != ST_IDLE & q_req),
 .d(next_nrzi), .q(nrzi));

assign q = nrzi; 

assign q_empty = d_empty & ~supress_empty; 

endmodule
