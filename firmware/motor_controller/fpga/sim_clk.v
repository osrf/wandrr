`timescale 1ps/1ps
module sim_clk
#(parameter MHZ=1)
(output reg clk);

realtime t_half_cycle;

initial begin
  t_half_cycle = 1000000 / MHZ / 2;
end

initial begin
  clk = 0;
  forever begin
    #t_half_cycle clk = 0;
    #t_half_cycle clk = 1;
  end
end

endmodule
