module sim_mlvds_xcvr(input di, input de, output ro, inout bus);

assign bus = de ? di : 1'bz;
tri0 bus_pulldown = bus;
assign ro  = bus_pulldown;

endmodule
