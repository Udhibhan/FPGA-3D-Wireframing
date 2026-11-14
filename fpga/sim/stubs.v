// Simulation stand-ins for the ECP5 oscillator and PLL
`timescale 1ns/1ps
module OSCG(output reg OSC); parameter DIV = "6"; initial OSC = 0; always #6.667 OSC = ~OSC; endmodule
module ecp5_oled_pll(input clki, output clko, output reg locked);
  assign clko = clki; initial begin locked = 0; #200 locked = 1; end
endmodule
