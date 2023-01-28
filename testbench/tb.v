`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/11/2022 11:54:23 AM
// Design Name: 
// Module Name: tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module tb;


reg ssd_clk;
reg clk;
reg rst;
reg[1 : 0] ledSel;
reg[3 : 0] ssdSel;
wire[15 : 0] leds;
wire[12 : 0] ssd;

ProcessorMain mmmmmmmmmmmm(ssd_clk, clk, rst, ledSel, ssdSel, leds, ssd);


initial begin 
rst = 1; #40; rst = 0; 
end

initial begin
ssd_clk = 0; clk = 0; 
ledSel = 2;
ssdSel = 0;
end

always begin 
   clk = ~clk;  #20; 
end

always begin 
   ssd_clk = ~ssd_clk; #5; 
end
endmodule
