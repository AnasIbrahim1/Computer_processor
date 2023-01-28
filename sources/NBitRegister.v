/*******************************************************************
*
* Module: NBitRegister.v
* Project: Processor
* Authors: Anas A. Ibrahim - anas2@aucegypt.edu, Ibrahim Gohar - 
abdelmaksou@aucegypt.edu
* Description: Register that stores n bits
*
* Change history: 06/11/2022 - Added from Lab 6 and polished
*
**********************************************************************/

module NBitRegister #(parameter n = 32) (
    input clk, 
    input rst, 
    input load, 
    input [n - 1 : 0] D, 
    output reg[n - 1 : 0] Q
);
    always @ (posedge clk) begin
        if (rst) 
            Q <= 0;
        else 
            Q <= (load ? D : Q);
    end
endmodule 