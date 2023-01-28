/*******************************************************************
*
* Module: BranchControl.v
* Project: Processor
* Authors: Anas A. Ibrahim - anas2@aucegypt.edu, Ibrahim Gohar - 
abdelmaksou@aucegypt.edu

* Description: Used to control the different kind of flags when branching
*
* Change history: 06/11/2022 - Coded down 
*
**********************************************************************/
`include "defines.v"
module BranchControlUnit (
    input Branch,
    input[2 : 0] funct3,
    input cf,
    input zf,
    input vf,
    input sf,
    output reg[1 : 0] signal
);

    always @(*) begin
        if (funct3 == `BR_BEQ) signal <= zf;
        else if (funct3 == `BR_BNE) signal <= ~zf;
        else if (funct3 == `BR_BLT) signal <= (sf != vf);
        else if (funct3 == `BR_BGE) signal <= (sf == vf);
        else if (funct3 == `BR_BLTU) signal = ~cf;
        else if (funct3 == `BR_BGEU) signal = cf;    
        else signal <= zf;
    end
endmodule
