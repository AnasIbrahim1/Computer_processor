/*******************************************************************
*
* Module: Shifter.v
* Project: Processor
* Authors: Anas A. Ibrahim - anas2@aucegypt.edu, Ibrahim Gohar - 
abdelmaksou@aucegypt.edu

* Description: processor instruction memory
*
* Change history: 06/11/2022 - Was typed down
*
**********************************************************************/
`include "defines.v"
module Shifter(
    input[31:0] a, 
    input[4:0] shamt, 
    input[1:0] type, 
    output reg[31:0] r
);

    always @(*) begin
        if (type == `SHIFTER_SRL)
            r = $unsigned(a) >> shamt;
        else if (type == `SHIFTER_SLL)
            r = a << shamt;
        else if (type == `SHIFTER_SRA)
            r = $signed(a) >>> shamt;
        else 
            r = a;   
    end
endmodule
