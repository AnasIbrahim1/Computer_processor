/*******************************************************************
*
* Module: MUXFour.v
* Project: Processor
* Authors: Anas A. Ibrahim - anas2@aucegypt.edu, Ibrahim Gohar - 
abdelmaksou@aucegypt.edu

* Description: MUX between four different inputs with 2 select lines
*
* Change history: 06/11/2022 - Coded down 
*
**********************************************************************/
module MUXFour #(parameter n = 32) (
    input[n - 1 : 0] a_00, 
    input[n - 1 : 0] a_01, 
    input[n - 1 : 0] a_10,
    input[n - 1 : 0] a_11,
    input[1 : 0] sel, 
    output[n - 1 : 0] f
);
    
    assign f = (sel[1] ? (sel[0] ? a_11 : a_10) : (sel[0] ? a_01 : a_00));
endmodule