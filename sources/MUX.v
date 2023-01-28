/*******************************************************************
*
* Module: MUX.v
* Project: Processor
* Authors: Anas A. Ibrahim - anas2@aucegypt.edu, Ibrahim Gohar - 
abdelmaksou@aucegypt.edu

* Description: n-bit 2 by 1 multiplexer, where a is when the select is 0 and b is when the select is 1
*
* Change history: 06/11/2022 - Added from Lab 6 and polished
*
**********************************************************************/

module MUX #(parameter n = 32) (
    input[n - 1 : 0] a, 
    input[n - 1 : 0] b, 
    input sel, 
    output[n - 1 : 0] c
);
    assign c = (sel ? b : a);
endmodule