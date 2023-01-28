/*******************************************************************
*
* Module: FullAdder.v
* Project: Processor
* Authors: Anas A. Ibrahim - anas2@aucegypt.edu, Ibrahim Gohar - 
abdelmaksou@aucegypt.edu

* Description: standard full adder circuit
*
* Change history: 06/11/2022 - Added from Lab 6 and polished
*
**********************************************************************/
module FullAdder (
    input a, 
    input b, 
    input cin, 
    output sum, 
    output cout
);

    wire c;
    assign c = a ^ b;
    assign sum = c ^ cin;
    assign cout = (c & cin) | (a & b);
endmodule
