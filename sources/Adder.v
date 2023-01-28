/*******************************************************************
*
* Module: Adder.v
* Project: Processor
* Authors: Anas A. Ibrahim - anas2@aucegypt.edu, Ibrahim Gohar - 
abdelmaksou@aucegypt.edu

* Description: n-bit ripple carry adder
*
* Change history: 06/11/2022 - Added from Lab 6 and polished
*
**********************************************************************/
module Adder #(parameter n = 32) (
    input[n - 1 : 0] a,
    input[n - 1 : 0] b,
    input cin,
    output[n - 1 : 0] s,
    output cout
);
    
    genvar i;

    wire[n - 1 : 0] b_prime;
    assign b_prime = (cin ? ~b : b);

    wire[n : 0] c;
    assign c[0] = cin;
    generate 
        for (i = 0; i < n; i = i + 1) begin: my_block
            FullAdder fullAdder(a[i], b_prime[i], c[i], s[i], c[i + 1]);
        end
    endgenerate

    assign cout = c[n];
endmodule