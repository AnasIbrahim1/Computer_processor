/*******************************************************************
*
* Module: RegFile.v
* Project: Processor
* Authors: Anas A. Ibrahim - anas2@aucegypt.edu, Ibrahim Gohar - 
abdelmaksou@aucegypt.edu

* Description: processor register file
*
* Change history: 06/11/2022 - Added from Lab 6 and polished
*
**********************************************************************/
module RegFile (
    input clk,
    input rst,
    input RegWrite,
    input[4 : 0] read_reg_1,
    input[4 : 0] read_reg_2,
    input[4 : 0] write,
    input[31 : 0] write_data,
    output[31 : 0] read_data_1,
    output[31 : 0] read_data_2
);

    // initialize register file
    wire[31 : 0] q[31 : 0];
    reg[31 : 0] load;
    // write
    integer j;
    always @(posedge clk) begin
        if (rst) load <= 0;
        else begin
            load[write] = RegWrite;
            for (j = 0; j < 32; j = j + 1)
                if (j != write || j == 0) 
                    load[j] = 0;     
        end
    end

    genvar i;
    generate 
        for (i = 0; i < 32; i = i + 1) begin: block2
            NBitRegister #(32) nBitRegister(clk, rst, load[i], write_data, q[i]);
        end 
    endgenerate
    
    // read
    assign read_data_1 = q[read_reg_1];
    assign read_data_2 = q[read_reg_2];
endmodule