/*******************************************************************
*
* Module: ImmGen.v
* Project: Processor
* Authors: Anas A. Ibrahim - anas2@aucegypt.edu, Ibrahim Gohar - 
abdelmaksou@aucegypt.edu

* Description: processor immediate generator 
*
* Change history: 06/11/2022 - Added from project support codes on BB
                  06/11/2022 - Added Load, SYSTEM, and edited other shifting to accomodate the 
                  instructions
*
**********************************************************************/
`include "defines.v"

module ImmGen (
    input  wire [31 : 0]  IR,
    output reg  [31 : 0]  imm
);

    always @(*) begin
        case (`OPCODE)
            `OPCODE_Arith_I   : 	imm = { {21{IR[31]}}, IR[30:25], IR[24:21], IR[20] };
            `OPCODE_Load      : 	imm = { {21{IR[31]}}, IR[30:25], IR[24:21], IR[20] };
            `OPCODE_Store     :     imm = { {21{IR[31]}}, IR[30:25], IR[11:8], IR[7] };
            `OPCODE_LUI       :     imm = { IR[31], IR[30:20], IR[19:12], 12'b0 };
            `OPCODE_AUIPC     :     imm = { 1'b0, IR[31], IR[30:20], IR[19:12], 11'b0 }; // because it will be shifted 1 again
            `OPCODE_JAL       : 	imm = { 1'b0, {12{IR[31]}}, IR[19:12], IR[20], IR[30:25], IR[24:21] }; // because it will be shifted one later
            `OPCODE_JALR      : 	imm = { {21{IR[31]}}, IR[30:25], IR[24:21], IR[20] };
            `OPCODE_SYSTEM    : 	imm = { {21{IR[31]}}, IR[30:25], IR[24:21], IR[20] };
            `OPCODE_Branch    : 	imm = { {20{IR[31]}}, IR[7], IR[30:25], IR[11:8], 1'b0};
            default           : 	imm = { {21{IR[31]}}, IR[30:25], IR[24:21], IR[20] }; // IMM_I
        endcase 
    end
    
endmodule