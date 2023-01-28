/*******************************************************************
*
* Module: ALUControl.v
* Project: Processor
* Authors: Anas A. Ibrahim - anas2@aucegypt.edu, Ibrahim Gohar - 
abdelmaksou@aucegypt.edu

* Description: processor ALU control
*
* Change history: 06/11/2022 - Added from Lab 6 and polished
                  06/11/2022 - Changed the design, added more instructions and more options according to 
                  with the help of defines.v
*
**********************************************************************/
`include "defines.v"

module ALUControl (
    input[2 : 0] ALUOp,
    input[2 : 0] funct3,
    input inst30,
    output reg[3 : 0] ALUSel
);

    always @(*) begin
        // R - Type
        if (ALUOp == `ALUOp_LW_SW) ALUSel = `ALU_LW_SW;
        else if (ALUOp == `ALUOp_Rtype && funct3 == `F3_ADD  && inst30 == 1'b0) ALUSel = `ALU_ADD;
        else if (ALUOp == `ALUOp_Rtype && funct3 == `F3_ADD  && inst30 == 1'b1) ALUSel = `ALU_SUB;
        else if (ALUOp == `ALUOp_Rtype && funct3 == `F3_SLL  && inst30 == 1'b0) ALUSel = `ALU_SLL;
        else if (ALUOp == `ALUOp_Rtype && funct3 == `F3_SLT  && inst30 == 1'b0) ALUSel = `ALU_SLT;
        else if (ALUOp == `ALUOp_Rtype && funct3 == `F3_SLTU && inst30 == 1'b0) ALUSel = `ALU_SLTU;
        else if (ALUOp == `ALUOp_Rtype && funct3 == `F3_XOR  && inst30 == 1'b0) ALUSel = `ALU_XOR; 
        else if (ALUOp == `ALUOp_Rtype && funct3 == `F3_SRL  && inst30 == 1'b0) ALUSel = `ALU_SRL; 
        else if (ALUOp == `ALUOp_Rtype && funct3 == `F3_SRL  && inst30 == 1'b1) ALUSel = `ALU_SRA; 
        else if (ALUOp == `ALUOp_Rtype && funct3 == `F3_OR   && inst30 == 1'b0) ALUSel = `ALU_OR; 
        else if (ALUOp == `ALUOp_Rtype && funct3 == `F3_AND  && inst30 == 1'b0) ALUSel = `ALU_AND;
        // I - Type
        else if (ALUOp == `ALUOp_Itype && funct3 == `F3_ADD ) ALUSel = `ALU_ADD;
        else if (ALUOp == `ALUOp_Itype && funct3 == `F3_SLL ) ALUSel = `ALU_SLL;
        else if (ALUOp == `ALUOp_Itype && funct3 == `F3_SLT ) ALUSel = `ALU_SLT;
        else if (ALUOp == `ALUOp_Itype && funct3 == `F3_SLTU) ALUSel = `ALU_SLTU;
        else if (ALUOp == `ALUOp_Itype && funct3 == `F3_XOR ) ALUSel = `ALU_XOR; 
        else if (ALUOp == `ALUOp_Itype && funct3 == `F3_SRL  && inst30 == 1'b0) ALUSel = `ALU_SRL; 
        else if (ALUOp == `ALUOp_Itype && funct3 == `F3_SRL  && inst30 == 1'b1) ALUSel = `ALU_SRA; 
        else if (ALUOp == `ALUOp_Itype && funct3 == `F3_OR ) ALUSel = `ALU_OR; 
        else if (ALUOp == `ALUOp_Itype && funct3 == `F3_AND) ALUSel = `ALU_AND;
        else if (ALUOp == `ALUOp_Btype) ALUSel = `ALU_SUB;
        // JAL and JALR and AUIPC - Type
        else if (ALUOp == `ALUOp_JALR) ALUSel = `ALU_ADD;
        // LUI - Type
        else if (ALUOp == `ALUOp_LUI) ALUSel = `ALU_PASS;
        else ALUSel = `ALU_PASS;
    end
    
endmodule
