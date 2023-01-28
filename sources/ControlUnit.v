/*******************************************************************
*
* Module: ControlUnit.v
* Project: Processor
* Authors: Anas A. Ibrahim - anas2@aucegypt.edu, Ibrahim Gohar - 
abdelmaksou@aucegypt.edu

* Description: processor control unit
*
* Change history: 06/11/2022 - Added from Lab 6 and polished
                  06/11/2022 - Change of design and adding new features to suppose new instructions
                  06/11/2022 - Repetitively added gate and wire-level detail to support different instructions
*
**********************************************************************/
`include "defines.v"

module ControlUnit (
    input[4 : 0] opcode,
    output reg Branch,
    output reg MemRead,
    output reg [1 : 0] MemtoReg,
    output reg [2 : 0] ALUOp,
    output reg MemWrite,
    output reg ALUSrc,
    output reg RegWrite,
    output reg [1 : 0] branch_signal
);
    
    always @(*) begin
        Branch <= (opcode == `OPCODE_Branch);
        MemRead <= (opcode == `OPCODE_Load);
        if (opcode === `OPCODE_Load)
            MemtoReg <= `MemToRegALU;
        else if (opcode == `OPCODE_JALR || opcode == `OPCODE_JAL)
            MemtoReg <= `MemToRegJAL;
        else if (opcode == `OPCODE_AUIPC)
            MemtoReg <= `MemToRegAUIPC;
        else 
            MemtoReg <= `MemToRegMemo;
        
        
        if (opcode == `OPCODE_JAL)
            branch_signal <= `BranchJAL;
        else if (opcode == `OPCODE_JALR)
            branch_signal <= `BranchJALR;
        else if (opcode == `OPCODE_FENCE || opcode == `OPCODE_SYSTEM)
            branch_signal <= `BranchSYSTEM;
        else
            branch_signal <= `BranchNormal;

        if (opcode == `OPCODE_Arith_R) 
            ALUOp <= `ALUOp_Rtype;
        else if (opcode == `OPCODE_Branch)
            ALUOp <= `ALUOp_Btype;
        else if (opcode == `OPCODE_Arith_I)
            ALUOp <= `ALUOp_Itype;
        else if (opcode == `OPCODE_JALR || opcode == `OPCODE_JAL || opcode == `OPCODE_AUIPC)
            ALUOp <= `ALUOp_JALR;
        else 
            ALUOp <= `ALUOp_LW_SW;

        MemWrite <= (opcode == `OPCODE_Store) ? 1 : 0;
        ALUSrc <= (opcode == `OPCODE_Load | opcode == `OPCODE_Store | 
        opcode == `OPCODE_Arith_I | opcode == `OPCODE_LUI | opcode == `OPCODE_JALR) ? 1 : 0;
        RegWrite <= (opcode == `OPCODE_Arith_R | opcode == `OPCODE_Load | 
        opcode == `OPCODE_Arith_I | opcode == `OPCODE_JAL | opcode == `OPCODE_JALR | 
        opcode == `OPCODE_LUI | opcode == `OPCODE_AUIPC) ? 1 : 0;
    end
    
endmodule
