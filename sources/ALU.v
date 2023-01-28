/*******************************************************************
*
* Module: ALU.v
* Project: Processor
* Authors: Anas A. Ibrahim - anas2@aucegypt.edu, Ibrahim Gohar - 
abdelmaksou@aucegypt.edu

* Description: processor main ALU
*
* Change history: 06/11/2022 - Added from project support codes on BB
*
**********************************************************************/
`include "defines.v"

module ALU (
	input   wire [31 : 0] a, b,
	input   wire [4 : 0]  shamt,
	output  reg  [31 : 0] r,
	output  wire          cf, zf, vf, sf,
	input   wire [3 : 0]  alufn
);

    wire [31 : 0] add, op_b;
    
    assign op_b = (~b);
    
    assign {cf, add} = alufn[0] ? (a + op_b + 1'b1) : (a + b);
    
    assign zf = (add == 0);
    assign sf = add[31];
    assign vf = (a[31] ^ (op_b[31]) ^ add[31] ^ cf);
    
    wire[31 : 0] sh;
    Shifter shifter0(.a(a), .shamt(shamt), .type(alufn[1:0]),  .r(sh));
    
    always @ * begin
        r = 0;
        (* parallel_case *)
        case (alufn)
            // arithmetic
            `ALU_ADD    : r = add;
            `ALU_SUB    : r = add;
            `ALU_LW_SW  : r = add;
            `ALU_PASS   : r = b;
            // logic
            `ALU_OR     :  r = a | b;
            `ALU_AND    :  r = a & b;
            `ALU_XOR    :  r = a ^ b;
            // shift
            `ALU_SRL    :  r = sh;
            `ALU_SLL    :  r = sh;
            `ALU_SRA    :  r = sh;
            // slt & sltu
            `ALU_SLT    :  r = {31'b0,(sf != vf)}; 
            `ALU_SLTU   :  r = {31'b0,(~cf)};    
            default     :  r = add;        	
        endcase
    end
endmodule