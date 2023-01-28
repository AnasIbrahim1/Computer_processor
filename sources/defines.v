`define     IR_rs1                  19:15
`define     IR_rs2                  24:20
`define     IR_rd                   11:7
`define     IR_opcode               6:2
`define     IR_funct3               14:12
`define     IR_funct7               31:25
`define     IR_shamt                24:20
`define     IR_funct7bit            30

`define     reg_IFID_pc_4           95:64
`define     reg_IFID_pc             63:32
`define     reg_IFID_inst           31:0

`define     reg_IDEX_rs1            195:191
`define     reg_IDEX_rs2            190:186
`define     reg_IDEX_pc_4           185:154
`define     reg_IDEX_inst_shamt     153:149
`define     reg_IDEX_Branch         148
`define     reg_IDEX_MemRead        147
`define     reg_IDEX_MemWrite       146
`define     reg_IDEX_ALUSrc         145
`define     reg_IDEX_RegWrite       144
`define     reg_IDEX_MemtoReg       143:142
`define     reg_IDEX_branch_cu      141:140
`define     reg_IDEX_ALUOp          139:137
`define     reg_IDEX_pc             136:105
`define     reg_IDEX_read_data_1    104:73
`define     reg_IDEX_read_data_2    72:41 
`define     reg_IDEX_imm            40:9  
`define     reg_IDEX_funct7bit      8     
`define     reg_IDEX_funct3         7:5   
`define     reg_IDEX_inst_rd        4:0

`define     reg_EXMEM_ALUB          212:181
`define     reg_EXMEM_RegWrite      180
`define     reg_EXMEM_inst_rd       179:175
`define     reg_EXMEM_MemtoReg      174:173
`define     reg_EXMEM_MemWrite      172
`define     reg_EXMEM_MemRead       171
`define     reg_EXMEM_imm           170
`define     reg_EXMEM_pc_imm        169:138
`define     reg_EXMEM_Branch        137
`define     reg_EXMEM_funct3        136:134
`define     reg_EXMEM_cf            133
`define     reg_EXMEM_zf            132
`define     reg_EXMEM_vf            131
`define     reg_EXMEM_sf            130
`define     reg_EXMEM_branch_cu     129:128
`define     reg_EXMEM_pc            127:96
`define     reg_EXMEM_alu           95:64
`define     reg_EXMEM_read_data_2   63:32
`define     reg_EXMEM_pc_4          31:0

`define     reg_MEMWB_RegWrite      136
`define     reg_MEMWB_MemRead       135
`define     reg_MEMWB_inst_rd       134:130
`define     reg_MEMWB_MemtoReg      129:128
`define     reg_MEMWB_alu           127:96
`define     reg_MEMWB_mem_read_data 95:64
`define     reg_MEMWB_pc_4          63:32
`define     reg_MEMWB_pc_imm        31:0

`define     reg_inst_mem            161:130

`define     SHIFTER_SRL             2'b00
`define     SHIFTER_SLL             2'b01
`define     SHIFTER_SRA             2'b10

`define     LS_FUNCT3_B             3'b000
`define     LS_FUNCT3_H             3'b001
`define     LS_FUNCT3_W             3'b010
`define     LS_FUNCT3_BU            3'b100
`define     LS_FUNCT3_HU            3'b101

`define     OPCODE_Branch           5'b11_000
`define     OPCODE_Load             5'b00_000
`define     OPCODE_Store            5'b01_000
`define     OPCODE_JALR             5'b11_001
`define     OPCODE_JAL              5'b11_011
`define     OPCODE_Arith_I          5'b00_100
`define     OPCODE_Arith_R          5'b01_100
`define     OPCODE_AUIPC            5'b00_101
`define     OPCODE_LUI              5'b01_101
`define     OPCODE_SYSTEM           5'b11_100 
`define     OPCODE_FENCE            5'b00_011

`define     ALUOp_LW_SW             3'b000
`define     ALUOp_Btype             3'b001
`define     ALUOp_Rtype             3'b010
`define     ALUOp_Itype             3'b011
`define     ALUOp_LUI               3'b100
`define     ALUOp_JALR              3'b101

`define     MemToRegMemo            2'b00
`define     MemToRegALU             2'b01
`define     MemToRegJAL             2'b10
`define     MemToRegAUIPC           2'b11

`define     BranchNormal            2'b00
`define     BranchJAL               2'b01
`define     BranchJALR              2'b10
`define     BranchSYSTEM            2'b11

`define     F3_ADD                  3'b000
`define     F3_SLL                  3'b001
`define     F3_SLT                  3'b010
`define     F3_SLTU                 3'b011
`define     F3_XOR                  3'b100
`define     F3_SRL                  3'b101
`define     F3_OR                   3'b110
`define     F3_AND                  3'b111

`define     BR_BEQ                  3'b000
`define     BR_BNE                  3'b001
`define     BR_BLT                  3'b100
`define     BR_BGE                  3'b101
`define     BR_BLTU                 3'b110
`define     BR_BGEU                 3'b111

`define     OPCODE                  IR[`IR_opcode]

`define     ALU_ADD                 4'b00_00
`define     ALU_SUB                 4'b00_01
`define     ALU_LW_SW               4'b00_10
`define     ALU_PASS                4'b00_11
`define     ALU_OR                  4'b01_00
`define     ALU_AND                 4'b01_01
`define     ALU_XOR                 4'b01_11
`define     ALU_SRL                 4'b10_00
`define     ALU_SRA                 4'b10_10
`define     ALU_SLL                 4'b10_01
`define     ALU_SLT                 4'b11_01
`define     ALU_SLTU                4'b11_11

`define     SYS_EC_EB               3'b000

