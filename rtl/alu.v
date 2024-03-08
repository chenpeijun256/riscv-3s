`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/19/2023 08:45:06 AM
// Design Name: 
// Module Name: alu
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
//`include "config.v"

module alu (
    //LSU master cmd to vrb
    output o_vrb_cmd_valid,
    input i_vrb_cmd_ready,
    output [`AW-1:0] o_vrb_cmd_addr, 
    output o_vrb_cmd_read, 
    output [`DW-1:0] o_vrb_cmd_wdata,
    output [`DW/8-1:0] o_vrb_cmd_wmask,
    //LSU master response from vrb
    input i_vrb_rsp_valid,
    output o_vrb_rsp_ready,
    input i_vrb_rsp_err,
    input [`DW-1:0] i_vrb_rsp_rdata,
    
    //reg file
    input [`DW-1:0] i_read_reg1_data,
    input [`DW-1:0] i_read_reg2_data,
    output [`DW-1:0] o_write_data,
    
    //exe ctrl
    input i_instr_illegal,
    input [`AW-1:0] i_pc_cur,
    
    input i_imm_en,
    input [`DW-1:0] i_imm,
    
    input [`IS_SYS:`IS_LUI] i_instr_type,
    input [`IS_SRAI:`IS_ADDI] i_instr_math_i,
    input [`IS_AND:`IS_ADD] i_instr_math,
    input [`IS_BGEU:`IS_BEQ] i_instr_jb,
    input [`IS_LHU:`IS_LB] i_instr_load,
    input [`IS_SW:`IS_SB] i_instr_store,
    input [`IS_CSR_RCI:`IS_ECALL] i_instr_sys,

    output o_jump_valid,
    output [`AW-1:0] o_jump_pc,

    input  clk,
    input  rst_n
);
    wire [`DW-1:0] math_i_write_data;
    wire [`DW-1:0] math_write_data;
    wire [`DW-1:0] load_write_data;
    
    wire [`DW-1:0] srai_t;
    wire [`DW-1:0] sra_t;
    
    wire [`AW-1:0] load_store_addr;
    wire [`DW-1:0] load_data_h;
    wire [`DW-1:0] load_data_b;
    wire [`DW-1:0] store_data_h;
    wire [`DW/8-1:0] store_mask_h;
    wire [`DW-1:0] store_data_b;
    wire [`DW/8-1:0] store_mask_b;
    
    assign srai_t = ($signed(i_read_reg1_data) >>> i_imm[4:0]);
    assign sra_t = ($signed(i_read_reg1_data) >>> i_read_reg2_data[4:0]);

    assign math_i_write_data = (i_instr_math_i[`IS_ADDI]) ? (i_read_reg1_data + i_imm) :(
                                  (i_instr_math_i[`IS_SLTI]) ? (($signed(i_read_reg1_data) < $signed(i_imm)) ? 32'h1 : 32'h0) : (
                                  (i_instr_math_i[`IS_SLTIU]) ? ((i_read_reg1_data < i_imm) ? 32'h1 : 32'h0) : (
                                  (i_instr_math_i[`IS_XORI]) ? (i_read_reg1_data ^ i_imm) : (
                                  (i_instr_math_i[`IS_ORI]) ? (i_read_reg1_data | i_imm) : (
                                  (i_instr_math_i[`IS_ANDI]) ? (i_read_reg1_data & i_imm) : (
                                  (i_instr_math_i[`IS_SLLI]) ? (i_read_reg1_data << i_imm[4:0]) : (
                                  (i_instr_math_i[`IS_SRLI]) ? (i_read_reg1_data >> i_imm[4:0]) : (
                                  (i_instr_math_i[`IS_SRAI]) ? (srai_t) : (0)))))))));
    assign math_write_data = (i_instr_math[`IS_ADD]) ? (i_read_reg1_data + i_read_reg2_data) :(
                              (i_instr_math[`IS_SUB]) ? (i_read_reg1_data - i_read_reg2_data) : (
                              (i_instr_math[`IS_SLT]) ? (($signed(i_read_reg1_data) < $signed(i_read_reg2_data)) ? 32'h1 : 32'h0) : (
                              (i_instr_math[`IS_SLTU]) ? ((i_read_reg1_data < i_read_reg2_data) ? 32'h1 : 32'h0) : (
                              (i_instr_math[`IS_XOR]) ? (i_read_reg1_data ^ i_read_reg2_data) : (
                              (i_instr_math[`IS_OR]) ? (i_read_reg1_data | i_read_reg2_data) : (
                              (i_instr_math[`IS_AND]) ? (i_read_reg1_data & i_read_reg2_data) : (
                              (i_instr_math[`IS_SLL]) ? (i_read_reg1_data << i_read_reg2_data[4:0]) : (
                              (i_instr_math[`IS_SRL]) ? (i_read_reg1_data >> i_read_reg2_data[4:0]) : (
                              (i_instr_math[`IS_SRA]) ? (sra_t) : (0))))))))));

    wire is_jb_valid = (i_instr_jb[`IS_BEQ]) ? (i_read_reg1_data == i_read_reg2_data) : (
                            (i_instr_jb[`IS_BNE]) ? (i_read_reg1_data != i_read_reg2_data) : (
                            (i_instr_jb[`IS_BLT]) ? ($signed(i_read_reg1_data) < $signed(i_read_reg2_data)) : (
                            (i_instr_jb[`IS_BGE]) ? ($signed(i_read_reg1_data) >= $signed(i_read_reg2_data)) : (
                            (i_instr_jb[`IS_BLTU]) ? (i_read_reg1_data < i_read_reg2_data) : (
                            (i_instr_jb[`IS_BGEU]) ? (i_read_reg1_data >= i_read_reg2_data) : (0))))));
    assign o_jump_valid = (i_instr_type[`IS_JALR] | i_instr_type[`IS_JAL]) ? (1'b1) : (
                            (i_instr_type[`IS_JB]) ? (is_jb_valid) : (0));
    assign o_jump_pc = (i_instr_type[`IS_JAL] | i_instr_type[`IS_JB]) ? (i_imm + i_pc_cur) : (
                        (i_instr_type[`IS_JALR]) ? (i_imm + i_read_reg1_data) : (0));

    assign store_data_h = (load_store_addr[1]) ? ((i_read_reg2_data<<16) & 32'hffff0000) : (i_read_reg2_data & 32'hffff);
    assign store_mask_h = 3<<load_store_addr[1:0];
    assign store_data_b = (load_store_addr[1:0] == 2'b11)? ((i_read_reg2_data<<24) & 32'hff000000) : (
                          (load_store_addr[1:0] == 2'b10) ? ((i_read_reg2_data<<16) & 32'hff0000) : (
                          (load_store_addr[1:0] == 2'b01) ? ((i_read_reg2_data<<8) & 32'hff00) : (i_read_reg2_data & 32'hff)));
    assign store_mask_b = 1<<load_store_addr[1:0];
    
    assign load_store_addr = i_read_reg1_data + i_imm;

    assign load_data_h = load_store_addr[1] ? (i_vrb_rsp_rdata>>16) : (i_vrb_rsp_rdata & 32'hffff);
    assign load_data_b = (load_store_addr[1:0] == 2'b11) ? ((i_vrb_rsp_rdata>>24) & 32'hff):  (
                          (load_store_addr[1:0] == 2'b10) ? ((i_vrb_rsp_rdata>>16) & 32'hff) : (
                          (load_store_addr[1:0] == 2'b01)  ? ((i_vrb_rsp_rdata>>8) & 32'hff) : (i_vrb_rsp_rdata & 32'hff)));
    assign load_write_data = (i_instr_load[`IS_LW]) ? (i_vrb_rsp_rdata) : (
                            (i_instr_load[`IS_LH]) ? ({{16{load_data_h[15]}}, load_data_h[15:0]}) : (
                            (i_instr_load[`IS_LHU]) ? (load_data_h) : (
                            (i_instr_load[`IS_LB]) ? ({{24{load_data_b[7]}}, load_data_b[7:0]}) : (
                            (i_instr_load[`IS_LBU]) ? (load_data_b) : (0)))));
    assign o_write_data = (i_instr_type[`IS_MATH_I]) ? (math_i_write_data) : (
                            (i_instr_type[`IS_MATH]) ? (math_write_data) : (
                            (i_instr_type[`IS_LOAD]) ? (load_write_data) : (
                            (i_instr_type[`IS_LUI]) ? (i_imm) : (
                            (i_instr_type[`IS_AUIPC]) ? (i_imm + i_pc_cur) : (
                            (i_instr_type[`IS_JALR] | i_instr_type[`IS_JAL]) ? (3'h4 + i_pc_cur) : (0))))));

    assign o_vrb_cmd_valid = i_instr_type[`IS_LOAD] | i_instr_type[`IS_STORE];
    assign o_vrb_cmd_addr = load_store_addr;
    assign o_vrb_cmd_read = i_instr_type[`IS_LOAD];
    assign o_vrb_cmd_wdata = i_instr_store[`IS_SW] ? (i_read_reg2_data) : (
                                i_instr_store[`IS_SH] ? (store_data_h) : (
                                i_instr_store[`IS_SB] ? (store_data_b) : (0)));
    assign o_vrb_cmd_wmask = i_instr_store[`IS_SW] ? (4'hf) : (
                              i_instr_store[`IS_SH] ? (store_mask_h) : (
                              i_instr_store[`IS_SB] ? (store_mask_b) : (0)));
    assign o_vrb_rsp_ready = 1'b1;

endmodule
