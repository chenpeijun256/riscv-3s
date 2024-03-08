`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/22/2023 03:00:08 PM
// Design Name: 
// Module Name: soc_top
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

module cpu_ctrl (
    input  [`AW-1:0] i_reset_pc,
    
    //IFU master cmd to vrb
    output o_ifu_vrb_cmd_valid,
    input i_ifu_vrb_cmd_ready,
    output [`AW-1:0] o_ifu_vrb_cmd_addr, 
    output o_ifu_vrb_cmd_read, 
    output [`DW-1:0] o_ifu_vrb_cmd_wdata,
    output [`DW/8-1:0] o_ifu_vrb_cmd_wmask,
    //IFU master response from vrb
    input i_ifu_vrb_rsp_valid,
    output o_ifu_vrb_rsp_ready,
    input i_ifu_vrb_rsp_err,
    input [`DW-1:0] i_ifu_vrb_rsp_rdata,
    
    //LSU master cmd to vrb
    output o_lsu_vrb_cmd_valid,
    input i_lsu_vrb_cmd_ready,
    output [`AW-1:0] o_lsu_vrb_cmd_addr, 
    output o_lsu_vrb_cmd_read, 
    output [`DW-1:0] o_lsu_vrb_cmd_wdata,
    output [`DW/8-1:0] o_lsu_vrb_cmd_wmask,
    //LSU master response from vrb
    input i_lsu_vrb_rsp_valid,
    output o_lsu_vrb_rsp_ready,
    input i_lsu_vrb_rsp_err,
    input [`DW-1:0] i_lsu_vrb_rsp_rdata,
    
    input clk, 
    input rst_n      // async reset
);
    /////////////////////////////////////////
    wire jump_valid;
    wire [`AW-1:0] jump_pc;
    //////////////////////////////////////////////////////
    wire [`RAW-1:0] write_reg;
    wire [`DW-1:0] write_reg_data;
    wire write_reg_en;
    
    wire read_reg1_en;
    wire read_reg2_en;
    wire [`RAW-1:0] read_reg1;
    wire [`RAW-1:0] read_reg2;
    wire [`DW-1:0] read_reg1_data;
    wire [`DW-1:0] read_reg2_data;
    
    reg_file u_reg_file (
        .i_write_reg(write_reg),
        .i_write_data(write_reg_data),
        .i_write_en(write_reg_en),
        
        .i_read_reg1_en(read_reg1_en),
        .i_read_reg1(read_reg1),
        .o_read_data1(read_reg1_data),
        
        .i_read_reg2_en(read_reg2_en),
        .i_read_reg2(read_reg2),
        .o_read_data2(read_reg2_data),
    
        .clk(clk),
        .rst_n(rst_n)
    );

    ////////////////////////////////////////
    ifu u_ifu (
        .i_reset_pc(i_reset_pc),
        .i_jump_valid(jump_valid),
        .i_jump_pc(jump_pc),
        
        .o_cmd_valid(o_ifu_vrb_cmd_valid),
        .i_cmd_ready(i_ifu_vrb_cmd_ready),
        .o_cmd_pc(o_ifu_vrb_cmd_addr),
        
        .i_rsp_valid(i_ifu_vrb_rsp_valid),
        .o_rsp_ready(o_ifu_vrb_rsp_ready),
        .i_rsp_err(i_ifu_vrb_rsp_err),
//        .i_rsp_instr(i_ifu_vrb_rsp_rdata),
        
        .clk(clk),
        .rst_n(rst_n)
    );
    assign o_ifu_vrb_cmd_read = 1'b1;
    assign o_ifu_vrb_cmd_wdata = 0;
    assign o_ifu_vrb_cmd_wmask = 0;
    
    //////////////////////////////////////////
    wire dec_instr_illegal;
    
    wire dec_rs_1_en;
    wire [`RAW-1:0] dec_rs_1;
    wire dec_rs_2_en;
    wire [`RAW-1:0] dec_rs_2;
    wire dec_rd_en;
    wire [`RAW-1:0] dec_rd;
    wire dec_imm_en;
    wire [`DW-1:0] dec_imm;
    
    wire [`IS_SYS:`IS_LUI] dec_instr_type;
    wire [`IS_SRAI:`IS_ADDI] dec_instr_math_i;
    wire [`IS_AND:`IS_ADD] dec_instr_math;
    wire [`IS_BGEU:`IS_BEQ] dec_instr_jb;
    wire [`IS_LHU:`IS_LB] dec_instr_load;
    wire [`IS_SW:`IS_SB] dec_instr_store;
    wire [`IS_CSR_RCI:`IS_ECALL] dec_instr_sys;
    
    dec u_dec (
        .i_instr_data(i_ifu_vrb_rsp_rdata),
        .i_jump_valid(jump_valid),
        
        .o_instr_illegal(dec_instr_illegal),
        
        .o_rs_1_en(dec_rs_1_en),
        .o_rs_1(dec_rs_1),
        .o_rs_2_en(dec_rs_2_en),
        .o_rs_2(dec_rs_2),
        .o_rd_en(dec_rd_en),
        .o_rd(dec_rd),
        .o_imm_en(dec_imm_en),
        .o_imm(dec_imm),

        .o_instr_type(dec_instr_type),
        .o_instr_math_i(dec_instr_math_i),
        .o_instr_math(dec_instr_math),
        .o_instr_jb(dec_instr_jb),
        .o_instr_load(dec_instr_load),
        .o_instr_store(dec_instr_store),
        .o_instr_sys(dec_instr_sys),
        
        .clk(clk),
        .rst_n(rst_n)
    );
    //DEC -- ALU Reg////////////////////////////////////////
    wire alu_instr_illegal;
    wire [`AW-1:0] alu_pc;

    wire alu_rs_1_en;
    wire [`RAW-1:0] alu_rs_1;
    wire alu_rs_2_en;
    wire [`RAW-1:0] alu_rs_2;
    wire alu_rd_en;
    wire [`RAW-1:0] alu_rd;
    wire alu_imm_en;
    wire [`DW-1:0] alu_imm;

    wire [`IS_SYS:`IS_LUI] alu_instr_type;
    wire [`IS_SRAI:`IS_ADDI] alu_instr_math_i;
    wire [`IS_AND:`IS_ADD] alu_instr_math;
    wire [`IS_BGEU:`IS_BEQ] alu_instr_jb;
    wire [`IS_LHU:`IS_LB] alu_instr_load;
    wire [`IS_SW:`IS_SB] alu_instr_store;
    wire [`IS_CSR_RCI:`IS_ECALL] alu_instr_sys;
    
    pipe # (
        .DATAW(1+`AW+1+`RAW+1+`RAW+1+`RAW+1+`DW+(`IS_SYS-`IS_LUI+1)+(`IS_SRAI-`IS_ADDI+1)
                +(`IS_AND-`IS_ADD+1)+(`IS_BGEU-`IS_BEQ+1)+(`IS_LHU-`IS_LB+1)
                +(`IS_SW-`IS_SB+1)+(`IS_CSR_RCI-`IS_ECALL+1))
    ) u_dec_to_alu (
        .enable(1'b1),
        .data_in({dec_instr_illegal, o_ifu_vrb_cmd_addr, dec_rs_1_en, 
                    dec_rs_1, dec_rs_2_en, dec_rs_2,
                    dec_rd_en, dec_rd, dec_imm_en,
                    dec_imm, dec_instr_type, 
                    dec_instr_math_i, dec_instr_math, dec_instr_jb,
                    dec_instr_load, dec_instr_store, dec_instr_sys}),
        .data_out({alu_instr_illegal, alu_pc, alu_rs_1_en,
                    alu_rs_1, alu_rs_2_en, alu_rs_2,
                    alu_rd_en, alu_rd, alu_imm_en,
                    alu_imm, alu_instr_type, 
                    alu_instr_math_i, alu_instr_math, alu_instr_jb,
                    alu_instr_load, alu_instr_store, alu_instr_sys}),
        
        .clk(clk),
        .rst((rst_n==0) | jump_valid)
    );

    //////////////////////////////////////////////////////
    
    assign read_reg1_en = alu_rs_1_en;
    assign read_reg1 = alu_rs_1;
    assign read_reg2_en = alu_rs_2_en;
    assign read_reg2 = alu_rs_2;
    assign write_reg_en = alu_rd_en;
    assign write_reg = alu_rd;
    
    alu u_alu (
        .i_read_reg1_data(read_reg1_data),
        .i_read_reg2_data(read_reg2_data),
        .o_write_data(write_reg_data),
        
        .i_instr_illegal(alu_instr_illegal),//(i_pipe_clean | i_instr_illegal | i_instr_fetch_err)
        .i_pc_cur(alu_pc),
        
        .i_imm_en(alu_imm_en),
        .i_imm(alu_imm),

        .i_instr_type(alu_instr_type),
        .i_instr_math_i(alu_instr_math_i),
        .i_instr_math(alu_instr_math),
        .i_instr_jb(alu_instr_jb),
        .i_instr_load(alu_instr_load),
        .i_instr_store(alu_instr_store),
        .i_instr_sys(alu_instr_sys),
        
        .o_jump_valid(jump_valid),
        .o_jump_pc(jump_pc),
        
        //LSU master cmd to vrb
        .o_vrb_cmd_valid(o_lsu_vrb_cmd_valid),
        .i_vrb_cmd_ready(i_lsu_vrb_cmd_ready),
        .o_vrb_cmd_addr(o_lsu_vrb_cmd_addr), 
        .o_vrb_cmd_read(o_lsu_vrb_cmd_read), 
        .o_vrb_cmd_wdata(o_lsu_vrb_cmd_wdata),
        .o_vrb_cmd_wmask(o_lsu_vrb_cmd_wmask),
        //LSU master response from vrb
        .i_vrb_rsp_valid(i_lsu_vrb_rsp_valid),
        .o_vrb_rsp_ready(o_lsu_vrb_rsp_ready),
        .i_vrb_rsp_err(i_lsu_vrb_rsp_err),
        .i_vrb_rsp_rdata(i_lsu_vrb_rsp_rdata),
    
        .clk(clk),
        .rst_n(rst_n)
    );
    ///////////////////////////////////////////////////////
endmodule
