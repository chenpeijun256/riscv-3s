`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/08/2023 09:41:05 AM
// Design Name: 
// Module Name: ifu
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

module ifu (
    input  [`AW-1:0] i_reset_pc,
    input i_jump_valid,
    input [`AW-1:0] i_jump_pc,
    
    //    * IFetch CMD channel
    output o_cmd_valid, // Handshake valid
    input  i_cmd_ready, // Handshake ready
    output [`AW-1:0] o_cmd_pc, // Fetch PC
    
    //    * IFetch RSP channel
    input  i_rsp_valid, // Response valid 
    output o_rsp_ready, // Response ready
    input  i_rsp_err,   // Response error
//    input  [DW-1:0] i_rsp_instr, // Response instruction
    
    input clk,
    input rst_n
);
    
    reg [`AW-1:0] pc_cur_r;
    reg reset_done;
    wire rsp_succ;
    
    assign o_cmd_pc = pc_cur_r;
    assign o_cmd_valid = 1'b1;//(rst_n == 1'b0) ? 1'b0 : 1'b1;
    
    assign o_rsp_ready = 1'b1;//(rst_n == 1'b0) ? 1'b0 : 1'b1;
    
    assign rsp_succ = i_rsp_valid & (~i_rsp_err);
    
    always @(posedge clk) begin 
        if (rst_n == 1'b0) begin
            pc_cur_r <= #1 i_reset_pc;
            reset_done <= 1'b0;
        end
        else begin
            reset_done <= 1'b1;
            if (i_jump_valid) begin
                pc_cur_r <= #1 i_jump_pc;
            end
            else if (rsp_succ & reset_done) begin
                pc_cur_r <= #1 pc_cur_r + 3'h4;
            end
            else begin
                pc_cur_r <= pc_cur_r;
            end
        end
        
    end
endmodule
