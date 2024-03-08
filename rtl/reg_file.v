`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/14/2023 02:02:21 PM
// Design Name: 
// Module Name: reg_file
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

module reg_file (
    input [`RAW-1:0] i_write_reg,
    input [`DW-1:0] i_write_data,
    input i_write_en,
    
    input i_read_reg1_en,
    input [`RAW-1:0] i_read_reg1,
    output [`DW-1:0] o_read_data1,
    
    input i_read_reg2_en,
    input [`RAW-1:0] i_read_reg2,
    output [`DW-1:0] o_read_data2,

    input  clk,
    input  rst_n
);
    localparam DP = (2**`RAW); 
    
    reg [`DW-1:0] reg_file [1:DP-1];
    
    integer i;
    
    always @(posedge clk) begin
        if (rst_n == 1'b0) begin
            for (i = 1; i < DP; i = i +1) begin
                reg_file[i] <= 32'h0;
            end
        end
        else if (i_write_en && (i_write_reg != 0)) begin
            reg_file[i_write_reg] <= #1 i_write_data;
        end
        else begin
            ;
        end
    end
    
    assign o_read_data1 = (!i_read_reg1_en | i_read_reg1== 0) ? 0 : reg_file[i_read_reg1];
    assign o_read_data2 = (!i_read_reg2_en | i_read_reg2== 0) ? 0 : reg_file[i_read_reg2];

endmodule
