`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/22/2024 08:35:05 AM
// Design Name: 
// Module Name: pipe
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


module pipe # (
    parameter DATAW  = 1, 
    parameter DEPTH  = 1
) (
    input enable,
    input [DATAW-1:0] data_in,
    output [DATAW-1:0] data_out,
    
    input clk,
    input rst
);
    if (DEPTH == 0) begin        
        assign data_out = data_in;  
    end else if (DEPTH == 1) begin        
        reg [DATAW-1:0] value;

        always @(posedge clk) begin
            if (rst) begin
                value <= #1 DATAW'(0);
            end
            else if (enable) begin
                value <= #1 data_in;
            end
            else begin
                ;
            end
        end
        assign data_out = value;
    end else begin
        wire [DEPTH:0][DATAW-1:0] data_delayed;        
        assign data_delayed[0] = data_in;
        for (genvar i = 1; i <= DEPTH; ++i) begin
            pipe #(
                .DATAW  (DATAW)
            ) pipe_reg (
                .clk      (clk),
                .rst      (rst),
                .enable   (enable),
                .data_in  (data_delayed[i-1]),
                .data_out (data_delayed[i])
            );
        end
        assign data_out = data_delayed[DEPTH];
    end
endmodule
