`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/01/15 09:20:52
// Design Name: 
// Module Name: cpu_ctrl_tb
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
// `include "../rtl/config.v"


module cpu_ctrl_tb(
);
    
   integer FN;

    integer ts_cycle[46: 0];
    string ts_filename[46: 0];

    string ts_dir = "../isa/";

    initial begin
        $dumpfile("cpu_ctrl_tb.vcd");
        $dumpvars(0, cpu_ctrl_tb);

        FN = 0;
        ts_filename[FN] = {ts_dir, "rv32ui-p-add.verilog"};   ts_cycle[FN] = 550;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-addi.verilog"};  ts_cycle[FN] = 300;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-and.verilog"};   ts_cycle[FN] = 550;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-andi.verilog"};  ts_cycle[FN] = 250;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-auipc.verilog"}; ts_cycle[FN] = 100;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-beq.verilog"};   ts_cycle[FN] = 400;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-bge.verilog"};   ts_cycle[FN] = 450;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-bgeu.verilog"};  ts_cycle[FN] = 500;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-blt.verilog"};   ts_cycle[FN] = 400;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-bltu.verilog"};  ts_cycle[FN] = 450;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-bne.verilog"};   ts_cycle[FN] = 400;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-jal.verilog"};   ts_cycle[FN] = 100;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-jalr.verilog"};  ts_cycle[FN] = 150;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-lb.verilog"};    ts_cycle[FN] = 300;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-lbu.verilog"};   ts_cycle[FN] = 300;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-lh.verilog"};    ts_cycle[FN] = 300;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-lhu.verilog"};   ts_cycle[FN] = 300;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-lui.verilog"};   ts_cycle[FN] = 100;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-lw.verilog"};    ts_cycle[FN] = 300;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-or.verilog"};    ts_cycle[FN] = 550;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-ori.verilog"};   ts_cycle[FN] = 250;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-sb.verilog"};    ts_cycle[FN] = 500;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-sh.verilog"};    ts_cycle[FN] = 550;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-simple.verilog"};ts_cycle[FN] = 50;    FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-sll.verilog"};   ts_cycle[FN] = 600;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-slli.verilog"};  ts_cycle[FN] = 300;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-slt.verilog"};   ts_cycle[FN] = 550;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-slti.verilog"};  ts_cycle[FN] = 250;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-sltiu.verilog"}; ts_cycle[FN] = 300;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-sltu.verilog"};  ts_cycle[FN] = 550;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-sra.verilog"};   ts_cycle[FN] = 600;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-srai.verilog"};  ts_cycle[FN] = 300;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-srl.verilog"};   ts_cycle[FN] = 600;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-srli.verilog"};  ts_cycle[FN] = 300;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-sub.verilog"};   ts_cycle[FN] = 550;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-sw.verilog"};    ts_cycle[FN] = 550;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-xor.verilog"};   ts_cycle[FN] = 550;   FN = FN + 1;
        ts_filename[FN] = {ts_dir, "rv32ui-p-xori.verilog"};  ts_cycle[FN] = 250;   FN = FN + 1;
        // ts_filename[FN] = {ts_dir, "rv32um-p-mul.verilog"};   ts_cycle[FN] = 550;   FN = FN + 1;
        // ts_filename[FN] = {ts_dir, "rv32um-p-mulh.verilog"};  ts_cycle[FN] = 550;   FN = FN + 1;
        // ts_filename[FN] = {ts_dir, "rv32um-p-mulhsu.verilog"};ts_cycle[FN] = 550;   FN = FN + 1;
        // ts_filename[FN] = {ts_dir, "rv32um-p-mulhu.verilog"}; ts_cycle[FN] = 550;   FN = FN + 1;
        // ts_filename[FN] = {ts_dir, "rv32um-p-divu.verilog"};  ts_cycle[FN] = 1000;  FN = FN + 1;
        // ts_filename[FN] = {ts_dir, "rv32um-p-div.verilog"};   ts_cycle[FN] = 1000;  FN = FN + 1;
        // ts_filename[FN] = {ts_dir, "rv32um-p-rem.verilog"};   ts_cycle[FN] = 1000;  FN = FN + 1;
        // ts_filename[FN] = {ts_dir, "rv32um-p-remu.verilog"};  ts_cycle[FN] = 1000;  FN = FN + 1;
        // ts_filename[FN] = {ts_dir, "rv32ui-p-fence_i.verilog"};ts_cycle[FN] = 100;  FN = FN + 1;
    end

    localparam ROM_AW = `CFG_ROM_ADDR_WIDTH;
    localparam RAM_AW = `CFG_RAM_ADDR_WIDTH;
    localparam ROM_LEN = (2**ROM_AW);
    localparam RAM_LEN = (2**RAM_AW);
    localparam DAT_LEN = (ROM_LEN + RAM_LEN);
    reg [7:0] data[0:DAT_LEN-1];
    
    reg clk;
    reg rst_n;
    
    //VRB(valid ready bus) master cmd from LSU
    wire lsu_vrb_cmd_valid;
    reg lsu_vrb_cmd_ready;
    wire [`AW-1:0]   lsu_vrb_cmd_addr; 
    wire lsu_vrb_cmd_read; 
    wire [`DW-1:0] lsu_vrb_cmd_wdata;
    wire [`DW/8-1:0] lsu_vrb_cmd_wmask;
    //VRB(valid ready bus) master response to LSU
    reg lsu_vrb_rsp_valid;
    wire lsu_vrb_rsp_ready;
    reg lsu_vrb_rsp_err;
    wire [`DW-1:0] lsu_vrb_rsp_rdata;
    wire [`DW-1:0] lsu_vrb_rsp_rdata_t;
    
    //VRB(valid ready bus) master cmd from IFU
    wire ifu_vrb_cmd_valid;
    reg ifu_vrb_cmd_ready;
    wire [`AW-1:0]   ifu_vrb_cmd_addr; 
    wire ifu_vrb_cmd_read; 
    wire [`DW-1:0] ifu_vrb_cmd_wdata;
    wire [`DW/8-1:0] ifu_vrb_cmd_wmask;
    //VRB(valid ready bus) master response to IFU
    reg ifu_vrb_rsp_valid;
    wire ifu_vrb_rsp_ready;
    reg ifu_vrb_rsp_err;
    wire [`DW-1:0] ifu_vrb_rsp_rdata;
    wire [`DW-1:0] ifu_vrb_rsp_rdata_t;
    
    integer k;
    integer failed_case = 0;

    always begin 
        #5 clk <= ~clk;
    end
 
    always @ (posedge clk) begin
        if (lsu_vrb_cmd_valid & ~lsu_vrb_cmd_read) begin
            if(lsu_vrb_cmd_wmask[0])
                data[{lsu_vrb_cmd_addr[`AW-1:2], {2'b00}}] <= lsu_vrb_cmd_wdata[7:0];
            if(lsu_vrb_cmd_wmask[1])
                data[{lsu_vrb_cmd_addr[`AW-1:2], {2'b01}}] <= lsu_vrb_cmd_wdata[15:8];
            if(lsu_vrb_cmd_wmask[2])
                data[{lsu_vrb_cmd_addr[`AW-1:2], {2'b10}}] <= lsu_vrb_cmd_wdata[23:16];
            if(lsu_vrb_cmd_wmask[3])
                data[{lsu_vrb_cmd_addr[`AW-1:2], {2'b11}}] <= lsu_vrb_cmd_wdata[31:24];
        end
    end
    
    assign ifu_vrb_rsp_rdata_t = {data[{ifu_vrb_cmd_addr[`AW-1:2], {2'b11}}], 
                                    data[{ifu_vrb_cmd_addr[`AW-1:2], {2'b10}}],
                                    data[{ifu_vrb_cmd_addr[`AW-1:2], {2'b01}}], 
                                    data[{ifu_vrb_cmd_addr[`AW-1:2], {2'b00}}]};
    assign ifu_vrb_rsp_rdata = ifu_vrb_cmd_read ? ifu_vrb_rsp_rdata_t : 0;
    assign lsu_vrb_rsp_rdata_t = {data[{lsu_vrb_cmd_addr[`AW-1:2], {2'b11}}], 
                                    data[{lsu_vrb_cmd_addr[`AW-1:2], {2'b10}}],
                                    data[{lsu_vrb_cmd_addr[`AW-1:2], {2'b01}}], 
                                    data[{lsu_vrb_cmd_addr[`AW-1:2], {2'b00}}]};
    assign lsu_vrb_rsp_rdata = lsu_vrb_cmd_read ? lsu_vrb_rsp_rdata_t : 0;
    
    initial begin
        clk = 1'b0;
        ifu_vrb_cmd_ready = 1;
        ifu_vrb_rsp_valid = 1;
        ifu_vrb_rsp_err = 0;
        
        lsu_vrb_cmd_ready = 1;
        lsu_vrb_rsp_valid = 1;
        lsu_vrb_rsp_err = 0;
        
        for (k = 0; k < FN; k=k+1) begin
            rst_n = 1'b0;
            
            $display("test file:%0s.",ts_filename[k]);
            $readmemh(ts_filename[k], data);
            #20;
            rst_n = 1'b1;
            
            #(10*(ts_cycle[k]));
            
            if (u_cpu_ctrl.u_reg_file.reg_file[27] == 0) begin
                $display("failed.");
                failed_case = failed_case + 1;
            end
            else begin
                $display("passed.");
            end
        end
        
        $display("total case: %0d, failed: %0d, success: %0d.", FN, failed_case, FN - failed_case);
        $finish(1);
    end
    
    cpu_ctrl u_cpu_ctrl (
        .i_reset_pc(32'h00000000),
        
        .o_ifu_vrb_cmd_valid(ifu_vrb_cmd_valid),
        .i_ifu_vrb_cmd_ready(ifu_vrb_cmd_ready),
        .o_ifu_vrb_cmd_addr(ifu_vrb_cmd_addr), 
        .o_ifu_vrb_cmd_read(ifu_vrb_cmd_read), 
        .o_ifu_vrb_cmd_wdata(ifu_vrb_cmd_wdata),
        .o_ifu_vrb_cmd_wmask(ifu_vrb_cmd_wmask),

        .i_ifu_vrb_rsp_valid(ifu_vrb_rsp_valid),
        .o_ifu_vrb_rsp_ready(ifu_vrb_rsp_ready),
        .i_ifu_vrb_rsp_err(ifu_vrb_rsp_err),
        .i_ifu_vrb_rsp_rdata(ifu_vrb_rsp_rdata),
        
        //LSU master cmd to vrb
        .o_lsu_vrb_cmd_valid(lsu_vrb_cmd_valid),
        .i_lsu_vrb_cmd_ready(lsu_vrb_cmd_ready),
        .o_lsu_vrb_cmd_addr(lsu_vrb_cmd_addr), 
        .o_lsu_vrb_cmd_read(lsu_vrb_cmd_read), 
        .o_lsu_vrb_cmd_wdata(lsu_vrb_cmd_wdata),
        .o_lsu_vrb_cmd_wmask(lsu_vrb_cmd_wmask),
        //LSU master response from vrb
        .i_lsu_vrb_rsp_valid(lsu_vrb_rsp_valid),
        .o_lsu_vrb_rsp_ready(lsu_vrb_rsp_ready),
        .i_lsu_vrb_rsp_err(lsu_vrb_rsp_err),
        .i_lsu_vrb_rsp_rdata(lsu_vrb_rsp_rdata),
        
        .clk(clk),
        .rst_n(rst_n)
    );
endmodule
