
RTL_DIR := ../rtl
TB_DIR := ../tb

IVERILOG     := /usr/bin/iverilog

RTL_SRCS := $(RTL_DIR)/config.v \
			$(RTL_DIR)/alu.v \
			$(RTL_DIR)/cpu_ctrl.v \
			$(RTL_DIR)/dec.v \
			$(RTL_DIR)/ifu.v \
			$(RTL_DIR)/pipe.sv \
			$(RTL_DIR)/reg_file.v

.PHONY: cpu_ctrl

cpu_ctrl: cpu_ctrl_tb.vvp

cpu_ctrl_tb.vvp: $(RTL_SRCS) $(TB_DIR)/cpu_ctrl_tb.v
	iverilog -g2005-sv -o cpu_ctrl_tb.vvp $(RTL_SRCS) $(TB_DIR)/cpu_ctrl_tb.v -s cpu_ctrl_tb
	vvp cpu_ctrl_tb.vvp

.PHONY: clean
clean:
	rm -f *.vvp *.vcd