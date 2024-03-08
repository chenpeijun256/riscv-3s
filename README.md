# riscv-3s
RISC-V core with 3 stage pipeline

1st stage: instruction fetch and decode.<br>
2nd stage: data fetch and execution.<br>
3rd stage: write back.

## get started:

**OS：** <br>
Ubuntu 22.04

**install:** <br>
sudo apt install iverilog gtkwave

**download code:** <br>
git clone https://github.com/chenpeijun256/riscv-3s.git

**make:** <br>
cd riscv-3s/iver <br>
make <br>
make clean <br>

after make done, no failed case found, it's ok. <br>
```
total case: 38, failed: 0, success: 38.
```