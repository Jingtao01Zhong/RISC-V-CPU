# RISC-V Pipeline CPU
RISC-V CPU design using Verilog HDL. 
This project is one of the lab assignment(also the hardest one) of the course: Computer Arhitecture(EE312) at KAIST(Korea Advance Institute of Science and Technology), Republic of Korea. Other assignmengts include the simple ALU, finite state machine, single-cycle CPU, multi-cycle CPU and a simple Cache. I finished this lab assignment in the fall semester of 2022, during my exchange period at KAIST.
The "RISCV_TOP.v" and "pipe_control.v" are the two verilog files that I designed. The other files, such as "Mem_Model.v", "REG_FILE.v" and all the files in the testbench and testcase is provided by the course. 
The CPU can devided into two parts: control path and data path. "RISCV_TOP.v" mainly contains the data path, and the "pipe_control.v" is the control path.
