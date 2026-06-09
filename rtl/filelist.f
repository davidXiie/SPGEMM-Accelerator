# SPGEMM-Accelerator Verilog RTL 文件列表

## include (全局头文件)
rtl/include/defines.vh
rtl/include/isa.vh

## Infrastructure (基础设施模块)
rtl/infrastructure/axi_interface.v
rtl/infrastructure/decode.v
rtl/infrastructure/fetch.v
rtl/infrastructure/load.v
rtl/infrastructure/store.v
rtl/infrastructure/scratchpad.v

## Core (核心加速模块)
rtl/core/core_top.v
rtl/core/scheduler.v
rtl/core/pe_top.v
rtl/core/pe_decompress.v
rtl/core/pe_mul_array.v
rtl/core/pe_aggregation.v
rtl/core/c_csr_writer.v

## Top Wrapper
rtl/wrapper.v

## Simulation (仿真)
rtl/sim/tb_core_top.v
