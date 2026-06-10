#=============================================================================
# File     : Makefile
# Project  : SPGEMM-Accelerator
# Brief    : Cocotb Makefile — builds and runs SpGEMM accelerator simulation
#
# Usage:
#   make                  # Run default test (test_spgemm_tc1) + waveform
#   make SIM=icarus       # Force Icarus Verilog
#   make SIM=verilator    # Force Verilator
#   make TESTCASE=test_quick_launch  # Run specific test
#   make WAVES=0          # Run without waveform
#   make wave             # Open waveform in gtkwave
#   make clean            # Remove build artifacts
#=============================================================================

# --- Waveform ---
WAVES ?= 1

# --- Simulation ---
SIM ?= icarus
TOPLEVEL_LANG = verilog

# --- DUT Top Module ---
TOPLEVEL = core_top

# --- Verilog Sources ---
# Parse filelist.f for source files
# Skip comment lines (#), resolve rtl/../rtl prefix
VERILOG_SOURCES  = $(shell grep -v '^#' ../rtl/filelist.f | grep -v '^$$' | sed 's|^+incdir+include$$||' | sed 's|^rtl|../rtl|')
# Add include directory explicitly
COMPILE_ARGS += -I../rtl/include

# For Verilator, add timing support
ifeq ($(SIM),verilator)
    COMPILE_ARGS += -Wno-fatal --timing
endif

# For Icarus, use SV2012
ifeq ($(SIM),icarus)
    COMPILE_ARGS += -g2012
endif

# --- Cocotb ---
MODULE = test_dut
TESTCASE ?= test_spgemm_tc1

# Enable waveform dumping (FST format, view with gtkwave)
# Usage: make WAVES=1
ifeq ($(WAVES),1)
    ifeq ($(SIM),icarus)
        COMPILE_ARGS += -DCOCOTB_DUMP_WAVE
    endif
endif

# --- Cocotb Plusargs ---
# Pass test parameters
export COCOTB_LOG_LEVEL ?= WARNING
# export COCOTB_LOG_LEVEL = DEBUG  # use for detailed debug

# --- Default target: gen data first, then simulate ---
all: gen_data sim

# --- Include Cocotb Makefiles ---
include $(shell cocotb-config --makefiles)/Makefile.sim

# --- Extra Targets ---

# Generate test data before simulation
gen_data:
	python3 gen_data.py

# Clean build artifacts
clean::
	rm -rf __pycache__ *.pyc sim_build results.xml
	rm -rf data/ram.txt data/ideal_result.txt
	rm -f dump.fst dump.vcd

# Open waveform with gtkwave (loads pre-configured signal layout)
wave:
	gtkwave sim_build/core_top.fst wave.gtkw &