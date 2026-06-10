#!/usr/bin/env python3
#=============================================================================
# File     : gen_data.py
# Project  : SPGEMM-Accelerator
# Brief    : Parse TC1_RAW reference test cases → CSR format → ISA instructions
#           → binary ram.txt for cocotb testbench.
#
# Usage    : python gen_data.py
# Input    : test_case_for_reference/TC1_RAW/*.txt
# Output   : data/ram.txt (binary memory image)
#            data/ideal_result.txt (software reference C = A × B)
#=============================================================================

import os
import struct
import numpy as np
from numpy import binary_repr

#=============================================================================
# Config
#=============================================================================
INST_WIDTH       = 256        # bits per instruction
AXI_DATA_WIDTH   = 512        # bits per AXI beat
AXI_BEAT_BYTES   = AXI_DATA_WIDTH // 8   # 64 bytes
DATA_WIDTH       = 16         # FP16 element width in bits
N_ELEM_PER_BEAT  = AXI_DATA_WIDTH // DATA_WIDTH  # 32 FP16 per beat

# Opcodes (3-bit)
OP_LOAD   = 0b001
OP_STORE  = 0b010
OP_SPGEMM = 0b011
OP_FINISH = 0b111

# Memory type IDs (3-bit)
MEM_ROW_PTR = 0b000
MEM_COL_IDX = 0b001
MEM_VAL     = 0b010
MEM_OUTPUT  = 0b011

#=============================================================================
# FP16 helpers (integer values → FP16 binary, lossless for small integers)
#=============================================================================
FP16_MAX = 65504.0  # IEEE 754 half-precision max representable value

def float_to_fp16(f):
    """Convert Python float to FP16 binary (uint16), clamp to FP16 range."""
    # Clamp to FP16 representable range
    f_clamped = max(min(f, FP16_MAX), -FP16_MAX)
    buf = struct.pack('e', f_clamped)  # 'e' = IEEE 754 binary16
    return struct.unpack('<H', buf)[0]

def fp16_binary_repr(val, width=16):
    """Return little-endian binary string of FP16 integer."""
    return binary_repr(val, width)[::-1]

#=============================================================================
# Instruction encoding (ISA per isa.vh) — 256-bit, bit-reversed little-endian
#=============================================================================
def encode_load(mem_id, dram_offset, sram_offset, xsize):
    """
    LOAD instr: [63:6]dram_addr, [95:64]sram_offset, [127:96]xsize
    bit[5:3]=mem_id, bit[2:0]=OP_LOAD
    """
    opcode = OP_LOAD
    inst = 0
    inst |= (opcode & 0x7)           # bits [2:0]
    inst |= (mem_id & 0x7) << 3      # bits [5:3]
    inst |= (dram_offset & ((1<<58)-1)) << 6     # bits [63:6]
    inst |= (sram_offset & 0xFFFFFFFF) << 64     # bits [95:64]
    inst |= (xsize & 0xFFFFFFFF) << 96           # bits [127:96]
    return inst

def encode_store(mem_id, dram_offset, sram_offset, xsize):
    """STORE instr: same layout as LOAD."""
    opcode = OP_STORE
    inst = 0
    inst |= (opcode & 0x7)
    inst |= (mem_id & 0x7) << 3
    inst |= (dram_offset & ((1<<58)-1)) << 6
    inst |= (sram_offset & 0xFFFFFFFF) << 64
    inst |= (xsize & 0xFFFFFFFF) << 96
    return inst

def encode_spgemm(a_row_sram, a_col_sram, a_val_sram,
                   b_row_sram, b_col_sram, b_val_sram,
                   M, K, N):
    """
    SPGEMM instr:
      [63:6]   A_row_ptr SRAM
      [95:64]  A_col_idx SRAM
      [127:96] A_val SRAM
      [159:128] B_row_ptr SRAM
      [191:160] B_col_idx SRAM
      [223:192] B_val SRAM
      [232:224] M (9 bits)
      [241:233] K (9 bits)
      [250:242] N (9 bits)
      bit[2:0] = OP_SPGEMM
    """
    opcode = OP_SPGEMM
    inst = 0
    inst |= (opcode & 0x7)
    # A_row_ptr: bits [63:6] (58-bit field, but we only use lower 32-bit sram offset)
    inst |= (a_row_sram & ((1<<58)-1)) << 6
    inst |= (a_col_sram & 0xFFFFFFFF) << 64
    inst |= (a_val_sram & 0xFFFFFFFF) << 96
    inst |= (b_row_sram & 0xFFFFFFFF) << 128
    inst |= (b_col_sram & 0xFFFFFFFF) << 160
    inst |= (b_val_sram & 0xFFFFFFFF) << 192
    inst |= (M & 0x1FF) << 224
    inst |= (K & 0x1FF) << 233
    inst |= (N & 0x1FF) << 242
    return inst

def encode_finish():
    """FINISH instr: bit[2:0] = OP_FINISH"""
    return OP_FINISH & 0x7

def inst_to_binary_str(inst_val):
    """Convert 256-bit instruction integer → little-endian binary string."""
    return binary_repr(inst_val, INST_WIDTH)[::-1]

#=============================================================================
# Parse TC1_RAW reference files
#=============================================================================
def parse_index_file(filepath):
    """
    Parse _Index.txt: each non-empty line = one row's column indices.
    Returns list of lists: row_cols[row_idx] = [col0, col1, ...]
    """
    rows = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # Split by whitespace (tabs/spaces)
            cols = [int(x) for x in line.split()]
            if cols:
                rows.append(cols)
    return rows

def parse_matrix_file(filepath):
    """
    Parse _Matrix.txt: each line = "weight  total_dimension".
    weight = nnz per row (A) or nnz per column (B)
    total_dimension = total columns (A) or total rows (B)

    Returns list of (weight, total_dim) tuples.
    """
    entries = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) >= 2:
                weight = int(parts[0])
                dim    = int(parts[1])
                entries.append((weight, dim))
    return entries


def build_csr_from_index(index_rows, K, start_row=0):
    """
    Build CSR arrays from parsed index file (row-major).
    - index_rows: list of [col_indices] per row
    - K: number of columns in the matrix
    - start_row: starting row ID for COO construction

    Returns (row_ptr, col_idx, val_fp16, M)
    """
    M = len(index_rows)
    row_ptr = [0]
    col_idx = []
    val_fp16 = []

    for rid, row in enumerate(index_rows):
        for c in row:
            col_idx.append(c)
            # Small values: 1~7, product max = 49, safe for FP16 accumulation
            actual_row = start_row + rid
            v = (actual_row * 37 + c * 13 + 1) % 7 + 1
            val_fp16.append(float_to_fp16(float(v)))
        row_ptr.append(len(col_idx))

    assert row_ptr[-1] == len(col_idx), f"CSR row_ptr mismatch: {row_ptr[-1]} != {len(col_idx)}"
    return row_ptr, col_idx, val_fp16, M


def build_csr_from_csc(index_cols, matrix_meta, K):
    """
    Build CSR arrays from column-major (CSC) format.
    TC1_RAW stores B in CSC: each index entry = rows where column c has non-zeros.

    - index_cols: list of [row_indices] per COLUMN (CSC format)
    - matrix_meta: list of (col_weight, total_rows) per column
    - K: total rows in B

    Returns (row_ptr, col_idx, val_fp16, M) in CSR format (row-major).
    """
    N = len(matrix_meta)  # number of columns in B
    # Build COO entries: (row, col, val)
    coo_entries = []
    for col in range(N):
        if col < len(index_cols):
            rows_in_col = index_cols[col]
        else:
            rows_in_col = []
        for row in rows_in_col:
            # Small values: 1~7, product max = 49, safe for FP16 accumulation
            v = (row * 37 + col * 13 + 1) % 7 + 1
            val_fp16_v = float_to_fp16(float(v))
            coo_entries.append((row, col, val_fp16_v))

    # Sort by row then column
    coo_entries.sort(key=lambda x: (x[0], x[1]))

    # Build CSR
    M = K  # B has K rows
    row_ptr = [0]
    col_idx = []
    val_fp16 = []

    current_row = 0
    for (r, c, v) in coo_entries:
        # Fill empty rows
        while current_row < r:
            row_ptr.append(len(col_idx))
            current_row += 1
        col_idx.append(c)
        val_fp16.append(v)

    # Fill remaining rows
    while current_row < M:
        row_ptr.append(len(col_idx))
        current_row += 1

    # row_ptr should have M+1 entries
    if len(row_ptr) < M + 1:
        row_ptr.append(len(col_idx))

    assert row_ptr[-1] == len(col_idx), \
        f"CSR row_ptr mismatch: {row_ptr[-1]} != {len(col_idx)}"
    return row_ptr, col_idx, val_fp16, M



#=============================================================================
# Software SpGEMM (compute ideal reference C = A×B)
#=============================================================================
def csr_spgemm_ideal(A_row, A_col, A_val_fp16, A_M, K,
                       B_row, B_col, B_val_fp16, K_B, N):
    assert K == K_B, f"Dimension mismatch: K={K}, K_B={K_B}"
    C_M = A_M
    C_N = N
    C_row_ptr = [0]
    C_col = []
    C_val = []
    for i in range(A_M):
        row_accum = {}
        a_start = A_row[i]
        a_end   = A_row[i + 1]
        for a_idx in range(a_start, a_end):
            a_col = A_col[a_idx]
            a_val = struct.unpack('e', struct.pack('<H', A_val_fp16[a_idx]))[0]
            if a_col >= len(B_row) - 1:
                continue
            b_start = B_row[a_col]
            b_end   = B_row[a_col + 1]
            for b_idx in range(b_start, b_end):
                b_col = B_col[b_idx]
                b_val = struct.unpack('e', struct.pack('<H', B_val_fp16[b_idx]))[0]
                product = a_val * b_val
                row_accum[b_col] = row_accum.get(b_col, 0.0) + product
        sorted_cols = sorted(row_accum.keys())
        for c in sorted_cols:
            C_col.append(c)
            C_val.append(float_to_fp16(row_accum[c]))
        C_row_ptr.append(len(C_col))
    return C_row_ptr, C_col, C_val, C_M, C_N


#=============================================================================
# Binary packing (based on reference Dram.py pattern)
#=============================================================================
def array32_to_binary(arr):
    """Pack list of 32-bit integers → little-endian binary string."""
    s = ''
    for v in arr:
        s += binary_repr(v, 32)[::-1]
    return s

def array16_to_binary(arr):
    """Pack list of 16-bit integers → little-endian binary string."""
    s = ''
    for v in arr:
        s += binary_repr(v, 16)[::-1]
    return s

def pad_to_multiple(bin_str, byte_multiple):
    """Pad binary string to multiple of byte_multiple bytes."""
    current_bytes = len(bin_str) // 8
    if current_bytes % byte_multiple != 0:
        pad_bytes = byte_multiple - (current_bytes % byte_multiple)
        return bin_str + '0' * (pad_bytes * 8)
    return bin_str

#=============================================================================
# Main: generate data for all test-case pairs
#=============================================================================
def main():
    base = os.path.dirname(os.path.abspath(__file__))
    ref_dir = os.path.join(base, 'test_case_for_reference', 'TC1_RAW')
    data_dir = os.path.join(base, 'data')
    os.makedirs(data_dir, exist_ok=True)

    # Test case pairs: (A_i, B_i) for i = 0, 1, 2
    pairs = [(0, 0), (1, 1), (2, 2)]

    # Collect all instructions and packed data
    all_instructions = []       # list of 256-bit instruction integers
    all_packed_data = []        # list of (address, binary_string) pairs
    all_ideal = []              # list of ideal C reference outputs

    # SRAM base addresses (reused per pair, since SRAM is scratchpad)
    INSTR_BASE = 0x0000
    A_ROW_SRAM  = 0x1000
    A_COL_SRAM  = 0x2000
    A_VAL_SRAM  = 0x3000
    B_ROW_SRAM  = 0x4000
    B_COL_SRAM  = 0x5000
    B_VAL_SRAM  = 0x6000
    OUT_SRAM    = 0x8000

    # DRAM layout: each section = 64KB (0x10000 bytes), large enough for any CSR array
    # DATA_BASE offsets all data after the instruction area (instructions ≈ few KB)
    DATA_BASE   = 0x10000   # start data at 64KB, safely after instructions
    SEC_SIZE    = 0x40000   # 256KB per section
    PAIR_SIZE   = 8 * SEC_SIZE  # 8 sections per pair = 512KB

    pair_info = []

    for pair_idx, (a_id, b_id) in enumerate(pairs):
        pair_base = DATA_BASE + pair_idx * PAIR_SIZE
        A_ROW_DRAM = pair_base + 0 * SEC_SIZE
        A_COL_DRAM = pair_base + 1 * SEC_SIZE
        A_VAL_DRAM = pair_base + 2 * SEC_SIZE
        B_ROW_DRAM = pair_base + 3 * SEC_SIZE
        B_COL_DRAM = pair_base + 4 * SEC_SIZE
        B_VAL_DRAM = pair_base + 5 * SEC_SIZE
        OUT_DRAM   = pair_base + 6 * SEC_SIZE
        a_idx_file = os.path.join(ref_dir, f'A_{a_id}_Index.txt')
        a_mat_file = os.path.join(ref_dir, f'A_{a_id}_Matrix.txt')
        b_idx_file = os.path.join(ref_dir, f'B_{b_id}_Index.txt')
        b_mat_file = os.path.join(ref_dir, f'B_{b_id}_Matrix.txt')

        # Parse A (row-major: Index gives column indices per row)
        a_index_rows = parse_index_file(a_idx_file)
        a_matrix_meta = parse_matrix_file(a_mat_file)
        A_M = len(a_index_rows)              # M = number of rows
        A_K = a_matrix_meta[0][1] if a_matrix_meta else 256  # K from total_cols

        # Parse B (column-major CSC: Index gives row indices per column)
        # Need to transpose B from CSC to CSR for DUT
        b_index_cols = parse_index_file(b_idx_file)   # per-column row indices
        b_matrix_meta = parse_matrix_file(b_mat_file)  # per-column weights
        B_K = b_matrix_meta[0][1] if b_matrix_meta else 256  # total rows of B
        B_N = len(b_matrix_meta)                       # N = number of columns

        # Build CSR arrays
        A_row_ptr, A_col_idx, A_val_fp16, _ = build_csr_from_index(a_index_rows, A_K)
        # B: convert from CSC (column-major) to CSR (row-major)
        B_row_ptr, B_col_idx, B_val_fp16, _ = build_csr_from_csc(b_index_cols, b_matrix_meta, B_K)

        print(f"[GEN] Pair A_{a_id}×B_{b_id}: A={A_M}×{A_K} (nnz={len(A_col_idx)}), "
              f"B={B_K}×{B_N} (nnz={len(B_col_idx)})")

        # Compute ideal reference C = A × B
        C_row_ptr, C_col_idx, C_val_fp16, C_M, C_N = \
            csr_spgemm_ideal(A_row_ptr, A_col_idx, A_val_fp16, A_M, A_K,
                             B_row_ptr, B_col_idx, B_val_fp16, B_K, B_N)
        print(f"[GEN]   Ideal C={C_M}×{C_N} (nnz={len(C_col_idx)})")

        # Pack CSR arrays to binary
        # row_ptr: 32-bit elements
        a_row_bin = array32_to_binary(A_row_ptr)
        a_col_bin = array16_to_binary(A_col_idx)
        a_val_bin = array16_to_binary(A_val_fp16)

        b_row_bin = array32_to_binary(B_row_ptr)
        b_col_bin = array16_to_binary(B_col_idx)
        b_val_bin = array16_to_binary(B_val_fp16)

        # Pad to AXI beat boundaries (64 bytes = 512 bits)
        a_row_bin = pad_to_multiple(a_row_bin, AXI_BEAT_BYTES)
        a_col_bin = pad_to_multiple(a_col_bin, AXI_BEAT_BYTES)
        a_val_bin = pad_to_multiple(a_val_bin, AXI_BEAT_BYTES)
        b_row_bin = pad_to_multiple(b_row_bin, AXI_BEAT_BYTES)
        b_col_bin = pad_to_multiple(b_col_bin, AXI_BEAT_BYTES)
        b_val_bin = pad_to_multiple(b_val_bin, AXI_BEAT_BYTES)

        # Generate LOAD instructions (6 per pair)
        # dram_offset = where data lives in DRAM, sram_offset = where it's placed in SRAM
        load_a_row = encode_load(MEM_ROW_PTR, A_ROW_DRAM, A_ROW_SRAM, len(A_row_ptr))
        load_a_col = encode_load(MEM_COL_IDX, A_COL_DRAM, A_COL_SRAM, len(A_col_idx))
        load_a_val = encode_load(MEM_VAL,     A_VAL_DRAM, A_VAL_SRAM, len(A_val_fp16))
        load_b_row = encode_load(MEM_ROW_PTR, B_ROW_DRAM, B_ROW_SRAM, len(B_row_ptr))
        load_b_col = encode_load(MEM_COL_IDX, B_COL_DRAM, B_COL_SRAM, len(B_col_idx))
        load_b_val = encode_load(MEM_VAL,     B_VAL_DRAM, B_VAL_SRAM, len(B_val_fp16))

        # SPGEMM instruction (references SRAM addresses where data is loaded)
        spgemm = encode_spgemm(A_ROW_SRAM, A_COL_SRAM, A_VAL_SRAM,
                               B_ROW_SRAM, B_COL_SRAM, B_VAL_SRAM,
                               A_M, A_K, B_N)
        # STORE instruction (output from SRAM to DRAM)
        out_xsize = len(C_row_ptr) + len(C_col_idx) + len(C_val_fp16)
        store_out = encode_store(MEM_OUTPUT, OUT_DRAM, OUT_SRAM, out_xsize)

        pair_insts = [load_a_row, load_a_col, load_a_val,
                      load_b_row, load_b_col, load_b_val,
                      spgemm, store_out]

        all_instructions.extend(pair_insts)

        # Store packed data at DRAM addresses (unique per pair)
        all_packed_data.append((A_ROW_DRAM, a_row_bin))
        all_packed_data.append((A_COL_DRAM, a_col_bin))
        all_packed_data.append((A_VAL_DRAM, a_val_bin))
        all_packed_data.append((B_ROW_DRAM, b_row_bin))
        all_packed_data.append((B_COL_DRAM, b_col_bin))
        all_packed_data.append((B_VAL_DRAM, b_val_bin))

        all_ideal.append({
            'C_row_ptr': C_row_ptr,
            'C_col_idx': C_col_idx,
            'C_val_fp16': C_val_fp16,
            'C_M': C_M,
            'C_N': C_N,
        })

        pair_info.append({
            'A_M': A_M, 'A_K': A_K, 'A_nnz': len(A_col_idx),
            'B_K': B_K, 'B_N': B_N, 'B_nnz': len(B_col_idx),
            'C_M': C_M, 'C_N': C_N, 'C_nnz': len(C_col_idx),
        })

    # Add FINISH instruction
    all_instructions.append(encode_finish())

    # Pad instruction count to even (reference convention)
    if len(all_instructions) % 2 != 0:
        all_instructions.append(0)  # NOP

    total_inst_count = len(all_instructions)

    # Write info.txt (metadata for testbench)
    info_path = os.path.join(data_dir, 'info.txt')
    with open(info_path, 'w') as f:
        f.write(f"INST_COUNT={total_inst_count}\n")
        f.write(f"INST_BYTES={total_inst_count * 32}\n")
        f.write(f"PAIR_COUNT={len(pairs)}\n")
        for i, info in enumerate(pair_info):
            f.write(f"PAIR_{i}_M={info['A_M']}\n")
            f.write(f"PAIR_{i}_K={info['A_K']}\n")
            f.write(f"PAIR_{i}_N={info['C_N']}\n")
            f.write(f"PAIR_{i}_A_NNZ={info['A_nnz']}\n")
            f.write(f"PAIR_{i}_B_NNZ={info['B_nnz']}\n")
            f.write(f"PAIR_{i}_C_NNZ={info['C_nnz']}\n")
    print(f"[GEN] info.txt written ({total_inst_count} instructions)")

    # Build binary ram image
    # Part 1: Instructions at INSTR_BASE
    dram = ''
    for inst in all_instructions:
        dram += inst_to_binary_str(inst)

    # Part 2: Data sections
    # Sort by address
    all_packed_data.sort(key=lambda x: x[0])

    # Fill gaps with zeros
    current_bit_pos = len(dram)  # in bits
    for addr, bin_str in all_packed_data:
        target_bit_pos = addr * 8
        assert target_bit_pos >= current_bit_pos, \
            f"Address {addr:#x} overlaps with previous data at bit {current_bit_pos}"
        gap = target_bit_pos - current_bit_pos
        if gap > 0:
            dram += '0' * gap
        dram += bin_str
        current_bit_pos = len(dram)

    # Write ram.txt
    ram_path = os.path.join(data_dir, 'ram.txt')
    with open(ram_path, 'w') as f:
        f.write(dram)
    print(f"\n[GEN] ram.txt written: {ram_path}")
    print(f"[GEN]   Size: {len(dram)} bits = {len(dram)//8} bytes")
    print(f"[GEN]   Instructions: {total_inst_count} × 256-bit")
    print(f"[GEN]   Data sections: {len(all_packed_data)}")

    # Write ideal_result.txt (software reference for testbench comparison)
    ideal_path = os.path.join(data_dir, 'ideal_result.txt')
    with open(ideal_path, 'w') as f:
        f.write(f"# Ideal reference for {len(pairs)} SpGEMM pairs\n")
        for i, g in enumerate(all_ideal):
            f.write(f"# --- Pair {i}: C={g['C_M']}×{g['C_N']}, nnz={len(g['C_col_idx'])} ---\n")
            f.write(f"M={g['C_M']} N={g['C_N']} NNZ={len(g['C_col_idx'])}\n")
            f.write(f"ROW_PTR: {','.join(str(x) for x in g['C_row_ptr'])}\n")
            f.write(f"COL_IDX: {','.join(str(x) for x in g['C_col_idx'])}\n")
            f.write(f"VAL:     {','.join(str(x) for x in g['C_val_fp16'])}\n")
            f.write("\n")
    print(f"[GEN] ideal_result.txt written: {ideal_path}")

    # Summary
    print(f"\n[GEN] ============ Summary ============")
    for i, info in enumerate(pair_info):
        print(f"[GEN] Pair {i}: A({info['A_M']}×{info['A_K']}, {info['A_nnz']}nnz) "
              f"× B({info['B_K']}×{info['B_N']}, {info['B_nnz']}nnz) "
              f"→ C({info['C_M']}×{info['C_N']}, {info['C_nnz']}nnz)")


if __name__ == '__main__':
    main()
