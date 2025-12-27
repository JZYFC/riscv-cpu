// Copyright lowRISC contributors.
// Copyright 2017 ETH Zurich and University of Bologna, see also CREDITS.md.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

/**
 * Package with constants used by Ibex
 */
package ibex_pkg;

typedef enum logic [6:0] {
// RV32I ================================== //
  // Arithmetics
  ALU_ADD,
  ALU_SUB,
  // ME
  ALU_MADD,  // rd = rs1*rs2+rs3
  // ME

  // Logics
  ALU_XOR,
  ALU_OR,
  ALU_AND,
  // RV32B
  ALU_XNOR,
  ALU_ORN,
  ALU_ANDN,

  // Shifts
  ALU_SRA,
  ALU_SRL,
  ALU_SLL,
  // RV32B
  ALU_SRO,
  ALU_SLO,
  ALU_ROR,
  ALU_ROL,
  ALU_GREV,
  ALU_GORC,
  ALU_SHFL,
  ALU_UNSHFL,

  // Comparisons
  ALU_LT,
  //ALU_PLUSONELT,
  ALU_LTU,
  ALU_GE,
  ALU_GEU,
  ALU_EQ,
  ALU_NE,
  // RV32B
  ALU_MIN,
  ALU_MINU,
  ALU_MAX,
  ALU_MAXU,

  // Pack
  // RV32B
  ALU_PACK,
  ALU_PACKU,
  ALU_PACKH,

  // Sign-Extend
  // RV32B
  ALU_SEXTB,
  ALU_SEXTH,

  // Bitcounting
  // RV32B
  ALU_CLZ,
  ALU_CTZ,
  ALU_PCNT,

  // Set lower than
  ALU_SLT,
  ALU_SLTU,

  // Ternary Bitmanip Operations
  // RV32B
  ALU_CMOV,
  ALU_CMIX,
  ALU_FSL,
  ALU_FSR,

  // Single-Bit Operations
  // RV32B
  ALU_SBSET,
  ALU_SBCLR,
  ALU_SBINV,
  ALU_SBEXT,

  // Bit Extract / Deposit
  // RV32B
  ALU_BEXT,
  ALU_BDEP,

  // Bit Field Place
  // RV32B
  ALU_BFP,

  // Carry-less Multiply
  // RV32B
  ALU_CLMUL,
  ALU_CLMULR,
  ALU_CLMULH,

  // Cyclic Redundancy Check
  ALU_CRC32_B,
  ALU_CRC32C_B,
  ALU_CRC32_H,
  ALU_CRC32C_H,
  ALU_CRC32_W,
  ALU_CRC32C_W,
// RV32I ================================== //

// RV32F ================================== //
  // Arithmetics
  ALU_FADD,
  ALU_FSUB,
  ALU_FMUL,
  ALU_FDIV,

  ALU_FMADD,   
  ALU_FMSUB,    
  ALU_FNMSUB,
  ALU_FNMADD,

  ALU_FADDDIV,  

  ALU_FSUBABS, 

  // Comparision
  ALU_FMIN,
  ALU_FMAX,
  ALU_FLT,
  ALU_FEQ,

  // Conversion
  ALU_FCVTSW,
  
  // Combine
  ALU_PLUSONELT,

  // ADDR
  ALU_ADDRTWO,
  ALU_ADDRFIVE
// RV32F ================================== //
} alu_op_e;

// RV32F ================================== //

///////////////////
// Rounding mode //
///////////////////

typedef enum logic [2:0] {
  RNE,      // Round to Nearest, ties to Even
  RTZ,      // Round towards Zero
  RDN,      // Round Down (towards minus infinity)
  RUP,      // Round Up (towards infinity)
  RMM,      // Round to Nearest, ties to Max Magnitude
  DYN       // select dynamic rounding mode

} rounding_mode_e;