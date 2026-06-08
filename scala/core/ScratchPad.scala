package gcn.core

import scala.math.pow

import chisel3._
import chisel3.util._
import vta.util.config._
import os.write
import gcn.core.util.MuxTree
import ISA._


/** Scratchpad Logic.
 *
 * Load 1D and 2D tensors from main memory (DRAM) to input/weight
 * scratchpads (SRAM). Also, there is support for zero padding, while
 * doing the load. Zero-padding works on the y and x axis, and it is
 * managed by TensorPadCtrl. The TensorDataCtrl is in charge of
 * handling the way tensors are stored on the scratchpads.
 * 
 */

 class SPWriteCmdWithSel(implicit p: Parameters) extends Bundle{
  val mp = p(AccKey).memParams
  val cp = p(AccKey).coreParams
  val spWriteCmd = new SPWriteCmd
  val spSel = UInt(cp.nScratchPadMem.W)
}


class SPWriteCmd(val scratchType: String = "Col", val mode: String = "multiple")(implicit p: Parameters) extends Bundle{
  val mp = p(AccKey).memParams
  val cp = p(AccKey).coreParams
  val addr = UInt(M_SRAM_OFFSET_BITS.W)
  val data = if(mode == "single"){UInt(cp.blockSize.W)} else if(scratchType == "Global"){UInt(mp.dataBits.W)}else{UInt(cp.bankBlockSize.W)}
}

class SPReadCmd(implicit p: Parameters) extends Bundle{
  val cp = p(AccKey).coreParams
  val addr = UInt(M_SRAM_OFFSET_BITS.W)
}


class SPReadData(val scratchType: String = "Col")(implicit p: Parameters) extends Bundle{
  val cp = p(AccKey).coreParams
  val mp = p(AccKey).memParams
  val data = if(scratchType == "Out"){UInt(mp.dataBits.W)}else if(scratchType == "Global"){UInt(cp.bankBlockSize.W)} else{UInt(cp.blockSize.W)}
}

// banked scratch
class BankedScratchpad(scratchType: String = "Col")(implicit p: Parameters)extends Module with ISAConstants{
  val mp = p(AccKey).memParams
  val cp = p(AccKey).coreParams

    // Scratch size params
  val blockSize = cp.blockSize
  val nBanks = cp.nColInDense
  val scratchSize = cp.scratchSizeMap(scratchType)/nBanks
  
  
  val io = IO(new Bundle {
    val spWrite = Input(new SPWriteCmd)
    val spReadCmd = Input(Vec(nBanks, new SPReadCmd))
    val spReadData = Output(Vec(nBanks, new SPReadData))
    val writeEn = Input(Bool())
  })

  // Write
  val waddr = WireDefault(io.spWrite.addr)
  val wdata = WireDefault(io.spWrite.data)

  // Read
  val raddr = io.spReadCmd.map(_.addr)
  val rdata = Wire(Vec(nBanks, UInt(blockSize.W)))

  val ram = Seq.fill(nBanks){
    SyncReadMem(scratchSize, UInt(blockSize.W))
  }

  when(io.writeEn){
  val writeIdx = waddr >> log2Ceil(cp.bankBlockSize/8)
    for (i <- 0 until (nBanks)){
      ram(i).write(writeIdx, wdata((i+1)*blockSize - 1, i*blockSize))
    }
  }

  
 
  val raddrByteAlign =  (raddr.map(_ >> log2Ceil(blockSize/8)))
  val bankIdx = raddrByteAlign.map(_ >> (log2Ceil(nBanks)))
  for (i <- 0 until (nBanks)){
    rdata(i) := ram(i).read(bankIdx(i), true.B)
  }
  for(i <- 0 until nBanks){
    io.spReadData(i).data := rdata(i)
  }

}

// Masked Writes 512 bits and reads 512 bits at a time.
class GlobalBuffer(scratchType: String = "Global")(implicit p: Parameters)extends Module with ISAConstants{
  val mp = p(AccKey).memParams
  val cp = p(AccKey).coreParams

    // Scratch size params
  val bankBlockSize = cp.bankBlockSize
  var scratchSize = 0
  if(scratchType == "Global"){
    scratchSize = cp.globalBufferSize/mp.dataBits
  }else{
    scratchSize = cp.scratchSizeMap(scratchType)/mp.dataBits
  }
  val nBanks = mp.dataBits/cp.bankBlockSize
  
  val io = IO(new Bundle {
    val spWrite = Input(new SPWriteCmd(scratchType = "Global"))
    val spReadCmd = Input(new SPReadCmd)
    val spReadData = Output(new SPReadData(scratchType = "Global"))
    val writeEn = Input(Bool())
  })

  // Write
  val waddr = WireDefault(io.spWrite.addr)
  val wdata = WireDefault(io.spWrite.data)

  // Read
  val raddr = WireDefault(io.spReadCmd.addr)
  val rdata = Wire(Vec(nBanks, UInt(bankBlockSize.W)))
  val bankSelPrev = RegInit(0.U(log2Ceil(nBanks).W))

  val ram = Seq.fill(nBanks){
    SyncReadMem(scratchSize, UInt(bankBlockSize.W))
  }

  when(io.writeEn){
    val writeIdx = waddr >> log2Ceil(mp.dataBits/8)
    for (i <- 0 until (nBanks)){
      ram(i).write(writeIdx, wdata((i+1)*bankBlockSize - 1, i*bankBlockSize))
    }
  }

  val raddrByteAlign =  (raddr >> log2Ceil(bankBlockSize/8))
  val bankSel = (raddrByteAlign)(log2Ceil(nBanks) - 1, 0)
  val bankIdx = (raddrByteAlign) >> (log2Ceil(nBanks))
  bankSelPrev := bankSel
  for (i <- 0 until (nBanks)){
    rdata(i) := ram(i).read(bankIdx, true.B)
  }
  io.spReadData.data := MuxTree(bankSelPrev, rdata)

}

// Writes 512 bits and reads 32 bits at a time 
class Scratchpad(scratchType: String = "Col", masked: Boolean = false)(implicit p: Parameters)extends Module with ISAConstants{
  val mp = p(AccKey).memParams
  val cp = p(AccKey).coreParams

    // Scratch size params
  val blockSize = cp.blockSize
  val nBanks = cp.nColInDense
  val scratchSize = cp.scratchSizeMap(scratchType)/nBanks
  
  
  val io = IO(new Bundle {
    val spWrite = Input(new SPWriteCmd)
    val spReadCmd = Input(new SPReadCmd)
    val spReadData = Output(new SPReadData)
    val writeEn = Input(Bool())
    val mask = if (masked) Some(Input(UInt((nBanks).W))) else None
  })

  // Write
  val waddr = WireDefault(io.spWrite.addr)
  val wdata = WireDefault(io.spWrite.data)

  // Read
  val raddr = WireDefault(io.spReadCmd.addr)
  val rdata = Wire(Vec(nBanks, UInt(blockSize.W)))
  val bankSelPrev = RegInit(0.U(log2Ceil(nBanks).W))

  val ram = Seq.fill(nBanks){
    SyncReadMem(scratchSize, UInt(blockSize.W))
  }

  when(io.writeEn){
    val writeIdx = waddr >> log2Ceil((nBanks * blockSize)/8)
    if(masked){
      for (i <- 0 until (nBanks)){
        when(io.mask.get(i).asBool){
          ram(i).write(writeIdx, wdata((i+1)*blockSize - 1, i*blockSize))
        }
      }
    }else{
      for (i <- 0 until (nBanks)){
        ram(i).write(writeIdx, wdata((i+1)*blockSize - 1, i*blockSize))
      }
    }
  }
  
 
  val raddrByteAlign =  (raddr >> log2Ceil(blockSize/8))
  val bankSel = (raddrByteAlign)(log2Ceil(nBanks) - 1, 0)
  val bankIdx = (raddrByteAlign) >> (log2Ceil(nBanks))
  bankSelPrev := bankSel
  for (i <- 0 until (nBanks)){
    rdata(i) := ram(i).read(bankIdx, (bankSel === i.U))
  }
  io.spReadData.data := MuxTree(bankSelPrev, rdata)

}

class SingleScratchpad(scratchType: String = "Ptr", masked: Boolean = false)(implicit p: Parameters)extends Module with ISAConstants{
  val mp = p(AccKey).memParams
  val cp = p(AccKey).coreParams

    // Scratch size params
  val blockSize = cp.blockSize
  val nBanks = 1
  val scratchSize = cp.scratchSizeMap(scratchType)/nBanks
  
  
  val io = IO(new Bundle {
    val spWrite = Input(new SPWriteCmd(mode = "single"))
    val spReadCmd = Input(new SPReadCmd)
    val spReadData = Output(new SPReadData)
    val writeEn = Input(Bool())
    val mask = if (masked) Some(Input(UInt((nBanks).W))) else None
  })

  // Write
  val waddr = WireDefault(io.spWrite.addr)
  val wdata = WireDefault(io.spWrite.data)

  // Read
  val raddr = WireDefault(io.spReadCmd.addr)
  val rdata = Wire( UInt(blockSize.W))

  val ram = SyncReadMem(scratchSize, UInt(blockSize.W))

  when(io.writeEn){
    val writeIdx = waddr >> log2Ceil(blockSize/8)
    ram.write(writeIdx, wdata)
  }
  
 
  val raddrByteAlign =  (raddr >> log2Ceil(blockSize/8))
  val bankIdx = (raddrByteAlign)

  rdata := ram.read(bankIdx, true.B)
  io.spReadData.data := rdata

}

// Reads 512 bits and write 32 bits at a time 
class OutputScratchpad(scratchType: String = "Out", debug: Boolean = false)(implicit p: Parameters)extends Module with ISAConstants{
  val mp = p(AccKey).memParams
  val cp = p(AccKey).coreParams

    // Scratch size params
  val blockSize = (cp.blockSize * cp.nColInDense)
  val scratchSize = cp.scratchSizeMap(scratchType)/mp.dataBits
  val nBanks = mp.dataBits/blockSize
  
  val io = IO(new Bundle {
    val spWrite = Input(new SPWriteCmd(scratchType = "Out"))
    val spReadCmd = Input(new SPReadCmd)
    val spReadData = Output(new SPReadData(scratchType = "Out"))
    val writeEn = Input(Bool())
  })

  // Write
  val waddr = WireDefault(io.spWrite.addr)
  val waddrByteAlign =  (waddr >> log2Ceil(blockSize/8))
  val wdata = WireDefault(io.spWrite.data)
  val writeBankSel = (waddrByteAlign)(log2Ceil(nBanks) - 1, 0)
  val writeIdx = waddrByteAlign >> log2Ceil(nBanks)
  // Read

  val raddr = WireDefault(io.spReadCmd.addr)
  val rdata = Wire(Vec(nBanks, UInt(blockSize.W)))

  val ram = Seq.fill(nBanks){
    SyncReadMem(scratchSize, UInt(blockSize.W))
  }


  for (i <- 0 until (nBanks)){
    when(io.writeEn && (writeBankSel === i.U)){
      ram(i).write(writeIdx, wdata)
    }
  }
 
  val readIdx = raddr >> log2Ceil(nBanks*blockSize/8)
  for (i <- 0 until (nBanks)){
    rdata(i) := ram(i).read(readIdx, true.B)
  }
  io.spReadData.data := rdata.reverse.reduce(Cat(_,_))

}
