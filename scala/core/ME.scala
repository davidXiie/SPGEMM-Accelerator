package gcn.core
import chisel3._
import chisel3.util._
import vta.util.config._
import vta.util.genericbundle._

/** MEBase. Parametrize base class. */
abstract class MEBase(implicit p: Parameters) extends GenericParameterizedBundle(p)

/** MECmd.
 *
 * This interface is used for creating write and read requests to memory.
 */
class clientTag(implicit p:Parameters) extends Bundle{
  val clientBits = p(AccKey).meParams.clientBits
  val RequestQueueDepth = p(AccKey).meParams.RequestQueueDepth
  val RequestQueueMaskBits = p(AccKey).meParams.RequestQueueMaskBits
  val client_id  = UInt(clientBits.W)
  val client_tag = UInt(p(AccKey).meParams.clientTagBitWidth.W)
  val client_mask = UInt(RequestQueueMaskBits.W)
}

class MECmd(implicit p: Parameters) extends MEBase {
  val addrBits = p(AccKey).memParams.addrBits
  val lenBits = p(AccKey).memParams.lenBits
  val tagBits  = p(AccKey).meParams.clientTagBitWidth
  val addr = UInt(addrBits.W)
  val len = UInt(lenBits.W)
  val tag = UInt(tagBits.W)
}
class MECmdData(implicit p: Parameters) extends MEBase {
  val data = UInt(p(AccKey).memParams.dataBits.W)
  val last = Bool()
}

class MEData(implicit p: Parameters) extends MEBase {
  val dataBits = p(AccKey).memParams.dataBits
  val data = UInt(dataBits.W)
  val tag = UInt(p(AccKey).meParams.clientTagBitWidth.W)
  val last = Bool()
}

/** MEReadMaster.
 *
 * This interface is used by modules inside the core to generate read requests
 * and receive responses from ME.
 */
class MEReadMaster(implicit p: Parameters) extends Bundle {
  val dataBits = p(AccKey).memParams.dataBits
  val cmd = Decoupled(new MECmd)
  val data = Flipped(Decoupled(new MEData))
}

/** MEReadClient.
 *
 * This interface is used by the ME to receive read requests and generate
 * responses to modules inside the core.
 */
class MEReadClient(implicit p: Parameters) extends Bundle {
  val dataBits = p(AccKey).memParams.dataBits
  val cmd = Flipped(Decoupled(new MECmd))
  val data = Decoupled(new MEData)
}

/** MEWriteData.
 *
 * This interface is used by the ME to handle write requests from modules inside
 * the core.
 */
class MEWriteData(implicit p: Parameters) extends Bundle {
  val dataBits = p(AccKey).memParams.dataBits
  val strbBits = dataBits/8

  val data = UInt(dataBits.W)
  val strb = UInt(strbBits.W)

}

/** MEWriteMaster.
 *
 * This interface is used by modules inside the core to generate write requests
 * to the ME.
 */
class MEWriteMaster(implicit p: Parameters) extends Bundle {
  val dataBits = p(AccKey).memParams.dataBits
  val cmd = Decoupled(new MECmd)
  val data = Decoupled(new MEWriteData)
  val ack = Input(Bool())
}

/** MEWriteClient.
 *
 * This interface is used by the ME to handle write requests from modules inside
 * the core.
 */
class MEWriteClient(implicit p: Parameters) extends Bundle {
  val dataBits = p(AccKey).memParams.dataBits
  val cmd = Flipped(Decoupled(new MECmd))
  val data = Flipped(Decoupled(new MEWriteData))
  val ack = Output(Bool())
}

/** MEMaster.
 *
 * Pack nRd number of MEReadMaster interfaces and nWr number of MEWriteMaster
 * interfaces.
 */
class MEMaster(implicit p: Parameters) extends Bundle {
  val nRd = p(AccKey).meParams.nReadClients
  val nWr = p(AccKey).meParams.nWriteClients
  val rd = Vec(nRd, new MEReadMaster)
  val wr = Vec(nWr, new MEWriteMaster)
}

/** MEClient.
 *
 * Pack nRd number of MEReadClient interfaces and nWr number of MEWriteClient
 * interfaces.
 */
class MEClient(implicit p: Parameters) extends Bundle {
  val nRd = p(AccKey).meParams.nReadClients
  val nWr = p(AccKey).meParams.nWriteClients
  val rd = Vec(nRd, new MEReadClient)
  val wr = Vec(nWr, new MEWriteClient)
}

/** Memory Engine (ME).
 *
 * This unit multiplexes the memory controller interface for the Core. Currently,
 * it supports single-writer and multiple-reader mode and it is also based on AXI.
 */
class ME(implicit p: Parameters) extends Module {
  val io = IO(new Bundle {
    val mem = new AXIMaster(p(AccKey).memParams)
    val me = new MEClient
  })
  val clientCmdQueueDepth = p(AccKey).meParams.clientCmdQueueDepth
  val clientDataQueueDepth = p(AccKey).meParams.clientDataQueueDepth
  val RequestQueueDepth = p(AccKey).meParams.RequestQueueDepth
  val RequestQueueAddrWidth = log2Ceil(RequestQueueDepth.toInt)
  val dataBits = p(AccKey).memParams.dataBits
  val nReadClients = p(AccKey).meParams.nReadClients
  val addrBits = p(AccKey).memParams.addrBits
  val lenBits = p(AccKey).memParams.lenBits
  val idBits = p(AccKey).memParams.idBits
  val meTag_array = SyncReadMem(RequestQueueDepth,(new(clientTag)))
  val meTag_array_wr_data = Wire(new(clientTag))
  val meTag_array_wr_addr = Wire(UInt(RequestQueueAddrWidth.W))
  val meTag_array_rd_addr = Wire(UInt(RequestQueueAddrWidth.W))
  val meTag_array_wr_en  = Wire(Bool())
  val localTag_out  = Wire(new(clientTag))
  val availableEntriesEn = Wire(Bool())
  val availableEntriesNext = Wire(UInt(RequestQueueDepth.W))
  val availableEntries     = Reg(availableEntriesNext.cloneType)
  val freeTagLocation  = Wire(UInt(RequestQueueDepth.W))
  val (resetEntry,newEntry,firstPostn) = firstOneOH(availableEntries.asUInt)
  val updateEntry = Wire(UInt(RequestQueueDepth.W))
  when(io.mem.r.bits.last & io.mem.r.valid){
  availableEntriesNext := updateEntry | availableEntries
  }.elsewhen(availableEntriesEn && availableEntries =/= 0.U && !(io.mem.r.bits.last & io.mem.r.valid)){
  availableEntriesNext:= newEntry
  }.otherwise{
  availableEntriesNext:= availableEntries
  }
  when(reset.asBool){
  availableEntries := VecInit(Seq.fill(RequestQueueDepth)(true.B)).asUInt
  updateEntry := 0.U
  }.otherwise{
  availableEntries := availableEntriesNext
  updateEntry := VecInit(IndexedSeq.tabulate(RequestQueueDepth){ i => i.U === (io.mem.r.bits.id).asUInt }).asUInt
  }
  // Cmd Queues for each ME client
  val MEcmd_Qs = IndexedSeq.fill(nReadClients){ Module(new Queue(new MECmd, clientCmdQueueDepth))}

  //---------------------------------------
  //--- Find available buffer entries -----
  //---------------------------------------
  def firstOneOH (in: UInt) = {
    val oneHotIdx = for(bitIdx <- 0 until in.getWidth) yield {
      if (bitIdx == 0){
        in(0)
      }
      else{
        in(bitIdx) && ~in(bitIdx-1,0).orR
      }
    }
    val oHot = VecInit(oneHotIdx).asUInt
    val newVec = in&(~oHot) // turn bit to 0
    val bitPostn = PriorityEncoder(oneHotIdx)
    (oHot, newVec,bitPostn)
  }
  val default_tag = Wire(new(clientTag))
  default_tag.client_tag  := 0.U
  default_tag.client_id  := 0.U
  default_tag.client_mask := 0.U

  val cmd_valids = for { q <- MEcmd_Qs } yield q.io.deq.valid

  val me_select = PriorityEncoder(cmd_valids :+ true.B)
  val any_cmd_valid = cmd_valids.foldLeft(false.B){ case (x,y) => x || y}
  availableEntriesEn := io.mem.ar.ready & any_cmd_valid

  for { i <- 0 until nReadClients} {
    MEcmd_Qs(i).io.enq.valid := io.me.rd(i).cmd.valid  & MEcmd_Qs(i).io.enq.ready
    MEcmd_Qs(i).io.enq.bits  := io.me.rd(i).cmd.bits
    MEcmd_Qs(i).io.deq.ready := io.mem.ar.ready &
    (me_select === i.U) & (availableEntries.asUInt =/= 0.U) &
    !(io.mem.r.bits.last & io.mem.r.valid)
    io.me.rd(i).cmd.ready := MEcmd_Qs(i).io.enq.ready
  }

  meTag_array_wr_addr := firstPostn.asUInt


  val cmd_readys = for { q <- MEcmd_Qs} yield q.io.deq.ready
  val any_cmd_ready = cmd_readys.foldLeft(false.B){ case (x,y) => x || y}

  meTag_array_wr_en := any_cmd_ready

  when(meTag_array_wr_en){
    val rdwrPort = meTag_array(meTag_array_wr_addr)
    rdwrPort  := meTag_array_wr_data
  }

  io.mem.ar.bits.addr := 0.U
  io.mem.ar.bits.len  := 0.U
  io.mem.ar.valid     := 0.U
  io.mem.ar.bits.id   := 0.U
  meTag_array_wr_data := default_tag

  // Last assign wins so do this in reverse order
  for { i <- nReadClients -1 to 0 by -1} {
    when(MEcmd_Qs(i).io.deq.ready){
      io.mem.ar.bits.addr := MEcmd_Qs(i).io.deq.bits.addr
      io.mem.ar.bits.len  := MEcmd_Qs(i).io.deq.bits.len
      io.mem.ar.valid     := MEcmd_Qs(i).io.deq.valid
      io.mem.ar.bits.id   := meTag_array_wr_addr
      meTag_array_wr_data.client_id  := i.U
      meTag_array_wr_data.client_tag := MEcmd_Qs(i).io.deq.bits.tag
      meTag_array_wr_data.client_mask := resetEntry
    }
  }

  // We need one clock cycle to look up the local tag from the
  // centralized tag buffer meTag_array
  // Adding a flop stage for mem.r.data, mem.r.last, mem.r.valid
  // till local tag lookup is performed.
  io.mem.r.ready  := true.B
  meTag_array_rd_addr :=  io.mem.r.bits.id
  localTag_out         :=  meTag_array(meTag_array_rd_addr)
  freeTagLocation      :=  localTag_out.client_mask

  for (i <- 0 until nReadClients) {
    io.me.rd(i).data.valid := ((RegNext(io.mem.r.valid, init = false.B)) && ((localTag_out.client_id) === i.U)
    && io.me.rd(i).data.ready)
    //ME doesnt stop on not ready
    assert(io.me.rd(i).data.ready || ~io.me.rd(i).data.valid)
    io.me.rd(i).data.bits.data := RegNext(io.mem.r.bits.data, init = false.B)
    io.me.rd(i).data.bits.last := RegNext(io.mem.r.bits.last, init = false.B)
    io.me.rd(i).data.bits.tag  := localTag_out.client_tag
  }

  // ME <-> AXI write interface
  val wr_len = RegInit(0.U(lenBits.W))
  val wr_addr = RegInit(0.U(addrBits.W))
  val sWriteIdle :: sWriteAddr :: sWriteData :: sWriteResp :: Nil = Enum(4)
  val wstate = RegInit(sWriteIdle)
  val wr_cnt = RegInit(0.U(lenBits.W))
  io.me.wr(0).cmd.ready := wstate === sWriteIdle
  io.me.wr(0).ack := io.mem.b.fire
  io.me.wr(0).data.ready := wstate === sWriteData & io.mem.w.ready
  io.mem.aw.valid := wstate === sWriteAddr
  io.mem.aw.bits.addr := wr_addr
  io.mem.aw.bits.len := wr_len
  io.mem.aw.bits.id  := p(AccKey).memParams.idConst.U // no support for multiple writes
  io.mem.w.valid := wstate === sWriteData & io.me.wr(0).data.valid
  io.mem.w.bits.data := io.me.wr(0).data.bits.data
  io.mem.w.bits.strb := io.me.wr(0).data.bits.strb
  io.mem.w.bits.last := wr_cnt === wr_len
  io.mem.w.bits.id   := p(AccKey).memParams.idConst.U // no support for multiple writes
  io.mem.b.ready := wstate === sWriteResp
  when(io.me.wr(0).cmd.fire) {
    wr_len := io.me.wr(0).cmd.bits.len
    wr_addr := io.me.wr(0).cmd.bits.addr
  }
  when(wstate === sWriteIdle) {
    wr_cnt := 0.U
  }
  .elsewhen(io.mem.w.fire){
    wr_cnt := wr_cnt + 1.U
  }
  switch(wstate){
    is(sWriteIdle){
      when(io.me.wr(0).cmd.valid){
        wstate := sWriteAddr
      }
    }
    is(sWriteAddr){
      when(io.mem.aw.ready){
        wstate := sWriteData
      }
    }
    is(sWriteData){
      when(io.me.wr(0).data.valid && io.mem.w.ready && wr_cnt === wr_len) {
        wstate := sWriteResp
      }
    }
    is(sWriteResp) {
      when(io.mem.b.valid) {
        wstate := sWriteIdle
      }
    }
  }
  // AXI constants - statically define
  io.mem.setConst()
}

