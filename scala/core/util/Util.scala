package gcn.core.util

import chisel3._
import chisel3.util._

// Tree of 2 to 1 Muxes to support vector selection based on select

object MuxTree {
  def apply[T <: Data](idx: UInt, vec: Seq[T]): T = {
    require(vec.size > 0)
    require(idx.getWidth >= log2Ceil(vec.size), s"idx.getWidth=${idx.getWidth} should cover vec.size=${vec.size}")
    if (vec.size == 1) {
      vec(0)
    } else if (vec.size == 2) {
      Mux(idx(0), vec(1), vec(0))
    } else { // vec.size > 2
      val idx_msb  = log2Ceil(vec.size) - 1
      val vec_half = 1 << idx_msb
      Mux(idx(idx_msb), apply(idx(idx_msb - 1, 0), vec.drop(vec_half)), apply(idx(idx_msb - 1, 0), vec.take(vec_half)))
    }
  }
}

// RRArbiter with lastGrant register initialization to allow simulation

private object MyArbiterCtrl {
  def apply(request: Seq[Bool]): Seq[Bool] = request.length match {
    case 0 => Seq()
    case 1 => Seq(true.B)
    case _ => true.B +: request.tail.init.scanLeft(request.head)(_ || _).map(!_)
  }
}

class MyLockingRRArbiter[T <: Data](gen: T, n: Int, count: Int, needsLock: Option[T => Bool] = None)
    extends LockingArbiterLike[T](gen, n, count, needsLock) {
  lazy val lastGrant = RegEnable(io.chosen, io.out.fire, true.B)
  lazy val grantMask = (0 until n).map(_.asUInt > lastGrant)
  lazy val validMask = io.in.zip(grantMask).map { case (in, g) => in.valid && g }

  override def grant: Seq[Bool] = {
    val ctrl = MyArbiterCtrl((0 until n).map(i => validMask(i)) ++ io.in.map(_.valid))
    (0 until n).map(i => ctrl(i) && grantMask(i) || ctrl(i + n))
  }

  override lazy val choice = WireDefault((n - 1).asUInt)
  for (i <- n - 2 to 0 by -1)
    when(io.in(i).valid) { choice := i.asUInt }
  for (i <- n - 1 to 1 by -1)
    when(validMask(i)) { choice := i.asUInt }
}


class MyRRArbiter[T <: Data](val gen: T, val n: Int) extends MyLockingRRArbiter[T](gen, n, 1)

/** Hardware module that is used to sequence n producers into 1 consumer.
  * Priority is given to lower producer.
  *
  * @param gen data type
  * @param n number of inputs
  *
  * @example {{{
  * val arb = Module(new Arbiter(UInt(), 2))
  * arb.io.in(0) <> producer0.io.out
  * arb.io.in(1) <> producer1.io.out
  * consumer.io.in <> arb.io.out
  * }}}
  */

