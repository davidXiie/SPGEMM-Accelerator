Q1: 为什么有两个 always 块？为什么不合并成一个？
时序 always 块（带时钟）用来更新寄存器（state、load_sub_state 等）。

组合 always 块用来计算下一个状态 state_next。

分开写是两段式状态机的经典写法，清晰且不容易出错。如果合并成一个时序块，state_next 也需要变成寄存器，会多打一拍，导致状态转移延迟一个周期，不符合设计意图。

Q2: state_next = state; 默认赋值有什么作用？
在组合逻辑中，如果某个状态没有对 state_next 赋值，它会保持原来的值（即隐式存储器），综合时会生成锁存器，这通常不是我们想要的。

显式地先赋值为当前状态，然后只在跳转条件满足时改变，可以避免锁存器。

Q3: load_sub_state 为什么在时序块中更新而不是组合块？
load_sub_state 是一个寄存器，需要记忆加载进度。它的更新依赖于当前状态和 load_done 信号，所以放在时钟沿驱动下最合适。

如果放在组合逻辑中，可能会产生毛刺或不必要的环路。

Q4: 为什么 ins_count_curr 只在 STATE_STORE 中增加？
因为每完成一条 SpGEMM 指令（包括加载、计算、存储），才会计数一次。STORE 阶段是最后一步，所以在这里递增。

Q5: cr_launch 是外部控制信号，为什么在 STATE_IDLE 中锁存 ins_count_total 而不是在启动时一次性锁存？
cr_launch 是一个脉冲信号（可能只高一个周期）。在 IDLE 状态下检测到它，就锁存当前的 ins_count（指令条数）到内部寄存器 ins_count_total。这样即使后续 ins_count 变化也不会影响当前运行批次。