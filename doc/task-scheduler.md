# ArceOS 任务调度框架分析（axtask）

> 分析对象：`modules/axtask/`（本仓库 ArceOS 内核，`make ARCH=aarch64 A=examples/ivc_tester build` 构建）
> 调度算法实现来自外部 crate：`scheduler = { git = "https://github.com/arceos-org/scheduler.git", tag = "v0.1.0" }`

---

## 1. 总体架构

### 1.1 分层结构

```
+-------------------------------------------------------------------+
| axfeat / axstd  (feature 组合层)                                   |
|   sched_fifo = ["axtask/sched_fifo"]                              |
|   sched_rr   = ["axtask/sched_rr", "irq"]                         |
|   sched_cfs  = ["axtask/sched_cfs", "irq"]                        |
+-------------------------------------------------------------------+
| axtask  (调度框架层, modules/axtask/)                              |
|   api.rs        对外 API: spawn / yield_now / sleep / exit ...     |
|   run_queue.rs  每 CPU 运行队列 AxRunQueue + 核心调度逻辑           |
|   task.rs       任务本体 TaskInner / CurrentTask / 状态机 / 抢占计数 |
|   wait_queue.rs 阻塞等待队列 WaitQueue (锁、join、notify 唤醒)      |
|   timers.rs     定时器到点唤醒 (依赖 irq feature)                   |
+-------------------------------------------------------------------+
| scheduler crate (算法层, 统一 trait: BaseScheduler)                |
|   FifoScheduler / RRScheduler / CFScheduler                       |
+-------------------------------------------------------------------+
| axhal  (底层: 中断寄存器/定时器/TaskContext 上下文切换)              |
+-------------------------------------------------------------------+
```

### 1.2 调度器选择（`api.rs:22-35`）

通过 cargo feature 在编译期选择，`AxTask` / `Scheduler` 两个类型别名被整个框架使用：

| feature | Scheduler 类型 | 说明 |
|---|---|---|
| `sched_fifo`（默认） | `FifoScheduler<TaskInner>` | 协作式 FIFO，只隐含 `multitask` |
| `sched_rr` | `RRScheduler<TaskInner, 5>` | 抢占式轮转，隐含 `multitask + preempt`，时间片 = 5 tick |
| `sched_cfs` | `CFScheduler<TaskInner>` | 抢占式完全公平调度，隐含 `multitask + preempt` |

注意 feature 级联关系（`axtask/Cargo.toml`）：

```
multitask = [...]                 # 多任务基础
irq       = []                    # 中断使能（定时器 tick 依赖它）
preempt   = ["irq", ...]          # 抢占必须依赖 irq（抢占最终由时钟中断驱动）
sched_fifo = ["multitask"]        # FIFO 不需要 preempt
sched_rr   = ["multitask", "preempt"]
sched_cfs  = ["multitask", "preempt"]
```

因此组合可能是：`fifo(无irq)` / `fifo+irq(无preempt)` / `rr 或 cfs (必含 irq+preempt)`。

### 1.3 核心数据结构

**每 CPU 运行队列**（`run_queue.rs:32-54`，percpu 静态变量）：

```rust
RUN_QUEUE:    LazyInit<AxRunQueue>   // 当前 CPU 的运行队列
IDLE_TASK:    LazyInit<AxTaskRef>    // idle 任务（就绪队列为空时运行）
EXITED_TASKS: VecDeque<AxTaskRef>    // 已退出任务列表，由 gc 任务回收
WAIT_FOR_EXIT: WaitQueue             // gc 任务睡眠在此
```

`AxRunQueue`（`run_queue.rs:182-189`）只包含两个字段：

```rust
pub struct AxRunQueue {
    cpu_id: usize,
    scheduler: SpinRaw<Scheduler>,   // 具体调度算法（Fifo/RR/CFS）
}
```

**任务状态机**（`task.rs:27-39`）：

```
             spawn/add_task
        +----------------------+
        v                      |
 Running ---yield/put_prev---> Ready ---pick_next---+
     |                                              |
     | wait/sleep/阻塞                              |
     v                                              |
  Blocked ---notify/timer 到点---> Ready            |
     |                                              |
     +--------- resched/switch_to ------------------+
     |
  Exited ----> EXITED_TASKS --> gc 任务 drop
```

**kernel_guard 关键区保护**（外部 crate `kernel_guard`）：调度操作全部通过
`current_run_queue::<G>()` / `select_run_queue::<G>()` 获取带 guard 的队列引用，
guard Drop 时恢复中断/抢占状态：

| Guard | 行为 |
|---|---|
| `NoOp` | 不做任何事（仅限已关中断的上下文，如 IRQ handler 内） |
| `IrqSave` | 关/恢复本地中断 |
| `NoPreempt` | 关/开内核抢占（计数 `preempt_disable_count`） |
| `NoPreemptIrqSave` | 关抢占 + 关中断（调度 API 的标准 guard） |

`preempt` feature 打开时，`enable_preempt(resched=true)` 在抢占计数减到 0 时会检查
`need_resched` 挂起标志并触发真正的抢占（`task.rs:377-396`），这是"延迟抢占"机制的核心。

### 1.4 一次调度的完整流程

以 `yield_now()`（`api.rs:181-183`）为例：

```
yield_now()
 └─ current_run_queue::<NoPreemptIrqSave>()     // 关抢占+关中断
     └─ yield_current()                          // run_queue.rs:286
         ├─ put_task_with_state(curr, Running, false)
         │    └─ scheduler.put_prev_task(prev, preempt)  // prev 放回就绪队列
         └─ resched()                            // run_queue.rs:509
              ├─ scheduler.pick_next_task()      // 按算法选出 next
              ├─ (空则取 IDLE_TASK)
              └─ switch_to(current, next)        // run_queue.rs:527
                   ├─ assert IRQs disabled       // 调度过程必须关中断
                   ├─ next.set_state(Running)
                   ├─ CurrentTask::set_current(prev, next)
                   └─ prev_ctx.switch_to(&next_ctx)   // axhal 汇编上下文切换
```

---

## 2. 三种调度算法

三者都实现统一 trait `BaseScheduler`（scheduler crate `lib.rs:28-68`）：

```rust
fn init(&mut self);
fn add_task(&mut self, task: Self::SchedItem);              // 新任务入队
fn remove_task(&mut self, task: &Self::SchedItem) -> ...;   // 出队
fn pick_next_task(&mut self) -> Option<Self::SchedItem>;    // 选下一个运行任务
fn put_prev_task(&mut self, prev: Self::SchedItem, preempt: bool); // 前任务回队
fn task_tick(&mut self, current: &Self::SchedItem) -> bool; // 每个 tick 调用，返回是否需要重调度
fn set_priority(&mut self, task: &Self::SchedItem, prio: isize) -> bool;
```

### 2.1 FifoScheduler —— 协作式 FIFO（`fifo.rs`）

- 就绪队列：侵入式链表 `linked_list::List<Arc<FifoTask<T>>>`，入队/出队 O(1)。
- `add_task` / `put_prev_task`：一律 `push_back`（**忽略 `preempt` 参数**，永远放队尾）。
- `pick_next_task`：`pop_front`，先来先运行。
- `task_tick`：**恒返回 `false`** —— 时钟 tick 对它毫无作用，从不因时间片到期而切换。
- `set_priority`：不支持，恒返回 `false`。

结论：FIFO 是纯协作式调度，任务切换只发生在任务**主动**调用调度 API 时。

### 2.2 RRScheduler —— 抢占式轮转（`round_robin.rs`）

- 就绪队列：`VecDeque`，`remove_task` 需 O(n) 遍历。
- 每个任务带原子时间片计数器 `time_slice`，创建时初始化为 `MAX_TIME_SLICE = 5`
  （`api.rs:24`，即 5 个时钟 tick）。
- `task_tick`：`time_slice.fetch_sub(1)`，当旧值 `<= 1`（即减到 0）返回 `true`，
  通知框架当前任务时间片耗尽、需要重调度。
- `put_prev_task(prev, preempt)` 的语义（`round_robin.rs:96-103`）：
  - `preempt == true && time_slice > 0`：任务是被高优先级/事件抢占的，**放回队首**
    （保留剩余时间片，回来后优先继续运行）；
  - 否则：重置时间片并 `push_back` 到队尾（时间片用完的正常轮转）。
- `set_priority`：不支持。

### 2.3 CFScheduler —— 完全公平调度（`cfs.rs`）

- 就绪队列：`BTreeMap<(vruntime, taskid), task>`，key 排序即"最小 vruntime 最先运行"，
  `pick_next_task` 即 `pop_first`，O(log n)。
- 每任务维护 `vruntime = init_vruntime + delta * 1024 / weight`：
  - `delta` 每 tick 加 1（实际运行时间）；
  - `weight` 由 nice 值查表得到（Linux 权重表，nice=0 时 weight=1024），
    nice 越小（优先级越高）权重越大，vruntime 增长越慢，从而获得更多 CPU 时间。
- `task_tick`：`delta += 1`；当**当前任务 vruntime > 队列最小 vruntime** 时返回 `true`
  （当前任务已经"跑过头"了，应当让给 vruntime 最小的任务）→ 触发抢占。
- `set_priority`：支持，`prio` 为 nice 值，范围 `-20..=19`，越界返回 `false`。

### 2.4 三者对比

| | FifoScheduler | RRScheduler | CFScheduler |
|---|---|---|---|
| 调度方式 | 协作式 FIFO | 抢占式时间片轮转 | 抢占式按 vruntime 公平 |
| 就绪队列 | 侵入式链表 O(1) | VecDeque，remove O(n) | BTreeMap O(log n) |
| task_tick | 恒 false | 时间片减到 0 返回 true | vruntime 超过 min 返回 true |
| 优先级 | 不支持 | 不支持 | nice (-20..19) |
| 需要 preempt/irq | 否 | 是 | 是 |
| put_prev_task 抢占插入 | 无视，放队尾 | 抢占时保片放队首 | 忽略，按 vruntime 重新入树 |

---

## 3. 任务切换的触发点：结合 irq 与 preempt

`switch_to()` 开头有断言（`run_queue.rs:529-533`）：

```rust
assert!(!axhal::arch::irqs_enabled(), "IRQs must be disabled during scheduling");
```

即**所有切换都发生在关中断 + 关抢占的关键区内**，由 `kernel_guard`（或 IRQ handler 自身）
保证。切换可分为两大类：

### 3.1 主动切换（任务自身调用调度 API，不依赖 irq/preempt）

| API | 路径 | 场景 |
|---|---|---|
| `yield_now()` | `yield_current()` → put_prev(Running) + resched | 任务主动让出 CPU |
| `exit(code)` | `exit_current()` → 置 Exited、通知 join 者、进 EXITED_TASKS、resched | 任务结束（`task_entry` 执行完入口函数后也会自动 `exit(0)`） |
| `WaitQueue::wait()` / `wait_until()` | `blocked_resched()` → 置 Blocked、挂入等待队列、resched | 等锁 / 等条件 / `task.join()` |
| `sleep(dur)` / `sleep_until()` | `sleep_until()` → 挂定时器 + 置 Blocked + resched；**无 irq 时退化为 `busy_wait_until` 忙等、不切换** | 定时睡眠 |
| `set_current_affinity()`（smp） | `migrate_current()` → 切到 migration 任务迁移自身 | 亲和性改变 |

### 3.2 被动切换（抢占，依赖 irq + preempt）

抢占链路（各文件行号为当前代码位置）：

1. **时钟中断**：`axruntime/src/lib.rs:257-261` 注册 `TIMER_IRQ_NUM` 处理函数，
   每 `TICKS_PER_SEC` 周期触发，中断上下文中调用 `axtask::on_timer_tick()`（`api.rs:94-100`）。
   此处 IRQ 已被硬件关闭，因此用 `NoOp` guard 即可。

2. **tick 推进调度器状态**：`scheduler_timer_tick()`（`run_queue.rs:274-281`）：
   ```rust
   if !curr.is_idle() && scheduler.task_tick(curr) {
       curr.set_preempt_pending(true);   // 仅置"待抢占"标志，不立即切换！
   }
   ```
   注意这里**并没有直接切换**，只是给当前任务打上 `need_resched` 挂起标记
   （idle 任务永不抢占）。对 FIFO 调度器 `task_tick` 恒为 false，此链路无效。

3. **定时器到点唤醒别的任务**：`timers.rs:35` 中 `unblock_task(task, resched=true)`，
   如果被唤醒任务就在本 CPU 的队列上且 `preempt` 开启，同样只置
   `set_preempt_pending(true)`（`run_queue.rs:263-267`）。`WaitQueue::notify_one(true)` 唤醒
   本 CPU 任务同理。

4. **延迟抢占点**：真正的切换发生在接下来某个"抢占重新打开"的边界：
   - 内核里所有 `NoPreempt` / `NoPreemptIrqSave` guard Drop 时，若抢占计数从 1 减到 0，
     调用 `enable_preempt(true)` → `current_check_preempt_pending()`（`task.rs:384-396`）：
     检查 `need_resched && can_preempt(0)`，满足则进入 `preempt_resched()`（`run_queue.rs:328-352`）
     —— 把当前任务以 `preempt=true` 放回就绪队列（RR 会因此保片插队首）并 resched；
   - `preempt_resched` 内部再次校验 `can_preempt(1)`：若此时嵌套关了抢占（如又进了别的
     关键区），则只能把 pending 标志留着，推迟到下一次 guard 释放。

5. **多核（smp）补充**：跨 CPU 唤醒时无法直接置远端 CPU 的 pending，
   本实现选择"唤醒的任务进入远端队列，由远端 CPU 下次调度时自然选出"。

```
时钟中断 ──► task_tick()==true ──► need_resched = true ─┐
                                                        ├─► guard Drop (enable_preempt)
notify/timer 唤醒本CPU任务 (resched=true) ─► need_resched┘        │
                                                                 ▼
                                                  can_preempt(0)? ──yes──► preempt_resched()
                                                                 │              put_prev(preempt=true)
                                                                 └──no(仍关抢占)  resched() → switch_to
                                                                     │
                                                                     └─► 保持 pending，下个边界再试
```

### 3.3 各 feature 组合下的切换时机汇总

| 配置 | 会被时钟 tick 打断？ | 切换时机 |
|---|---|---|
| fifo（无 irq，无 preempt） | 否（根本没有 tick） | 仅 `yield_now` / `exit` / `WaitQueue::wait`（等价于死等）；`sleep` 为忙等 |
| fifo + irq（无 preempt） | tick 到来但 `task_tick` 恒 false，不打断 | 同上；新增 `sleep`/`wait_timeout` 真正睡眠，由定时器到点唤醒后重新入队 |
| rr / cfs（irq + preempt） | 是：时间片耗尽（RR）或 vruntime 超前（CFS），以及高优先唤醒（notify/sleep 到点，resched=true） | 上述被动抢占点 + 全部主动切换点 |

---

## 4. 无中断 FIFO 调度的切换条件（问题解答）

`FifoScheduler` 下 `task_tick` 恒返回 `false`，且若连 `irq` feature 都未开启，系统完全没有
时钟中断，因此**不存在任何被动切换**。任务一旦运行，将一直占用 CPU，直到发生下列
事件之一（全部属于"任务主动交出 CPU"）：

1. **任务调用 `axtask::yield_now()`** —— 自愿让出，重新排队尾。
2. **任务退出** —— `axtask::exit(code)`，或任务入口函数正常返回后由 `task_entry`
   自动 `exit(0)`（`task.rs:543-557`），触发 `exit_current()` 切换到下一任务。
3. **任务在 `WaitQueue` 上阻塞** —— 调用 `wait()` / `wait_until()`（锁、条件变量、
   `task.join()` 等），`blocked_resched()` 使其进入 Blocked 态并切换出去；
   之后须由其他任务调用 `notify_one/notify_all/notify_task` 将其放回就绪队列。
4. （若启用了 `irq` 但仍用 FIFO）**`sleep_until` / `wait_timeout` 阻塞** ——
   挂定时器后切换出去，到点由时钟中断唤醒。无 irq 时此条不存在（sleep 退化为忙等）。

与之相对，下列情况在无中断 FIFO 下**不会**引起切换：

- 时钟 tick：既无中断，`task_tick` 也恒 false；
- `spawn()` 新任务：只是 `add_task` 入队尾，当前任务继续运行；
- 修改优先级：FIFO 不支持，无效果；
- idle 任务：就绪队列空时运行 `run_idle()`，它内部就是死循环 `yield_now()`
  （无 irq 时连 `wait_for_irqs` 都没有，纯自旋），一旦有任务入队即被选中。

一句话总结：**无中断的 FIFO 调度是完全协作式的，只有当前任务"自己不想跑了"
（yield、exit、阻塞等待）时才会发生任务切换；任何外部事件（时间流逝、新任务诞生）
都不能强制夺走它的 CPU。**

---

## 5. 其他设计要点

- **GC 任务**：每 CPU 队列初始化时内置一个 `gc` 任务（`run_queue.rs:449-460`），
  退出任务先进 `EXITED_TASKS`，由 gc 任务延迟 `drop`，避免在切换临界区里释放栈内存
  （`switch_to` 之后 prev 的强引用计数才递减，见 `run_queue.rs:561-568` 注释）。
- **idle 任务**：每 CPU 一个，仅当 `pick_next_task()` 返回 None 时兜底运行；
  被 `resched()` 显式排除在 `put_task_with_state` 之外，永不入队。
- **smp**：每 CPU 独立运行队列；新任务按 cpumask + 全局轮转计数选队列
  （`select_run_queue_index`）；`on_cpu` 标志 + 自旋等待（`put_task_with_state`，
  `run_queue.rs:491-496`）保证远端核唤醒阻塞任务时，等对方 CPU 的切换流程走完；
  `set_current_affinity` 通过临时 migration 任务实现自迁移。
- **上下文切换的原子性**：切换全程要求 IRQ 关闭（`switch_to` 有断言），
  `CurrentTask::set_current` 在切换前更新 percpu 的 current 指针，
  保证任意时刻每核的 `current()` 都指向真实运行中的任务。

---

## 6. 深入：运行任务的"被动切换"有哪些方式

"被动切换"指：当前任务没有调用任何让出 CPU 的 API，却被外部事件夺走 CPU。
它的**唯一实现机制**是"抢占挂起标志 + 延迟抢占点"：

```
外部事件 ──► 置位 current.need_resched (preempt pending) ──► 某个抢占检查点真正执行切换
```

框架中只有两处代码会置位 pending（都要求 `feature = "preempt"`）：

### 6.1 触发方式一：时钟 tick 到期（`scheduler_timer_tick`）

代码路径（`run_queue.rs:274-281`）：

```rust
pub fn scheduler_timer_tick(&mut self) {
    let curr = &self.current_task;
    if !curr.is_idle() && self.inner.scheduler.lock().task_tick(curr.as_task_ref()) {
        #[cfg(feature = "preempt")]
        curr.set_preempt_pending(true);   // 只打标记，不切换
    }
}
```

`task_tick()` 返回 `true` 的条件因调度器而异：

| 调度器 | tick 触发抢占的条件 | 备注 |
|---|---|---|
| RR | 时间片自减后耗尽（旧值 `<= 1`，即减到 0） | 时间片 = 5 tick |
| CFS | 当前任务 `vruntime > min_vruntime`（跑得比最该跑的久） | nice 权重影响 vruntime 增速 |
| FIFO | 永不（恒返回 false） | FIFO 无 tick 抢占 |

额外条件：当前任务必须是普通任务，**idle 任务永不因 tick 抢占**（`!curr.is_idle()` 判断）。

### 6.2 触发方式二：唤醒任务时附带 resched 请求（`unblock_task`）

当一个 Blocked 任务被唤醒入队时，唤醒者可携带 `resched=true` 语义：

```rust
// run_queue.rs:247-268
pub fn unblock_task(&mut self, task: AxTaskRef, resched: bool) {
    if put_task_with_state(task, Blocked, resched) {   // Blocked → Ready
        if resched && cpu_id == this_cpu_id() {
            #[cfg(feature = "preempt")]
            crate::current().set_preempt_pending(true); // 给"当前任务"打标记
        }
    }
}
```

关键限制：
- **只在本 CPU 生效**：`cpu_id == this_cpu_id()`，即被唤醒任务恰好落到了唤醒者所在 CPU
  的运行队列上，才会请求抢占当前任务；跨 CPU 唤醒只是把任务放入远端队列，
  由远端 CPU 自行调度（无 IPI 强制重调度）。
- **只影响"当前正在运行的任务"**：pending 标志打在 `crate::current()` 上。

上游调用 `resched=true` 的场景：

| 场景 | 代码位置 | 执行上下文 |
|---|---|---|
| sleep 到点唤醒 | `timers.rs:35` `TaskWakeupEvent::callback` | **时钟中断内** |
| `WaitQueue::notify_one/notify_all/notify_task(true)` | `wait_queue.rs:171-203` | 普通任务上下文（如解锁、条件变量通知、`join` 通知） |
| `notify_exit`（任务退出通知 join 者） | `task.rs:399-402`，`notify_all(false)` | resched=false，不请求抢占 |

> 注：`resched` 只是一个"请求"。即使打了 pending，是否真切换还取决于抢占检查点处的
> `can_preempt()` 状态（见第 7 节）。而 `smp` 之外的单核场景同样遵循此机制。

### 6.3 汇总：被动切换全景表

| # | 被动切换方式 | 需要 irq | 需要 preempt | FIFO 下有效？ |
|---|---|---|---|---|
| 1 | RR 时间片耗尽（时钟 tick） | ✔ | ✔ | ✘（task_tick 恒 false） |
| 2 | CFS vruntime 超前（时钟 tick） | ✔ | ✔ | ✘ |
| 3 | 本 CPU 唤醒其他任务时 `resched=true`（notify/定时器到点） | ✔ | ✔ | ✘（pending 机制被编译剔除） |
| 4 | IPI 回调（`axipi::ipi_handler`）间接唤醒任务 | ✔(ipi) | ✔ | 间接：IPI 只是通用回调队列，若回调中 `notify(true)` 唤醒本 CPU 任务同样置 pending |

除上述方式外没有任何其他被动切换入口——框架中置位 `need_resched` 的代码点仅
`run_queue.rs:265`、`run_queue.rs:279`、`run_queue.rs:350`（fallback 重置）三处。

---

## 7. 任务 preempt 抢占的完整过程

ArceOS 采用 **"置位标记 + 延迟检查点"** 的抢占模型（类似 Linux 的 need_resched +
TIF_NEED_RESCHED），而非在事件发生点直接切换。下面按时序完整走一遍（以
aarch64 + `sched_rr`、任务 T1 正常运行、无嵌套关键区为例）：

### 7.1 阶段一：中断进入，执行 tick 逻辑

```
1. 时钟中断触发（GIC → CPU），硬件自动关中断，跳转 trap.S 异常向量
2. 保存被打断任务 T1 的 TrapFrame（寄存器现场）到 T1 的内核栈
3. handle_irq_exception() → handle_trap!(IRQ) → axhal::irq::handler_irq()   [axhal/src/irq.rs:53]
4. handler_irq 创建 kernel_guard::NoPreempt：
       T1.preempt_disable_count: 0 → 1        ← 关键：给"被打断的任务"加抢占计数
5. dispatch_irq() → 平台 GIC 分发 → 注册的时钟处理函数:
       update_timer();                        [axruntime/src/lib.rs:246-255]
       axtask::on_timer_tick();               [api.rs:94-100]
           ├─ timers::check_events()          // 先处理到点的睡眠唤醒（unblock_task(resched=true)）
           └─ scheduler_timer_tick()          // 再推进调度状态
               └─ task_tick(T1) == true       // RR: 时间片耗尽
                   └─ T1.need_resched = true  // 只置位，不切换
```

### 7.2 阶段二：中断退出时的延迟抢占检查点（真正切换发生处）

```
6. handler_irq 中 drop(guard):
       NoPreempt::release → enable_preempt(true)        [api.rs:49-53]
       T1.preempt_disable_count: 1 → 0
       计数减到 0 且 resched=true → current_check_preempt_pending()  [task.rs:384-396]
           条件: T1.need_resched == true && T1.can_preempt(0)（计数==0）
           ↓ 满足
7. current_run_queue::<NoPreemptIrqSave>()     // 再次关抢占(0→1) + 关中断
   → rq.preempt_resched()                     [run_queue.rs:328-352]
       ├─ can_preempt(1) 校验：计数必须恰为 1（本 guard 独占）
       │    若不满足（嵌套了其他关键区）→ 保留 pending，本次放弃，return
       ├─ put_task_with_state(T1, Running, preempt=true)
       │    └─ scheduler.put_prev_task(T1, preempt=true)
       │         RR:  T1.time_slice>0? → push_front（保剩余片，优先回跑）
       │               T1.time_slice==0 → reset_time_slice + push_back（正常轮转）
       │         CFS: 忽略 preempt，按 vruntime 重新插入 BTreeMap
       ├─ resched():
       │    ├─ pick_next_task() → T2          // RR/CFS 选出下一个任务
       │    └─ switch_to(T1, T2)              [run_queue.rs:527-575]
       │         ├─ assert IRQs disabled      // 调度必须关中断
       │         ├─ T2.set_state(Running)，清 T2 的 pending
       │         ├─ CurrentTask::set_current(T1, T2)  // 更新 percpu current 指针
       │         └─ T1.ctx.switch_to(&T2.ctx) // 汇编上下文切换，SP 切到 T2 内核栈
8. 此刻 CPU 已经在执行 T2 的代码（仍处于 T1 被打断时的那次中断处理流程内）
9. handler_irq 返回 → trap.S 以 T2 的栈继续执行中断返回（eret）
```

### 7.3 阶段三：被抢占任务 T1 的恢复

T1 的执行流被冻结在 `switch_to` 内 `(*prev_ctx_ptr).switch_to(...)` 这条调用里
（上下文保存在 T1 的 `ctx` + 内核栈上的 TrapFrame）。未来某次 `resched()` 选中 T1 时：

```
switch_to(T_prev, T1) → 恢复 T1 的寄存器上下文
  → T1 从 switch_to 内部继续返回
  → 逐层返回: preempt_resched → current_check_preempt_pending → enable_preempt
  → handler_irq 的 drop(guard) 之后 → 返回 true
  → trap.S 恢复 T1 当初中断时压栈的 TrapFrame → eret 回到被打断的下一条指令
```

即：**抢占对被中断任务完全透明，它恢复时就像什么都没发生过**（时钟中断是异步的，
T1 的 pending 标志已被 `switch_to` 开头为 next 清 pending 的逻辑清掉，
时间片在下次 put_prev_task 时重置）。

### 7.4 抢占的"推迟"情形

若打 pending 时任务正处在更深的关键区，抢占会被推迟到计数归零的边界：

```rust
// task.rs:377-382
pub(crate) fn enable_preempt(&self, resched: bool) {
    if self.preempt_disable_count.fetch_sub(1) == 1 && resched {
        Self::current_check_preempt_pending();   // 只有最后一次 unlock 才检查
    }
}
```

- `preempt_resched` 中 `can_preempt(1)` 不满足时，重新置位 pending（`run_queue.rs:349-351`），
  等待下一个边界再试；
- 典型推迟场景：任务持有 `SpinNoIrq`/`SpinNoPreempt` 自旋锁（各自通过 `kernel_guard`
  增加计数）期间收到 tick → 直到最外层锁 guard 释放才发生切换。这保证了持锁临界区
  不会被切走，避免死锁与数据竞争。

### 7.5 其他抢占检查点

除 `handler_irq` 的 `drop(guard)` 外，所有 `NoPreempt`/`NoPreemptIrqSave` guard 的
Drop 都是潜在抢占点，例如：

- 任务调用任何调度 API（`yield_now`/`exit`/`sleep`/`wait`...）时 `current_run_queue`
  guard 的释放；
- 任务释放 `WaitQueue` 的自旋锁后（`wait_queue.rs:100` 注释 "Preemption may occur here"）——
  这就是"唤醒者唤醒了本 CPU 更优任务后自己被抢占"的路径；
- 新任务入口 `task_entry` 开中断前（`task.rs:549-551`）。

本仓库 `axhal/src/irq.rs:56-62` 的中文注释还记录了一个实测问题：抢占切换发生在
中断处理函数内部，`drop(guard)` 会导致当前 SP 改变；在 hypervisor（`hv` feature）
场景下，若中断返回路径 `arm_vcpu::HANDLE_CURRENT_IRQ` 沿用切换前的 SP 执行
`.Lexception_return_el2`，就会产生 Instruction Abort（EL2 取指异常）——这正是
"切换发生在中断退出检查点"这一设计的直接后果，也是本移植版特别标注的注意点。

---

## 8. 无 IRQ 中断时的被动切换分析

### 8.1 结论先行

**没有 irq 时，系统中不存在任何被动切换。** 运行中的任务只会在主动调用调度 API
时切换走，任何外部事件都无法强制夺走它的 CPU。

### 8.2 为什么无 irq 就没有被动切换（三层保障）

1. **feature 依赖层面（编译期剔除）**：
   ```
   preempt = ["irq", ...]     // axtask/Cargo.toml:26 —— 抢占强依赖中断
   ```
   关闭 `irq` 时 `preempt` 无法启用，于是：
   - `TaskInner::need_resched`、`preempt_disable_count` 字段根本不存在（`task.rs:70-73` 的
     `#[cfg(feature = "preempt")]`）；
   - `set_preempt_pending()` / `set_preempt_pending` 调用点全部编译剔除
     （`run_queue.rs:264-266`、`run_queue.rs:278-280`）；
   - `unblock_task` 的 `resched` 参数被完全忽略；
   - `kernel_guard` 的抢占开关退化为 no-op。

2. **运行时层面**：`axruntime::init_interrupt()` 只在 `feature = "irq"` 下编译
   （`axruntime/src/lib.rs:235`），无 irq 就没有时钟中断注册，`on_timer_tick()`
   永远不会被调用，tick 类抢占（RR 时间片/CFS vruntime）无从谈起。

3. **调度器层面**：即便强行给出 `task_tick` 的调用（不可能），FIFO 调度器也恒返回
   `false`；而 RR/CFS 本身要求 `preempt`，与无 irq 前提矛盾。

### 8.3 无 irq 时逐一检查"疑似被动切换"的路径

| 路径 | 无 irq 时的实际行为 | 是否被动切换 |
|---|---|---|
| 时钟 tick 抢占 | 中断不存在 | ✘ |
| 唤醒任务 `unblock_task(resched=true)` | `resched` 被忽略，目标任务仅进入就绪队列排队，当前任务继续运行 | ✘（只是改变未来谁先被选中） |
| `notify_one/notify_all` | 同上，唤醒即入队，不切换 | ✘ |
| IPI | `ipi` feature 依赖 `irq`（`axfeat/Cargo.toml:24` `ipi = ["irq", ...]`），无 irq 时不存在 | ✘ |
| `sleep()` / `sleep_until()` | 退化为 `axhal::time::busy_wait_until()` 忙等（`api.rs:195-200`），**不切换、不让出 CPU** | ✘（连主动切换都没有） |
| spawn 新任务 | `add_task` 入队尾，当前任务继续 | ✘ |
| `yield_now` / `exit` / `WaitQueue::wait` | 正常工作（协作式主动切换） | 属主动切换 |

### 8.4 无 irq 时任务的完整行为画像

无 irq 的 FIFO 系统（如 `sched_fifo` 且不带 `irq`）本质是一个**纯协作式**的多任务系统：

- 调度点只有三个：`yield_now()`（让出）、`exit()`（退出）、`WaitQueue::wait()/wait_until()`
  （阻塞等通知）；
- `sleep` 是自旋忙等，占用 CPU 直到超时；
- 就绪队列空时 idle 任务死循环 `yield_now()`（无 irq 连 `wait_for_irqs` 都没有，
  `api.rs:210-216`）；
- "唤醒"（notify）只是把目标任务放回就绪队列，要等到未来某个调度点才会真正运行。

> 对比：`fifo + irq`（但无 preempt）配置下虽然有时钟中断、能真正睡眠/超时唤醒，
> 但同样没有被动切换——因为 `need_resched` 机制被编译剔除，tick 对 FIFO 又无效。
> **被动切换当且仅当 `preempt` feature 开启（即 `sched_rr` 或 `sched_cfs`）才存在。**

### 8.5 工程含义

- 无 irq FIFO 适合确定性要求高、任务自身会主动让出的场景（如裸机轮询式服务循环）；
- 任何计算密集型死循环（`loop {}`、长耗时数值计算且不调用 yield/锁等待）在无 irq
  配置下会**永久霸占 CPU**，饿死同核其他任务——部署前必须确认每个任务都有让出点；
- 需要"任务失控也能被夺回 CPU"的保证时，必须启用 `sched_rr`/`sched_cfs`
  （隐含 irq + preempt）。
