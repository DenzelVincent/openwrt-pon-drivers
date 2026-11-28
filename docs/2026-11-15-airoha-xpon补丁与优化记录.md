# airoha-xpon 补丁与优化记录（bugfix）

起始：2026-11-15
上游：`pbs05/openwrt-pon-drivers`，对比基准 `upstream/main 47ad313`（2026-11-16，2026-11-24 拉取）。
重建基线：`main 8ca3357`（2026-11-28）；旧 bugfix 保存在 `backup/bugfix-pre-rebuild-20261128`，本轮不恢复旧功能。
范围：本分支记录相对上游与相对主分支的优化；固件侧见 PonWrt 台账，用户态见 userspace 台账。

记录规范：

- 已合入主分支的项目归「相对上游」；bugfix 独有项目归「相对主分支」，chore/文档整理合并成一行。
- 表列：优化项、优化方案简介、关联提交、验收状态、验收说明。
- 验收状态列只记录实机验证结果（`已验收`/`待验收`，括号内可注场景或上机情况）；其余记 `—`。

## 一、相对上游

| 优化项 | 优化方案简介 | 关联提交 | 验收状态 | 验收说明 |
|---|---|---|---|---|
| XG-PON MBI RX 溢出风暴自愈 | 移植上游 #49：IRQ 累计 RX MBI overrun，连续 32 次且距上次复位 ≥60 s 才复位 MAC epoch（保留 PHY RX，ONU 回 O1，O5 约 17 s 重建） | `3929816` | 待验收（已上机观察中） | 正常时 overrun 应恒 0；下次卡死看自愈触发与 O5 重建耗时 |
| PLOAMd IRQ 锁反转 + AES 切换超时传播 | 移植上游 #1：修 PLOAMd IRQ 锁反转，AES key switch 超时向上传播 | `5898477` | 待验收（已上机） | PLOAMd 事件不丢不挂；AES key switch 失败能上报 |
| MBI RX 卡死现场与分段诊断 | 纯观测：首 overrun 时快照 PHY sync/PLOAMd FIFO/XGEM RX/MBI stop 并打环形增量；O5 稳态 idle 采样；latched 值跨 epoch 保留 | `ea612e5` | 待验收（11-24 已上机） | 不改阈值与复位路径；采集与判读见第三节 |
| MBI 250ms 预故障环形观测 | 纯观测：采样 5 s→250 ms、保留 16 点≈4 s；首 overrun 冻结 `mbi_fault_trace` 并打增量+逐点；`bwm_check`/`rx_error` 记本 epoch 首现时间；`mac` dump 增实时 XGEM/XGTC/PLOAMD | `04f2b21` | 待验收（未上机） | 下次 MBI 卡死按 `MBI pre-fault trace` 逐点判先后；读 `mac` 对比实时计数 |
| 忽略 IDE 与 tmp、忽略文件补末尾换行 | 忽略 IDE 与 tmp 目录，忽略文件补末尾换行 | `1971bd6`、`bd82f7a` | — | 仅仓库整理，不进固件 |

## 二、相对主分支（bugfix 新增）

本轮第一、二梯队没有驱动仓库代码项；RX 环、QDMA 诊断、SRAM 与 fullcone2 均在 PonWrt 固件仓库记录。

## 三、MBI RX 卡死观测（`ea612e5` + `04f2b21`）

### 补丁行为

- 每个 MAC epoch 的首个 RX MBI overrun：在 W1C 清中断前快照 PHY sync、PLOAMd FIFO、XGEM RX 计数、MBI stop，并在 dmesg 打出 4 s 环形增量与逐点明细（每 epoch 一次）。
- O5 稳态期间每 250 ms 采样 MAC 入口（`RX_XGEM_CNT`）、`RX_XGTC_CNT`、`RX_PLOAMD_CNT`、GDM 交付/丢弃（`airoha_eth` 统计），保留 16 点≈4 s；首个 overrun 时冻结到 `mbi_fault_trace`，因此 epoch 复位后仍能从 debugfs 读回。
- `bwm_check`(0x1)/`rx_error`(0x80) 没有中断源，仅由采样器记录本 epoch 首次非零的 uptime；epoch 复位时转存 `last_epoch_*_first_ms`。
- 累计到 32 次、执行复位前打印两段停滞时长；上一 epoch 的 `fifo_error`/`bwm_check` 锁存值另存到 `last_epoch_latched_*`。
- 除上述观测外，阈值、复位路径与既有自愈行为都不变。

### 实机事件

- 2026-11-23 19:14:46（带 `mbi_*` 固件，开机 38 分钟）：`first RX MBI overrun` → int=0x8000 连报 → storm（xgem_rx=56059162 / delivered=9222597 / dropped=0；两者均"4490 ms 前"停，当时 5 s 采样分辨率无法判先后）；自愈 epoch reset 成功，19:15:57 OLT Deactivate，19:16:16 PPPoE 恢复，**中断 89 s**；此后 19.7 h 无复发。
- 2026-11-24 14:57:07：`PON line restart requested`（用户手动触发 `airoha-pon-debug`）→ 17 s 恢复；不是 MBI 故障。
- 2026-11-24 下载期约 11 分钟（35–36 MB/s）：`mbi_payload_overrun_irq=32 / header=29 / mbi_reset=1` 全为开机累计、零变化 ⇒ 下载负载不是触发器；计数非零只说明开机/注册期发生过。

### 信息在哪看

两条现场诊断都是**内核日志**（`dev_warn`），不是用户态服务日志：

```sh
dmesg | grep -E "first RX MBI|MBI RX overrun storm|MBI pre-fault|  MBI trace"
```

也可在 LuCI“状态 → 内核日志”查看；`logread` 只有在构建把内核消息汇入 logd 时才含这些行。

- `first RX MBI overrun this epoch: uptime=… events=… fifo=… tx=… rx=… bwm=… phy_sync=… ploamd_fifo=… xgem_rx=… mbi_stop=… bwm_first_ms=… rx_err_first_ms=…`
  首次 overrun 即出现，不必到阈值；`fifo` 的 `0x80` 是 header overrun、`0x100` 是 payload overrun，可确认事故类型与上游是否同步；`*_first_ms` 是本 epoch `bwm_check`/`rx_error` 首次非零距今的毫秒数。
- `MBI pre-fault trace: N samples over M ms; xgem +… xgtc +… ploamd +… delivered +… dropped +…`
  紧跟在首行之后，是冻结的 4 s 环形窗口的净增量；随后每两点打一行 `  MBI trace t-N ms: xgem=… xgtc=… ploamd=… delivered=… dropped=… bwm=… rx_error=…`，`t-0` 是最早的一点。看谁先停。
- `MBI RX overrun storm: XGEM RX changed N ms ago, GDM RX delivered changed N ms ago (xgem_rx=… delivered=… dropped=…)`
  只有累计 32 次、即将复位时才出现。线正常时应完全没有这几行。

实时值看 debugfs 伪文件（不是日志）：

```sh
cat /sys/kernel/debug/airoha-xpon-pon0/mac
```

关键字段：实时 `rx_xgem_cnt`/`rx_xgtc_cnt`/`rx_ploamd_cnt`、`xgem_rx_idle_ms`、`rx_delivered_idle_ms`、`mbi_first_fault_seen/uptime_ms/fifo_error/xgem_rx/phy_sync/ploamd_fifo/mbi_stop`、`latched_fifo_error_prev`、`latched_bwm_check_prev`、`bwm_first_ms`/`rx_err_first_ms`（本 epoch）与 `bwm_first_ms_prev`/`rx_err_first_ms_prev`（上一 epoch）、`mbi_header_overrun_irq`、`mbi_payload_overrun_irq`、`mbi_reset`，以及 `mbi_trace_count`/`mbi_fault_trace_count` 与逐点 `mbi_fault_trace[i]`。实时的三个计数器每次读都刷新，可以直接 `a`/`b` 对比停滞与否。

`mbi_first_fault_*` 在新 MAC epoch 开始时清零，因此要么在首次 overrun 后、复位前抓，要么只看复位后仍保留的 `*_prev` 与 dmesg 留痕。`idle_ms` 是“距最后一次变化”的时长，只在 O5 稳态下才有判读意义。

提交号与合入情况只看本仓库 git 和本文表格，日志里没有。

### 判读

| XGEM RX | GDM delivered | 指向 |
|---|---|---|
| 停滞 | 停滞 | MAC 入口没再收帧。看 `mbi_first_fault_phy_sync`/`ploamd_fifo`：正常则更像 MBI 入口状态机失步；异常则是上游断光/PLOAM 问题 |
| 停滞 | 仍增长 | 入口已停但 MBI 仍在排空缓存；隔一轮再采，持续则同上 |
| 增长 | 停滞 | MAC 已收、MBI→GDM2/QDMA 交接卡住（GDM 背压/环满），与 11-23 拨号异常方向一致 |
| 增长 | 增长 | 未卡死，或采样窗口落在恢复之后 |

`delivered` 来自 `airoha_eth` 统计接口，读取失败时 storm 行会打印 0，需结合同一行的 `xgem_rx` 与下一轮 debugfs 判断。

先后顺序以 `MBI pre-fault trace` 的 250 ms 逐点为准：`xgem` 与 `xgtc` 同停 ⇒ MAC 输入端/上游；仅 `delivered` 停、`dropped` 起涨 ⇒ GDM/QDMA 背压；`xgtc`/`ploamd` 仍涨而 `xgem` 停 ⇒ MBI 内部。`bwm`/`rx_error` 长期锁存，只能看 `*_first_ms` 是否远早于事故。

## 四、待办

- 重新编译上机；下次 MBI 卡死按第三节两条命令取证，再用 250 ms 逐点定段。
- 待 250 ms 数据落地后再定 GDM RX 环水位/page_pool 补位失败（`airoha_qdma_fill_rx_queue`，PonWrt 922/144 补丁区）是否值得加。
