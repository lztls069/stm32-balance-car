# 转向环 SWD 在线调参工具

不依赖串口、只靠 ST-Link/SWD 的调参链路：固件把 100Hz 波形抓进 RAM，
主机端用 STM32CubeProgrammer CLI 写入参数 + 读出波形，再离线算指标。

> **硬件安全前提**：ST-Link 与小车之间的杜邦线长度有限，**小车单次只能转 90°**（左或右）。
> 所有试验都用短脉冲（0.6~0.8s ≈ 27°）并且左右交替，保证净转角接近 0、车不离开原位。
> 测试时务必有人守在电源旁。

## 文件

| 文件 | 作用 |
| --- | --- |
| `build.ps1` | 用 PATH 上的 `arm-none-eabi-gcc` 复刻 CMake Release(`-Os`) 构建，产出 `build/tune/Bcar.elf/.hex` 并打印 mailbox 符号地址 |
| `swd.ps1` | 烧写 / 读状态 / 写参数 / 触发试验 / dump 波形 / 复算指标 |
| `sweep.ps1` | 按 (Kp,Kd) 网格自动跑试验并输出指标汇总 |

固件侧实现在 `Core/Src/tune.c` 与 `Core/Inc/tune.h`。

## 常用命令

```powershell
# 1) 编译（必须 Release，Debug 已占 Flash 93%）
powershell -ExecutionPolicy Bypass -File tools\tune\build.ps1

# 2) 烧写（烧完复位即开始跑直立环，先让车处于安全状态）
powershell -ExecutionPolicy Bypass -File tools\tune\swd.ps1 flash

# 3) 看状态 / 在线改参数 / 急停
powershell -ExecutionPolicy Bypass -File tools\tune\swd.ps1 status
powershell -ExecutionPolicy Bypass -File tools\tune\swd.ps1 set -Kp -0.2 -Kd 0.02 -Max 40 -Sign -1
powershell -ExecutionPolicy Bypass -File tools\tune\swd.ps1 stop

# 4) 一次转向试验（右转 0.72s ≈ 27°）+ dump + 复算
powershell -ExecutionPolicy Bypass -File tools\tune\sweep.ps1 -KpList "-0.20" -KdList "0.02" `
    -TMax 40 -Amp 150 -Samples 120 -Delay 300 -OutDir build\tune\v1
powershell -ExecutionPolicy Bypass -File tools\tune\swd.ps1 report -In build\tune\v1\kp-200_kd20_ramp0.csv
```

## 指标含义（sweep.ps1 输出）

| 列 | 含义 | 判据 |
| --- | --- | --- |
| `ss` | 稳态偏航角速度（gyroz 原始值，16.4 LSB/dps） | 与指令同号；`+150` 指令下约 600（37 dps） |
| `rise` | 指令起始到 \|gyroz\| 达到稳态 90% 的时间 | < 400ms |
| `ripple` | 稳态段标准差 / \|ss\| | 越小越好；本车平台约 100% |
| `osc` | 稳态段相对均值的过零频率 | 越低越好；实测 12~16Hz |
| `roll_pp` | 转向中 \|roll\| 峰值（度） | < 5~6° |
| `sat` | \|moto\|=100 的时间占比 | < 10% |
| `spin` | 左右轮速度差 ≥4 的采样占比（轮子是否真在转） | > 60% |

## 已定档结论（2026-09-25）

- 极性：`Turn_Sign` 必须为 **-1**，且 `Kp` 与 `Kd` 必须**异号**（推导见 `过程笔记.md`）。
- 参数：`Turn_Kp=-0.2`、`Turn_Kd=0.02`、`Turn_Out_Max=40`、`Turn_Sign=-1`。
  满指令约 37 dps（约 2.4s 转 90°），上升 40ms，转向时 roll 扰动约 5°。
- **12~16Hz 振铃不来自转向环**：把 `Turn_Kd` 设为 0（完全没有偏航反馈）后仍有 10.4Hz/732% 的振铃，
  且 roll 同时摆 ±3°、平衡环公共模 PWM 在 ±15 极限环。差速转向会削弱平衡环的有效权威；
  要更干净的转向应改直立环滚转阻尼 `Vertical_Kd`，而不是继续加转向环增益。
- 小指令无效：`|Turn_out| < 10` 基本在电机死区内。手机 App 按住按键时 `Target_turn`
  以 30/拍递增，50ms 内就到 150，正常操作不受影响。

## 排错记录

- **`-w32` 写负数**：PowerShell 5.1 里 `0xFFFFFFFF` 是 Int32 `-1`，`-band` 不截断，
  `'X8'` 对负的 Int64 会输出 16 位十六进制（如 `0xFFFFFFFFFFFEC780`），CLI 遇到这种值会
  **停止解析后面的字**，造成"参数只写进去一半"。脚本统一用 `[BitConverter]` 取低 4 字节，并且写完回读校验。
- **`-r32` 的长度单位是字节**，不是字数（传字数会少读 3/4）。
- **PS 5.1 解析 .ps1**：无 BOM 的 UTF-8 会被当成 GBK，中文注释会吞掉换行导致语法错误；
  本目录脚本都带 UTF-8 BOM。
- 调参固件必须用 Release(`-Os`) 构建，Debug(`-O0`) 已占 61.1KB/64KB Flash。
