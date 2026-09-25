<#
  swd.ps1 - 通过 STM32CubeProgrammer CLI (ST-Link/SWD) 驱动片上调参 mailbox。

  依赖: build\tune\Bcar.elf / Bcar.hex 已由 build.ps1 生成。

  用法:
    swd.ps1 flash                        烧写当前固件
    swd.ps1 status                       读 mailbox 头部并打印可读状态
    swd.ps1 set   -Kp -0.08 -Kd 0.003    只改参数（cmd=APPLY）
    swd.ps1 arm   -Samples 500           武装采集，无转向阶跃（手推阻尼测试用）
    swd.ps1 step  -Amp 40 -Samples 500   武装采集 + 转向阶跃（试次用）
    swd.ps1 stop                         立刻停电机（cmd=STOP）
    swd.ps1 trials                       读试次指标表
    swd.ps1 clr                          清空试次表 + 耗时统计
    swd.ps1 dump  -Out trial.csv         把波形缓冲读成 CSV
    swd.ps1 report -In trial.csv         从 CSV 复算指标（与片上算法对照）
    swd.ps1 raw -RawArgs "-c port=SWD ..."   直接透传给 CLI

  注意: 单次试验期间不要做任何 SWD 访问（写入用 -Delay 500 把停机影响挪到采样前），
        采集结束后再 dump。
#>
param(
    [Parameter(Mandatory = $true, Position = 0)]
    [ValidateSet('flash', 'status', 'set', 'arm', 'step', 'stop', 'trials', 'clr', 'dump', 'report', 'raw')]
    [string]$Action,

    [string]$Cli  = '',
    [string]$Elf  = '',
    [string]$In   = '',
    [string]$Out  = '',
    [string]$RawArgs = '',

    [double]$Kp = -0.08,
    [double]$Kd = 0.003,
    [int]$Max   = 25,
    [int]$Sign  = 1,
    [int]$Samples = 500,
    [int]$Delay   = 500,
    [int]$Amp     = 40,
    [int]$Speed   = 0,
    [double]$VKp = -3.9,
    [double]$VKd = -0.0102,
    [double]$SKp = -1,
    [double]$SKi = -0.005
)

$ErrorActionPreference = 'Continue'

$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
if ($Elf -eq '') { $Elf = Join-Path $root 'build\tune\Bcar.elf' }
$Hex = [System.IO.Path]::ChangeExtension($Elf, '.hex')

if ($Cli -eq '') {
    $cand = @(
        (Join-Path $env:LOCALAPPDATA 'stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'),
        'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe',
        'C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    )
    $Cli = ($cand | Where-Object { Test-Path $_ } | Select-Object -First 1)
    if (-not $Cli) { throw "找不到 STM32_Programmer_CLI.exe，请用 -Cli 指定路径" }
}

# mailbox 字段偏移（与 Core/Inc/tune.h 的编译期断言一一对应）
$OFF = @{
    arg0 = 0x00; arg1 = 0x04; arg2 = 0x08
    kp = 0x0C; kd = 0x10; max = 0x14; sign = 0x18
    vkp = 0x1C; vkd = 0x20; skp = 0x24; ski = 0x28
    amp = 0x2C; cmd = 0x30
    ack = 0x34; magic = 0x38; state = 0x3C; count = 0x40; tick = 0x44
    isr = 0x48; isrmax = 0x4C; err = 0x50
    gyroz = 0x54; tturn = 0x58; tout = 0x5C; m1 = 0x60; m2 = 0x64
    roll = 0x68; encl = 0x6C; encr = 0x70; spd = 0x74; vout = 0x78; vel = 0x7C
    trials = 0x80; akp = 0x84; akd = 0x88; amax = 0x8C; asign = 0x90
    trial = 0x94; buf = 0x454
}
$HDR_WORDS = 37          # 0x00..0x90
$TRIAL_ROWS = 20
$TRIAL_COLS = 12
$BUF_SAMPLES = 500
$BUF_CH = 8

function Invoke-Cli {
    param([string[]]$Args)
    $out = & $Cli @Args 2>&1
    if ($LASTEXITCODE -ne 0) {
        $out | ForEach-Object { Write-Host $_ }
        throw "STM32_Programmer_CLI 失败 (exit $LASTEXITCODE)"
    }
    return $out
}

function Get-Base {
    $nm = (Get-Command arm-none-eabi-nm -ErrorAction Stop).Source
    $line = (& $nm --print-size $Elf) | Select-String -Pattern ' g_tune$' | Select-Object -First 1
    if (-not $line) { throw "在 $Elf 里找不到 g_tune" }
    return [Convert]::ToInt64((($line.Line.Trim() -split '\s+')[0]), 16)
}

function Hex32([int64]$v) {
    # 必须用 BitConverter 取低 4 字节：
    # PS 5.1 里 0xFFFFFFFF 是 Int32 -1，'$v -band 0xFFFFFFFF' 不截断，
    # 'X8' 对负的 Int64 会输出 16 位十六进制（如 0xFFFFFFFFFFFEC780），
    # 而 CLI 遇到这种非法值就停止解析 —— 后面的字全都写不进去。
    return ('0x{0:X8}' -f [BitConverter]::ToUInt32([BitConverter]::GetBytes($v), 0))
}

function Read-Words {
    param([int64]$Addr, [int]$Count)
    # 注意: CLI 的 -r32 第二个参数是「字节数」，不是字数（传字数会少读 3/4）
    $out = Invoke-Cli @('-c', 'port=SWD', 'mode=hotplug', '-r32', (Hex32 $Addr), "$($Count * 4)")
    $words = New-Object System.Collections.Generic.List[uint32]
    foreach ($ln in $out) {
        if ($ln -match '^\s*0x[0-9A-Fa-f]+\s*:\s*(.*)$') {
            foreach ($t in ($matches[1] -split '\s+')) {
                if ($t -match '^[0-9A-Fa-f]{1,8}$') { $words.Add([Convert]::ToUInt32($t, 16)) }
            }
        }
    }
    if ($words.Count -lt $Count) { throw "只读到 $($words.Count)/$Count 个字" }
    return $words.GetRange(0, $Count)
}

function Write-Words {
    param([int64]$Addr, [int64[]]$Values)
    $a = @('-c', 'port=SWD', 'mode=hotplug', '-w32', (Hex32 $Addr))
    foreach ($v in $Values) { $a += (Hex32 $v) }
    Invoke-Cli $a | Out-Null
    # 回读校验：hot-plug 写 RAM 必须验证，否则坏值/漏写会静默通过
    $back = Read-Words -Addr $Addr -Count $Values.Count
    for ($i = 0; $i -lt $Values.Count; $i++) {
        $want = [BitConverter]::ToUInt32([BitConverter]::GetBytes([int64]$Values[$i]), 0)
        if ($back[$i] -ne $want) {
            throw ("写入校验失败 @0x{0:X} 第{1}个字: 期望 0x{2:X8} 实际 0x{3:X8}" -f $Addr, $i, $want, $back[$i])
        }
    }
}

# 注意: PowerShell 的 [int32]/[int16] 强制转换遇到超范围值会抛异常，
# 负数在上位机里是 0xFFFFFFxx 这种形式，必须走 BitConverter 位转换。
function To-I32([uint32]$u) { return [BitConverter]::ToInt32([BitConverter]::GetBytes($u), 0) }
function To-I16([uint32]$u, [bool]$hi) {
    $v = if ($hi) { [int](($u -shr 16) -band 0xFFFF) } else { [int]($u -band 0xFFFF) }
    if ($v -ge 0x8000) { $v -= 0x10000 }
    return $v
}
function To-F([int32]$v) { return $v / 1000000.0 }

function Write-Cmd {
    param([int64]$Base, [int]$Cmd)
    Write-Words ($Base + $OFF['cmd']) @([int64]$Cmd)
}

function Get-Word([object]$w, [int]$i) { return $w[$i] }

switch ($Action) {

    'flash' {
        if (-not (Test-Path $Hex)) { throw "找不到 $Hex，先跑 build.ps1" }
        Write-Host "烧写 $Hex ..."
        Invoke-Cli @('-c', 'port=SWD', 'mode=normal', '-w', $Hex, '-v', '-rst') |
            Select-String -Pattern 'Erasing|Download|verified|Error|error|Reset|Device name' |
            ForEach-Object { $_.Line.Trim() }
    }

    'status' {
        $base = Get-Base
        $w = Read-Words -Addr $base -Count $HDR_WORDS
        $st = $w[$OFF['state'] / 4]
        $stTxt = @()
        if ($st -band 0x01) { $stTxt += 'ARMED' }
        if ($st -band 0x02) { $stTxt += 'FULL' }
        if ($st -band 0x04) { $stTxt += 'FELL' }
        if ($st -band 0x08) { $stTxt += 'MANUAL' }
        "{0,-12} 0x{1:X8}  {2}" -f 'magic', $w[$OFF['magic'] / 4], $(if ($w[$OFF['magic'] / 4] -eq 0x544E3031) { 'OK (TN01)' } else { 'BAD!' })
        "{0,-12} {1} [{2}]" -f 'state', $st, ($stTxt -join ',')
        "{0,-12} {1}" -f 'ack', $w[$OFF['ack'] / 4]
        "{0,-12} {1}" -f 'live_tick', $w[$OFF['tick'] / 4]
        "{0,-12} {1} us (max {2} us)" -f 'isr', $w[$OFF['isr'] / 4], $w[$OFF['isrmax'] / 4]
        "{0,-12} {1} / {2}" -f 'err_count', $w[$OFF['err'] / 4], 'capture_count=' + $w[$OFF['count'] / 4]
        "{0,-12} kp={1} kd={2} max={3} sign={4}" -f 'applied',
            (To-F (To-I32 $w[$OFF['akp'] / 4])), (To-F (To-I32 $w[$OFF['akd'] / 4])),
            (To-I32 $w[$OFF['amax'] / 4]), (To-I32 $w[$OFF['asign'] / 4])
        "{0,-12} gyroz={1} target_turn={2} turn_out={3}" -f 'live',
            (To-I32 $w[$OFF['gyroz'] / 4]), (To-I32 $w[$OFF['tturn'] / 4]), (To-I32 $w[$OFF['tout'] / 4])
        "{0,-12} moto1={1} moto2={2} roll={3} deg" -f '',
            (To-I32 $w[$OFF['m1'] / 4]), (To-I32 $w[$OFF['m2'] / 4]), ((To-I32 $w[$OFF['roll'] / 4]) / 100.0)
        "{0,-12} enc_l={1} enc_r={2} spd_tgt={3} vert={4} vel={5}" -f '',
            (To-I32 $w[$OFF['encl'] / 4]), (To-I32 $w[$OFF['encr'] / 4]), (To-I32 $w[$OFF['spd'] / 4]),
            (To-I32 $w[$OFF['vout'] / 4]), (To-I32 $w[$OFF['vel'] / 4])
        "{0,-12} {1}" -f 'trials', $w[$OFF['trials'] / 4]
    }

    'set'   { $cmd = 1 }
    'arm'   { $cmd = 2 }
    'step'  { $cmd = 4 }

    'stop' {
        $base = Get-Base
        Write-Cmd $base 3
        Write-Host "STOP 已下发 (cmd=3)"
    }

    'clr' {
        $base = Get-Base
        Write-Cmd $base 5
        Start-Sleep -Milliseconds 50
        Write-Cmd $base 6
        Write-Host "试次表与统计已清零"
    }

    'trials' {
        $base = Get-Base
        $n = $TRIAL_ROWS * $TRIAL_COLS
        $w = Read-Words -Addr ($base + $OFF['trial']) -Count $n
        $done = (Read-Words -Addr ($base + $OFF['trials']) -Count 1)[0]
        "{0,4} {1,8} {2,8} {3,5} {4,5} {5,6} {6,8} {7,8} {8,5} {9,7} {10,7} {11,6} {12}" -f 'row', 'Kp', 'Kd', 'Tmax', 'sign', 'amp', 'ss', 'rise_ms', 'over%', 'osc%', 'roll', 'sat%', 'flags'
        for ($r = 0; $r -lt $TRIAL_ROWS; $r++) {
            $o = $r * $TRIAL_COLS
            $kp = To-I32 $w[$o + 0]; $kd = To-I32 $w[$o + 1]
            if ($kp -eq 0 -and $kd -eq 0 -and (To-I32 $w[$o + 5]) -eq 0) { continue }
            "{0,4} {1,8:n4} {2,8:n4} {3,5} {4,5} {5,6} {6,8} {7,8} {8,5:n1} {9,7:n1} {10,7:n1} {11,6:n1} 0x{12:X}" -f `
                $r, (To-F $kp), (To-F $kd), (To-I32 $w[$o + 2]), (To-I32 $w[$o + 3]), (To-I32 $w[$o + 4]),
                (To-I32 $w[$o + 5]), (To-I32 $w[$o + 6]), ((To-I32 $w[$o + 7]) / 10.0), ((To-I32 $w[$o + 8]) / 10.0),
                ((To-I32 $w[$o + 9]) / 100.0), ((To-I32 $w[$o + 10]) / 10.0), (To-I32 $w[$o + 11])
        }
        "已完成试次: $done"
    }

    'dump' {
        if ($Out -eq '') { throw "dump 需要 -Out <csv 路径>" }
        $base = Get-Base
        $n = $BUF_SAMPLES * $BUF_CH
        $w = Read-Words -Addr ($base + $OFF['buf']) -Count ($n / 2)
        $vals = New-Object System.Collections.Generic.List[int]
        foreach ($x in $w) {
            $vals.Add((To-I16 $x $false)); $vals.Add((To-I16 $x $true))
        }
        $sb = New-Object System.Text.StringBuilder
        [void]$sb.AppendLine('t_ms,gyroz,target_turn,turn_out,moto1,moto2,roll_cdeg,enc_l,enc_r')
        for ($i = 0; $i -lt $BUF_SAMPLES; $i++) {
            $o = $i * $BUF_CH
            [void]$sb.AppendLine(("{0},{1},{2},{3},{4},{5},{6},{7},{8}" -f `
                ($i * 10), $vals[$o + 0], $vals[$o + 1], $vals[$o + 2], $vals[$o + 3], $vals[$o + 4],
                $vals[$o + 5], $vals[$o + 6], $vals[$o + 7]))
        }
        [System.IO.File]::WriteAllText($Out, $sb.ToString())
        "已写出 $Out ($BUF_SAMPLES 点，10ms/点)"
    }

    'report' {
        if ($In -eq '') { throw "report 需要 -In <csv 路径>" }
        $rows = Import-Csv $In
        if ($rows.Count -lt 10) { throw "CSV 数据太少" }
        $g = $rows | ForEach-Object { [int]$_.gyroz }
        $tt = $rows | ForEach-Object { [int]$_.target_turn }
        $roll = $rows | ForEach-Object { [int]$_.roll_cdeg }
        $m1 = $rows | ForEach-Object { [Math]::Abs([int]$_.moto1) }
        $m2 = $rows | ForEach-Object { [Math]::Abs([int]$_.moto2) }
        $base0 = $tt[0]
        $stepAt = -1; $relAt = -1
        for ($i = 0; $i -lt $tt.Count; $i++) { if ($tt[$i] -ne $base0) { $stepAt = $i; break } }
        if ($stepAt -ge 0) { for ($i = $stepAt + 1; $i -lt $tt.Count; $i++) { if ($tt[$i] -eq $base0) { $relAt = $i; break } } }
        if ($stepAt -lt 0) { $stepAt = 10; $relAt = $g.Count }
        $w0 = $stepAt; $w1 = $relAt
        $s0 = [int]($w1 - [int](($w1 - $w0) * 0.3))
        $ss = 0.0
        if ($w1 -gt $s0) { $sum = 0; for ($i = $s0; $i -lt $w1; $i++) { $sum += $g[$i] }; $ss = $sum / ($w1 - $s0) }
        $peak = 0; for ($i = $w0; $i -lt $w1; $i++) { if ([Math]::Abs($g[$i]) -gt $peak) { $peak = [Math]::Abs($g[$i]) } }
        $mn = ($g[$s0..($w1 - 1)] | Measure-Object -Minimum).Minimum
        $mx = ($g[$s0..($w1 - 1)] | Measure-Object -Maximum).Maximum
        $rise = -1
        if ([Math]::Abs($ss) -ge 50) {
            $thr = [Math]::Abs($ss) * 0.9
            for ($i = $w0; $i -lt $w1; $i++) { if ([Math]::Abs($g[$i]) -ge $thr) { $rise = ($i - $w0) * 10; break } }
        }
        $over = 0; $osc = 0
        if ([Math]::Abs($ss) -ge 50) { $over = ($peak - [Math]::Abs($ss)) * 100 / [Math]::Abs($ss); if ($over -lt 0) { $over = 0 }; $osc = ($mx - $mn) * 100 / [Math]::Abs($ss) }
        $rollPeak = 0; for ($i = $w0; $i -lt $w1; $i++) { if ([Math]::Abs($roll[$i]) -gt $rollPeak) { $rollPeak = [Math]::Abs($roll[$i]) } }
        $satN = 0
        for ($i = $w0; $i -lt $w1; $i++) { if ($m1[$i] -ge 99 -or $m2[$i] -ge 99) { $satN++ } }
        $sat = 0.0; if ($w1 -gt $w0) { $sat = $satN * 100.0 / ($w1 - $w0) }
        "步阶起点 index={0} ({1} ms), 结束 index={2}" -f $stepAt, ($stepAt * 10), $relAt
        "稳态偏航角速度 ss = {0:n1} (raw, 16.4 LSB/dps -> {1:n1} dps)" -f $ss, ($ss / 16.4)
        "峰值 |gyroz| = {0}, 上升时间 = {1} ms, 超调 = {2:n1}%, 稳态段峰峰值 = {3:n1}%" -f $peak, $rise, $over, $osc
        "|roll| 峰值 = {0:n2} 度, 电机饱和占比 = {1:n1}%" -f ($rollPeak / 100.0), $sat
        if ($stepAt -lt 10 -and $peak -eq 0) { "（无阶跃：这是阻尼测试，看 gyroz 衰减）" }
    }

    'raw' {
        if ($RawArgs -eq '') { throw "raw 需要 -RawArgs" }
        Invoke-Cli ($RawArgs -split '\s+')
    }
}

# set/arm/step 走到这里统一写 13 个字（cmd 最后，保证参数先落地）
if ($Action -in @('set', 'arm', 'step')) {
    $base = Get-Base
    $vals = @(
        [int64]$Delay, [int64]$Samples, [int64]$Speed,
        [int64][int32][Math]::Round($Kp * 1000000), [int64][int32][Math]::Round($Kd * 1000000),
        [int64]$Max, [int64]$Sign,
        [int64][int32][Math]::Round($VKp * 1000000), [int64][int32][Math]::Round($VKd * 1000000),
        [int64][int32][Math]::Round($SKp * 1000000), [int64][int32][Math]::Round($SKi * 1000000),
        [int64]$Amp, [int64]$cmd
    )
    Write-Words $base $vals
    "已写入: Kp={0} Kd={1} Max={2} Sign={3} Amp={4} Samples={5} Delay={6}ms Speed={7} cmd={8}" -f `
        $Kp, $Kd, $Max, $Sign, $Amp, $Samples, $Delay, $Speed, $cmd
    "trial CSVs 请用: swd.ps1 dump -Out <csv>"
}
