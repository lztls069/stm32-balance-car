<#
  sweep.ps1 - 自动跑 (Kp, Kd) 网格：写参数 -> 触发转向阶跃 -> dump 波形 -> 算指标。

  每个试次用一次 SWD 触发；采集期间不做任何 SWD 访问。
  指标定义见 README：ss=稳态偏航角速度, ripple=稳态段相对波动, osc=摆动频率,
  roll_pp=直立扰动峰峰值, sat=电机饱和占比, spin=是否真的在转。

  用法:
    tools\tune\sweep.ps1 -KpList "-0.10,-0.20,-0.35" -KdList "0.01,0.03,0.06" `
                         -TMax 40 -Amp 150 -Samples 200 -Delay 500 -OutDir build\tune\sweep
#>
param(
    [string]$KpList = "-0.20",
    [string]$KdList = "0.02",
    [int]$Max = 40,
    [int]$Sign = -1,
    [int]$Amp = 150,
    [int]$Samples = 200,
    [int]$Delay = 500,
    [int]$Ramp = 0,          # 0 = 硬阶跃；>0 = 用这么多拍(10ms)线性升到 Amp
    [int]$Speed = 0,         # 强制 Target_speed（0=原地转；6=一边前进一边转）
    [double]$VKp = -3.9,     # 直立环参数（默认沿用编译值，用于验证瓶颈在不在直立环）
    [double]$VKd = -0.0102,
    [string]$VKdList = '',   # 非空时优先：对每个 VKd 各跑一遍（配合 -Repeat 做左右交替重复）
    [int]$Repeat = 1,        # 每个参数点重复次数；奇偶次自动左右交替，保证净转角≈0
    [double]$SKp = -1.0,
    [double]$SKi = -0.005,
    [string]$OutDir = '',
    [string]$Elf = '',
    [string]$Cli = ''
)

$ErrorActionPreference = 'Continue'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
if ($Elf -eq '') { $Elf = Join-Path $root 'build\tune\Bcar.elf' }
if ($OutDir -eq '') { $OutDir = Join-Path $root 'build\tune\sweep' }
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

if ($Cli -eq '') {
    $cand = @(
        (Join-Path $env:LOCALAPPDATA 'stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'),
        'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe',
        'C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    )
    $Cli = ($cand | Where-Object { Test-Path $_ } | Select-Object -First 1)
    if (-not $Cli) { throw "找不到 STM32_Programmer_CLI.exe" }
}

$OFF = @{ arg0=0x00; arg1=0x04; arg2=0x08; kp=0x0C; kd=0x10; max=0x14; sign=0x18;
          vkp=0x1C; vkd=0x20; skp=0x24; ski=0x28; amp=0x2C; cmd=0x30
          ack=0x34; state=0x3C; count=0x40; trials=0x80; trial=0x94; buf=0x454 }
$BUF_SAMPLES = 500
$BUF_CH = 8

function ToHex32([int64]$v) { '0x{0:X8}' -f [BitConverter]::ToUInt32([BitConverter]::GetBytes($v),0) }
function ToI16([uint32]$u, [bool]$hi) {
    $x = if ($hi) { [int](($u -shr 16) -band 0xFFFF) } else { [int]($u -band 0xFFFF) }
    if ($x -ge 0x8000) { $x -= 0x10000 }
    return $x
}
function Invoke-Cli { param([string[]]$a) $o = & $Cli @a 2>&1; if ($LASTEXITCODE -ne 0) { $o | ForEach-Object { Write-Host $_ }; throw "CLI 失败" }; return $o }
function Get-Base {
    $nm = (Get-Command arm-none-eabi-nm -ErrorAction Stop).Source
    $l = (& $nm --print-size $Elf) | Select-String ' g_tune$' | Select-Object -First 1
    if (-not $l) { throw "找不到 g_tune" }
    return [Convert]::ToInt64((($l.Line.Trim() -split '\s+')[0]), 16)
}
function Read-Words {
    param([int64]$Addr, [int]$Count)
    $o = Invoke-Cli @('-c','port=SWD','mode=hotplug','-r32',(ToHex32 $Addr),"$($Count*4)")
    $v = New-Object System.Collections.Generic.List[uint32]
    foreach ($ln in $o) {
        if ($ln -match '^\s*0x[0-9A-Fa-f]+\s*:\s*(.*)$') {
            foreach ($t in ($matches[1] -split '\s+')) { if ($t -match '^[0-9A-Fa-f]{1,8}$') { $v.Add([Convert]::ToUInt32($t,16)) } }
        }
    }
    if ($v.Count -lt $Count) { throw "只读到 $($v.Count)/$Count 个字" }
    return $v.GetRange(0,$Count)
}
function Write-Words {
    param([int64]$Addr, [int64[]]$Values)
    $a = @('-c','port=SWD','mode=hotplug','-w32',(ToHex32 $Addr))
    foreach ($v in $Values) { $a += (ToHex32 $v) }
    Invoke-Cli $a | Out-Null
    $back = Read-Words -Addr $Addr -Count $Values.Count
    for ($i = 0; $i -lt $Values.Count; $i++) {
        $want = [BitConverter]::ToUInt32([BitConverter]::GetBytes([int64]$Values[$i]), 0)
        # cmd 会被目标清零，跳过校验
        if ($i -eq 12 -and $back[$i] -eq 0) { continue }
        if ($back[$i] -ne $want) { throw ("写校验失败 word{0}: 期望 0x{1:X8} 实际 0x{2:X8}" -f $i,$want,$back[$i]) }
    }
}

function Get-Metrics {
    param([string]$Csv)
    $r = Import-Csv $Csv
    $n = $r.Count
    $g  = @(); $tt = @(); $to = @(); $m1 = @(); $m2 = @(); $rl = @(); $el = @(); $er = @()
    foreach ($x in $r) {
        $g += [int]$x.gyroz; $tt += [int]$x.target_turn; $to += [int]$x.turn_out
        $m1 += [int]$x.moto1; $m2 += [int]$x.moto2; $rl += [int]$x.roll_cdeg
        $el += [int]$x.enc_l; $er += [int]$x.enc_r
    }
    # 起始: tt 第一次离开基线；保持段: |tt| == 该次试验的最大指令（斜坡模式下会排除升/降段）
    $base0 = $tt[0]
    $startI = -1
    for ($i = 0; $i -lt $n; $i++) { if ($tt[$i] -ne $base0) { $startI = $i; break } }
    if ($startI -lt 0) { return $null }
    $ampMax = 0
    foreach ($v in $tt) { if ([Math]::Abs($v) -gt $ampMax) { $ampMax = [Math]::Abs($v) } }
    $w0 = -1; $w1 = -1
    for ($i = 0; $i -lt $n; $i++) {
        if ([Math]::Abs($tt[$i]) -eq $ampMax) { if ($w0 -lt 0) { $w0 = $i }; $w1 = $i }
    }
    if ($w0 -lt 0) { $w0 = $startI; $w1 = $n - 1 }
    if (($w1 - $w0) -lt 10) { return $null }

    # 稳态窗口 = 保持段后 40%
    $s0 = [int]($w1 - [int](($w1 - $w0) * 0.4))
    $sum = 0.0; for ($i = $s0; $i -lt $w1; $i++) { $sum += $g[$i] }
    $ss = $sum / ($w1 - $s0 + 1)

    # 波动（稳态段标准差）、摆动频率（相对均值的过零率）、峰峰值
    $var = 0.0; for ($i = $s0; $i -le $w1; $i++) { $var += ($g[$i] - $ss) * ($g[$i] - $ss) }
    $sd = [Math]::Sqrt($var / ($w1 - $s0 + 1))
    $zc = 0; $prev = $g[$s0] - $ss
    for ($i = $s0 + 1; $i -le $w1; $i++) { $cur = $g[$i] - $ss; if (($prev -lt 0 -and $cur -ge 0) -or ($prev -ge 0 -and $cur -lt 0)) { $zc++ }; $prev = $cur }
    $oscHz = $zc / 2.0 / (($w1 - $s0) * 0.01)

    $peak = 0; $rollPk = 0; $sat = 0; $wheel = 0
    for ($i = $w0; $i -le $w1; $i++) {
        if ([Math]::Abs($g[$i]) -gt $peak) { $peak = [Math]::Abs($g[$i]) }
        if ([Math]::Abs($rl[$i]) -gt $rollPk) { $rollPk = [Math]::Abs($rl[$i]) }
        if ([Math]::Abs($m1[$i]) -ge 99 -or [Math]::Abs($m2[$i]) -ge 99) { $sat++ }
        # 轮子真的转了吗：左右轮速度差的绝对值
        if ([Math]::Abs($el[$i] - $er[$i]) -ge 4) { $wheel++ }
    }
    # 上升时间：从指令起始到 |gyroz| 首次达到稳态 90%
    $riseMs = -1
    if ([Math]::Abs($ss) -ge 50) {
        $thr = [Math]::Abs($ss) * 0.9
        for ($i = $startI; $i -le $w1; $i++) { if ([Math]::Abs($g[$i]) -ge $thr) { $riseMs = ($i - $startI) * 10; break } }
    }
    $m = ($w1 - $w0 + 1)
    return [pscustomobject]@{
        rise_ms     = $riseMs
        ss_raw      = [Math]::Round($ss,1)
        ss_dps      = [Math]::Round($ss / 16.4, 1)
        ripple_pct  = [Math]::Round(100.0 * $sd / [Math]::Max([Math]::Abs($ss),1), 0)
        osc_hz      = [Math]::Round($oscHz, 1)
        peak_raw    = $peak
        roll_pp_deg = [Math]::Round($rollPk / 100.0, 2)
        sat_pct     = [Math]::Round(100.0 * $sat / $m, 1)
        spin_pct    = [Math]::Round(100.0 * $wheel / $m, 0)
        step_ms     = [int](($w1 - $w0) * 10)
    }
}

$kps = $KpList -split ',' | ForEach-Object { [double]$_.Trim() }
$kds = $KdList -split ',' | ForEach-Object { [double]$_.Trim() }
$vkds = if ($VKdList -ne '') { $VKdList -split ',' | ForEach-Object { [double]$_.Trim() } } else { @($VKd) }
$base = Get-Base
"mailbox base = 0x{0:X8}   网格 Kp x Kd x VKd = {1} x {2} x {3}，TMAX={4} Amp={5} Samples={6} Repeat={7}" -f $base, $kps.Count, $kds.Count, $vkds.Count, $Max, $Amp, $Samples, $Repeat

$results = New-Object System.Collections.Generic.List[object]
$trialNo = 0
foreach ($kp in $kps) {
    foreach ($kd in $kds) {
      foreach ($vkd in $vkds) {
       for ($rep = 0; $rep -lt $Repeat; $rep++) {
        $ampNow = if ($trialNo % 2 -eq 0) { $Amp } else { -$Amp }   # 左右交替
        $trialNo++
        $tag = "kp{0}_kd{1}_vkd{2}_r{3}_a{4}" -f ([int][Math]::Round($kp*1000)), ([int][Math]::Round($kd*1000)), ([int][Math]::Round($vkd*1000)), $rep, $ampNow
        $vals = @(
            [int64]$Delay, [int64]($Samples -bor ($Ramp -shl 16)), [int64]$Speed,
            [int64][int32][Math]::Round($kp*1000000), [int64][int32][Math]::Round($kd*1000000),
            [int64]$Max, [int64]$Sign,
            [int64][int32][Math]::Round($VKp*1000000), [int64][int32][Math]::Round($vkd*1000000),
            [int64][int32][Math]::Round($SKp*1000000), [int64][int32][Math]::Round($SKi*1000000),
            [int64]$ampNow, [int64]4
        )
        Write-Words $base $vals
        Start-Sleep -Milliseconds ($Delay + $Ramp*10 + $Samples*10 + 800)

        $w = Read-Words -Addr ($base + $OFF['buf']) -Count ($BUF_SAMPLES * $BUF_CH / 2)
        $sb = New-Object System.Text.StringBuilder
        [void]$sb.AppendLine('t_ms,gyroz,target_turn,turn_out,moto1,moto2,roll_cdeg,enc_l,enc_r')
        for ($i = 0; $i -lt $Samples; $i++) {
            $o = $i * 4
            [void]$sb.AppendLine("$($i*10),$(ToI16 $w[$o] $false),$(ToI16 $w[$o] $true),$(ToI16 $w[$o+1] $false),$(ToI16 $w[$o+1] $true),$(ToI16 $w[$o+2] $false),$(ToI16 $w[$o+2] $true),$(ToI16 $w[$o+3] $false),$(ToI16 $w[$o+3] $true)")
        }
        $csv = Join-Path $OutDir "$tag.csv"
        [System.IO.File]::WriteAllText($csv, $sb.ToString())
        $mt = Get-Metrics $csv
        $row = [pscustomobject]@{ kp = $kp; kd = $kd; vkd = $vkd; amp = $ampNow; csv = $tag }
        if ($mt) {
            $row | Add-Member rise_ms $mt.rise_ms
            $row | Add-Member ss_raw $mt.ss_raw; $row | Add-Member ss_dps $mt.ss_dps
            $row | Add-Member ripple_pct $mt.ripple_pct; $row | Add-Member osc_hz $mt.osc_hz
            $row | Add-Member roll_pp_deg $mt.roll_pp_deg; $row | Add-Member sat_pct $mt.sat_pct
            $row | Add-Member spin_pct $mt.spin_pct
        }
        $results.Add($row)
        "{0,8} {1,6} | ss={2,7} ({3,6} dps) rise={4,5}ms ripple={5,5}% osc={6,4}Hz roll_pp={7,5}deg sat={8,5}% spin={9,4}%" -f `
            $vkd, $ampNow, $row.ss_raw, $row.ss_dps, $row.rise_ms, $row.ripple_pct, $row.osc_hz, $row.roll_pp_deg, $row.sat_pct, $row.spin_pct
       }
      }
    }
}

$summary = Join-Path $OutDir 'summary.csv'
$results | Export-Csv -NoTypeInformation -Encoding UTF8 $summary
"`n汇总: $summary"
