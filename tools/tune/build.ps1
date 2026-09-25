<#
  build.ps1 - 用 PATH 上的 arm-none-eabi-gcc 复刻 STM32CubeIDE(CMake) Release 构建。

  为什么不用 cmake：本项目 CubeIDE 使用的 cube-cmake 与工具链都在
  %LOCALAPPDATA%\stm32cube\bundles 下，在受限环境里不可执行；而 PATH 上的
  arm-none-eabi-gcc 14.3.1 与 bundle 内是同一套编译器。
  这里用的 define / include / -Os -g0 -std=gnu11 / 链接脚本 / 库 与
  build/Release/build.ninja 完全一致，产物等价。

  用法:
    powershell -ExecutionPolicy Bypass -File tools\tune\build.ps1
    powershell -ExecutionPolicy Bypass -File tools\tune\build.ps1 -Clean

  注意: Debug(-O0) 已占 61.1KB/64KB Flash，必须用 Release(-Os) 才有余量。
#>
param(
    [switch]$Clean
)

# 原生命令的 stderr 在 PS 5.1 里会被包成 ErrorRecord；用 'Continue' + 显式检查
# $LASTEXITCODE 才能在失败时把真正的编译错误完整读出来（'Stop' 会提前抛异常）
$ErrorActionPreference = 'Continue'

$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$out  = Join-Path $root 'build\tune'

if ($Clean -and (Test-Path $out)) { Remove-Item $out -Recurse -Force }
New-Item -ItemType Directory -Force -Path $out | Out-Null

$gcc = (Get-Command arm-none-eabi-gcc -ErrorAction Stop).Source
$nm  = (Get-Command arm-none-eabi-nm  -ErrorAction Stop).Source
$sz  = (Get-Command arm-none-eabi-size -ErrorAction Stop).Source

$defines = @('-DSTM32F103xB', '-DUSE_HAL_DRIVER')
# 注意: 不能在数组字面量里写 '-I' + (Join-Path ...)，PowerShell 会把它折叠成一个字符串
$incDirs = @('Core\Inc', 'MyCode\Inc', 'Drivers\STM32F1xx_HAL_Driver\Inc',
             'Drivers\STM32F1xx_HAL_Driver\Inc\Legacy',
             'Drivers\CMSIS\Device\ST\STM32F1xx\Include', 'Drivers\CMSIS\Include')
$includes = foreach ($d in $incDirs) { '-I' + (Join-Path $root $d) }
$cflags = @('-mcpu=cortex-m3', '-Wall', '-fdata-sections', '-ffunction-sections',
            '-fstack-usage', '-Os', '-g0', '-std=gnu11')

# 与 cmake/stm32cubemx/CMakeLists.txt 的 MX_Application_Src 一致
$appSrc = @(
    'Core\Src\main.c', 'Core\Src\gpio.c', 'Core\Src\i2c.c', 'Core\Src\tim.c',
    'Core\Src\usart.c', 'Core\Src\stm32f1xx_it.c', 'Core\Src\stm32f1xx_hal_msp.c',
    'Core\Src\sysmem.c', 'Core\Src\syscalls.c'
)
# 与 STM32_Drivers_Src 一致
$drvSrc = @(
    'Core\Src\system_stm32f1xx.c',
    'Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_gpio_ex.c',
    'Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_i2c.c',
    'Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal.c',
    'Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_rcc.c',
    'Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_rcc_ex.c',
    'Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_gpio.c',
    'Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_dma.c',
    'Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_cortex.c',
    'Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_pwr.c',
    'Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_flash.c',
    'Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_flash_ex.c',
    'Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_exti.c',
    'Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_tim.c',
    'Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_tim_ex.c',
    'Drivers\STM32F1xx_HAL_Driver\Src\stm32f1xx_hal_uart.c'
)
# 用户源码（CMakeLists.txt target_sources）
$userSrc = @(
    'Core\Src\oled.c', 'Core\Src\font.c', 'Core\Src\sr04.c', 'Core\Src\motor.c',
    'Core\Src\encoder.c', 'Core\Src\pid.c', 'Core\Src\tune.c',
    'MyCode\Src\IIC.c', 'MyCode\Src\inv_mpu_dmp_motion_driver.c',
    'MyCode\Src\inv_mpu.c', 'MyCode\Src\mpu6050.c'
)

$asmSrc = @('startup_stm32f103xb.s')

$objs = New-Object System.Collections.Generic.List[string]
$warnCount = 0

function Compile-One([string]$rel) {
    $src = Join-Path $root $rel
    if (-not (Test-Path $src)) { throw "missing source: $src" }
    $obj = Join-Path $out (($rel -replace '[\\/]', '_') + '.o')
    $objLog = "$obj.log"
    $a = @($cflags + $defines + $includes + @('-c', $src, '-o', $obj))
    & $gcc @a 2>&1 | Tee-Object -FilePath $objLog | Out-Null
    if ($LASTEXITCODE -ne 0) {
        Get-Content $objLog | Write-Host
        throw "compile failed: $rel"
    }
    if (Test-Path $objLog) {
        $w = (Select-String -Path $objLog -Pattern 'warning:' -ErrorAction SilentlyContinue | Measure-Object).Count
        $script:warnCount += $w
        if ($w -gt 0) { Select-String -Path $objLog -Pattern 'warning:' | ForEach-Object { "  [$rel] " + $_.Line.Trim() } }
    }
    $script:objs.Add($obj)
}

Write-Host "=== compile (Release -Os) ==="
$sw = [System.Diagnostics.Stopwatch]::StartNew()
foreach ($s in ($appSrc + $drvSrc + $userSrc)) { Compile-One $s }
Compile-One $asmSrc[0]
$sw.Stop()
Write-Host ("compiled {0} objects in {1:n1}s, warnings={2}" -f $objs.Count, $sw.Elapsed.TotalSeconds, $warnCount)

Write-Host "=== link ==="
$elf = Join-Path $out 'Bcar.elf'
$map = Join-Path $out 'Bcar.map'
$ld  = Join-Path $root 'STM32F103XX_FLASH.ld'
$mapArg = '-Wl,-Map=' + $map
$la = @('-mcpu=cortex-m3', '-T', $ld, '--specs=nano.specs', $mapArg,
        '-Wl,--gc-sections', '-Wl,--print-memory-usage', '-u_printf_float')
$la += $objs.ToArray()
$la += @('-lm', '-o', $elf)
& $gcc @la 2>&1 | ForEach-Object { $_ }
if ($LASTEXITCODE -ne 0) { throw "link failed" }

Write-Host "=== size ==="
& $sz $elf | ForEach-Object { $_ }

Write-Host "=== hex (给 CubeProgrammer CLI 烧写用) ==="
$objcopy = (Get-Command arm-none-eabi-objcopy -ErrorAction Stop).Source
$hex = Join-Path $out 'Bcar.hex'
& $objcopy -O ihex $elf $hex
if ($LASTEXITCODE -ne 0) { throw "objcopy failed" }
"  $hex"

Write-Host "=== key symbols ==="
# nm --print-size 输出为 "地址 大小 类型 符号"，地址是第 0 个字段
$sym = & $nm --print-size $elf | Select-String -Pattern ' g_tune$' | Select-Object -First 1
if (-not $sym) { throw "g_tune not found in ELF" }
$f = ($sym.Line.Trim() -split '\s+')
$gAddr = [Convert]::ToInt64($f[0], 16)
"  {0}  (size {1} bytes)" -f $sym.Line.Trim(), [Convert]::ToInt64($f[1], 16)
"  g_tune base  = 0x{0:X8}" -f $gAddr
"  cmd   @0x30  = 0x{0:X8}" -f ($gAddr + 0x30)
"  trial @0x94  = 0x{0:X8}" -f ($gAddr + 0x94)
"  buf   @0x454 = 0x{0:X8}" -f ($gAddr + 0x454)
"  elf : $elf"
