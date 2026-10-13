# 连续作业一键脚本。用法（在 PowerShell 或 PyCharm 的 Terminal 里）：
#   .\运行.ps1 红-蓝-黄        跑一组（带篮）
#   .\运行.ps1                 交互模式：每次输入一组颜色，q 退出
#   .\运行.ps1 -All            依次跑全部24组（每组后复位篮子、自动回P0）
#   .\运行.ps1 -Check 红-蓝-黄 只看编译结果和用时估计，不连机械臂
#   .\运行.ps1 -Empty 红-蓝-黄 空载跑（不放篮子）
# 运行前：机械臂在P0，上位机断开串口，四个篮子在原位，P1~P3清空。
param(
    [string]$Command = '',
    [switch]$All,
    [switch]$Check,
    [switch]$Empty
)
$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $here
$py = 'C:\Users\1\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
$env:PYTHONPATH = 'A:\NEXarm\Claude\.runtime_deps\serial_capture'
$env:PYTHONIOENCODING = 'utf-8'
$lock = Join-Path $here '..\handoff\lock.py'

if ($Check) {
    if (-not $Command) { throw '用法：.\运行.ps1 -Check 红-蓝-黄' }
    & $py sequence.py $Command
    & $py executor.py $Command --dry-run
    exit $LASTEXITCODE
}

$load = if ($Empty) { '--empty-load' } else { '--loaded' }

# 2026-10-13 用户要求：不再因锁被占用而拒绝，直接接管锁。
# 串口本身是独占的：若别的程序（上位机等）真的开着COM11，连接会失败，不会两边同时控制。
& $py $lock release COM11 --holder claude --force | Out-Null
& $py $lock acquire COM11 --holder claude --minutes 120 --purpose '连续作业'
if ($LASTEXITCODE -ne 0) { throw '取锁失败' }
try {
    if ($All) {
        & $py run_all_combos.py --port COM11 --go $load
    } elseif ($Command) {
        & $py executor.py $Command --port COM11 --go $load
    } else {
        & $py executor.py --port COM11 --go $load
    }
    $code = $LASTEXITCODE
} finally {
    & $py $lock release COM11 --holder claude | Out-Null
    Write-Host '已释放 COM11 锁'
}
exit $code
