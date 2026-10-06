$ErrorActionPreference = 'Stop'
$env:TEMP = 'A:\NEXarm\Claude\.gcc-temp-readonly-oled-r3'
$env:TMP = $env:TEMP
New-Item -ItemType Directory -Force -Path $env:TEMP | Out-Null
$cliArgs = @(
    'compile', '-v', '--jobs', '1', '--fqbn', 'esp32:esp32:esp32',
    '--libraries', 'A:\NEXarm\Claude\match_compile_libraries',
    '--build-path', 'A:\NEXarm\Claude\.arduino-build-readonly-oled-r3',
    '--libraries', 'A:\NEXarm\Claude\.toolchains\esp32-core-clean-20260919\esp32-2.0.17\libraries',
    '--build-property', 'build.core.path=A:\NEXarm\Claude\.toolchains\esp32-core-clean-20260919\esp32-2.0.17\cores\esp32',
    '--build-property', 'build.variant.path=A:\NEXarm\Claude\.toolchains\esp32-core-clean-20260919\esp32-2.0.17\variants\esp32',
    '--build-property', 'runtime.platform.path=A:\NEXarm\Claude\.toolchains\esp32-core-clean-20260919\esp32-2.0.17',
    '--build-property', 'runtime.tools.xtensa-esp32-elf-gcc.path=A:\NEXarm\Claude\.toolchains\xtensa-clean-20260919\xtensa-esp32-elf',
    '--build-property', 'runtime.tools.esptool_py.path=A:\NEXarm\Claude\.toolchains\python-wrappers',
    '--build-property', 'tools.esptool_py.cmd=esptool.cmd',
    '--build-property', 'tools.esptool_py.cmd.windows=esptool.cmd',
    '--build-property', 'tools.gen_esp32part.cmd.windows=A:\NEXarm\Claude\.toolchains\python-wrappers\gen_esp32part.cmd',
    '--build-property', 'compiler.sdk.path=A:\NEXarm\Claude\.toolchains\esp32-core-clean-20260919\esp32-2.0.17\tools\sdk\esp32',
    'A:\NEXarm\Claude\readonly_oled_20260923\NexArm_ReadOnly_OLED'
)
& 'A:\NEXarm\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' @cliArgs *> 'A:\NEXarm\Claude\build_match_readonly_oled_r3.log'
$result = $LASTEXITCODE
Get-Content 'A:\NEXarm\Claude\build_match_readonly_oled_r3.log' -Tail 24 | Where-Object { $_.Length -lt 1200 }
exit $result





