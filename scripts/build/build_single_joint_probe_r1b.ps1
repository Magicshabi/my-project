$ErrorActionPreference = 'Stop'
$env:PATH='A:\NEXarm\Claude\.toolchains\xtensa-clean-20260919\xtensa-esp32-elf\bin;'+$env:PATH
$env:TEMP = 'A:\NEXarm\Claude\.gcc-temp-single-joint-probe-r1b'
$env:TMP = $env:TEMP
New-Item -ItemType Directory -Force -Path $env:TEMP | Out-Null
$cliArgs = @(
    '--log', '--log-level', 'debug', 'compile', '-v', '--jobs', '1', '--fqbn', 'esp32:esp32:esp32',
    '--libraries', 'A:\NEXarm\Claude\match_compile_libraries',
    '--build-path', 'A:\NEXarm\Claude\.arduino-build-single-joint-probe-r1b',
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
    'A:\NEXarm\Claude\single_joint_probe_r1b_20261006\NexArm_Single_Joint_Probe'
)
& 'A:\NEXarm\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' @cliArgs *> 'A:\NEXarm\Claude\build_single_joint_probe_r1b.log'
$result = $LASTEXITCODE
Get-Content 'A:\NEXarm\Claude\build_single_joint_probe_r1b.log' -Tail 24 | Where-Object { $_.Length -lt 1200 }
exit $result









