param([string]$PythonExecutable = 'python', [string]$RuntimePath = '')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
Push-Location $projectRoot
try {
    New-Item -ItemType Directory -Path build/tests -Force | Out-Null
    $roots = @('Inc', 'Drivers', 'Middlewares', 'bsp', 'modules', 'application')
    $includes = @($roots | ForEach-Object {
        '-I' + $_
        Get-ChildItem -LiteralPath $_ -Directory -Recurse | ForEach-Object { '-I' + $_.FullName }
    })
    $flags = @('-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft', '-std=gnu11', '-O1',
        '-ffunction-sections', '-fdata-sections', '-DUSE_HAL_DRIVER', '-DSTM32F407xx',
        '-DARM_MATH_CM4', '-DDISABLE_LOG_SYSTEM=1')
    $sources = @('tests/integration.c', 'tests/startup.s', 'modules/algorithm/controller.c',
        'modules/message_center/message_center.c', 'modules/daemon/daemon.c',
        'modules/motor/DJImotor/dji_motor.c', 'modules/motor/power_control.c',
        'modules/swerve/swerve_kinematics.c', 'modules/super_cap/super_cap.c',
        'modules/super_cap/super_cap_sdk.c')
    $previousPath = $env:PYTHONPATH
    try {
        if ($RuntimePath) { $env:PYTHONPATH = $RuntimePath }
        # Deliberately omit every application/BSP/RTOS include directory.
        $geometryElf = 'build/tests/kinematics-standalone.elf'
        & arm-none-eabi-gcc @flags -Imodules/swerve tests/kinematics_standalone.c `
            tests/startup.s modules/swerve/swerve_kinematics.c `
            -nostartfiles '-specs=nano.specs' '-specs=nosys.specs' `
            '-Wl,--gc-sections' -T tests/test.ld -lm -lc -o $geometryElf
        if ($LASTEXITCODE) { throw 'Standalone kinematics compilation failed' }
        & $PythonExecutable tests/emulate.py $geometryElf
        if ($LASTEXITCODE) { throw 'Standalone kinematics tests failed' }
        foreach ($sourceMode in 0, 1) {
            $elf = "build/tests/integration-$sourceMode.elf"
            & arm-none-eabi-gcc @flags "-DSWERVE_COMMAND_SOURCE=$sourceMode" @includes @sources `
                -nostartfiles '-specs=nano.specs' '-specs=nosys.specs' `
                '-Wl,--gc-sections' -T tests/test.ld -lm -lc -o $elf
            if ($LASTEXITCODE) { throw "Test compilation failed for command source $sourceMode" }
            & $PythonExecutable tests/emulate.py $elf
            if ($LASTEXITCODE) { throw "Integration tests failed for command source $sourceMode" }
        }
        $bspElf = 'build/tests/can-bsp.elf'
        & arm-none-eabi-gcc @flags @includes tests/can_bsp.c tests/startup.s `
            -nostartfiles '-specs=nano.specs' '-specs=nosys.specs' `
            '-Wl,--gc-sections' -T tests/test.ld -lm -lc -o $bspElf
        if ($LASTEXITCODE) { throw 'CAN BSP test compilation failed' }
        & $PythonExecutable tests/emulate.py $bspElf
        if ($LASTEXITCODE) { throw 'CAN BSP tests failed' }
    } finally { $env:PYTHONPATH = $previousPath }
} finally { Pop-Location }
