@echo off
setlocal
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cd /d "%~dp0"
if not exist build mkdir build
cl /nologo /W4 /WX /std:c11 /TC /I ../../Algorithm ../../Algorithm/pid.c ../../Algorithm/kalman.c ../../Algorithm/interpolation.c test_algorithm.c /Fe:build/test_algorithm.exe /Fo:build/
if errorlevel 1 exit /b 1
build\test_algorithm.exe
if errorlevel 1 exit /b 1
cl /nologo /W4 /WX /std:c11 /TC /DBSP_HOST_TEST /I ../../BSP ../../BSP/bsp_pwm.c ../../BSP/bsp_encoder.c ../../BSP/bsp_timebase.c test_bsp_math.c /Fe:build/test_bsp_math.exe /Fo:build/
if errorlevel 1 exit /b 1
build\test_bsp_math.exe
if errorlevel 1 exit /b 1
cl /nologo /W4 /WX /std:c11 /TC /I ../../Platform /I ../../Device /I ../../BSP /I ../../Config ../../Device/mdd3a.c ../../Device/motor.c ../../Device/encoder.c ../../Device/sg90.c ../../Device/limit_switch.c test_device.c /Fe:build/test_device.exe /Fo:build/
if errorlevel 1 exit /b 1
build\test_device.exe
if errorlevel 1 exit /b 1
cl /nologo /W4 /WX /std:c11 /TC /I ../../Platform /I ../../Config ../../Platform/Simulation/platform_sim.c test_platform_sim.c /Fe:build/test_platform_sim.exe /Fo:build/
if errorlevel 1 exit /b 1
build\test_platform_sim.exe
if errorlevel 1 exit /b 1
cl /nologo /W4 /WX /std:c11 /TC /I ../../Controller /I ../../Algorithm /I ../../Platform /I ../../Config ../../Controller/joint_controller.c ../../Algorithm/pid.c ../../Algorithm/kalman.c test_joint_controller.c /Fe:build/test_joint_controller.exe /Fo:build/
if errorlevel 1 exit /b 1
build\test_joint_controller.exe
if errorlevel 1 exit /b 1
cl /nologo /W4 /WX /std:c11 /TC /I ../../Communication /I ../../Config ../../Communication/protocol.c ../../Communication/crc.c ../../Communication/frame.c ../../Communication/parser.c ../../Communication/command.c ../../Communication/telemetry.c test_communication.c /Fe:build/test_communication.exe /Fo:build/
if errorlevel 1 exit /b 1
build\test_communication.exe
if errorlevel 1 exit /b 1
cl /nologo /W4 /WX /std:c11 /TC /I ../../Application /I ../../Communication /I ../../Config ../../Application/state_machine.c test_state_machine.c /Fe:build/test_state_machine.exe /Fo:build/
if errorlevel 1 exit /b 1
build\test_state_machine.exe
if errorlevel 1 exit /b 1
cl /nologo /W4 /WX /std:c11 /TC /I ../../Application /I ../../Communication /I ../../Config ../../Application/safety.c test_safety.c /Fe:build/test_safety.exe /Fo:build/
if errorlevel 1 exit /b 1
build\test_safety.exe
if errorlevel 1 exit /b 1
cl /nologo /W4 /WX /std:c11 /TC /I ../../Application /I ../../Platform /I ../../Config ../../Application/homing.c ../../Platform/Simulation/platform_sim.c test_homing.c /Fe:build/test_homing.exe /Fo:build/
if errorlevel 1 exit /b 1
build\test_homing.exe
if errorlevel 1 exit /b 1
cl /nologo /W4 /WX /std:c11 /TC /I ../../Application /I ../../Platform /I ../../Config ../../Application/gripper.c ../../Platform/Simulation/platform_sim.c test_gripper.c /Fe:build/test_gripper.exe /Fo:build/
if errorlevel 1 exit /b 1
build\test_gripper.exe
if errorlevel 1 exit /b 1
cl /nologo /W4 /WX /std:c11 /TC /I ../../Application /I ../../Controller /I ../../Algorithm /I ../../Platform /I ../../Config /I ../../Communication ../../Application/robot.c ../../Application/state_machine.c ../../Application/safety.c ../../Application/homing.c ../../Application/gripper.c ../../Controller/joint_controller.c ../../Controller/trajectory_controller.c ../../Algorithm/interpolation.c ../../Algorithm/pid.c ../../Algorithm/kalman.c ../../Platform/Simulation/platform_sim.c ../../Communication/protocol.c ../../Communication/telemetry.c test_robot.c /Fe:build/test_robot.exe /Fo:build/
if errorlevel 1 exit /b 1
build\test_robot.exe
if errorlevel 1 exit /b 1
cl /nologo /W4 /WX /std:c11 /TC /I ../../Controller /I ../../Config /I ../../Algorithm ../../Controller/trajectory_controller.c ../../Algorithm/interpolation.c test_trajectory_controller.c /Fe:build/test_trajectory_controller.exe /Fo:build/
if errorlevel 1 exit /b 1
build\test_trajectory_controller.exe
if errorlevel 1 exit /b 1
cl /nologo /W4 /WX /std:c11 /TC /DTASKS_HOST_TEST /I ../../Tasks /I ../../Application /I ../../Controller /I ../../Algorithm /I ../../Platform /I ../../Communication /I ../../Config ../../Tasks/shared_data.c ../../Tasks/control_task.c test_shared_data.c /Fe:build/test_shared_data.exe /Fo:build/
if errorlevel 1 exit /b 1
build\test_shared_data.exe
if errorlevel 1 exit /b 1
cl /nologo /W4 /WX /std:c11 /TC /DTASKS_HOST_TEST /I ../../BSP /I ../../Tasks /I ../../Application /I ../../Controller /I ../../Algorithm /I ../../Platform /I ../../Communication /I ../../Config ../../Tasks/shared_data.c ../../Tasks/communication_task.c ../../Communication/protocol.c ../../Communication/crc.c ../../Communication/frame.c ../../Communication/parser.c ../../Communication/command.c ../../Communication/telemetry.c test_communication_task.c /Fe:build/test_communication_task.exe /Fo:build/
if errorlevel 1 exit /b 1
build\test_communication_task.exe
if errorlevel 1 exit /b 1
cl /nologo /W4 /WX /std:c11 /TC /DBSP_UART_HOST_TEST /I ../Integration/uart_stub /I ../../BSP ../../BSP/bsp_uart.c ../Integration/test_uart_dma.c /Fe:build/test_uart_dma.exe /Fo:build/
if errorlevel 1 exit /b 1
build\test_uart_dma.exe
if errorlevel 1 exit /b 1
cl /nologo /W4 /WX /std:c11 /TC /DTASKS_HOST_TEST /I ../../Tasks /I ../../BSP /I ../../Application /I ../../Controller /I ../../Algorithm /I ../../Platform /I ../../Config /I ../../Communication ../../Application/robot.c ../../Application/state_machine.c ../../Application/safety.c ../../Application/homing.c ../../Application/gripper.c ../../Controller/joint_controller.c ../../Controller/trajectory_controller.c ../../Algorithm/interpolation.c ../../Algorithm/pid.c ../../Algorithm/kalman.c ../../Platform/Simulation/platform_sim.c ../../Communication/protocol.c ../../Communication/telemetry.c ../../Tasks/shared_data.c ../../Tasks/communication_task.c ../../Communication/crc.c ../../Communication/frame.c ../../Communication/parser.c ../../Communication/command.c ../Integration/test_communication_robot.c /Fe:build/test_communication_robot.exe /Fo:build/
if errorlevel 1 exit /b 1
build\test_communication_robot.exe
if errorlevel 1 exit /b 1
cl /nologo /W4 /WX /std:c11 /TC /DTASKS_HOST_TEST /I ../../BSP /I ../../Tasks /I ../../Application /I ../../Controller /I ../../Algorithm /I ../../Platform /I ../../Communication /I ../../Config ../../Tasks/shared_data.c ../../Tasks/communication_task.c ../../Communication/protocol.c ../../Communication/crc.c ../../Communication/frame.c ../../Communication/parser.c ../../Communication/command.c ../../Communication/telemetry.c ../../Tasks/robot_task.c ../../Tasks/telemetry_task.c ../Integration/test_task_pipeline.c /Fe:build/test_task_pipeline.exe /Fo:build/
if errorlevel 1 exit /b 1
build\test_task_pipeline.exe
if errorlevel 1 exit /b 1
exit /b 0
