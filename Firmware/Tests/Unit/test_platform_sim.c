#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "communication_config.h"
#include "control_config.h"
#include "hardware_config.h"
#include "platform.h"

int main(void)
{
    platform_t platform = {0};
    platform_sim_state_t sim = {0};
    uint32_t raw = 99U;
    uint32_t now = 99U;
    bool active = false;

    assert(HW_JOINT_IO[ROBOT_J1].encoder_timer == 2U);
    assert(HW_JOINT_IO[ROBOT_J2].pwm_channel_b == 4U);
    assert(HW_JOINT_IO[ROBOT_J3].pwm_timer == 12U);
    assert(HW_GRIPPER_CHANNEL == 3U && HW_GRIPPER_PIN == 2U);
    assert(HW_UART_RX_DMA_STREAM == 2U && HW_UART_TX_DMA_STREAM == 7U);
    assert(HW_FREERTOS_TASK_STACK_WORDS == 512U);
    assert(HW_CONTROL_TASK_CMSIS_PRIORITY == 40U);
    assert(CONTROL_PERIOD_MS == 1U && COMM_UART_BAUD == HW_UART_BAUD);
    assert(COMM_SOF_0 == 0xAAU && COMM_SOF_1 == 0x55U);
    assert(ROBOT_ENCODER_CPR_VERIFIED == 0U);
    assert(CONTROL_PID_GAINS_VERIFIED == 0U);

    assert(platform_read_encoder(&platform, ROBOT_J1, &raw) == PLATFORM_NOT_READY);
    assert(raw == 99U);
    assert(platform_sim_bind(&platform, &sim) == PLATFORM_OK);
    assert(platform_read_encoder(&platform, ROBOT_J1, &raw) == PLATFORM_NOT_READY);
    assert(platform_set_motor_pwm(&platform, ROBOT_J1, 0.2f) == PLATFORM_NOT_READY);
    assert(sim.motor_command[ROBOT_J1] == 0.0f);

    sim.encoder_raw[ROBOT_J2] = 65535U;
    sim.limit_active[ROBOT_J2] = true;
    sim.now_ms = 1234U;
    sim.ready = true;

    assert(platform_read_encoder(&platform, ROBOT_J2, &raw) == PLATFORM_OK);
    assert(raw == 65535U);
    assert(platform_read_limit(&platform, ROBOT_J2, &active) == PLATFORM_OK);
    assert(active);
    assert(platform_time_ms(&platform, &now) == PLATFORM_OK);
    assert(now == 1234U);
    assert(platform_set_motor_pwm(&platform, ROBOT_J3, -0.5f) == PLATFORM_OK);
    assert(sim.motor_command[ROBOT_J3] == -0.5f);
    assert(platform_set_gripper(&platform, PLATFORM_GRIPPER_CLOSE) == PLATFORM_OK);
    assert(sim.gripper_command == PLATFORM_GRIPPER_CLOSE);

    assert(platform_read_encoder(&platform, ROBOT_JOINT_COUNT, &raw) == PLATFORM_INVALID_ARGUMENT);
    assert(platform_read_limit(&platform, ROBOT_J1, 0) == PLATFORM_INVALID_ARGUMENT);
    assert(platform_set_motor_pwm(&platform, ROBOT_J1, 1.1f) == PLATFORM_INVALID_ARGUMENT);
    assert(platform_set_motor_pwm(&platform, ROBOT_J1, NAN) == PLATFORM_INVALID_ARGUMENT);
    assert(platform_set_gripper(&platform, PLATFORM_GRIPPER_UNKNOWN) == PLATFORM_INVALID_ARGUMENT);
    assert(sim.motor_command[ROBOT_J1] == 0.0f);

    puts("platform_sim: PASS");
    return 0;
}
