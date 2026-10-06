#include "Arduino.h"
#include "system_task_handle.h"

// 增大 loopTask 栈，OTA + 初始化需要更多栈空间
SET_LOOP_TASK_STACK_SIZE(16384);

esp_event_loop_handle_t loop_handle;

void setup()
{
    esp_event_loop_args_t loop_args = {
        .queue_size = 10,
        .task_name = "sys_evt_loop",
        .task_priority = uxTaskPriorityGet(NULL),
        .task_stack_size = 4096,
        .task_core_id = tskNO_AFFINITY
    };

    esp_event_loop_create(&loop_args, &loop_handle);
    register_system_task(&loop_handle);
}

void loop()
{
    system_loop_handler();
}