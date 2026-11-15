/*
 * Intelligent transport FreeRTOS pseudocode.
 *
 * DESIGN ONLY - DO NOT ADD THIS FILE TO THE KEIL BUILD.
 * The types and helper functions below describe ownership, sequencing and
 * failure handling. They are intentionally incomplete and are not expected to
 * compile until the BSP and protocol contracts are frozen.
 */

/* ---------- System model ---------- */

enum SystemEventBits {
    EV_SELF_TEST_OK          = 1u << 0,
    EV_START_LATCHED         = 1u << 1,
    EV_MISSION_VALID         = 1u << 2,
    EV_RUNNING               = 1u << 3,
    EV_PAYLOAD_ON_PLATFORM   = 1u << 4,
    EV_OBSTACLE              = 1u << 5,
    EV_ESTOP                 = 1u << 6,
    EV_FAULT                 = 1u << 7,
    EV_FINISHED              = 1u << 8,
};

enum FaultCode {
    FAULT_NONE,
    FAULT_START_INVALID,
    FAULT_MISSION_PARSE,
    FAULT_CAN_TIMEOUT,
    FAULT_CAN_BUS_OFF,
    FAULT_VISION_TIMEOUT,
    FAULT_SENSOR_STALE,
    FAULT_MOTOR,
    FAULT_LIMIT_CONFLICT,
    FAULT_PAYLOAD_MISSING,
    FAULT_STACK_COLOR_MISMATCH,
    FAULT_ACTION_TIMEOUT,
    FAULT_QUEUE_OVERFLOW,
    FAULT_TASK_HEARTBEAT,
    FAULT_STACK_OVERFLOW,
    FAULT_MALLOC,
};

enum MatchState {
    MATCH_BOOT,
    MATCH_SELF_TEST,
    MATCH_WAIT_START,
    MATCH_READ_MISSION,
    MATCH_PICK_BATCH1_ITEM1,
    MATCH_PICK_BATCH1_ITEM2,
    MATCH_PICK_BATCH1_ITEM3,
    MATCH_MOVE_PROCESSING1,
    MATCH_PLACE_PROCESSING1,
    MATCH_TRANSFER_BATCH1_TO_TEMP,
    MATCH_RETURN_RAW,
    MATCH_PICK_BATCH2_ITEM1,
    MATCH_PICK_BATCH2_ITEM2,
    MATCH_PICK_BATCH2_ITEM3,
    MATCH_MOVE_PROCESSING2,
    MATCH_PLACE_PROCESSING2,
    MATCH_MOVE_TEMP2,
    MATCH_STACK_BATCH2,
    MATCH_RETURN_START,
    MATCH_FINISH,
    MATCH_SAFE_STOP,
};

enum MatchEventType {
    ME_NONE,
    ME_START,
    ME_MISSION_OK,
    ME_MISSION_BAD,
    ME_MOTION_DONE,
    ME_MOTION_FAILED,
    ME_ACTUATOR_DONE,
    ME_ACTUATOR_FAILED,
    ME_PAYLOAD_CONFIRMED,
    ME_OBSTACLE,
    ME_TIMEOUT,
    ME_FAULT,
};

typedef struct {
    uint8_t colors[3];
    uint8_t rings[3];
} MissionBatch;

typedef struct {
    MissionBatch batch[2];
    char canonical_code[20];
    uint32_t sequence;
    bool valid;
} Mission;

typedef struct {
    float x_m;
    float y_m;
    float yaw_rad;
    float vx_mps;
    float vy_mps;
    float quality;
    uint32_t timestamp_ms;
    uint32_t sequence;
    bool valid;
} NavEstimate;

typedef struct {
    float vx_mps;
    float vy_mps;
    float wz_radps;
    uint32_t valid_until_ms;
    uint32_t sequence;
    bool enabled;
} MotionSetpoint;

typedef struct {
    float wheel_rpm[4];
    uint32_t sequence;
    bool emergency_stop;
} WheelSetpoint;

typedef struct {
    uint32_t request_id;
    int destination;
    float max_speed;
    uint32_t timeout_ms;
    bool require_vision_alignment;
} MotionRequest;

typedef struct {
    uint32_t request_id;
    int action;
    uint8_t color;
    uint8_t ring;
    uint8_t platform_slot;
    uint32_t timeout_ms;
} ActuatorRequest;

typedef struct {
    enum MatchEventType type;
    uint32_t request_id;
    int result;
    uint32_t timestamp_ms;
} MatchEvent;

/* ---------- RTOS handles ---------- */

static osEventFlagsId_t eg_system;
static osMessageQueueId_t q_match_event;
static osMessageQueueId_t q_motion_request;
static osMessageQueueId_t q_actuator_request;
static osMessageQueueId_t q_can_rx;
static osMessageQueueId_t q_can_tx;
static osMessageQueueId_t q_can_urgent;
static osMessageQueueId_t q_display;

static osThreadId_t safety_task;
static osThreadId_t control_task;
static osThreadId_t sensor_fusion_task;
static osThreadId_t comms_task;
static osThreadId_t motion_task;
static osThreadId_t actuator_task;
static osThreadId_t perception_task;
static osThreadId_t match_task;
static osThreadId_t display_task;
static osThreadId_t telemetry_task;

static volatile bool hardware_estop_latched;
static volatile bool stop_all_outputs;
static volatile uint32_t heartbeat_bits;
static volatile enum FaultCode latched_fault;

/* Single-writer versioned snapshots. The real implementation must make each
 * write atomic with a short critical section or an equivalent seqlock. */
static NavEstimate nav_snapshot;
static MotionSetpoint motion_snapshot;
static WheelSetpoint wheel_snapshot;
static Mission mission_snapshot;

/* ---------- Creation ---------- */

void AppTasks_Init(void)
{
    eg_system         = osEventFlagsNew(NULL);
    q_match_event     = osMessageQueueNew(16, sizeof(MatchEvent), NULL);
    q_motion_request  = osMessageQueueNew(4, sizeof(MotionRequest), NULL);
    q_actuator_request= osMessageQueueNew(4, sizeof(ActuatorRequest), NULL);
    q_can_rx          = osMessageQueueNew(32, sizeof(CanFrame), NULL);
    q_can_tx          = osMessageQueueNew(16, sizeof(CanFrame), NULL);
    q_can_urgent      = osMessageQueueNew(4, sizeof(CanFrame), NULL);
    q_display         = osMessageQueueNew(8, sizeof(DisplayMessage), NULL);

    ASSERT_ALL_OBJECTS_CREATED();

    safety_task       = CREATE_TASK(SafetyTask,       osPriorityRealtime,    768);
    control_task      = CREATE_TASK(ControlTask,      osPriorityHigh,       1024);
    sensor_fusion_task= CREATE_TASK(SensorFusionTask, osPriorityAboveNormal2,1536);
    comms_task        = CREATE_TASK(CommsTask,        osPriorityAboveNormal1,1024);
    motion_task       = CREATE_TASK(MotionTask,       osPriorityAboveNormal,1536);
    actuator_task     = CREATE_TASK(ActuatorTask,     osPriorityNormal3,    1536);
    perception_task   = CREATE_TASK(PerceptionTask,   osPriorityNormal2,    1536);
    match_task        = CREATE_TASK(MatchTask,        osPriorityNormal,     2048);
    display_task      = CREATE_TASK(DisplayTask,      osPriorityLow3,       1024);
    telemetry_task    = CREATE_TASK(TelemetryTask,    osPriorityLow,        1536);
}

/* ---------- ISR boundary ---------- */

void ESTOP_IRQHandler(void)
{
    /* This interrupt may run above the FreeRTOS syscall priority. */
    HARDWARE_ACTUATOR_GATE_OFF();
    hardware_estop_latched = true;
    stop_all_outputs = true;
    /* Do not call an RTOS API here when NVIC priority is numerically < 5. */
}

void CONTROL_TIMER_IRQHandler(void)
{
    CLEAR_CONTROL_TIMER_IRQ();
    osThreadFlagsSet(control_task, CONTROL_TICK_FLAG);
    /* The dedicated control timer must have an RTOS-safe NVIC priority. */
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    if (uart == QR_UART) {
        STREAM_BUFFER_SEND_FROM_ISR(sb_qr_rx, qr_dma_buffer, size);
    } else if (uart == VISION_UART) {
        STREAM_BUFFER_SEND_FROM_ISR(sb_vision_rx, vision_dma_buffer, size);
    }
    RESTART_UART_RECEIVE_TO_IDLE_DMA(uart);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *can, uint32_t flags)
{
    CanFrame frame;
    while (BSP_CAN_READ_FIFO(can, &frame)) {
        if (!QUEUE_PUT_FROM_ISR(q_can_rx, &frame)) {
            RECORD_ISR_OVERFLOW();
        }
    }
}

/* ---------- Safety ---------- */

void SafetyTask(void *argument)
{
    bool start_consumed = false;
    uint32_t missing_heartbeat_windows = 0;

    PERFORM_POWER_ON_SELF_TEST();
    if (SELF_TEST_PASSED()) {
        osEventFlagsSet(eg_system, EV_SELF_TEST_OK);
    } else {
        LATCH_FATAL_FAULT(FAULT_MOTOR);
    }

    for (;;) {
        osDelay(10);

        if (hardware_estop_latched) {
            LATCH_FATAL_FAULT(FAULT_NONE);
            osEventFlagsSet(eg_system, EV_ESTOP | EV_FAULT);
        }

        if (!start_consumed && START_BUTTON_DEBOUNCED_PRESSED() &&
            SYSTEM_READY_AND_SAFE()) {
            start_consumed = true;
            osEventFlagsSet(eg_system, EV_START_LATCHED);
            POST_MATCH_EVENT(ME_START, 0, RESULT_OK);
        }

        if (SYSTEM_IS_RUNNING()) {
            if (ALL_REQUIRED_HEARTBEATS_SEEN()) {
                missing_heartbeat_windows = 0;
                CLEAR_HEARTBEAT_WINDOW();
            } else if (++missing_heartbeat_windows >= HEARTBEAT_FAIL_LIMIT) {
                LATCH_FATAL_FAULT(FAULT_TASK_HEARTBEAT);
            }
        }

        if (FAULT_OR_ESTOP_LATCHED()) {
            stop_all_outputs = true;
            WRITE_ZERO_MOTION_SNAPSHOT();
            HARDWARE_ACTUATOR_GATE_OFF();
            NOTIFY_COMMS_EMERGENCY_STOP();
            POST_MATCH_EVENT_NONBLOCKING(ME_FAULT, 0, latched_fault);
            SET_ERROR_LED();
        }

        if (SYSTEM_HEALTHY_FOR_WATCHDOG()) {
            HAL_IWDG_Refresh(&hiwdg);
        }
    }
}

/* ---------- Deterministic control ---------- */

void ControlTask(void *argument)
{
    for (;;) {
        osThreadFlagsWait(CONTROL_TICK_FLAG, osFlagsWaitAny, osWaitForever);

        MotionSetpoint target = READ_CONSISTENT_MOTION_SNAPSHOT();
        NavEstimate nav = READ_CONSISTENT_NAV_SNAPSHOT();
        WheelSetpoint output = {0};

        if (stop_all_outputs || !target.enabled || TARGET_IS_EXPIRED(target) ||
            NAV_IS_STALE(nav)) {
            output.emergency_stop = stop_all_outputs;
            SET_ALL_WHEEL_RPM(output, 0.0f);
        } else {
            BodyVelocity corrected = RUN_HEADING_AND_VELOCITY_CONTROL(target, nav, 0.001f);
            corrected = APPLY_ACCELERATION_AND_SPEED_LIMITS(corrected);
            output = MECANUM_INVERSE_KINEMATICS(corrected);
        }

        WRITE_WHEEL_SNAPSHOT(output);
        SET_HEARTBEAT(HB_CONTROL);
    }
}

/* ---------- IMU, optical flow and wheel feedback ---------- */

void SensorFusionTask(void *argument)
{
    uint32_t next = osKernelGetTickCount();
    NavEstimate estimate = RESET_NAV_ESTIMATE();

    for (;;) {
        next += MS_TO_TICKS(5);

        ImuSample imu = READ_IMU_SAMPLE();
        FlowSample flow = READ_OPTICAL_FLOW_SAMPLE();
        WheelFeedback wheel = READ_LATEST_WHEEL_FEEDBACK();

        if (IMU_SAMPLE_VALID(imu)) {
            UPDATE_YAW(&estimate, imu);
        }

        if (FLOW_SAMPLE_VALID(flow) && FLOW_QUALITY_OK(flow) &&
            SENSOR_HEIGHT_WITHIN_CALIBRATED_RANGE()) {
            BodyDelta body = FLOW_TO_BODY_DELTA(flow);
            FieldDelta field = ROTATE_BY_YAW(body, estimate.yaw_rad);
            UPDATE_XY_FROM_FLOW(&estimate, field);
        } else {
            REDUCE_POSITION_QUALITY(&estimate);
        }

        CROSS_CHECK_FLOW_AND_WHEEL_FEEDBACK(&estimate, flow, wheel);
        estimate.timestamp_ms = NOW_MS();
        WRITE_NAV_SNAPSHOT(estimate);
        SET_HEARTBEAT(HB_SENSOR_FUSION);
        osDelayUntil(next);
    }
}

/* ---------- CAN ownership ---------- */

void CommsTask(void *argument)
{
    uint32_t next_send = osKernelGetTickCount();

    for (;;) {
        PROCESS_ALL_AVAILABLE_CAN_RX_FRAMES();

        if (EMERGENCY_STOP_NOTIFICATION_PENDING() || stop_all_outputs) {
            DISCARD_NORMAL_MOTION_FRAMES();
            SEND_CHASSIS_IMMEDIATE_STOP();
            SEND_ARM_IMMEDIATE_STOP();
        } else if (TICK_REACHED(next_send)) {
            WheelSetpoint latest = READ_CONSISTENT_WHEEL_SNAPSHOT();
            SEND_FOUR_ZDT_TARGETS_BUFFERED(latest);
            SEND_ZDT_SYNCHRONOUS_TRIGGER();
            next_send += MS_TO_TICKS(5);  /* 200 Hz CAN command rate */
        }

        SEND_URGENT_QUEUE_BEFORE_NORMAL_QUEUE();
        UPDATE_CAN_LINK_HEALTH_AND_TIMEOUTS();
        SET_HEARTBEAT(HB_COMMS);
        WAIT_FOR_CAN_EVENT_OR_1MS();
    }
}

/* ---------- Motion action state machine ---------- */

void MotionTask(void *argument)
{
    MotionAction action = MOTION_IDLE;
    MotionRequest request = {0};

    for (;;) {
        if (action == MOTION_IDLE &&
            osMessageQueueGet(q_motion_request, &request, NULL, 10) == osOK) {
            action = MOTION_COARSE_APPROACH;
            START_MOTION_DEADLINE(request);
        }

        if (stop_all_outputs) {
            WRITE_ZERO_MOTION_SNAPSHOT();
            FAIL_ACTIVE_MOTION(request, FAULT_NONE);
            action = MOTION_IDLE;
            continue;
        }

        if (DEADLINE_EXPIRED(request)) {
            WRITE_ZERO_MOTION_SNAPSHOT();
            FAIL_ACTIVE_MOTION(request, FAULT_ACTION_TIMEOUT);
            action = MOTION_IDLE;
            continue;
        }

        switch (action) {
        case MOTION_COARSE_APPROACH:
            if (OBSTACLE_ACTIVE()) {
                WRITE_DECELERATE_TO_STOP_TARGET();
                action = MOTION_OBSTACLE_HOLD;
            } else {
                WRITE_ROUTE_SEGMENT_TARGET(request.destination);
                if (COARSE_DESTINATION_REACHED()) {
                    action = request.require_vision_alignment ?
                             MOTION_FINE_ALIGN : MOTION_DONE;
                }
            }
            break;

        case MOTION_OBSTACLE_HOLD:
            if (OBSTACLE_CLEAR_CONFIRMED()) {
                action = MOTION_COARSE_APPROACH;
            } else if (OBSTACLE_REQUIRES_BYPASS()) {
                action = MOTION_BYPASS;
            }
            break;

        case MOTION_BYPASS:
            RUN_BOUNDED_BYPASS_MANEUVER();
            if (ROUTE_REACQUIRED()) action = MOTION_COARSE_APPROACH;
            break;

        case MOTION_FINE_ALIGN:
            if (!FRESH_HIGH_CONFIDENCE_RING_RESULT()) {
                WRITE_ZERO_MOTION_SNAPSHOT();
                REQUEST_NEW_VISION_FRAME();
            } else {
                WRITE_VISION_ALIGNMENT_TARGET();
                if (ALIGNMENT_STABLE_FOR_N_FRAMES()) action = MOTION_DONE;
            }
            break;

        case MOTION_DONE:
            WRITE_ZERO_MOTION_SNAPSHOT();
            POST_MATCH_EVENT(ME_MOTION_DONE, request.request_id, RESULT_OK);
            action = MOTION_IDLE;
            break;
        }

        SET_HEARTBEAT(HB_MOTION);
        osDelay(10);
    }
}

/* ---------- Manipulator action state machine ---------- */

void ActuatorTask(void *argument)
{
    ActuatorRequest request = {0};
    ActuatorActionState state = ACT_IDLE;

    for (;;) {
        if (state == ACT_IDLE &&
            osMessageQueueGet(q_actuator_request, &request, NULL, 10) == osOK) {
            if (!ACTUATOR_REQUEST_RULES_VALID(request, mission_snapshot)) {
                POST_MATCH_EVENT(ME_ACTUATOR_FAILED, request.request_id, RESULT_RULE_REJECTED);
            } else {
                state = ACT_MOVE_TO_SAFE_APPROACH;
                START_ACTUATOR_DEADLINE(request);
            }
        }

        if (stop_all_outputs) {
            CANCEL_ACTUATOR_OUTPUTS_SAFELY();
            state = ACT_IDLE;
            continue;
        }

        if (ACTUATOR_DEADLINE_EXPIRED(request)) {
            CANCEL_ACTUATOR_OUTPUTS_SAFELY();
            POST_MATCH_EVENT(ME_ACTUATOR_FAILED, request.request_id, FAULT_ACTION_TIMEOUT);
            state = ACT_IDLE;
            continue;
        }

        /* Each step sends a target once and waits for feedback or a sensor.
         * No HAL_Delay and no busy-wait loop is allowed. */
        switch (state) {
        case ACT_MOVE_TO_SAFE_APPROACH:
            COMMAND_SAFE_APPROACH_POSE_ONCE();
            if (SAFE_APPROACH_REACHED()) state = ACT_ALIGN_GRIPPER;
            break;

        case ACT_ALIGN_GRIPPER:
            COMMAND_ALIGNMENT_POSE_ONCE();
            if (ALIGNMENT_POSE_REACHED()) state = ACT_GRIP_OR_RELEASE;
            break;

        case ACT_GRIP_OR_RELEASE:
            COMMAND_GRIPPER_ONCE(request.action);
            if (GRIPPER_ACTION_CONFIRMED()) state = ACT_VERIFY_PAYLOAD;
            break;

        case ACT_VERIFY_PAYLOAD:
            if (REQUEST_IS_PICK(request)) {
                if (PAYLOAD_ON_PLATFORM_CONFIRMED()) {
                    osEventFlagsSet(eg_system, EV_PAYLOAD_ON_PLATFORM);
                    state = ACT_RETURN_SAFE;
                }
            } else if (PLACEMENT_CONFIRMED_BY_SENSOR_OR_VISION()) {
                state = ACT_RETURN_SAFE;
            }
            break;

        case ACT_RETURN_SAFE:
            COMMAND_TRANSPORT_SAFE_POSE_ONCE();
            if (TRANSPORT_SAFE_POSE_REACHED()) state = ACT_DONE;
            break;

        case ACT_DONE:
            POST_MATCH_EVENT(ME_ACTUATOR_DONE, request.request_id, RESULT_OK);
            state = ACT_IDLE;
            break;
        }

        SET_HEARTBEAT(HB_ACTUATOR);
        osDelay(10);
    }
}

/* ---------- Perception ---------- */

void PerceptionTask(void *argument)
{
    for (;;) {
        if (QR_STREAM_HAS_FRAME()) {
            TextFrame qr = READ_QR_FRAME();
            Mission parsed;
            if (FRAME_CRC_OK(qr) && PARSE_FOUR_GROUP_MISSION(qr, &parsed)) {
                WRITE_MISSION_SNAPSHOT(parsed);
                osEventFlagsSet(eg_system, EV_MISSION_VALID);
                POST_MATCH_EVENT(ME_MISSION_OK, 0, RESULT_OK);
                POST_DISPLAY_MISSION(parsed);
            } else {
                POST_MATCH_EVENT(ME_MISSION_BAD, 0, FAULT_MISSION_PARSE);
            }
        }

        if (VISION_STREAM_HAS_FRAME()) {
            VisionFrame frame = READ_VISION_FRAME();
            if (FRAME_LENGTH_CRC_SEQUENCE_OK(frame)) {
                UPDATE_TIMESTAMPED_PERCEPTION_SNAPSHOT(frame);
                UPDATE_OBSTACLE_EVENT_WITH_HYSTERESIS(frame);
            } else {
                RECORD_VISION_FRAME_ERROR();
            }
        }

        INVALIDATE_STALE_PERCEPTION_RESULTS();
        SET_HEARTBEAT(HB_PERCEPTION);
        WAIT_FOR_UART_DATA_OR_10MS();
    }
}

/* ---------- Match owner ---------- */

void MatchTask(void *argument)
{
    enum MatchState state = MATCH_BOOT;
    MatchEvent event = {0};
    uint32_t active_request = 0;
    uint8_t item_index = 0;

    for (;;) {
        osMessageQueueGet(q_match_event, &event, NULL, 10);

        if (event.type == ME_FAULT || FATAL_SYSTEM_EVENT_ACTIVE()) {
            state = MATCH_SAFE_STOP;
        }

        switch (state) {
        case MATCH_BOOT:
            state = MATCH_SELF_TEST;
            break;

        case MATCH_SELF_TEST:
            if (EVENT_IS_SET(EV_SELF_TEST_OK)) state = MATCH_WAIT_START;
            break;

        case MATCH_WAIT_START:
            if (event.type == ME_START) {
                osEventFlagsSet(eg_system, EV_RUNNING);
                REQUEST_QR_RECOGNITION();
                START_STATE_DEADLINE(QR_TIMEOUT_MS);
                state = MATCH_READ_MISSION;
            }
            break;

        case MATCH_READ_MISSION:
            if (event.type == ME_MISSION_OK) {
                item_index = 0;
                active_request = REQUEST_PICK_ITEM(0, item_index);
                state = MATCH_PICK_BATCH1_ITEM1;
            } else if (STATE_DEADLINE_EXPIRED()) {
                RAISE_FATAL_FAULT(FAULT_MISSION_PARSE);
            }
            break;

        case MATCH_PICK_BATCH1_ITEM1:
        case MATCH_PICK_BATCH1_ITEM2:
        case MATCH_PICK_BATCH1_ITEM3:
            if (MATCHING_ACTION_DONE(event, active_request) &&
                EVENT_IS_SET(EV_PAYLOAD_ON_PLATFORM)) {
                UPDATE_PICK_STATISTICS(0, item_index);
                if (++item_index < 3) {
                    active_request = REQUEST_PICK_ITEM(0, item_index);
                    state = NEXT_BATCH1_PICK_STATE(state);
                } else {
                    active_request = REQUEST_MOVE(DEST_PROCESSING, true);
                    state = MATCH_MOVE_PROCESSING1;
                }
            }
            break;

        case MATCH_MOVE_PROCESSING1:
            if (MATCHING_MOTION_DONE(event, active_request)) {
                active_request = REQUEST_PLACE_BATCH_AT_PROCESSING(0);
                state = MATCH_PLACE_PROCESSING1;
            }
            break;

        case MATCH_PLACE_PROCESSING1:
            if (MATCHING_ACTION_DONE(event, active_request)) {
                active_request = REQUEST_TRANSFER_PROCESSING_TO_TEMP(0);
                state = MATCH_TRANSFER_BATCH1_TO_TEMP;
            }
            break;

        case MATCH_TRANSFER_BATCH1_TO_TEMP:
            if (MATCHING_ACTION_DONE(event, active_request)) {
                active_request = REQUEST_MOVE(DEST_RAW_MATERIAL, false);
                state = MATCH_RETURN_RAW;
            }
            break;

        case MATCH_RETURN_RAW:
            if (MATCHING_MOTION_DONE(event, active_request)) {
                item_index = 0;
                active_request = REQUEST_PICK_ITEM(1, item_index);
                state = MATCH_PICK_BATCH2_ITEM1;
            }
            break;

        case MATCH_PICK_BATCH2_ITEM1:
        case MATCH_PICK_BATCH2_ITEM2:
        case MATCH_PICK_BATCH2_ITEM3:
            if (MATCHING_ACTION_DONE(event, active_request) &&
                EVENT_IS_SET(EV_PAYLOAD_ON_PLATFORM)) {
                UPDATE_PICK_STATISTICS(1, item_index);
                if (++item_index < 3) {
                    active_request = REQUEST_PICK_ITEM(1, item_index);
                    state = NEXT_BATCH2_PICK_STATE(state);
                } else {
                    active_request = REQUEST_MOVE(DEST_PROCESSING, true);
                    state = MATCH_MOVE_PROCESSING2;
                }
            }
            break;

        case MATCH_MOVE_PROCESSING2:
            if (MATCHING_MOTION_DONE(event, active_request)) {
                active_request = REQUEST_PLACE_BATCH_AT_PROCESSING(1);
                state = MATCH_PLACE_PROCESSING2;
            }
            break;

        case MATCH_PLACE_PROCESSING2:
            if (MATCHING_ACTION_DONE(event, active_request)) {
                active_request = REQUEST_MOVE(DEST_TEMP_STORAGE, true);
                state = MATCH_MOVE_TEMP2;
            }
            break;

        case MATCH_MOVE_TEMP2:
            if (MATCHING_MOTION_DONE(event, active_request)) {
                if (!SECOND_BATCH_STACK_TARGETS_VALID(mission_snapshot)) {
                    RAISE_FATAL_FAULT(FAULT_STACK_COLOR_MISMATCH);
                } else {
                    active_request = REQUEST_STACK_SECOND_BATCH();
                    state = MATCH_STACK_BATCH2;
                }
            }
            break;

        case MATCH_STACK_BATCH2:
            if (MATCHING_ACTION_DONE(event, active_request)) {
                active_request = REQUEST_MOVE(DEST_ORIGINAL_START_ZONE, false);
                state = MATCH_RETURN_START;
            }
            break;

        case MATCH_RETURN_START:
            if (MATCHING_MOTION_DONE(event, active_request) && CHASSIS_STOPPED()) {
                state = MATCH_FINISH;
            }
            break;

        case MATCH_FINISH:
            osEventFlagsClear(eg_system, EV_RUNNING);
            osEventFlagsSet(eg_system, EV_FINISHED);
            POST_FINAL_STATISTICS_TO_DISPLAY();
            WRITE_ZERO_MOTION_SNAPSHOT();
            break;

        case MATCH_SAFE_STOP:
            osEventFlagsClear(eg_system, EV_RUNNING);
            WRITE_ZERO_MOTION_SNAPSHOT();
            CANCEL_ALL_ACTIVE_REQUESTS();
            POST_FAULT_TO_DISPLAY(latched_fault);
            /* One-start rule: wait for a complete system reset, not a retry. */
            break;
        }

        if (ACTION_FAILURE_MATCHES_ACTIVE_REQUEST(event, active_request)) {
            RAISE_FATAL_FAULT(TRANSLATE_ACTION_FAILURE(event));
        }

        SET_HEARTBEAT(HB_MATCH);
    }
}

/* ---------- Low-priority services ---------- */

void DisplayTask(void *argument)
{
    DisplayMessage message;
    for (;;) {
        if (osMessageQueueGet(q_display, &message, NULL, 100) == osOK) {
            MERGE_MESSAGE_INTO_DISPLAY_MODEL(message);
        }
        SEND_ONLY_CHANGED_SCREEN_FIELDS_OVER_USART3();
        SET_HEARTBEAT(HB_DISPLAY);
    }
}

void TelemetryTask(void *argument)
{
    for (;;) {
        osDelay(100);
        if (DEBUG_MODE_ENABLED_AND_NOT_IN_MATCH_CONTROL_MODE()) {
            REPORT_STACK_HIGH_WATER_MARKS();
            REPORT_QUEUE_PEAKS_AND_DROPS();
            REPORT_CPU_AND_CAN_LOAD();
            REPORT_CURRENT_STATE_AND_FAULT();
        }
        /* Never accept a chassis or actuator command in competition mode. */
        SET_HEARTBEAT(HB_TELEMETRY);
    }
}

/* ---------- Fatal hooks ---------- */

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    HARDWARE_ACTUATOR_GATE_OFF();
    PERSIST_CRASH_REASON(FAULT_STACK_OVERFLOW, name);
    SYSTEM_RESET_OR_HALT_SAFELY();
}

void vApplicationMallocFailedHook(void)
{
    HARDWARE_ACTUATOR_GATE_OFF();
    PERSIST_CRASH_REASON(FAULT_MALLOC, NULL);
    SYSTEM_RESET_OR_HALT_SAFELY();
}
