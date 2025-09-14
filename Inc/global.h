//
// Created by dextercai on 25-8-6.
//

#ifndef GLOBAL_H
#define GLOBAL_H

#include "FreeRTOS.h"
#include "queue.h"
#include "../Src/fliter/kalman_filter.h"
#include "../Src/fliter/averageFilter.h"
#include "../Src/fliter/median_filter.h"
#include "../Src/mcp3208/mcp3208.h"
#include "../Src/report/report_0x14.h"

extern MCP3208 mcp3208;


extern SlipAvgFilter_t *slipAvgFilter_Ch;

extern HID_Report14_t report14;

extern uint16_t led_delay_ms;

extern QueueHandle_t hid_rx_queue;

// extern uint16_t roll_input_ref_clib;
extern uint16_t rudder_input_min_clib;
extern uint16_t rudder_input_max_clib;

// extern uint16_t trim_input_center_clib;

extern uint16_t pic_left_break_input_min_clib;
extern uint16_t pic_left_break_input_max_clib;

extern uint16_t pic_right_break_input_min_clib;
extern uint16_t pic_right_break_input_max_clib;

extern uint16_t pf_left_break_input_min_clib;
extern uint16_t pf_left_break_input_max_clib;

extern uint16_t pf_right_break_input_min_clib;
extern uint16_t pf_right_break_input_max_clib;

// #define RUDDER_INPUT_REF_OFFSET_ADDR 0x00
// #define YOKE_ROLL_INPUT_CENTER_TARGE 8192
#define RUDDER_INPUT_MIN_OFFSET_ADDR 0x02
#define RUDDER_INPUT_MAX_OFFSET_ADDR 0x04
// #define TRIM_INPUT_CENTER_OFFSET_ADDR 0x02
// #define TRIM_INPUT_CENTER_TARGE 8192

#define PIC_LEFT_BREAK_INPUT_MIN_OFFSET_ADDR 0x06
#define PIC_LEFT_BREAK_INPUT_MAX_OFFSET_ADDR 0x08

#define PIC_RIGHT_BREAK_INPUT_MIN_OFFSET_ADDR 0x10
#define PIC_RIGHT_BREAK_INPUT_MAX_OFFSET_ADDR 0x12

#define PF_LEFT_BREAK_INPUT_MIN_OFFSET_ADDR 0x06
#define PF_LEFT_BREAK_INPUT_MAX_OFFSET_ADDR 0x08

#define PF_RIGHT_BREAK_INPUT_MIN_OFFSET_ADDR 0x10
#define PF_RIGHT_BREAK_INPUT_MAX_OFFSET_ADDR 0x12



#endif //GLOBAL_H
