//
// Created by dextercai on 25-8-6.
//
#include "global.h"


MCP3208 mcp3208;


SlipAvgFilter_t *slipAvgFilter_Ch;

HID_Report14_t report14;

uint16_t led_delay_ms = 255;

QueueHandle_t hid_rx_queue;


uint16_t rudder_input_min_clib = 0;
uint16_t rudder_input_max_clib = (1 << 12) - 1;

uint16_t pic_left_break_input_min_clib = 0;
uint16_t pic_left_break_input_max_clib = (1 << 12) - 1;

uint16_t pic_right_break_input_min_clib = 0;
uint16_t pic_right_break_input_max_clib = (1 << 12) - 1;

uint16_t pf_left_break_input_min_clib = 0;
uint16_t pf_left_break_input_max_clib = (1 << 12) - 1;

uint16_t pf_right_break_input_min_clib = 0;
uint16_t pf_right_break_input_max_clib = (1 << 12) - 1;