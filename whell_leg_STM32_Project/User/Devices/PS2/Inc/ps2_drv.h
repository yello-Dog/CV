#ifndef __PSTWO_TASK_H
#define __PSTWO_TASK_H

#include "main.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint8_t connected;     // 1: connected, 0: disconnected
    uint8_t mode;          // 0x41 digital, 0x73/0x79 analog, etc.

    uint16_t buttons;      // low active

    uint8_t rx_raw;
    uint8_t ry_raw;
    uint8_t lx_raw;
    uint8_t ly_raw;

    int16_t rx;
    int16_t ry;
    int16_t lx;
    int16_t ly;

} PS2_Data_t;

/* button bit definitions, low active */
#define PS2_BTN_SELECT     (1U << 0)
#define PS2_BTN_L3         (1U << 1)
#define PS2_BTN_R3         (1U << 2)
#define PS2_BTN_START      (1U << 3)
#define PS2_BTN_UP         (1U << 4)
#define PS2_BTN_RIGHT      (1U << 5)
#define PS2_BTN_DOWN       (1U << 6)
#define PS2_BTN_LEFT       (1U << 7)

#define PS2_BTN_L2         (1U << 8)
#define PS2_BTN_R2         (1U << 9)
#define PS2_BTN_L1         (1U << 10)
#define PS2_BTN_R1         (1U << 11)
#define PS2_BTN_TRIANGLE   (1U << 12)
#define PS2_BTN_CIRCLE     (1U << 13)
#define PS2_BTN_CROSS      (1U << 14)
#define PS2_BTN_SQUARE     (1U << 15)

#define PS2_IS_PRESSED(data_ptr, button)   (((data_ptr)->buttons & (button)) == 0U)

/*
 * Debug variables:
 * PS2_RawData[]    : latest raw packet, may contain bad frames.
 * PS2_StableData[] : last valid packet, bad frames will not overwrite it.
 */
extern volatile PS2_Data_t ps2_data;

extern volatile uint8_t PS2_RawData[9];
extern volatile uint8_t PS2_StableData[9];

extern volatile uint8_t PS2_FrameOK;
extern volatile uint8_t PS2_AnalogOK;
extern volatile uint8_t PS2_Mode;

extern volatile uint32_t PS2_OkCnt;
extern volatile uint32_t PS2_BadCnt;
extern volatile uint32_t PS2_LostCnt;

void PS2_Init(void);
bool PS2_Update(void);
const PS2_Data_t *PS2_GetData(void);
void PS2_task(void);

#endif
