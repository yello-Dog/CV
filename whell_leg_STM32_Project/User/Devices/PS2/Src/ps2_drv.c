#include "ps2_drv.h"
#include "chassisL_task.h"
#include "VMC.h"
#include "main.h"
#include "gpio.h"
#include "cmsis_os2.h"
#include "bsp_dwt.h"
#include "pid.h"
#include "INS_task.h"
#include "observe_task.h"

/*
 * Stable PS2 GPIO bit-bang driver.
 *
 * This file keeps the original external API:
 *   PS2_Init()
 *   PS2_Update()
 *   PS2_GetData()
 *   PS2_task()
 *
 * But the communication core is changed from HAL_SPI to GPIO bit-bang.
 * Required CubeMX GPIO labels:
 *   PS2_DI  : input, pull-up recommended
 *   PS2_DO  : output push-pull
 *   PS2_CS  : output push-pull
 *   PS2_CLK : output push-pull
 *
 * Important:
 *   DWT_Init(...) must be called before PS2_Init()/PS2_task().
 */

#define PS2_BIT_SETUP_US          4U
#define PS2_CLK_LOW_US           12U
#define PS2_CLK_HIGH_US          12U
#define PS2_PACKET_GAP_US        50U
#define PS2_CONFIG_GAP_MS        2U
#define PS2_LOST_RESET_COUNT     20U     // 20 * 10ms = about 200ms if PS2_task period is 10ms

#define PS2_DI_READ()            HAL_GPIO_ReadPin(PS2_DI_GPIO_Port, PS2_DI_Pin)

#define PS2_DO_HIGH()            HAL_GPIO_WritePin(PS2_DO_GPIO_Port, PS2_DO_Pin, GPIO_PIN_SET)
#define PS2_DO_LOW()             HAL_GPIO_WritePin(PS2_DO_GPIO_Port, PS2_DO_Pin, GPIO_PIN_RESET)

#define PS2_CLK_HIGH()           HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET)
#define PS2_CLK_LOW()            HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_RESET)

#define PS2_CS_LOW()             HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET)
#define PS2_CS_HIGH()            HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET)

extern chassis_t chassis_move;
extern vmc_leg_t VMC_right;
extern vmc_leg_t VMC_left;

volatile PS2_Data_t ps2_data;

/* Debug buffers */
volatile uint8_t PS2_RawData[9] = {0};
volatile uint8_t PS2_StableData[9] = {
    0xFF, 0x73, 0x5A, 0xFF, 0xFF, 128, 128, 128, 128
};

volatile uint8_t PS2_FrameOK = 0;
volatile uint8_t PS2_AnalogOK = 0;
volatile uint8_t PS2_Mode = 0;

volatile uint32_t PS2_OkCnt = 0;
volatile uint32_t PS2_BadCnt = 0;
volatile uint32_t PS2_LostCnt = 0;

static uint8_t ps2_motor1 = 0x00;
static uint8_t ps2_motor2 = 0x00;

// ============================================================FM看数据用
volatile uint8_t  fm_ps2_connected;
volatile uint8_t  fm_ps2_mode;
volatile uint16_t fm_ps2_buttons;

volatile int16_t  fm_ps2_rx;
volatile int16_t  fm_ps2_ry;
volatile int16_t  fm_ps2_lx;
volatile int16_t  fm_ps2_ly;

volatile uint32_t fm_ps2_ok_cnt;
volatile uint32_t fm_ps2_bad_cnt;
volatile uint32_t fm_ps2_lost_cnt;

uint8_t yaw_pd_position_OR_velo_FLAG = 0;
uint8_t LQR_pd_position_OR_velo_FLAG = 0;
const INS_t* INS_DATA_HANDLELER_PS2;

extern uint8_t OFF_GROUND_FLAG;

static void PS2_FreeMasterMirrorUpdate(void)
{
    fm_ps2_connected = ps2_data.connected;
    fm_ps2_mode      = ps2_data.mode;
    fm_ps2_buttons   = ps2_data.buttons;

    fm_ps2_rx = ps2_data.rx;
    fm_ps2_ry = ps2_data.ry;
    fm_ps2_lx = ps2_data.lx;
    fm_ps2_ly = ps2_data.ly;

    fm_ps2_ok_cnt   = PS2_OkCnt;
    fm_ps2_bad_cnt  = PS2_BadCnt;
    fm_ps2_lost_cnt = PS2_LostCnt;
}

// ============================================================FM看数据结束
static void PS2_BusIdle(void)
{
    PS2_CS_HIGH();
    PS2_CLK_HIGH();
    PS2_DO_HIGH();
}

static bool PS2_IsValidHeader(const uint8_t *buf)
{
    if (buf[2] != 0x5A)
    {
        return false;
    }

    return (buf[1] == 0x41 || buf[1] == 0x73 || buf[1] == 0x79);
}

static bool PS2_IsAnalogMode(uint8_t mode)
{
    return (mode == 0x73 || mode == 0x79);
}

static void PS2_ResetDataToSafe(void)
{
    ps2_data.connected = 0;
    ps2_data.mode = 0x00;
    ps2_data.buttons = 0xFFFF;

    ps2_data.rx_raw = 128;
    ps2_data.ry_raw = 128;
    ps2_data.lx_raw = 128;
    ps2_data.ly_raw = 128;

    ps2_data.rx = 0;
    ps2_data.ry = 0;
    ps2_data.lx = 0;
    ps2_data.ly = 0;

    PS2_StableData[0] = 0xFF;
    PS2_StableData[1] = 0x73;
    PS2_StableData[2] = 0x5A;
    PS2_StableData[3] = 0xFF;
    PS2_StableData[4] = 0xFF;
    PS2_StableData[5] = 128;
    PS2_StableData[6] = 128;
    PS2_StableData[7] = 128;
    PS2_StableData[8] = 128;

    PS2_FrameOK = 0;
    PS2_AnalogOK = 0;
    PS2_Mode = 0x00;
}

/*
 * PS2 is LSB-first.
 * Send one byte and receive one byte through GPIO bit-bang.
 */
static uint8_t PS2_TransferByte(uint8_t tx)
{
    uint8_t rx = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (tx & 0x01U)
        {
            PS2_DO_HIGH();
        }
        else
        {
            PS2_DO_LOW();
        }

        DWT_Delay_us(PS2_BIT_SETUP_US);

        PS2_CLK_LOW();
        DWT_Delay_us(PS2_CLK_LOW_US);

        if (PS2_DI_READ() == GPIO_PIN_SET)
        {
            rx |= (1U << i);
        }

        PS2_CLK_HIGH();
        DWT_Delay_us(PS2_CLK_HIGH_US);

        tx >>= 1;
    }

    PS2_DO_HIGH();
    DWT_Delay_us(PS2_BIT_SETUP_US);

    return rx;
}

static void PS2_ExchangePacket(const uint8_t *tx_buf, uint8_t *rx_buf, uint8_t len)
{
    PS2_BusIdle();
    DWT_Delay_us(PS2_PACKET_GAP_US);

    /*
     * 关键：锁住调度器，防止 PS2 一帧通信中途被其他任务切走。
     * 这里不要用 HAL_Delay，只用 DWT_Delay_us。
     */
    osKernelLock();

    PS2_CS_LOW();
    DWT_Delay_us(PS2_PACKET_GAP_US);

    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t rx = PS2_TransferByte(tx_buf[i]);

        if (rx_buf != NULL)
        {
            rx_buf[i] = rx;
        }

        DWT_Delay_us(10U);
    }

    PS2_CS_HIGH();
    DWT_Delay_us(PS2_PACKET_GAP_US);

    PS2_DO_HIGH();
    PS2_CLK_HIGH();

    osKernelUnlock();
}

static void PS2_SendPacket(const uint8_t *tx_buf, uint8_t len)
{
    PS2_ExchangePacket(tx_buf, NULL, len);
    HAL_Delay(PS2_CONFIG_GAP_MS);
}

static void PS2_ShortPoll(void)
{
    const uint8_t tx[] = {
        0x01, 0x42, 0x00, 0x00, 0x00
    };

    PS2_SendPacket(tx, sizeof(tx));
}

static void PS2_EnterConfig(void)
{
    const uint8_t tx[] = {
        0x01, 0x43, 0x00, 0x01, 0x00,
        0x00, 0x00, 0x00, 0x00
    };

    PS2_SendPacket(tx, sizeof(tx));
}

static void PS2_TurnOnAnalogMode(void)
{
    const uint8_t tx[] = {
        0x01, 0x44, 0x00,
        0x01,       // 0x01: analog/red-light mode
        0x03,       // 0x03: lock analog mode, more stable than 0xEE
        0x00, 0x00, 0x00, 0x00
    };

    PS2_SendPacket(tx, sizeof(tx));
}

static void PS2_VibrationMode(void)
{
    const uint8_t tx[] = {
        0x01, 0x4D, 0x00,
        0x00, 0x01,
        0x00, 0x00, 0x00, 0x00
    };

    PS2_SendPacket(tx, sizeof(tx));
}

static void PS2_ExitConfig(void)
{
    const uint8_t tx[] = {
        0x01, 0x43, 0x00,
        0x00,
        0x5A, 0x5A, 0x5A, 0x5A, 0x5A
    };

    PS2_SendPacket(tx, sizeof(tx));
}

static void PS2_ConfigAnalogMode(void)
{
    for (uint8_t retry = 0; retry < 5; retry++)
    {
        PS2_ShortPoll();
        PS2_ShortPoll();
        PS2_ShortPoll();

        PS2_EnterConfig();
        PS2_TurnOnAnalogMode();
        PS2_VibrationMode();
        PS2_ExitConfig();

        HAL_Delay(20);

        for (uint8_t i = 0; i < 3; i++)
        {
            if (PS2_Update() && PS2_AnalogOK)
            {
                return;
            }

            HAL_Delay(10);
        }
    }
}

uint8_t FM_START = 0;
uint8_t FM_JUMP = 0;
uint8_t FM_STOP = 0;

void PS2_Init(void)
{        
		FM_START = 0;
		FM_JUMP = 0;
		FM_STOP = 0;
		INS_DATA_HANDLELER_PS2 = INS_GetData();
	  chassis_move.v_set = 0.0f;
    chassis_move.x_set = chassis_move.x_filter;
    chassis_move.turn_set = chassis_move.total_yaw;
		chassis_move.leg_set = 0.08f;
		chassis_move.left_leg_set = 0.08f; 
		chassis_move.right_leg_set = 0.08f; 
		chassis_move.roll_set = 0.0f;
    PS2_BusIdle();
    PS2_ResetDataToSafe();

    PS2_OkCnt = 0;
    PS2_BadCnt = 0;
    PS2_LostCnt = 0;

    /*
     * Give receiver time to power up and pair.
     * If your main already delays after enabling 5V, this is still harmless.
     */
    HAL_Delay(50);

    PS2_ConfigAnalogMode();
}

bool PS2_Update(void)
{
    uint8_t rx_buf[9];
    const uint8_t tx_buf[9] = {
        0x01, 0x42, 0x00,	
        ps2_motor1, ps2_motor2,
        0x00, 0x00, 0x00, 0x00
    };

    PS2_ExchangePacket(tx_buf, rx_buf, 9);

    for (uint8_t i = 0; i < 9; i++)
    {
        PS2_RawData[i] = rx_buf[i];
    }

    if (!PS2_IsValidHeader(rx_buf))
    {
        PS2_FrameOK = 0;
        PS2_BadCnt++;
        PS2_LostCnt++;

        /*
         * Short bad-frame bursts do not overwrite ps2_data.
         * This prevents joystick values from jumping to 0.
         */
        if (PS2_LostCnt > PS2_LOST_RESET_COUNT)
        {
            PS2_ResetDataToSafe();
        }

        return false;
    }

    PS2_FrameOK = 1;
    PS2_Mode = rx_buf[1];
    PS2_OkCnt++;
    PS2_LostCnt = 0;

    ps2_data.connected = 1;
    ps2_data.mode = rx_buf[1];
    ps2_data.buttons = ((uint16_t)rx_buf[4] << 8) | rx_buf[3];

    PS2_StableData[0] = rx_buf[0];
    PS2_StableData[1] = rx_buf[1];
    PS2_StableData[2] = rx_buf[2];
    PS2_StableData[3] = rx_buf[3];
    PS2_StableData[4] = rx_buf[4];

    if (PS2_IsAnalogMode(rx_buf[1]))
    {
        PS2_AnalogOK = 1;

        PS2_StableData[5] = rx_buf[5];
        PS2_StableData[6] = rx_buf[6];
        PS2_StableData[7] = rx_buf[7];
        PS2_StableData[8] = rx_buf[8];

        ps2_data.rx_raw = rx_buf[5];
        ps2_data.ry_raw = rx_buf[6];
        ps2_data.lx_raw = rx_buf[7];
        ps2_data.ly_raw = rx_buf[8];

        ps2_data.rx = (int16_t)ps2_data.rx_raw - 128;
				if(ps2_data.rx == -1)
				{
					ps2_data.rx = 0;
				}
        ps2_data.ry = -(int16_t)ps2_data.ry_raw + 128;
        ps2_data.lx = (int16_t)ps2_data.lx_raw - 128;
				if(ps2_data.lx == -1)
				{
					ps2_data.lx = 0;
				}
        ps2_data.ly = -(int16_t)ps2_data.ly_raw + 128;
    }
    else
    {
        /*
         * Valid digital packet.
         * Buttons are valid, but analog bytes are meaningless.
         * Keep last analog values, do not force them to 0.
         */
        PS2_AnalogOK = 0;
    }

    return true;
}

void reset_settings(chassis_t *chassis)
{
		chassis->roll_set = 0.0f;
		chassis->v_set = 0.0f;
		chassis->x_set = 0.0f;
		chassis->x_filter = 0.0f;
		chassis->L_x_filter = 0.0f;
		chassis->R_x_filter = 0.0f;
		chassis->L_v_filter = 0.0f;
		chassis->R_v_filter = 0.0f;
		chassis->turn_set = INS_DATA_HANDLELER_PS2->YawTotalAngle;
		chassis->leg_set = 0.08f;
}

float roll_err = 0;
extern uint8_t self_lock_flag;
void PS2_data_process(volatile PS2_Data_t *data, chassis_t *chassis, float dt)
{
    if (data == NULL || chassis == NULL)
    {
        return;
    }

    /*
     * If receiver has been lost for too long, ps2_data is already reset to neutral.
     * This branch is only an extra safety gate.
     */
    if (data->connected == 0)
    {
        chassis->v_set = 0.0f;
        chassis->x_set = chassis->x_filter;
        chassis->turn_set = chassis->total_yaw;
        chassis->leg_set = 0.08f;
        chassis->roll_set = 0.0f;
        return;
    }

    if (PS2_IS_PRESSED(data, PS2_BTN_START)||FM_START == 1)
    {
				FM_START = 0;
        // 手柄上的Start按键被按下
        chassis->start_flag = 1;
				chassis->recover_flag = 0;
			  self_lock_flag = 0;
				reset_settings(chassis);
    }
		if (PS2_IS_PRESSED(data, PS2_BTN_CIRCLE))
    {
        // 手柄上的Start按键被按下
        chassis->start_flag = 0;
    }

    if (chassis->start_flag == 1)
    {
				chassis->v_set = ((float)data->ry) * (0.008f); // 往前大于0
        // 启动
				if(data->ry != 0 )
				{   
           chassis->x_set = chassis->x_set + chassis->v_set * dt;
					LQR_pd_position_OR_velo_FLAG = 1;
				}
				else
				{
					if(LQR_pd_position_OR_velo_FLAG == 1)
					{
						LQR_pd_position_OR_velo_FLAG = 0;
						chassis->x_set = chassis_move.x_filter;
					}
				}

			  // 速度模式
				if(data->rx != 0 )
				{
					chassis->turn_set=chassis->turn_set-data->rx*(0.0005f);//往右大于0
					yaw_pd_position_OR_velo_FLAG = 1;
				}else // 位置模式
				{	
					if(yaw_pd_position_OR_velo_FLAG  == 1)
					{
						// 单次进入
							yaw_pd_position_OR_velo_FLAG = 0;
							chassis->turn_set = INS_DATA_HANDLELER_PS2->YawTotalAngle;
					}
				}
				
				chassis->roll_set = ((float)data->lx)/400.f;
				roll_err = INS_DATA_HANDLELER_PS2->Roll - chassis->roll_set;
				if(chassis_move.jump_flag != 1)
				{
					// 控制腿长
					chassis->left_leg_set += ((float)data->ly) * 0.000008f;
					chassis->right_leg_set += ((float)data->ly) * 0.000008f;
					// 控制roll
					if(OFF_GROUND_FLAG == 0)
					{
						chassis->left_leg_set += (roll_err) * 0.005f;
						chassis->right_leg_set -= (roll_err) * 0.005f;
						mySaturate(&chassis->right_leg_set, 0.065f, 0.18f); // 腿长限幅在0.065m到0.18m之间
						mySaturate(&chassis->left_leg_set, 0.065f, 0.18f); // 腿长限幅在0.065m到0.18m之间
					}
					else
					{
						float mid_leg_length = (chassis->left_leg_set + chassis->right_leg_set)/2.0f;
						chassis->left_leg_set = mid_leg_length;
						chassis->right_leg_set = mid_leg_length;
					}
				}
				
        if (data->lx != 0 && data->ly != 0)
        {
            // 遥控器控制腿长在变化
            VMC_right.leg_flag = 1;
            VMC_left.leg_flag = 1;
        }else
				{
            VMC_right.leg_flag = 0;
            VMC_left.leg_flag = 0;
				}

    }
    else
    {
        // 关闭
        chassis->v_set = 0.0f;
        //chassis->x_set = chassis->x_filter;
        chassis->turn_set = INS_DATA_HANDLELER_PS2->YawTotalAngle;
        chassis->leg_set = 0.08f;
        chassis->roll_set = 0.0f;
    }

    if (PS2_IS_PRESSED(data, PS2_BTN_CIRCLE)||FM_STOP == 1)
    {
			FM_STOP = 0;
			reset_settings(chassis);
    }
		if (PS2_IS_PRESSED(data, PS2_BTN_SQUARE))
    {
			float mid_leg_length = (chassis->left_leg_set + chassis->right_leg_set)/2.0f;
			chassis->left_leg_set = mid_leg_length;
			chassis->right_leg_set = mid_leg_length;
    }
		if (PS2_IS_PRESSED(data, PS2_BTN_TRIANGLE)||FM_JUMP == 1)
    {
			FM_JUMP = 0;
			chassis->attampt_jump_flag = 1;
    }
}

uint8_t PS2_TIME = 10;
uint32_t check_clock = 0;
void PS2_task(void)
{
    PS2_Init();

    while (1)
    {
        PS2_Update();

        PS2_data_process(&ps2_data, &chassis_move, (float)PS2_TIME / 1000.0f);
				check_clock++;
				PS2_FreeMasterMirrorUpdate();
        // 读取周期10ms
        osDelay(PS2_TIME);
    }
}

/*
 * 查看PS2数据的时候开一个PS2数据类型实例，然后传指针进去。
 * Note: ps2_data is volatile internally, but the public API remains compatible.
 */
const PS2_Data_t *PS2_GetData(void)
{
    return (const PS2_Data_t *)&ps2_data;
}
