/******************************************************************************
 * @file : main.c
 * @brief : The Last of EE2028 - with OLED dashboard + IR target sensor
 * (c) EE2028 Teaching Team
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "stdio.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h" // LED header
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"

#include <stdarg.h>
#include <math.h>
#include <string.h>

/* ======================== ENHANCEMENT SWITCHES ========================
 * Set any switch to 0 to fall back to the core assignment behaviour.   */
#define ENABLE_OLED 1 // Grove 128x64 OLED on the Base Shield I2C port (= D14 SDA / D15 SCL)
#define ENABLE_IR 1 // Grove IR reflective sensor on the Base Shield D2 port
#define ENABLE_GYRO_CAL 1 // measure gyro bias at power-on (keep the board still)
#define ENABLE_GREYMECHA 1 // GreyMecha Command Centre link: UART4, Arduino D1 = PA0 (TX), D0 = PA1 (RX)

#define GM_USB_BRIDGE 1 // 1 = no wires: also send "$" lines and accept commands on USART1 (laptop bridge)
#define GM_PACKET_MS 200U // status line to the Command Centre 5 times per second
#define GM_ATTITUDE_MS 25U // attitude line for the Command Centre's 3D view, 40 times per second
#define MAX_HP 5U // drone health in the zombie game: small zombie -1, big zombie -2, 0 = attacked

#define OLED_IS_SH1106 0 // set to 1 if the picture is shifted or has a garbage edge (1.3" screens)
#define OLED_RENDER_MS 200U // redraw the screen 5 times per second

#define IR_ACTIVE_LOW 1 // most IR modules pull OUT low when an object is close
#define IR_SAMPLE_MS 50U
#define IR_GPIO_PORT GPIOD
#define IR_GPIO_PIN GPIO_PIN_14 // Arduino D2 = PD14 on B-L4S5I-IOT01A

// Temporary detection thresholds for physical testing
#define GYRO_TRIGGER_DPS 20.0f
#define INVERTED_Z_MG -700 // approximately 135 degree from upright
#define UPRIGHT_Z_MG 700
#define FLIP_WINDOW_MS 4000U
#define INVERT_CONFIRM_MS 1000U
#define BUTTON_DEBOUNCE_MS 50U // unwanted electrical transistions due to mechanics of button
#define DOUBLE_PRESS_MS 500U // assignment specified
#define RESCUE_TIMEOUT_MS 10000U // attack must terminate program ( last of ee2028 assignment specified)
#define ATTACK_MESSAGE_MS 1000U
#define SAMPLE_PERIOD_MS 50U
#define TELEMETRY_PERIOD_MS 1000U

// Configurable telemetry thresholds (provisional as assignment left it up to us to decide
#define TEMP_MIN_C 0.0f
#define TEMP_MAX_C 50.0f // fur tests to see if warning works make it 20.0f otherwise 50.0f

#define PRESS_MIN_HPA 950.0f
#define PRESS_MAX_HPA 1050.0f

#define HUM_MIN_PCT 20.0f
#define HUM_MAX_PCT 90.0f

#define ACCEL_MAX_MS2 19.6f
#define GYRO_MAX_DPS 250.0f

#define MAG_MIN_UT 15.0f
#define MAG_MAX_UT 300.0f // board reads ~200-220 uT at rest: Earth's ~40 uT plus the board's own magnetic offset; 300 flags a magnet brought close

typedef enum
{ // drone states
    STANDBY,
    BATTLE,
    ATTACKED,
    TERMINATED
} DroneState;

// Values shown on the OLED (copied from the main loop)
typedef struct
{
    DroneState state;
    float temp, press, hum, accel, gyro, mag, alt;
    uint8_t badMask; // bit0 T, bit1 P, bit2 H, bit3 A, bit4 G, bit5 M
    uint8_t charge, hits, armed, ir, hp;
    uint32_t attackStartedAt;
} UiData;

static UiData ui;
static char uiEvent[22] = ""; // short message shown on the OLED for 1.5 s
static uint32_t uiEventAt = 0;

// Gyro bias in deg/s. Defaults are from earlier stationary tests;
// replaced by the power-on calibration when ENABLE_GYRO_CAL is 1.
static float gyroBias[3] = {0.38f, -1.30f, 0.0f};

// UART handle shared by all serial output
static UART_HandleTypeDef huart1;

// Print formatted text over the real UART connection
static void uart_printf(const char *format, ...)
{
    char buffer[256];

    va_list args;
    va_start(args, format);

    int length = vsnprintf(buffer, sizeof(buffer), format, args);

    va_end(args);

    if (length <= 0)
        return;

    // Never transmit beyond the buffer's actual capacity
    if (length >= (int)sizeof(buffer))
        length = sizeof(buffer) - 1;

    HAL_UART_Transmit(&huart1,
                      (uint8_t *)buffer,
                      (uint16_t)length,
                      100);
}

// All serial output goes through uart_printf() (HAL_UART_Transmit), never printf()

/* ===================== GREYMECHA COMMAND CENTRE LINK ===================== */
#if ENABLE_GREYMECHA
// Second UART, only for the Command Centre. USART1 stays the readable PC log.
static UART_HandleTypeDef huart4;

static void gm_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();

    // PA0 = UART4 TX (Arduino D1), PA1 = UART4 RX (Arduino D0)
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP; // RX idles high if the wire falls off
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &g);

    huart4.Instance = UART4;
    huart4.Init.BaudRate = 115200;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart4) != HAL_OK)
        uart_printf("Command Centre link: UART4 init failed\r\n");
    else
        uart_printf("Command Centre link on D0/D1 (UART4, 115200)\r\n");
#if GM_USB_BRIDGE
    uart_printf("USB bridge mode: ON ($ lines + D/S/P commands on this port too)\r\n");
#else
    uart_printf("USB bridge mode: OFF (wires only)\r\n");
#endif
}

// Send one machine-readable line to the Command Centre.
// Short timeout, so a missing Command Centre never stalls the drone.
static void gm_send(const char *format, ...)
{
    char buffer[128];

    va_list args;
    va_start(args, format);
    int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length <= 0)
        return;
    if (length >= (int)sizeof(buffer))
        length = sizeof(buffer) - 1;

    HAL_UART_Transmit(&huart4, (uint8_t *)buffer, (uint16_t)length, 20);
#if GM_USB_BRIDGE
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)length, 20);
#endif
}

// Angle difference folded into -180..+180 degrees (so 179 and -179 are 2 apart)
static float wrap180(float a)
{
    while (a > 180.0f)
        a -= 360.0f;
    while (a < -180.0f)
        a += 360.0f;
    return a;
}

// Non-blocking read of one byte from a UART: returns -1 if nothing arrived
static int uart_read_byte(UART_HandleTypeDef *h)
{
    // An overrun (byte arrived while we were busy) blocks reception: clear it
    if (__HAL_UART_GET_FLAG(h, UART_FLAG_ORE))
        __HAL_UART_CLEAR_FLAG(h, UART_CLEAR_OREF);

    if (__HAL_UART_GET_FLAG(h, UART_FLAG_RXNE))
        return (int)(h->Instance->RDR & 0xFFU);

    return -1;
}

// One command byte from the Command Centre, or -1
static int gm_read(void)
{
    int c = uart_read_byte(&huart4);
#if GM_USB_BRIDGE
    if (c < 0)
        c = uart_read_byte(&huart1);
#endif
    return c;
}
#endif // ENABLE_GREYMECHA

// Shared between the button interrupt and main loop.
// volatile prevents the compiler from assuming these never change.
static volatile uint8_t waitingSecondPress = 0;
static volatile uint8_t singlePressPending = 0;
static volatile uint8_t doublePressPending = 0;

static volatile uint32_t firstPressAt = 0;
static volatile uint32_t lastButtonEdgeAt = 0;
static volatile uint32_t secondPressAt = 0;

/* ============================ SMALL HELPERS ============================ */

// Remember a short event for the OLED's bottom line
static void ui_event(const char *text, uint32_t now)
{
    strncpy(uiEvent, text, sizeof(uiEvent) - 1);
    uiEvent[sizeof(uiEvent) - 1] = '\0';
    uiEventAt = now;
}

// International Standard Atmosphere: height change from a reference pressure.
// About 0.12 hPa per metre near sea level.
static float pressure_to_height(float p, float p0)
{
    if (p <= 0.0f || p0 <= 0.0f)
        return 0.0f;
    return 44330.0f * (1.0f - powf(p / p0, 0.190295f));
}

// 1 once the power-on gyro calibration has finished (or is switched off)
static uint8_t gyroCalDone = 0;

#if ENABLE_GYRO_CAL
// Gyro bias calibration without blocking: called every main-loop pass, it takes
// one sample every 20 ms, starting 200 ms after power-on (gyro settling time).
// After 40 samples (~0.8 s) it averages them. If the board moved, keep the defaults.
#define GYRO_CAL_SAMPLES 40
#define GYRO_CAL_SETTLE_MS 200U
#define GYRO_CAL_STEP_MS 20U

static void gyro_cal_step(uint32_t now, uint32_t bootAt)
{
    static int n = 0;
    static uint32_t lastSampleAt = 0;
    static float sum[3] = {0}, mn[3] = {0}, mx[3] = {0};

    if (gyroCalDone || (uint32_t)(now - bootAt) < GYRO_CAL_SETTLE_MS)
        return;
    if (n > 0 && (uint32_t)(now - lastSampleAt) < GYRO_CAL_STEP_MS)
        return;
    lastSampleAt = now;

    float g[3] = {0};
    BSP_GYRO_GetXYZ(g);

    for (int k = 0; k < 3; k++)
    {
        float v = g[k] / 1000.0f; // mdps -> deg/s
        sum[k] += v;
        if (n == 0 || v < mn[k])
            mn[k] = v;
        if (n == 0 || v > mx[k])
            mx[k] = v;
    }
    n++;
    if (n < GYRO_CAL_SAMPLES)
        return;

    gyroCalDone = 1;
    for (int k = 0; k < 3; k++)
    {
        // Spread > 3 deg/s means the board was moving; > 10 deg/s average is not a bias
        if ((mx[k] - mn[k]) > 3.0f || fabsf(sum[k] / GYRO_CAL_SAMPLES) > 10.0f)
        {
            uart_printf("Gyro calibration skipped (board moved) - using default bias\r\n");
            return;
        }
    }

    for (int k = 0; k < 3; k++)
        gyroBias[k] = sum[k] / GYRO_CAL_SAMPLES;

    uart_printf("Gyro bias: X=%.2f Y=%.2f Z=%.2f deg/s\r\n",
                gyroBias[0], gyroBias[1], gyroBias[2]);
}
#endif

/* ============================== IR SENSOR ============================== */
#if ENABLE_IR
static uint8_t ir_read(void)
{
    GPIO_PinState s = HAL_GPIO_ReadPin(IR_GPIO_PORT, IR_GPIO_PIN);
#if IR_ACTIVE_LOW
    return (s == GPIO_PIN_RESET) ? 1U : 0U;
#else
    return (s == GPIO_PIN_SET) ? 1U : 0U;
#endif
}

static void ir_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();

    g.Pin = IR_GPIO_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP; // reads "no object" if the wire falls off
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(IR_GPIO_PORT, &g);

    uart_printf("IR sensor on D2: %s\r\n", ir_read() ? "OBJECT" : "CLEAR");
}
#endif

/* ============================ OLED (SSD1306) ============================ */
#if ENABLE_OLED
static I2C_HandleTypeDef hi2c1;
static uint8_t oledOk = 0;
static uint8_t oledAddr = 0; // 8-bit address used by HAL
static uint8_t oledErrors = 0;
static uint8_t oledInverted = 0;
static uint8_t oledNextPage = 0;
static uint8_t oledBuf[8][128]; // picture we want (8 pages x 128 columns)
static uint8_t oledSent[8][128]; // picture the screen currently shows

// 5x7 font, ASCII 0x20-0x5F (upper case only). Each byte is one column, bit 0 = top.
static const uint8_t font5x7[64][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // backslash
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // ^
    {0x40, 0x40, 0x40, 0x40, 0x40} // _
};

// After 3 failed transfers, give up on the display so the drone keeps working
static void oled_fail(void)
{
    if (++oledErrors >= 3U && oledOk)
    {
        oledOk = 0;
        uart_printf("OLED not responding - continuing without display\r\n");
    }
}

static void oled_cmds(const uint8_t *cmds, uint16_t n)
{
    if (!oledOk)
        return;
    // Control byte 0x00 = the following bytes are commands
    if (HAL_I2C_Mem_Write(&hi2c1, oledAddr, 0x00, I2C_MEMADD_SIZE_8BIT,
                          (uint8_t *)cmds, n, 20) != HAL_OK)
        oled_fail();
}

static void oled_cmd1(uint8_t c)
{
    oled_cmds(&c, 1);
}

// Send one 128-byte page (8 pixel rows) to the screen, ~12 ms at 100 kHz
static void oled_send_page(uint8_t page)
{
    uint8_t col = OLED_IS_SH1106 ? 2U : 0U; // SH1106 RAM is 132 wide, centred
    uint8_t addr[3] = {(uint8_t)(0xB0U | page),
                       (uint8_t)(col & 0x0FU),
                       (uint8_t)(0x10U | (col >> 4))};

    oled_cmds(addr, 3);
    if (!oledOk)
        return;

    // Control byte 0x40 = the following bytes are pixel data
    if (HAL_I2C_Mem_Write(&hi2c1, oledAddr, 0x40, I2C_MEMADD_SIZE_8BIT,
                          oledBuf[page], 128, 50) == HAL_OK)
        memcpy(oledSent[page], oledBuf[page], 128);
    else
        oled_fail();
}

// Send at most ONE changed page per main-loop pass, so the loop is
// never blocked for long (button, LED and flip timing stay accurate)
static void oled_flush_step(void)
{
    for (uint8_t i = 0; i < 8U; i++)
    {
        uint8_t p = (uint8_t)((oledNextPage + i) & 7U);
        if (memcmp(oledBuf[p], oledSent[p], 128) != 0)
        {
            oled_send_page(p);
            oledNextPage = (uint8_t)((p + 1U) & 7U);
            return;
        }
    }
}

static void oled_flush_all(void)
{
    for (uint8_t p = 0; p < 8U && oledOk; p++)
        oled_send_page(p);
}

static void oled_set_invert(uint8_t inv)
{
    if (!oledOk || inv == oledInverted)
        return;
    oled_cmd1(inv ? 0xA7 : 0xA6);
    oledInverted = inv;
}

/* ---- drawing into the RAM picture ---- */
static void oled_clear(void)
{
    memset(oledBuf, 0, sizeof(oledBuf));
}

static void oled_pixel(int x, int y, uint8_t on)
{
    if (x < 0 || x > 127 || y < 0 || y > 63)
        return;
    if (on)
        oledBuf[y >> 3][x] |= (uint8_t)(1U << (y & 7));
    else
        oledBuf[y >> 3][x] &= (uint8_t) ~(1U << (y & 7));
}

static void oled_fill(int x, int y, int w, int h, uint8_t on)
{
    for (int i = x; i < x + w; i++)
        for (int j = y; j < y + h; j++)
            oled_pixel(i, j, on);
}

static void oled_char(int x, int y, char c, int s, uint8_t on)
{
    if (c >= 'a' && c <= 'z')
        c = (char)(c - 32); // font is upper case only
    if (c < 0x20 || c > 0x5F)
        c = '?';

    const uint8_t *glyph = font5x7[c - 0x20];

    for (int col = 0; col < 5; col++)
        for (int row = 0; row < 7; row++)
            if (glyph[col] & (1U << row))
                oled_fill(x + col * s, y + row * s, s, s, on);
}

static int oled_width(const char *t, int s)
{
    int n = (int)strlen(t);
    return n ? n * 6 * s - s : 0;
}

static void oled_text(int x, int y, const char *t, int s, uint8_t on)
{
    while (*t)
    {
        oled_char(x, y, *t++, s, on);
        x += 6 * s;
    }
}

static void oled_text_center(int y, const char *t, int s)
{
    oled_text((128 - oled_width(t, s)) / 2, y, t, s, 1);
}

// White title bar with black text, optional tag on the right
static void oled_header(const char *left, const char *right)
{
    oled_fill(0, 0, 128, 9, 1);
    oled_text(2, 1, left, 1, 0);
    if (right)
        oled_text(126 - oled_width(right, 1), 1, right, 1, 0);
}

static void oled_init(void)
{
    static const uint8_t initSeq[] = {
        0xAE, // display off
        0xD5, 0x80, // clock divide
        0xA8, 0x3F, // 64 rows
        0xD3, 0x00, // no vertical offset
        0x40, // start line 0
        0x8D, 0x14, // charge pump on (SSD1306)
        0x20, 0x02, // page addressing mode
        0xA1, // mirror columns (normal orientation)
        0xC8, // scan rows top to bottom
        0xDA, 0x12, // COM pins for 128x64
        0x81, 0xCF, // contrast
        0xD9, 0xF1, // pre-charge
        0xDB, 0x40, // VCOM level
        0xA4, // show RAM contents
        0xA6, // normal (not inverted)
        0xAF // display on
    };
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CONFIG(RCC_I2C1CLKSOURCE_PCLK1);
    __HAL_RCC_I2C1_CLK_ENABLE();

    // PB8 = Arduino D15 (SCL), PB9 = Arduino D14 (SDA)
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode = GPIO_MODE_AF_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &g);

    // ~100 kHz at any clock: prescale the I2C clock to ~4 MHz (250 ns ticks),
    // then SCL low = 20 ticks (5 us), SCL high = 16 ticks (4 us)
    uint32_t presc = (HAL_RCC_GetPCLK1Freq() + 3999999U) / 4000000U;
    if (presc < 1U)
        presc = 1U;
    if (presc > 16U)
        presc = 16U;

    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = ((presc - 1U) << 28) | (0x4U << 20) | (0x2U << 16) |
                        (0x0FU << 8) | 0x13U;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        uart_printf("OLED: I2C init failed - continuing without display\r\n");
        return;
    }

    // Most 128x64 modules answer at 0x3C, some at 0x3D
    if (HAL_I2C_IsDeviceReady(&hi2c1, 0x3C << 1, 3, 20) == HAL_OK)
        oledAddr = 0x3C << 1;
    else if (HAL_I2C_IsDeviceReady(&hi2c1, 0x3D << 1, 3, 20) == HAL_OK)
        oledAddr = 0x3D << 1;
    else
    {
        uart_printf("OLED: not found on D14/D15 - continuing without display\r\n");
        return;
    }

    oledOk = 1;
    oled_cmds(initSeq, sizeof(initSeq));
    if (!oledOk)
        return;

    memset(oledSent, 0xAA, sizeof(oledSent)); // force the first full refresh
    oled_clear();
    oled_flush_all();

    uart_printf("OLED found at 0x%02X\r\n", (unsigned int)(oledAddr >> 1));
}

static void oled_show_boot(void)
{
    if (!oledOk)
        return;
    oled_clear();
    oled_text_center(8, "EE2028", 2);
    oled_text_center(30, "DRONE BOOTING", 1);
    oled_text_center(44, "KEEP BOARD STILL", 1);
    oled_flush_all();
}

// Build the picture for the current state (RAM only, nothing sent yet)
static void oled_render(uint32_t now)
{
    char line[24];
#if ENABLE_IR
    const char *irTag = ui.ir ? "IR:OBJ" : "IR:--";
#else
    const char *irTag = NULL;
#endif

    oled_clear();

    if (ui.state == STANDBY)
    {
        oled_header("STANDBY", irTag);

        snprintf(line, sizeof(line), "T %.1fC  H %.1f%%", ui.temp, ui.hum);
        oled_text(0, 11, line, 1, 1);

        snprintf(line, sizeof(line), "P %.1f HPA", ui.press);
        oled_text(0, 20, line, 1, 1);

        snprintf(line, sizeof(line), "HEIGHT %+.2f M", ui.alt);
        oled_text(0, 29, line, 1, 1);

        snprintf(line, sizeof(line), "A %.2f M/S2", ui.accel);
        oled_text(0, 38, line, 1, 1);

        snprintf(line, sizeof(line), "G %.1f DPS  M %.0f UT", ui.gyro, ui.mag);
        oled_text(0, 47, line, 1, 1);

        if (ui.badMask == 0U)
        {
            oled_text(0, 56, "ALL SENSORS OK", 1, 1);
        }
        else
        {
            // Highlighted bar listing which sensors are out of range
            static const char tags[6] = {'T', 'P', 'H', 'A', 'G', 'M'};
            char warn[24] = "WARNING:";
            size_t n = strlen(warn);

            for (int i = 0; i < 6; i++)
            {
                if (ui.badMask & (1U << i))
                {
                    warn[n++] = ' ';
                    warn[n++] = tags[i];
                }
            }
            warn[n] = '\0';

            oled_fill(0, 55, 128, 9, 1);
            oled_text(2, 56, warn, 1, 0);
        }
    }
    else if (ui.state == BATTLE)
    {
        oled_header("BATTLE MODE", irTag);

        snprintf(line, sizeof(line), "LASER CHARGE %u%%", (unsigned int)ui.charge);
        oled_text(0, 12, line, 1, 1);

        // Charge bar outline
        oled_fill(0, 22, 128, 1, 1);
        oled_fill(0, 30, 128, 1, 1);
        oled_fill(0, 22, 1, 9, 1);
        oled_fill(127, 22, 1, 9, 1);

        // Bar fill (0-100%) and a tick at the 50% firing level
        int pct = (ui.charge > 100U) ? 100 : (int)ui.charge;
        oled_fill(2, 24, pct * 124 / 100, 5, 1);
        oled_fill(64, 32, 1, 3, 1);

#if ENABLE_GREYMECHA
        snprintf(line, sizeof(line), "HITS %u   HP %u/%u", (unsigned int)ui.hits,
                 (unsigned int)ui.hp, (unsigned int)MAX_HP);
        oled_text(0, 37, line, 1, 1);
#elif ENABLE_IR
        snprintf(line, sizeof(line), "HITS %u", (unsigned int)ui.hits);
        oled_text(0, 37, line, 1, 1);
#endif
#if ENABLE_IR
        oled_text(0, 46, ui.ir ? "ZOMBIE IN RANGE!" : "NO TARGET", 1, 1);
#endif

        if (uiEvent[0] && (uint32_t)(now - uiEventAt) < 1500U)
        {
            oled_fill(0, 55, 128, 9, 1);
            oled_text(2, 56, uiEvent, 1, 0);
        }
        else if (!ui.armed)
        {
            oled_text(0, 56, "TURN UPRIGHT TO ARM", 1, 1);
        }
        else
        {
            oled_text(0, 56, "FLIP DETECTOR ARMED", 1, 1);
        }
    }
    else if (ui.state == ATTACKED)
    {
        oled_header("LAST OF EE2028", NULL);

        uint32_t elapsed = (uint32_t)(now - ui.attackStartedAt);
        uint32_t remaining = (elapsed >= RESCUE_TIMEOUT_MS) ? 0U : (RESCUE_TIMEOUT_MS - elapsed + 999U) / 1000U;

        snprintf(line, sizeof(line), "%lu", (unsigned long)remaining);
        oled_text_center(14, line, 4); // big countdown digits

        oled_text_center(46, "DRONE ATTACKED!", 1);
        oled_text_center(56, "2X PRESS = RESCUE", 1);
    }
}

// Called every main-loop pass
static void oled_update(uint32_t now)
{
    static uint32_t lastRenderAt = 0;
    static DroneState lastState = TERMINATED;

    if (!oledOk)
        return;
    if (!gyroCalDone)
        return; // keep "KEEP BOARD STILL" on screen while the gyro calibrates

    // Redraw 5x per second, or at once when the mode changes
    if (ui.state != lastState ||
        (uint32_t)(now - lastRenderAt) >= OLED_RENDER_MS)
    {
        lastState = ui.state;
        lastRenderAt = now;
        oled_render(now);
    }

    // Under attack the whole screen flashes in step with the 2 Hz LED
    uint8_t wantInvert = (ui.state == ATTACKED) &&
                         ((((uint32_t)(now - ui.attackStartedAt) / 250U) & 1U) != 0U);
    oled_set_invert(wantInvert);

    oled_flush_step();
}

// Last screen before termination: "SIGNAL LOST" stays on screen
static void oled_show_terminated(void)
{
    if (!oledOk)
        return;

    oled_set_invert(0);
    oled_clear();
    oled_text_center(12, "SIGNAL", 2);
    oled_text_center(34, "LOST", 2);
    oled_flush_all(); // stays on screen: the program halts right after this
}
#endif // ENABLE_OLED

int main(void)
{

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    // Configure USART1: 115200 baud, 8 data bits, no parity, 1 stop bit
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    // BSP connects COM1 to USART1 (PB6 TX, PB7 RX)
    BSP_COM_Init(COM1, &huart1);

    uart_printf("EE2028 UART ONLINE\r\n");

#if ENABLE_GREYMECHA
    gm_init();
    gm_send("$HELLO\n");
#endif


    /* Peripheral initializations using BSP functions */
    BSP_ACCELERO_Init();
    BSP_TSENSOR_Init();
    if (BSP_GYRO_Init() != GYRO_OK)
    {
        uart_printf("Gyroscope initialization failed!\r\n");
    }

    // Initialize humidity sensor (HTS221)
    if (BSP_HSENSOR_Init() != HSENSOR_OK)
    {
        uart_printf("ERROR: Humidity sensor initialization failed\r\n");
        while (1)
        {
        }
    }

    // Initialize pressure sensor (LPS22HB)
    if (BSP_PSENSOR_Init() != PSENSOR_OK)
    {
        uart_printf("ERROR: Pressure sensor initialization failed\r\n");
        while (1)
        {
        }
    }

    // Initialize magnetometer (LIS3MDL)
    if (BSP_MAGNETO_Init() != MAGNETO_OK)
    {
        uart_printf("ERROR: Magnetometer initialization failed\r\n");
        while (1)
        {
        }
    }

#if !ENABLE_GYRO_CAL
    gyroCalDone = 1; // calibration switched off: use the default bias
#endif

    // Reference pressure: "height" is measured from where the drone powered on.
    // p0 = 0 means "not set yet": the first STANDBY telemetry reading becomes p0.
    float p0 = 0.0f;
    float pFiltered = 0.0f;
    uint32_t bootAt = HAL_GetTick();
    (void)bootAt; // only used by the gyro calibration and OLED start-up
#if ENABLE_OLED
    uint8_t oledStarted = 0;
#endif

#if ENABLE_IR
    ir_init();
    uint32_t lastIrReadAt = HAL_GetTick();
    uint8_t irDetected = ir_read();
    uint8_t irPrevRaw = irDetected;
#endif

    BSP_LED_Init(LED2); // Configure PB14 as LED output
    BSP_LED_On(LED2); // Start with LED illuminated
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI); // Interrupt for button

    DroneState currentState = STANDBY; // starting at state Standby

    // Virtual laser energy starts empty.
    // Each valid single press adds 30 percentage points.
    uint8_t laserChargePercent = 0;
    uint8_t hitCount = 0;
#if ENABLE_GREYMECHA
    uint8_t healthPoints = MAX_HP; // zombie game health
#endif
    (void)hitCount; // only used when IR or OLED is enabled

#if ENABLE_GREYMECHA
    uint8_t shotCount = 0; // every laser shot, hit or miss (for the Command Centre)
    uint32_t lastGmPacketAt = 0;
    uint32_t lastGmAttitudeAt = 0;
    float attRoll = 0.0f, attPitch = 0.0f; // tilt for the 3D view, degrees
    float attYaw = 0.0f; // heading for the 3D view, degrees, from the gyro
#endif

    uart_printf("Entering STANDBY MODE\r\n");

    // Timing variables: all measured in milliseconds
    uint32_t lastSampleAt = HAL_GetTick() - SAMPLE_PERIOD_MS; // a trick to sample quicker from sensor the values
    uint32_t lastTelemetryAt = HAL_GetTick() - TELEMETRY_PERIOD_MS;
    uint32_t lastLedToggleAt = HAL_GetTick();

    uint32_t flipStartedAt = 0; // for significant Gyro
    uint32_t invertedStartedAt = 0; // az negative
    uint32_t attackStartedAt = 0;

    // Flags for the flip detector
    uint8_t flipWindowActive = 0; // possile flip?
    uint8_t trackingInverted = 0; // flipped confirm?
    uint8_t detectorArmed = 1;

    // Sensor data
    int16_t accelRaw[3] = {0};
    float gyroData[3] = {0};
    uint32_t lastAttackReportAt = 0;

    while (1)
    {
        uint32_t now = HAL_GetTick(); // has at least 50 ms passed for new readings from sensor?

#if ENABLE_GYRO_CAL
        gyro_cal_step(now, bootAt); // power-on gyro calibration, one sample per 20 ms
#endif
#if ENABLE_OLED
        // Start the OLED 100 ms after power-on (its power-up time), without blocking
        if (!oledStarted && (uint32_t)(now - bootAt) >= 100U)
        {
            oledStarted = 1;
            oled_init(); // OLED uses I2C1; the on-board sensors use I2C2
            oled_show_boot();
        }
#endif
        uint8_t newSample = 0;

        if (currentState != STANDBY &&
            (uint32_t)(now - lastSampleAt) >= SAMPLE_PERIOD_MS)
        {
            lastSampleAt = now;

            BSP_ACCELERO_AccGetXYZ(accelRaw); // az being read silently
            BSP_GYRO_GetXYZ(gyroData);

            newSample = 1;
        } // read the sensor when at least 50ms have passed from last reading

#if ENABLE_IR
        // IR sensor: a change is accepted only when two readings
        // 50 ms apart agree (filters flicker at the detection edge)
        uint8_t irChanged = 0;
        if ((uint32_t)(now - lastIrReadAt) >= IR_SAMPLE_MS)
        {
            lastIrReadAt = now;
            uint8_t raw = ir_read();
            if (raw == irPrevRaw && raw != irDetected)
            {
                irDetected = raw;
                irChanged = 1;
            }
            irPrevRaw = raw;
        }
#endif

#if ENABLE_GREYMECHA
        // Commands from the Command Centre behave exactly like the drone's own button:
        // 'D' = double press, 'S' = single press. The state machine below decides
        // what they mean (e.g. 'D' within 10 s of an attack = rescue).
        int cmd = gm_read();
        if (cmd == 'D' || cmd == 'S')
        {
            __disable_irq();
            if (cmd == 'D')
            {
                doublePressPending = 1;
                secondPressAt = HAL_GetTick(); // checked against the 10 s rescue deadline
            }
            else
            {
                singlePressPending = 1;
            }
            __enable_irq();

            uart_printf("Command Centre: remote %s press\r\n",
                   (cmd == 'D') ? "double" : "single");
        }
        else if (cmd == 'P')
        {
            gm_send("$PONG\n"); // link test
        }
        else if (currentState == BATTLE)
        {
            // Zombie game events from the Command Centre. The IR sensor only spawns
            // zombies; a shot hits if the drone (yaw) is aimed at one when it fires.
            switch (cmd)
            {
            case 'K': // hit, zombie destroyed
                hitCount++;
                uart_printf("ZOMBIE HIT - DESTROYED! (hits: %u)\r\n", (unsigned int)hitCount);
                ui_event("ZOMBIE DESTROYED!", now);
                break;
            case 'H': // hit, big zombie still standing
                hitCount++;
                uart_printf("BIG ZOMBIE HIT - still standing (hits: %u)\r\n", (unsigned int)hitCount);
                ui_event("BIG ZOMBIE HIT!", now);
                break;
            case 'M': // the shot found no zombie in the aim
                uart_printf("LASER MISSED - no zombie in the aim\r\n");
                ui_event("MISSED", now);
                break;
            case 'A': // a zombie came close
                uart_printf("WARNING: ZOMBIE APPROACHING!\r\n");
                ui_event("ZOMBIE APPROACHING", now);
                break;
            case 'B': // a small zombie reached the drone: -1 health
            case 'G': // a big zombie reached the drone: -2 health
            {
                uint8_t damage = (cmd == 'G') ? 2U : 1U;
                healthPoints = (healthPoints > damage) ? (uint8_t)(healthPoints - damage) : 0U;
                uart_printf("%s ZOMBIE REACHED THE DRONE! -%u HP (health: %u/%u)\r\n",
                            (cmd == 'G') ? "BIG" : "A", (unsigned int)damage,
                            (unsigned int)healthPoints, (unsigned int)MAX_HP);
                ui_event((cmd == 'G') ? "BIG ZOMBIE BITE!" : "ZOMBIE BITE!", now);

                if (healthPoints == 0U)
                {
                    // No health left: same rescue mode as a flip (10 s, double press)
                    uart_printf("DRONE OVERRUN: HEALTH 0/%u\r\n", (unsigned int)MAX_HP);
                    currentState = ATTACKED;
                    attackStartedAt = now;
                    lastAttackReportAt = now - ATTACK_MESSAGE_MS;
                    __disable_irq(); // fresh button recognition for the rescue window
                    waitingSecondPress = 0;
                    singlePressPending = 0;
                    doublePressPending = 0;
                    __enable_irq();
                    flipWindowActive = 0;
                    trackingInverted = 0;
                    lastLedToggleAt = now;
                    BSP_LED_On(LED2);
                    uart_printf("\r\n*** LAST OF EE2028 ***\r\n");
                }
                break;
            }
            default:
                break;
            }
        }
#endif

        // button interrupt event handling
        uint8_t singlePress = 0;
        uint8_t doublePress = 0;
        uint32_t doublePressTime = 0;

        // Briefly prevent the interrupt from changing shared variables
        __disable_irq();
        uint32_t buttonNow = HAL_GetTick();

        // First press has waited 500 ms without a second press
        if (waitingSecondPress &&
            (uint32_t)(buttonNow - firstPressAt) >= DOUBLE_PRESS_MS)
        {
            waitingSecondPress = 0;
            singlePressPending = 1;
        }

        // Transfer pending events into this loop iteration
        if (singlePressPending)
        {
            singlePress = 1;
            singlePressPending = 0;
        }

        if (doublePressPending)
        {
            doublePress = 1;
            doublePressTime = secondPressAt;
            doublePressPending = 0;
        }

        __enable_irq();
        // Convert interrupt-generated button flags into one-time
        // single/double-press events for the state machine.

        // Introducing a Finite state machine
        switch (currentState)
        {

        case STANDBY:
        {
            // Double press enters BATTLE
            if (doublePress)
            {
                currentState = BATTLE;
                laserChargePercent = 0U;
                hitCount = 0U;
#if ENABLE_GREYMECHA
                healthPoints = MAX_HP;
#endif

                lastLedToggleAt = now;
                BSP_LED_On(LED2);

                uart_printf("Entering BATTLE MODE\r\n");
                break;
            }

            // STANDBY telemetry is sent once per second
            if ((uint32_t)(now - lastTelemetryAt) >= TELEMETRY_PERIOD_MS)
            {
                lastTelemetryAt = now;

                // Read all six sensors
                float temperature = BSP_TSENSOR_ReadTemp();
                float pressure = BSP_PSENSOR_ReadPressure();
                float humidity = BSP_HSENSOR_ReadHumidity();

                BSP_ACCELERO_AccGetXYZ(accelRaw);
                BSP_GYRO_GetXYZ(gyroData);

                int16_t magRaw[3] = {0};
                BSP_MAGNETO_GetXYZ(magRaw);

                // Accelerometer: mg -> m/s^2, magnitude of XYZ vector
                float accelMagnitude =
                    sqrtf((float)accelRaw[0] * accelRaw[0] +
                          (float)accelRaw[1] * accelRaw[1] +
                          (float)accelRaw[2] * accelRaw[2]) *
                    0.00980665f;

                // Gyroscope: mdps -> degrees/second, bias removed
                float gxs = gyroData[0] / 1000.0f - gyroBias[0];
                float gys = gyroData[1] / 1000.0f - gyroBias[1];
                float gzs = gyroData[2] / 1000.0f - gyroBias[2];
                float gyroMagnitude = sqrtf(gxs * gxs + gys * gys + gzs * gzs);

                // Magnetometer BSP output: milligauss -> microtesla
                // 1 milligauss = 0.1 microtesla
                float magMagnitude =
                    sqrtf((float)magRaw[0] * magRaw[0] +
                          (float)magRaw[1] * magRaw[1] +
                          (float)magRaw[2] * magRaw[2]) *
                    0.1f;

                // Height above the power-on point (smoothed against sensor noise)
                if (p0 < 800.0f) // no reference yet: the first reading is the ground level
                {
                    p0 = pressure;
                    pFiltered = pressure;
                }
                pFiltered = 0.5f * pFiltered + 0.5f * pressure;
                float height = pressure_to_height(pFiltered, p0);

                // Check all sensor thresholds
                uint8_t badTemp =
                    (temperature < TEMP_MIN_C || temperature > TEMP_MAX_C);

                uint8_t badPressure =
                    (pressure < PRESS_MIN_HPA || pressure > PRESS_MAX_HPA);

                uint8_t badHumidity =
                    (humidity < HUM_MIN_PCT || humidity > HUM_MAX_PCT);

                uint8_t badAccel =
                    (accelMagnitude > ACCEL_MAX_MS2);

                uint8_t badGyro =
                    (gyroMagnitude > GYRO_MAX_DPS);

                uint8_t badMag =
                    (magMagnitude < MAG_MIN_UT || magMagnitude > MAG_MAX_UT);

                // Copy to the OLED dashboard
                ui.temp = temperature;
                ui.press = pressure;
                ui.hum = humidity;
                ui.accel = accelMagnitude;
                ui.gyro = gyroMagnitude;
                ui.mag = magMagnitude;
                ui.alt = height;
                ui.badMask = (uint8_t)((badTemp << 0) | (badPressure << 1) |
                                       (badHumidity << 2) | (badAccel << 3) |
                                       (badGyro << 4) | (badMag << 5));

                // If ANY sensor violates its threshold:
                // pause normal telemetry and send warning messages.
                if (badTemp || badPressure || badHumidity ||
                    badAccel || badGyro || badMag)
                {
                    uart_printf("WARNING: TELEMETRY THRESHOLD VIOLATED\r\n");

                    if (badTemp)
                        uart_printf("TEMP WARNING: %.2f C (allowed %.1f to %.1f C)\r\n",
                               temperature, TEMP_MIN_C, TEMP_MAX_C);

                    if (badPressure)
                        uart_printf("PRESSURE WARNING: %.2f hPa (allowed %.1f to %.1f hPa)\r\n",
                               pressure, PRESS_MIN_HPA, PRESS_MAX_HPA);

                    if (badHumidity)
                        uart_printf("HUMIDITY WARNING: %.2f %%RH (allowed %.1f to %.1f %%RH)\r\n",
                               humidity, HUM_MIN_PCT, HUM_MAX_PCT);

                    if (badAccel)
                        uart_printf("ACCEL WARNING: %.2f m/s2 (max %.2f)\r\n",
                               accelMagnitude, ACCEL_MAX_MS2);

                    if (badGyro)
                        uart_printf("GYRO WARNING: %.2f deg/s (max %.2f)\r\n",
                               gyroMagnitude, GYRO_MAX_DPS);

                    if (badMag)
                        uart_printf("MAG WARNING: %.2f uT (allowed %.1f to %.1f uT)\r\n",
                               magMagnitude, MAG_MIN_UT, MAG_MAX_UT);
                }
                else
                {
                    // All sensors normal: send regular telemetry
                    uart_printf("TELEMETRY: T=%.2f C | P=%.2f hPa | H=%.2f %%RH\r\n",
                           temperature, pressure, humidity);

                    uart_printf("A=%.2f m/s2 | G=%.2f deg/s | M=%.2f uT\r\n",
                           accelMagnitude, gyroMagnitude, magMagnitude);

#if ENABLE_IR
                    uart_printf("HEIGHT=%+.2f m (from start) | IR=%s\r\n",
                           height, irDetected ? "OBJECT" : "CLEAR");
#else
                    uart_printf("HEIGHT=%+.2f m (from start)\r\n", height);
#endif
                }
            }

            // Single button presses are ignored in STANDBY
            break;
        }

        case BATTLE:
        {
            if (doublePress)
            {
                currentState = STANDBY;

                flipWindowActive = 0;
                trackingInverted = 0;

                BSP_LED_On(LED2);
                lastLedToggleAt = now;

                uart_printf("Entering STANDBY MODE\r\n");
                break;
            }

            if (singlePress)
            {
                // A single press adds 30% energy
                laserChargePercent += 30U;

                uart_printf("LASER CHARGING: %u%%\r\n",
                       (unsigned int)laserChargePercent);

                // Fire immediately when sufficient charge is available
                if (laserChargePercent >= 50U)
                {
                    laserChargePercent -= 50U;
#if ENABLE_GREYMECHA
                    shotCount++;
#endif

                    // Whether a zombie was hit depends on where the drone is aimed:
                    // the Command Centre's zombie game decides and reports back (K/H/M below)
                    uart_printf("LASER FIRED! Remaining charge: %u%%\r\n",
                           (unsigned int)laserChargePercent);
                    ui_event("LASER FIRED!", now);
                }
                else
                {
                    ui_event("CHARGING...", now);
                }
            }

#if ENABLE_IR
            if (irChanged)
            {
                if (irDetected)
                {
                    uart_printf("ZOMBIE IN RANGE!\r\n");
                    ui_event("ZOMBIE IN RANGE!", now);
                }
                else
                {
                    uart_printf("Zone clear\r\n");
                }
            }
#endif

            if (!newSample)
                break;

            // Gyroscope bias correction (measured at power-on)
            float gx = gyroData[0] / 1000.0f - gyroBias[0];
            float gy = gyroData[1] / 1000.0f - gyroBias[1];

            // After a previous attack, require upright orientation
            // before allowing another attack detection.
            if (!detectorArmed)
            {
                if (accelRaw[2] > UPRIGHT_Z_MG)
                {
                    detectorArmed = 1;
                    uart_printf("Inversion detector rearmed\r\n");
                }
                break;
            }
            // below is persistence checking

            // Stage 1: Detect significant rotation around X or Y
            if (!flipWindowActive &&
                (gx > GYRO_TRIGGER_DPS || gx < -GYRO_TRIGGER_DPS ||
                 gy > GYRO_TRIGGER_DPS || gy < -GYRO_TRIGGER_DPS))
            {
                flipWindowActive = 1;
                flipStartedAt = now;
                trackingInverted = 0;

                uart_printf("Rotation detected: checking inversion\r\n");
            }

            // Stage 2: Confirm upside-down orientation
            if (flipWindowActive)
            {
                // Give up if confirmation takes too long
                if ((uint32_t)(now - flipStartedAt) >= FLIP_WINDOW_MS)
                {
                    flipWindowActive = 0;
                    trackingInverted = 0;
                }
                else if (accelRaw[2] < INVERTED_Z_MG)
                {
                    // First negative-Z sample starts confirmation timer
                    if (!trackingInverted)
                    {
                        trackingInverted = 1;
                        invertedStartedAt = now;
                    }

                    // Negative Z has persisted for 1 second
                    if ((uint32_t)(now - invertedStartedAt) >= INVERT_CONFIRM_MS)
                    {
                        currentState = ATTACKED;
                        attackStartedAt = now;
                        lastAttackReportAt = now - ATTACK_MESSAGE_MS;

                        // Start fresh button recognition for the rescue window
                        __disable_irq();
                        waitingSecondPress = 0;
                        singlePressPending = 0;
                        doublePressPending = 0;
                        __enable_irq();

                        flipWindowActive = 0;
                        trackingInverted = 0;

                        lastLedToggleAt = now;
                        BSP_LED_On(LED2);

                        uart_printf("\r\n*** LAST OF EE2028 ***\r\n");
                    }
                }
                else
                {
                    // Negative Z was interrupted: restart confirmation
                    trackingInverted = 0;
                }
            }

            break;
        }

        case ATTACKED:
        {
            // Read current elapsed attack time
            uint32_t attackNow = HAL_GetTick();
            uint32_t elapsed = (uint32_t)(attackNow - attackStartedAt);

            // STEP 1: A double press within 10 s rescues the drone
            if (doublePress)
            {
                // Verify the second press occurred before the deadline
                if ((uint32_t)(doublePressTime - attackStartedAt) < RESCUE_TIMEOUT_MS)
                {
                    currentState = BATTLE;

                    // Disarm until the board is upright again,
                    // so it doesn't instantly re-trigger if still inverted
                    detectorArmed = 0;
                    flipWindowActive = 0;
                    trackingInverted = 0;

                    lastLedToggleAt = attackNow;
                    BSP_LED_On(LED2);

#if ENABLE_GREYMECHA
                    healthPoints = MAX_HP; // rescued drone is repaired
#endif
                    uart_printf("*** RESCUE SUCCESSFUL: RETURN TO BATTLE ***\r\n");
                    ui_event("RESCUED!", attackNow);
                    break;
                }
            }

            // STEP 2: Check for expiration of the rescue deadline
            if (elapsed >= RESCUE_TIMEOUT_MS)
            {
                currentState = TERMINATED;

                BSP_LED_Off(LED2);

                uart_printf("Rescue countdown: 0 seconds\r\n");
                uart_printf("*** DRONE TERMINATED ***\r\n");

#if ENABLE_GREYMECHA
                // Last word to the Command Centre (state 3 = lost), then cut that link too
                gm_send("$D,3,0,0,%u,%u,0,0.0,0.0,0.0,0.00,0\n",
                        (unsigned int)shotCount, (unsigned int)hitCount);
                HAL_UART_DeInit(&huart4);
#endif

#if ENABLE_OLED
                oled_show_terminated(); // "SIGNAL LOST" stays on screen
#endif

                BSP_COM_DeInit(COM1, &huart1);
                // Halt all application processing
                __disable_irq();

                while (1)
                {
                    __NOP();
                }
            }

            // STEP 3: Print the warning and remaining time every second
            if ((uint32_t)(attackNow - lastAttackReportAt) >= ATTACK_MESSAGE_MS)
            {
                lastAttackReportAt = attackNow;

                uint32_t remainingMs = RESCUE_TIMEOUT_MS - elapsed;
                uint32_t remainingSeconds = (remainingMs + 999U) / 1000U;

                uart_printf("Drone Was Attacked! %lu seconds remaining\r\n",
                       (unsigned long)remainingSeconds);
            }

            // Single button presses are ignored
            break;
        }

        case TERMINATED:
            // Terminated behaviour
            break;
        }

        // LED behaviour depends on the drone's current state

        if (currentState == STANDBY)
        {
            // Standby: LED must remain continuously ON
            BSP_LED_On(LED2);
        }
        else if (currentState == BATTLE || currentState == ATTACKED)
        {
            // Attacked: toggle every 250 ms -> 2 Hz blinking
            // Battle: toggle every 500 ms -> 1 Hz blinking
            uint32_t ledHalfPeriod =
                (currentState == ATTACKED) ? 250U : 500U;

            if ((uint32_t)(now - lastLedToggleAt) >= ledHalfPeriod)
            {
                lastLedToggleAt = now;
                BSP_LED_Toggle(LED2);
            }
        }

#if ENABLE_GREYMECHA
        // Status line for the Command Centre, 5 times per second:
        // $D,state,charge%,ir,shots,hits,secsLeft,T,P,H,height,heading
        if ((uint32_t)(now - lastGmPacketAt) >= GM_PACKET_MS)
        {
            lastGmPacketAt = now;

            uint32_t secsLeft = 0;
            if (currentState == ATTACKED)
            {
                uint32_t el = (uint32_t)(now - attackStartedAt);
                secsLeft = (el >= RESCUE_TIMEOUT_MS) ? 0U : (RESCUE_TIMEOUT_MS - el + 999U) / 1000U;
            }

#if ENABLE_IR
            uint8_t irNow = irDetected;
#else
            uint8_t irNow = 0;
#endif

            // Compass heading for the radar screen: angle of the field in the board's
            // X-Y plane (good while the drone is roughly level), 0..359 degrees
            int16_t magXYZ[3] = {0};
            BSP_MAGNETO_GetXYZ(magXYZ);
            float heading = atan2f((float)magXYZ[1], (float)magXYZ[0]) * 57.29578f;
            if (heading < 0.0f)
                heading += 360.0f;

            // Then: sensors out of range in STANDBY (bit0 T .. bit5 M, else 0), health
            gm_send("$D,%d,%u,%u,%u,%u,%lu,%.1f,%.1f,%.1f,%.2f,%d,%u,%u\n",
                    (int)currentState, (unsigned int)laserChargePercent,
                    (unsigned int)irNow, (unsigned int)shotCount,
                    (unsigned int)hitCount, (unsigned long)secsLeft,
                    ui.temp, ui.press, ui.hum, ui.alt, (int)heading,
                    (unsigned int)((currentState == STANDBY) ? ui.badMask : 0U),
                    (unsigned int)healthPoints);
        }

        // Attitude line for the Command Centre's 3D view, 40 times per second:
        // $A,roll,pitch,yaw in degrees.
        // Complementary filter: the gyro follows fast movement with no delay,
        // and gravity (accelerometer) slowly pulls out the gyro's drift.
        // Yaw has no gravity reference, so it is the gyro alone.
        if ((uint32_t)(now - lastGmAttitudeAt) >= GM_ATTITUDE_MS)
        {
            float dt = (float)(uint32_t)(now - lastGmAttitudeAt) / 1000.0f;
            lastGmAttitudeAt = now;
            if (dt > 0.2f)
                dt = 0.2f; // first pass, or after a long pause

            int16_t a[3] = {0};
            float g[3] = {0};
            BSP_ACCELERO_AccGetXYZ(a);
            BSP_GYRO_GetXYZ(g);

            // Tilt from gravity alone: correct when still, noisy when moving
            float ax = (float)a[0], ay = (float)a[1], az = (float)a[2];
            float accRoll = atan2f(ay, az) * 57.29578f;
            float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;
            float aMag = sqrtf(ax * ax + ay * ay + az * az); // mg, ~1000 at rest

            if (!gyroCalDone)
            {
                attRoll = accRoll; // gyro not ready yet: gravity only
                attPitch = accPitch;
            }
            else
            {
                // Body turn rates (deg/s, bias removed) -> roll, pitch, yaw rates
                float p = g[0] / 1000.0f - gyroBias[0];
                float q = g[1] / 1000.0f - gyroBias[1];
                float r = g[2] / 1000.0f - gyroBias[2];
                float sr = sinf(attRoll / 57.29578f), cr = cosf(attRoll / 57.29578f);
                float sp = sinf(attPitch / 57.29578f), cp = cosf(attPitch / 57.29578f);
                if (fabsf(cp) < 0.1f)
                    cp = (cp < 0.0f) ? -0.1f : 0.1f; // nose straight up/down
                float yawRate = (q * sr + r * cr) / cp;

                attRoll += (p + yawRate * sp) * dt;
                attPitch += (q * cr - r * sr) * dt;
                if (fabsf(yawRate) > 0.5f) // ignore tiny leftover gyro drift
                    attYaw += yawRate * dt;

                // Gravity correction, 5% per step, only when not being shaken
                if (aMag > 700.0f && aMag < 1300.0f)
                {
                    attRoll += 0.05f * wrap180(accRoll - attRoll);
                    attPitch += 0.05f * (accPitch - attPitch);
                }
            }
            attRoll = wrap180(attRoll);
            if (attPitch > 90.0f)
                attPitch = 90.0f;
            if (attPitch < -90.0f)
                attPitch = -90.0f;
            if (attYaw >= 360.0f)
                attYaw -= 360.0f;
            if (attYaw < 0.0f)
                attYaw += 360.0f;

            gm_send("$A,%d,%d,%d\n", (int)attRoll, (int)attPitch, (int)attYaw);
        }
#endif

#if ENABLE_OLED
        // Hand the latest values to the dashboard and send one page if needed
        ui.state = currentState;
        ui.charge = laserChargePercent;
        ui.hits = hitCount;
#if ENABLE_GREYMECHA
        ui.hp = healthPoints;
#endif
        ui.armed = detectorArmed;
        ui.attackStartedAt = attackStartedAt;
#if ENABLE_IR
        ui.ir = irDetected;
#endif
        oled_update(now);
#endif
    }
}

// BUTTON PRESS FUNCTION Definition
// below is the interrupt call back
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != USER_BUTTON_PIN)
        return;

    uint32_t t = HAL_GetTick();

    // Ignore rapid duplicate edges due to button bounce
    if ((uint32_t)(t - lastButtonEdgeAt) < BUTTON_DEBOUNCE_MS)
        return;

    lastButtonEdgeAt = t;

    if (waitingSecondPress &&
        (uint32_t)(t - firstPressAt) < DOUBLE_PRESS_MS)
    {
        // Two presses occurred within 500 ms
        waitingSecondPress = 0;
        doublePressPending = 1;
        secondPressAt = t;
    }
    else
    {
        // If the old first press expired, it was single
        if (waitingSecondPress)
            singlePressPending = 1;

        // This becomes the first press of a new sequence
        waitingSecondPress = 1;
        firstPressAt = t;
    }
}
