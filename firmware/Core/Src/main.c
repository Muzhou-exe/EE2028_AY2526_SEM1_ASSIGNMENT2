
/******************************************************************************
  * @file           : main.c
  * @brief          : EE2028 Integrated WiFi + OLED + Game System
  * Integration by 木舟
******************************************************************************/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ======== Extra includes for integration ======== */
#include "wifi.h"
#include "i2c.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

/* BSP sensor includes */
#include "stm32l4s5i_iot01_accelero.h"
#include "stm32l4s5i_iot01_gyro.h"
#include "stm32l4s5i_iot01_hsensor.h"
#include "stm32l4s5i_iot01_psensor.h"
#include "stm32l4s5i_iot01_magneto.h"
#include "stm32l4s5i_iot01_tsensor.h"

/* UART/I2C/SPI handles */
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef  hi2c1;

/* -------------------- WiFi Configuration -------------------- */
const char*      WiFi_SSID     = "Muzhou_exe";
const char*      WiFi_password = "20051216";
const WIFI_Ecn_t WiFi_security = WIFI_ECN_WPA2_PSK;
uint8_t          ipaddr[4]     = {192, 168, 137, 1};
const uint16_t   DEST_PORT     = 2028;
const uint16_t   SOURCE_PORT   = 2028;

/* -------------------------------------------------------------------------- */
void SystemClock_Config(void);
void UART_Print(const char *msg);
void Game_RedLightGreenLight(void);
void Game_CatchAndRun(void);
void ReadSensors(float *T, float *H, float *P, float *accel, float *gyro, float *mag);

/* -------------------------------------------------------------------------- */
typedef enum {
  MODE_RED_GREEN = 0,
  MODE_CATCH_RUN = 1
} GameMode;

GameMode currentMode = MODE_RED_GREEN;
float gyroOffset = 0.0f;
#define MAX_LENGTH 400
#define WIFI_READ_TIMEOUT 5000
static uint8_t loggedIn = 0;
volatile uint8_t rg_reset_request = 0;

/* EXTI 按键全局变量 */
volatile uint8_t buttonEvent = 0;        // 0:无事件, 1:单击, 2:双击
volatile uint8_t escapeTriggered = 0;    // 逃脱触发标志
volatile uint32_t lastPressTime = 0;
volatile uint8_t clickCount = 0;

/* ======== Utility ======== */
static void wait_ms(uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms) __NOP();
}

/* ======== OLED Helper ======== */
static void OLED_ShowStatus(const char* l1, const char* l2, const char* l3)
{
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString((char*)l1, Font_7x10, White);
    ssd1306_SetCursor(0, 12);
    ssd1306_WriteString((char*)l2, Font_7x10, White);
    ssd1306_SetCursor(0, 24);
    ssd1306_WriteString((char*)l3, Font_7x10, White);
    ssd1306_UpdateScreen();
}

/* ======== UART Print ======== */
void UART_Print(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}

/* ======== WiFi Initialization ======== */
static uint8_t WiFi_ConnectToServer(void)
{
    WIFI_Status_t status;
    char msg[100];

    UART_Print("\r\n=== WiFi Init ===\r\n");
    OLED_ShowStatus("WiFi Init", "Starting...", "");

    status = WIFI_Init();  // 会内部调用 SPI_WIFI_MspInit / HAL_SPI_Init，并开启 EXTI1
    if (status != WIFI_STATUS_OK)
    {
        UART_Print("WiFi init FAILED!\r\n");
        OLED_ShowStatus("WiFi Init", "FAILED", "");
        return 0;
    }

    OLED_ShowStatus("WiFi Init", "OK", "Connecting...");
    sprintf(msg, "Connecting to %s...\r\n", WiFi_SSID);
    UART_Print(msg);

    status = WIFI_Connect(WiFi_SSID, WiFi_password, WiFi_security);
    if (status != WIFI_STATUS_OK)
    {
        UART_Print("WiFi connect FAILED!\r\n");
        OLED_ShowStatus("WiFi Connect", "FAILED", "");
        return 0;
    }

    OLED_ShowStatus("WiFi Connect", "OK", "Connecting TCP...");
    sprintf(msg, "Connecting %d.%d.%d.%d:%d\r\n",
            ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3], DEST_PORT);
    UART_Print(msg);

    status = WIFI_OpenClientConnection(1, WIFI_TCP_PROTOCOL,
                                       "conn", ipaddr, DEST_PORT, SOURCE_PORT);
    if (status != WIFI_STATUS_OK)
    {
        UART_Print("TCP connect FAILED!\r\n");
        OLED_ShowStatus("TCP Connect", "FAILED", "");
        return 0;
    }

    OLED_ShowStatus("TCP Connect", "Success", "Ready for login");
    UART_Print("TCP connection established.\r\n");
    return 1;
}

/* -------------------------------------------------------------------------- */
/* ---------------------- Original Game Logic (Unchanged) -------------------- */
/* -------------------------------------------------------------------------- */
void ReadSensors(float *T, float *H, float *P, float *accel, float *gyro, float *mag)
{
    int16_t acc_i16[3];
    float   gyro_f[3];
    int16_t mag_i16[3];

    *T = BSP_TSENSOR_ReadTemp();
    *H = BSP_HSENSOR_ReadHumidity();
    *P = BSP_PSENSOR_ReadPressure();

    BSP_ACCELERO_AccGetXYZ(acc_i16);
    BSP_GYRO_GetXYZ(gyro_f);
    BSP_MAGNETO_GetXYZ(mag_i16);

    *accel = sqrtf(acc_i16[0]*acc_i16[0] + acc_i16[1]*acc_i16[1] + acc_i16[2]*acc_i16[2]) / 1000.0f / 9.81f;
    *gyro  = sqrtf(gyro_f[0]*gyro_f[0] + gyro_f[1]*gyro_f[1] + gyro_f[2]*gyro_f[2]) / 1000.0f - gyroOffset;
    if (*gyro < 0) *gyro = 0;
    *mag   = sqrtf(mag_i16[0]*mag_i16[0] + mag_i16[1]*mag_i16[1] + mag_i16[2]*mag_i16[2]);
}

/* -------------------- EXTI Callback -------------------- */
/* -------------------- EXTI Callback -------------------- */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13)  // 用户按钮
    {
        uint32_t now = HAL_GetTick();

        // 防抖处理
        if (now - lastPressTime < 50) {
            return;
        }
        lastPressTime = now;

        // 递增点击计数
        clickCount++;

        static uint32_t firstClickTime = 0;

        if (clickCount == 1) {
            // 第一次点击，记录时间
            firstClickTime = now;
        }
        else if (clickCount == 2) {
            // 第二次点击，检查时间间隔判断是否为双击
            if (now - firstClickTime < 400) {
                // 双击：切换模式
                buttonEvent = 2;
            }
            // 重置状态
            clickCount = 0;
            firstClickTime = 0;
        }

        // 如果是游戏2模式，单击立即触发逃脱
        if (currentMode == MODE_CATCH_RUN && clickCount == 1) {
            escapeTriggered = 1;
        }
    }
    else if (GPIO_Pin == GPIO_PIN_1) {
        // WiFi中断
        SPI_WIFI_ISR();
    }
}
/* -------------------- Game 1: Red Light / Green Light -------------------- */
void Game_RedLightGreenLight(void)
{
    static uint8_t  isGreen     = 1;
    static uint32_t lastToggle  = 0;
    static uint32_t lastEnvSend = 0;
    static uint8_t  printedGreen = 0, printedRed = 0;

    // 重置处理
    if (rg_reset_request) {
        isGreen = 1;
        lastToggle = HAL_GetTick();
        lastEnvSend = 0;
        printedGreen = 0;
        printedRed = 0;
        rg_reset_request = 0;

        UART_Print("Red-Green Game RESET\n");
        OLED_ShowStatus("WiFi OK", "Mode: Red-Green", "Game Reset");
        wait_ms(1000);
    }

    float T,H,P,accel,gyro,mag;
    char  msg[128];

    if (lastToggle == 0)
        lastToggle = HAL_GetTick();

    if (HAL_GetTick() - lastToggle > 10000)
    {
        isGreen = !isGreen;
        lastToggle = HAL_GetTick();
        printedGreen = 0;
        printedRed = 0;
    }

    ReadSensors(&T,&H,&P,&accel,&gyro,&mag);

    if (isGreen)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        if (!printedGreen) {
            UART_Print("Green Light!\n");
            OLED_ShowStatus("WiFi OK", "Mode: Red-Green", "Green Light!");
            printedGreen = 1;
        }

        // 恢复环境数据打印（每2秒）
        if (HAL_GetTick() - lastEnvSend > 2000)
        {
            lastEnvSend = HAL_GetTick();
            snprintf(msg, sizeof(msg), "Env T=%.1fC H=%.1f%% P=%.0fhPa\n", T, H, P);
            UART_Print(msg);
        }
    }
    else
    {
        if (!printedRed) {
            UART_Print("Red Light!\n");
            OLED_ShowStatus("WiFi OK", "Mode: Red-Green", "Red Light!");
            printedRed = 1;
        }

        if ((int)(HAL_GetTick()/500)%2==0)
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);

        // 恢复运动数据打印（每2秒）
        if (HAL_GetTick() - lastEnvSend > 2000)
        {
            lastEnvSend = HAL_GetTick();
            snprintf(msg, sizeof(msg), "Motion A=%.2fg G=%.2fdps\n", accel, gyro);
            UART_Print(msg);
        }

        if (fabs(accel) > 1.6f || fabs(gyro) > 10.0f)
        {
            UART_Print("Game Over! Player Moved.\n");
            OLED_ShowStatus("WiFi OK", "Game: Red-Green", "Game Over!");
            while(1);
        }
    }
}

/* -------------------- Game 2: Catch & Run -------------------- */
/* -------------------- Game 2: Catch & Run -------------------- */
void Game_CatchAndRun(void)
{
    float T, H, P, accel, gyro, mag;
    char  msg[256];
    static uint32_t lastEnv = 0;
    static uint32_t lastBlink = 0;
    static uint32_t lastDebug = 0;
    static uint32_t chaseStart = 0;
    static uint32_t lastCooldown = 0;
    static uint8_t  chasing = 0;
    static uint8_t  cooldown = 0;

    ReadSensors(&T, &H, &P, &accel, &gyro, &mag);

    // 传感器数据打印（每1秒）
    if (HAL_GetTick() - lastDebug > 1000)
    {
        lastDebug = HAL_GetTick();
        snprintf(msg, sizeof(msg),
                 "[DEBUG] T=%.2fC, H=%.1f%%, P=%.0fhPa, "
                 "A=%.2fg, G=%.2fdps, M=%.1fuT\n",
                 T, H, P, accel, gyro, mag);
        UART_Print(msg);

        // 调试信息：显示追逐状态和逃脱标志
        snprintf(msg, sizeof(msg),
                 "[STATUS] Chasing=%d, EscapeFlag=%d\n",
                 chasing, escapeTriggered);
        UART_Print(msg);
    }

    // 环境警报打印（每1秒）
    if (HAL_GetTick() - lastEnv > 1000)
    {
        lastEnv = HAL_GetTick();
        if (T > 35.0f)  {
            snprintf(msg, sizeof(msg), "[ALERT] Temp spike: %.1fC\n", T);
            UART_Print(msg);
        }
        if (H > 80.0f)  {
            snprintf(msg, sizeof(msg), "[ALERT] Humidity spike: %.1f%%\n", H);
            UART_Print(msg);
        }
        if (P > 1020.0f){
            snprintf(msg, sizeof(msg), "[ALERT] Pressure spike: %.0fhPa\n", P);
            UART_Print(msg);
        }
    }

    // 冷却期处理
    if (cooldown && (HAL_GetTick() - lastCooldown < 5000)) {
        uint32_t remaining = 5000 - (HAL_GetTick() - lastCooldown);
        snprintf(msg, sizeof(msg), "Cooldown: %lums", remaining);
        OLED_ShowStatus("WiFi OK", "Mode: Catch&Run", msg);
        return;
    } else {
        cooldown = 0;
    }

    // 安全区域检测
    if (mag <= 1800.0f && !chasing) {
        OLED_ShowStatus("WiFi OK", "Mode: Catch&Run", "Safe Zone");
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    }
    // 开始追逐
    else if (mag > 1800.0f && !chasing) {
        chasing = 1;
        chaseStart = HAL_GetTick();
        escapeTriggered = 0;  // 开始追逐时重置逃脱标志
        UART_Print("Enforcer nearby! Be careful.\n");
        OLED_ShowStatus("WiFi OK", "Mode: Catch&Run", "! CHASING!");
    }

    // 追逐状态处理
    if (chasing) {
        uint32_t chaseTime = HAL_GetTick() - chaseStart;

        // LED闪烁（根据距离调整频率）
        uint16_t blinkRate;
        if (mag > 2000) blinkRate = 100;
        else if (mag > 1900) blinkRate = 200;
        else blinkRate = 400;

        if (HAL_GetTick() - lastBlink > blinkRate) {
            lastBlink = HAL_GetTick();
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
        }

        // 显示追逐时间
        snprintf(msg, sizeof(msg), "Chase: %lu.%lus", chaseTime/1000, (chaseTime%1000)/100);
        OLED_ShowStatus("WiFi OK", "Press to escape!", msg);

        // 逃脱检查 - 关键修复！
        if (escapeTriggered && chaseTime < 3000) {
            // 逃脱成功！
            UART_Print("Player escaped, good job!\n");
            OLED_ShowStatus("WiFi OK", "Mode: Catch&Run", "ESCAPED!");

            chasing = 0;
            cooldown = 1;
            lastCooldown = HAL_GetTick();
            escapeTriggered = 0;  // 重要：清除标志

            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            wait_ms(2000); // 显示成功信息

            return;
        }

        // 如果按下按钮但不在逃脱时间内，清除标志但不逃脱
        if (escapeTriggered && chaseTime >= 3000) {
            UART_Print("Escape failed - too late!\n");
            escapeTriggered = 0;  // 清除标志
        }

        // 超时检查（必须在逃脱检查之后）
        if (chaseTime >= 3000) {
            UART_Print("Game Over! Player caught.\n");
            OLED_ShowStatus("Game Over", "Caught!", "Try again!");
            while (1);
        }
    }
    // 追捕者离开
    else if (mag < 1600.0f && chasing) {
        UART_Print("Enforcer left. Safe now.\n");
        chasing = 0;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    }
}
/* -------------------- Main -------------------- */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    wait_ms(200);  // 重要：WiFi 模块上电稳定
    ssd1306_Init();
    OLED_ShowStatus("System Boot", "WiFi Setup", "...");

    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    BSP_HSENSOR_Init();
    BSP_PSENSOR_Init();
    BSP_MAGNETO_Init();
    BSP_TSENSOR_Init();
    BSP_LED_Init(LED2);
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

    if (!WiFi_ConnectToServer())
        while(1);

    UART_Print("Connected to WiFi, waiting for login...\r\n");
    OLED_ShowStatus("WiFi OK", "Waiting login...", "");

    /* ========== 登录等待循环 ========== */
    uint8_t recvBuf[MAX_LENGTH];
    uint16_t recvLen = 0;
    WIFI_Status_t WiFi_Stat;

    while (!loggedIn)
    {
        memset(recvBuf, 0, MAX_LENGTH);
        recvLen = 0;

        WiFi_Stat = WIFI_ReceiveData(1, recvBuf, MAX_LENGTH, &recvLen, WIFI_READ_TIMEOUT);
        if (WiFi_Stat == WIFI_STATUS_OK && recvLen > 0)
        {
            recvBuf[recvLen] = '\0';
            UART_Print(" From Server: ");
            UART_Print((char*)recvBuf);
            UART_Print("\r\n");

            if (strstr((char*)recvBuf, "Register OK")) {
                UART_Print(" Registration successful!\r\n");
                OLED_ShowStatus("Register", "Success!", "");
            }
            else if (strstr((char*)recvBuf, "Register FAIL")) {
                UART_Print(" Registration failed.\r\n");
                OLED_ShowStatus("Register", "Failed", "");
            }
            else if (strstr((char*)recvBuf, "Login OK")) {
                loggedIn = 1;
                char username[32] = {0};
                sscanf((char*)recvBuf, "Login OK %s", username);
                char disp[64];
                snprintf(disp, sizeof(disp), "Welcome %s", username);
                OLED_ShowStatus("Login", "Success!", disp);
                UART_Print(" Login success, game start.\r\n");
            }
            else if (strstr((char*)recvBuf, "Login FAIL")) {
                loggedIn = 0;
                OLED_ShowStatus("Login", "Failed", "");
                UART_Print(" Login failed.\r\n");
            }
            else if (strstr((char*)recvBuf, "Game over")) {
                OLED_ShowStatus("Game", "Over", "");
                UART_Print(" Game over.\r\n");
                loggedIn = 0;
            }
        }
    }

    OLED_ShowStatus("WiFi OK", "Game Ready", "Press BTN");
    UART_Print("Entering Red Light, Green Light (EXTI version)\r\n");

    OLED_ShowStatus("WiFi OK", "Game Ready", "Press BTN");

    UART_Print("Entering Red Light, Green Light (EXTI version)\r\n");

    /* --------- Original calibration --------- */
    UART_Print("Calibrating gyro...\r\n");
    {
        float gtmp[3], sum = 0;
        uint32_t samples = 0, lastRead = HAL_GetTick();

        while (samples < 50)
        {
            if (HAL_GetTick() - lastRead >= 10)
            {
                lastRead = HAL_GetTick();
                BSP_GYRO_GetXYZ(gtmp);
                sum += sqrtf(gtmp[0]*gtmp[0] + gtmp[1]*gtmp[1] + gtmp[2]*gtmp[2]);
                samples++;
            }
        }

        gyroOffset = sum / 50.0f / 1000.0f;
        char msg[64];
        snprintf(msg, sizeof(msg), "Gyro offset = %.3f dps\r\n", gyroOffset);
        UART_Print(msg);
    }

    while (1)
    {
        // 处理按钮事件（包括模式切换）
        if (buttonEvent != 0) {
            if (buttonEvent == 2) {
                // 双击切换模式
                currentMode = (currentMode == MODE_RED_GREEN) ? MODE_CATCH_RUN : MODE_RED_GREEN;
                if (currentMode == MODE_RED_GREEN) {
                    UART_Print("Switching to Red Light, Green Light\n");
                    rg_reset_request = 1;  // 请求重置游戏1状态
                    OLED_ShowStatus("WiFi OK", "Mode: Red-Green", "Game Reset");
                    wait_ms(1000);
                } else {
                    UART_Print("Switching to Catch and Run\n");
                    OLED_ShowStatus("WiFi OK", "Mode: Catch&Run", "Ready");
                    // 切换到游戏2时重置逃脱标志
                    escapeTriggered = 0;
                }
            }
            buttonEvent = 0;  // 清除事件标志
        }

        // 单机事件处理（如果需要）
        // if (buttonEvent == 1) { ... }

        // 运行当前游戏
        if (currentMode == MODE_RED_GREEN) {
            Game_RedLightGreenLight();
        } else {
            Game_CatchAndRun();
        }

        // 处理单击超时（防止单击被误认为双击的第一击）
        if (clickCount == 1 && (HAL_GetTick() - lastPressTime > 400)) {
            // 单机事件处理
            if (currentMode == MODE_RED_GREEN) {
                // 游戏1的单击功能
                buttonEvent = 1;
            }
            clickCount = 0;
        }
    }
}

/* 时钟配置 ---------------------------------------------------------------- */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 40;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

/* 错误处理 ---------------------------------------------------------------- */
void Error_Handler(void)
{
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
    }
}




