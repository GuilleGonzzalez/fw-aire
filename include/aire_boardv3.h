/**
 * @file aire_boardv3.h
 *
 * @brief Board definition for Aire v3.0.0
 *
 * @version v1.0.0
 */

#ifndef __AIRE_BOARD3_H
#define __AIRE_BOARD3_H

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_ID 3

/** GENERIC **/
#define PIN_BTN_USR    0
#define PIN_FAN1       10
#define PIN_FAN2       9
#define PIN_FAN3       3

/** I2C **/
#define I2C_PRESENT 1
#define PIN_I2C_SDA 5
#define PIN_I2C_SCL 4

/** LEDS **/
#define PIN_LED_RED    6
#define PIN_LED_GREEN  7
#define PIN_LED_BLUE   17

#define LED_POWER      PIN_LED_GREEN
#define LED_GENERAL    PIN_LED_BLUE

/** BUZZER **/
#define PIN_BUZZ       8

/** SHT4X **/
#define SHT4X_PRESENT  1
#define TEMP_SENSOR    1
#define HUMD_SENSOR    1

/** BMP581 **/
#define BMP581_PRESENT 1
#define PRESS_SENSOR   1
#define PIN_INT_BMP581 14

#ifdef __cplusplus
}
#endif

#endif // __AIRE_BOARD3_H
