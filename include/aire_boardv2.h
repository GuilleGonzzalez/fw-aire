/**
 * @file aire_boardv2.h
 *
 * @brief Board definition for Aire v2.0.0
 *
 * @version v1.0.0
 */

#ifndef __AIRE_BOARD2_H
#define __AIRE_BOARD2_H

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_ID 2

/** GENERIC **/
#define PIN_BTN_USR    0
#define PIN_FAN1       14
#define PIN_FAN2       12
#define PIN_FAN3       13

/** I2C **/
#define I2C_PRESENT 1
#define PIN_I2C_SDA 4
#define PIN_I2C_SCL 5

/** PCA9536 **/ // I2C GPIO expander
#define PCA9536_PRESENT 1

/** LEDS **/
#define PIN_LED_RED    0 // PCA9536
#define PIN_LED_YELLOW 1 // PCA9536
#define PIN_LED_GREEN  2 // PCA9536

#define LED_POWER      PIN_LED_RED
#define LED_GENERAL    PIN_LED_YELLOW

/** BUZZER **/
#define PIN_BUZZ       2

/** SHT4X **/
#define SHT4X_PRESENT  1
#define TEMP_SENSOR    1
#define HUMD_SENSOR    1

/** BMP581 **/
#define BMP581_PRESENT 0
#define PRESS_SENSOR   1
#define PIN_INT_BMP581 16

#ifdef __cplusplus
}
#endif

#endif // __AIRE_BOARD2_H
