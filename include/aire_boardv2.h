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
#define PIN_LED_RED    16
#define PIN_LED_GREEN  9 // PROBLEMS!
#define PIN_BUZZ       2
#define PIN_BTN_USR    0 // PROBLEMS!
#define PIN_FAN1       13
#define PIN_FAN2       12
#define PIN_FAN3       14

/** LEDS **/
#define LEDS_NUMBER       2
#define LEDS_RGB          0
#define LED_POWER         PIN_LED_RED
#define LED_GENERAL       PIN_LED_YELLOW
#define LEDS_LIST         {PIN_LED_RED, PIN_LED_YELLOW}
#define LEDS_ACTIVE_STATE 0

/** SHT4X **/
#define SHT4X_PRESENT  1
#define TEMP_SENSOR    1
#define HUMD_SENSOR    1

/** BMP581 **/
#define BMP581_PRESENT 1
#define PRESS_SENSOR   1
#define PIN_INT_BMP581 15

#ifdef __cplusplus
}
#endif

#endif // __AIRE_BOARD2_H
