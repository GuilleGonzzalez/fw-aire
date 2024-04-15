#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoHA.h>
#include "Adafruit_SHT4x.h"
#include "SparkFun_PCA9536_Arduino_Library.h"
#include "SparkFun_BMP581_Arduino_Library.h"

#include "aire_boardv2.h"

// #define WIFI_SSID          "TycheTools"
// #define WIFI_PASS          "DanilleX2023"
// #define MQTT_IP            "192.168.0.100"
#define WIFI_SSID          "Gonmar-Livebox"
#define WIFI_PASS          "618995151609549464"
#define MQTT_IP            "192.168.1.100"
#define DEVICE_NAME        "Aire Guille Arduino"
#define FAN_NAME           "Fan Guille"
#define TEMP_SENSOR_NAME   "Temp Guille"
#define HUMD_SENSOR_NAME   "Humd Guille"
#define PRESS_SENSOR_NAME   "Press Guille"
#define GENERAL_LIGHT_NAME "General Light"
#define FW_VERSION         "0.1.0"
#define FAN_ID             "fan_guille"
#define TEMP_SENSOR_ID     "temp_fan_guille"
#define HUMD_SENSOR_ID     "humd_fan_guille"
#define PRESS_SENSOR_ID     "press_fan_guille"
#define GENERAL_LIGHT_ID   "general_light"
#define TEMP_PERIOD        10000 // ms
#define BUZZ_FREQ          220

WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device);
HAFan fan(FAN_ID, HAFan::SpeedsFeature);
HASensorNumber temp_sensor(TEMP_SENSOR_ID, HASensorNumber::PrecisionP2);
HASensorNumber humd_sensor(HUMD_SENSOR_ID, HASensorNumber::PrecisionP2);
HASensorNumber press_sensor(PRESS_SENSOR_ID, HASensorNumber::PrecisionP2);
HALight general_light(GENERAL_LIGHT_ID);

#if SHT4X_PRESENT == 1
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
#endif

#if BMP581_PRESENT == 1
BMP581 pressureSensor;
#endif

#if PCA9536_PRESENT == 1
PCA9536 io_exp;
#endif

/* Global variables ***********************************************************/

static volatile int bmp581_ready = 0;
static unsigned long last_time = 0;
static bool fan_state = false;
static uint16_t fan_speed = 1;

/* Function prototypes ********************************************************/

#if BMP581_PRESENT == 1
static void bmp581_cb();
#endif
static void usr_btn_cb();
static void fan_state_cmd_cb(bool l_fan_state, HAFan* sender);
static void fan_speed_cmd_cb(uint16_t speed, HAFan* sender);
static void light_state_cmd_cb(bool general_state, HALight* sender);

static void set_speed(uint16_t speed);
static void buzzer_set(uint8_t freq, uint16_t t1, uint16_t t2, uint16_t t3);

/* Callbacks ******************************************************************/

#if BMP581_PRESENT == 1
static void bmp581_cb()
{
	bmp581_ready = true;
}
#endif

static void usr_btn_cb()
{
	delay(50);
	if (fan_state == false) {
		fan_state = true;
		fan_speed = 1;
		fan.setSpeed(fan_speed);
		fan_state_cmd_cb(fan_state, &fan);
	} else {
		if (fan_speed == 3) {
			fan_state = false;
			fan_state_cmd_cb(fan_state, &fan);
		} else {
			fan_speed++;
			fan_speed_cmd_cb(fan_speed, &fan);
		}
	}
}


static void fan_state_cmd_cb(bool l_fan_state, HAFan* sender)
{

	fan_state = l_fan_state;

	Serial.print("Fan state: ");
	Serial.println(fan_state);

	sender->setState(fan_state); // Report state back to the Home Assistant

	if (fan_state) {
#if PCA9536_PRESENT == 1
		io_exp.digitalWrite(LED_POWER, HIGH);
#endif
		set_speed(fan_speed);
		buzzer_set(BUZZ_FREQ, 100, 100, 300); 
	} else {
#if PCA9536_PRESENT == 1
		io_exp.digitalWrite(LED_POWER, LOW);
#endif
		set_speed(0);
		buzzer_set(BUZZ_FREQ, 300, 100, 100); 
	}
}

static void fan_speed_cmd_cb(uint16_t speed, HAFan* sender)
{
	fan_speed = speed;

	Serial.print("Fan speed: ");
	Serial.println(fan_speed);

	sender->setSpeed(fan_speed);     // Report speed back to the Home Assistant

	set_speed(fan_speed);
	buzzer_set(BUZZ_FREQ, 100, 100, 100); 
}

static void light_state_cmd_cb(bool general_state, HALight* sender)
{
	Serial.print("Light state: ");
	Serial.println(general_state);

	if (general_state) {
#if PCA9536_PRESENT == 1
		io_exp.digitalWrite(LED_GENERAL, HIGH);
#endif
		buzzer_set(BUZZ_FREQ, 100, 0, 0); 
	} else {
#if PCA9536_PRESENT == 1
		io_exp.digitalWrite(LED_GENERAL, LOW);
#endif
		buzzer_set(BUZZ_FREQ, 100, 0, 0); 
	}
}

/* Function definitions *******************************************************/

static void set_speed(uint16_t speed)
{
	switch (speed) {
	case 1:
		digitalWrite(PIN_FAN2, LOW);
		digitalWrite(PIN_FAN3, LOW);
		digitalWrite(PIN_FAN1, HIGH);
		break;
	case 2:
		digitalWrite(PIN_FAN1, LOW);
		digitalWrite(PIN_FAN3, LOW);
		digitalWrite(PIN_FAN2, HIGH);
		break;
	case 3:
		digitalWrite(PIN_FAN1, LOW);
		digitalWrite(PIN_FAN2, LOW);
		digitalWrite(PIN_FAN3, HIGH);
		break;
	default:
		digitalWrite(PIN_FAN1, LOW);
		digitalWrite(PIN_FAN2, LOW);
		digitalWrite(PIN_FAN3, LOW);
		break;
	}
}

static void buzzer_set(uint8_t freq, uint16_t t1, uint16_t t2, uint16_t t3)
{
	analogWrite(PIN_BUZZ, freq);
	delay(t1);
	analogWrite(PIN_BUZZ, 0);
	delay(t2);
	analogWrite(PIN_BUZZ, freq);
	delay(t3);
	analogWrite(PIN_BUZZ, 0);
}

/* Setup and loop functions ***************************************************/

void setup()
{
	Serial.begin(9600);
	while (!Serial) {
		delay(10);
	}

	Serial.println("Home Assistant Arduino Aire");

	WiFi.begin(WIFI_SSID, WIFI_PASS);
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
	}
	Serial.println("Connected!");

	byte mac[WL_MAC_ADDR_LENGTH];
	WiFi.macAddress(mac);
	device.setUniqueId(mac, sizeof(mac));
	device.setName(DEVICE_NAME);
	device.setSoftwareVersion(FW_VERSION);

	fan.setName(FAN_NAME);
	fan.setSpeedRangeMin(1);
	fan.setSpeedRangeMax(3);
	fan.onStateCommand(fan_state_cmd_cb);
	fan.onSpeedCommand(fan_speed_cmd_cb);

	temp_sensor.setIcon("mdi:thermometer");
	temp_sensor.setName(TEMP_SENSOR_NAME);
	temp_sensor.setUnitOfMeasurement("°C");

	humd_sensor.setIcon("mdi:water-percent");
	humd_sensor.setName(HUMD_SENSOR_NAME);
	humd_sensor.setUnitOfMeasurement("%rH");

	press_sensor.setIcon("mdi:arrow-collapse-vertical");
	press_sensor.setName(PRESS_SENSOR_NAME);
	press_sensor.setUnitOfMeasurement("Pa");

	general_light.setName(GENERAL_LIGHT_NAME);
	general_light.onStateCommand(light_state_cmd_cb);

	mqtt.begin(MQTT_IP);

#if SHT4X_PRESENT == 1
	while (!sht4.begin()) {
		Serial.println("Couldn't find SHT4x");
		delay(1000);
		//TODO: led error and while(true)
	}
	Serial.print("Found SHT4x sensor. SN 0x");
	Serial.println(sht4.readSerial(), HEX);
	sht4.setPrecision(SHT4X_HIGH_PRECISION);
	sht4.setHeater(SHT4X_NO_HEATER);
#endif

#if BMP581_PRESENT == 1
	while (pressureSensor.beginI2C(BMP581_I2C_ADDRESS_DEFAULT) != BMP5_OK) {
		Serial.println("Couldn't find BMP581");
		delay(1000);
	}

	Serial.println("Found BMP581 sensor");

	// int8_t err = BMP5_OK;

    // // Configure the BMP581 to trigger interrupts whenever a measurement is performed
	// BMP581_InterruptConfig interruptConfig = {
	// 	.enable   = BMP5_INTR_ENABLE,    // Enable interrupts
	// 	.drive    = BMP5_INTR_PUSH_PULL, // Push-pull or open-drain
	// 	.polarity = BMP5_ACTIVE_HIGH,    // Active low or high
	// 	.mode     = BMP5_PULSED,         // Latch or pulse signal
	// 	.sources  = {
	// 		.drdy_en = BMP5_ENABLE,        // Trigger interrupts when data is ready
	// 		.fifo_full_en = BMP5_DISABLE,  // Trigger interrupts when FIFO is full
	// 		.fifo_thres_en = BMP5_DISABLE, // Trigger interrupts when FIFO threshold is reached
	// 		.oor_press_en = BMP5_DISABLE,  // Trigger interrupts when pressure goes out of range
	// 	}
	// };
    // err = pressureSensor.setInterruptConfig(&interruptConfig);
    // if(err != BMP5_OK)
    // {
    //     // Interrupt settings failed, most likely a communication error (code -2)
    //     Serial.print("Interrupt settings failed! Error code: ");
    //     Serial.println(err);
    // }

	attachInterrupt(digitalPinToInterrupt(PIN_INT_BMP581), bmp581_cb, RISING);

#endif

#if PCA9536_PRESENT == 1
	Wire.begin();

	while (!io_exp.begin()) {
		Serial.println("Couldn't find PCA9536");
		delay(1000);
		//TODO: led error and while(true)
	}
	Serial.println("Found PCA9536");

	io_exp.pinMode(PIN_LED_RED, OUTPUT);
	io_exp.pinMode(PIN_LED_YELLOW, OUTPUT);
	io_exp.pinMode(PIN_LED_GREEN, OUTPUT);
	io_exp.digitalWrite(LED_POWER, LOW);
	io_exp.digitalWrite(LED_GENERAL, LOW);
	io_exp.digitalWrite(PIN_LED_GREEN, LOW);
#endif

	pinMode(PIN_FAN1, OUTPUT);
	pinMode(PIN_FAN2, OUTPUT);
	pinMode(PIN_FAN3, OUTPUT);
	pinMode(PIN_BUZZ, OUTPUT);
	pinMode(PIN_BTN_USR, INPUT_PULLUP);

	digitalWrite(LED_POWER, LOW);
	digitalWrite(LED_GENERAL, LOW);
	digitalWrite(PIN_LED_GREEN, LOW);
}

void loop()
{
	mqtt.loop();
#if SHT4X_PRESENT == 1
	if ((millis() - last_time) > TEMP_PERIOD) {
		sensors_event_t sht_humd, sht_temp;
		sht4.getEvent(&sht_humd, &sht_temp);
		// temp_sensor.setValue(sht_temp.temperature);
		// humd_sensor.setValue(sht_humd.relative_humidity);
		last_time = millis();
		Serial.print(sht_temp.temperature);
		Serial.print("ºC ");
		Serial.print(sht_temp.relative_humidity);
		Serial.println("%rH");
	}
#endif

#if BMP581_PRESENT == 1
	if (bmp581_ready) {
		bmp581_ready = 0;

		uint8_t interrupt_status = 0;
		int err = pressureSensor.getInterruptStatus(&interrupt_status);
		if (err != BMP5_OK) {
			Serial.print("BMP581 error: ");
			Serial.println(err);
		}

		if (interrupt_status & BMP5_INT_ASSERTED_DRDY) {
			bmp5_sensor_data data = {0, 0};
			int8_t err = pressureSensor.getSensorData(&data);
			if (err == BMP5_OK) {
				Serial.println(data.pressure);
				Serial.print(" Pa ");
				Serial.print(data.temperature);
				Serial.println(" ºC");
			} else {
				Serial.print("BMP581 error: ");
				Serial.println(err);
			}
		}
	}
#endif

	if (digitalRead(PIN_BTN_USR) == 0) {
		usr_btn_cb();
	}
}