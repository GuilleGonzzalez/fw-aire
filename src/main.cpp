#include <Arduino.h>
#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include <ArduinoOTA.h>
#include <ArduinoHA.h>
#include <Wire.h>
#include "Adafruit_SHT4x.h"
#include "SparkFun_PCA9536_Arduino_Library.h"
#include "SparkFun_BMP581_Arduino_Library.h"

#include "aire_boardv3.h"

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
#define PRESS_SENSOR_NAME  "Press Guille"
#define GENERAL_LIGHT_NAME "General Light"
#define FW_VERSION         "0.1.0"
#define FAN_ID             "fan_guille"
#define TEMP_SENSOR_ID     "temp_fan_guille"
#define HUMD_SENSOR_ID     "humd_fan_guille"
#define PRESS_SENSOR_ID    "press_fan_guille"
#define GENERAL_LIGHT_ID   "general_light"
#define TEMP_PERIOD        60000 // ms
#define BUZZ_FREQ          220

#define LED_BRIGHT_LOW 240
#define LED_OFF        255

WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device);
HAFan fan(FAN_ID, HAFan::SpeedsFeature);
HASensorNumber temp_sensor(TEMP_SENSOR_ID, HASensorNumber::PrecisionP2);
HASensorNumber humd_sensor(HUMD_SENSOR_ID, HASensorNumber::PrecisionP2);
HASensorNumber press_sensor(PRESS_SENSOR_ID, HASensorNumber::PrecisionP2);
HALight general_light(GENERAL_LIGHT_ID);

// HAHVAC hvac("Guille HVAC", HAHVAC::PowerFeature | HAHVAC::TargetTemperatureFeature | HAHVAC::FanFeature);

#if I2C_PRESENT
TwoWire i2c = TwoWire(0);
#endif

#if SHT4X_PRESENT == 1
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
#endif

#if BMP581_PRESENT == 1
BMP581 pressureSensor;
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

// static void hvac_power_cmd_cb(bool state, HAHVAC* sender);
// static void hvac_fan_mode_cmd_cb(HAHVAC::FanMode mode, HAHVAC* sender);
// static void hvac_target_temperature_cmd_cb(HANumeric temperature, HAHVAC* sender);

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
		analogWrite(LED_POWER, LED_BRIGHT_LOW);
		set_speed(fan_speed);
		buzzer_set(BUZZ_FREQ, 100, 100, 300); 
	} else {
		analogWrite(LED_POWER, LED_OFF);
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
		analogWrite(LED_GENERAL, LED_BRIGHT_LOW);
		buzzer_set(BUZZ_FREQ, 100, 0, 0); 
	} else {
		analogWrite(LED_GENERAL, LED_OFF);
		buzzer_set(BUZZ_FREQ, 100, 0, 0); 
	}
}

// static void hvac_power_cmd_cb(bool state, HAHVAC* sender)
// {
// 	if (state) {
// 		Serial.println("HVAC power on");
// 	} else {
// 			Serial.println("HVAC power off");
// 	}
// }

// static void hvac_fan_mode_cmd_cb(HAHVAC::FanMode mode, HAHVAC* sender)
// {
// 	Serial.print("Fan mode: ");
// 	Serial.println(mode);
// 	sender->setFanMode(mode);
// }

// static void hvac_target_temperature_cmd_cb(HANumeric temperature, HAHVAC* sender)
// {
// 	Serial.print("Target temp: ");
// 	Serial.println(temperature.toFloat());
// 	sender->setTargetTemperature(temperature);

// }

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
	pinMode(PIN_FAN1, OUTPUT);
	pinMode(PIN_FAN2, OUTPUT);
	pinMode(PIN_FAN3, OUTPUT);
	pinMode(PIN_BUZZ, OUTPUT);
	pinMode(LED_POWER, OUTPUT);
	pinMode(LED_GENERAL, OUTPUT);
	pinMode(PIN_LED_RED, OUTPUT);
	pinMode(PIN_BTN_USR, INPUT_PULLUP);

	analogWrite(LED_POWER, LED_OFF);
	analogWrite(LED_GENERAL, LED_OFF);
	analogWrite(PIN_LED_RED, LED_OFF);

	Serial.begin(9600);
	while (!Serial) {
		delay(10);
	}

	Serial.println("Home Assistant Arduino Aire");

	WiFi.begin(WIFI_SSID, WIFI_PASS);
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
	}
	Serial.println("");
	Serial.print("Connected to WiFi network with IP Address: ");
	Serial.println(WiFi.localIP());

	byte mac[6];
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

	// hvac.onPowerCommand(hvac_power_cmd_cb);
	// hvac.onFanModeCommand(hvac_fan_mode_cmd_cb);
	// hvac.onTargetTemperatureCommand(hvac_target_temperature_cmd_cb);
	// hvac.setName("My HVAC");
	// hvac.setMinTemp(17);
	// hvac.setMaxTemp(30);
	// hvac.setTempStep(1);

	mqtt.begin(MQTT_IP);

	ArduinoOTA
		.onStart([]() {
			String type;
			if (ArduinoOTA.getCommand() == U_FLASH) {
				type = "sketch";
			} else { // U_SPIFFS
				type = "filesystem";
			}
			// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
			Serial.println("Start updating " + type);
		})
		.onEnd([]() {
			Serial.println("\nEnd");
		})
		.onProgress([](unsigned int progress, unsigned int total) {
			Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
		})
		.onError([](ota_error_t error) {
			Serial.printf("Error[%u]: ", error);
	});

	ArduinoOTA.begin();

#if I2C_PRESENT
	i2c.begin(PIN_I2C_SDA, PIN_I2C_SCL, 1e5);
#endif

#if SHT4X_PRESENT == 1
	while (!sht4.begin(&i2c)) {
		Serial.println("Couldn't find SHT4x");
		delay(1000);
	}
	Serial.print("Found SHT4x sensor. SN 0x");
	Serial.println(sht4.readSerial(), HEX);
	sht4.setPrecision(SHT4X_HIGH_PRECISION);
	sht4.setHeater(SHT4X_NO_HEATER);
#endif

#if BMP581_PRESENT == 1
	while (pressureSensor.beginI2C(BMP581_I2C_ADDRESS_SECONDARY, i2c) != BMP5_OK) {
		Serial.println("Couldn't find BMP581");
		delay(1000);
	}

	Serial.println("Found BMP581 sensor");

	attachInterrupt(digitalPinToInterrupt(PIN_INT_BMP581), bmp581_cb, RISING);
#endif
}

void loop()
{
	mqtt.loop();
	ArduinoOTA.handle();

#if SHT4X_PRESENT == 1
	if ((millis() - last_time) > TEMP_PERIOD) {
		sensors_event_t sht_humd, sht_temp;
		sht4.getEvent(&sht_humd, &sht_temp);
		temp_sensor.setValue(sht_temp.temperature);
		humd_sensor.setValue(sht_humd.relative_humidity);
		last_time = millis();
		Serial.print(sht_temp.temperature);
		Serial.print("ºC ");
		Serial.print(sht_humd.relative_humidity);
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
				press_sensor.setValue(data.pressure);
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