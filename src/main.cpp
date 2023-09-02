#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoHA.h>
#include "Adafruit_SHT4x.h"

#include "aire_boardv1.h"
// #include "aire_boardv2.h"

#define WIFI_SSID          "Gonmar-Livebox"
#define WIFI_PASS          "618995151609549464"
#define MQTT_IP            "192.168.1.100"
#define DEVICE_NAME        "Aire Guille Arduino"
#define FAN_NAME           "Fan Guille"
#define TEMP_SENSOR_NAME   "Temp Guille"
#define HUMD_SENSOR_NAME   "Humd Guille"
#define GENERAL_LIGHT_NAME "General Light"
#define FW_VERSION         "0.1.0"
#define FAN_ID             "fan_guille"
#define TEMP_SENSOR_ID     "temp_fan_guille"
#define HUMD_SENSOR_ID     "humd_fan_guille"
#define GENERAL_LIGHT_ID   "general_light"
#define TEMP_PERIOD        10000 // ms
#define BUZZ_FREQ          220

WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device);
HAFan fan(FAN_ID, HAFan::SpeedsFeature);
HASensorNumber temp_sensor(TEMP_SENSOR_ID, HASensorNumber::PrecisionP2);
HASensorNumber humd_sensor(HUMD_SENSOR_ID, HASensorNumber::PrecisionP2);
HALight general_light(GENERAL_LIGHT_ID);

#if SHT4X_PRESENT == 1
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
#endif

/* Global variables ***********************************************************/

static unsigned long last_time = 0;
static bool state = false;
static uint16_t fan_speed = 1;
static bool yellow_led_state = false;

/* Function prototypes ********************************************************/

static void usr_btn_cb();
static void fan_state_cmd_cb(bool l_state, HAFan* sender);
static void fan_speed_cmd_cb(uint16_t speed, HAFan* sender);
static void light_state_cmd_cb(bool state, HALight* sender);

static void set_speed(uint16_t speed);
static void buzzer_set(uint8_t freq, uint16_t t1, uint16_t t2, uint16_t t3);

/* Callbacks ******************************************************************/

static void usr_btn_cb()
{
	yellow_led_state = !yellow_led_state;
	digitalWrite(PIN_LED_YELLOW, yellow_led_state);
}

static void fan_state_cmd_cb(bool l_state, HAFan* sender)
{
	state = l_state;

	Serial.print("Fan state: ");
	Serial.println(state);

	sender->setState(state); // Report state back to the Home Assistant

	if (state) {
		digitalWrite(LED_POWER, HIGH);
		set_speed(fan_speed);
		buzzer_set(BUZZ_FREQ, 100, 100, 300); 
	} else {
		digitalWrite(LED_POWER, LOW);
		set_speed(0);
		buzzer_set(BUZZ_FREQ, 300, 100, 100); 
	}
}

static void fan_speed_cmd_cb(uint16_t speed, HAFan* sender)
{
	fan_speed = speed;

	Serial.print("Fan speed: ");
	Serial.println(fan_speed);

	sender->setSpeed(fan_speed); // Report speed back to the Home Assistant

	if (!state) { 
		return;
	}
	set_speed(fan_speed);
	buzzer_set(BUZZ_FREQ, 100, 100, 100); 
}

static void light_state_cmd_cb(bool state, HALight* sender)
{
	Serial.print("Light state: ");
	Serial.println(state);

	if (state) {
		digitalWrite(LED_GENERAL, HIGH);
		buzzer_set(BUZZ_FREQ, 50, 50, 10); 
	} else {
		digitalWrite(LED_GENERAL, LOW);
		set_speed(0);
		buzzer_set(BUZZ_FREQ, 100, 50, 50); 
	}

	sender->setState(state); // Report state back to the Home Assistant
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
		delay(500); // Waiting for the connection
	}
	Serial.println("Connected!");

#if SHT4X_PRESENT == 1
	while (!sht4.begin()) {
		Serial.println("Couldn't find SHT4x");
		delay(1000);
	}
	Serial.print("Found SHT4x sensor. SN 0x");
	Serial.println(sht4.readSerial(), HEX);
	sht4.setPrecision(SHT4X_HIGH_PRECISION);
	sht4.setHeater(SHT4X_NO_HEATER);
#endif

	byte mac[WL_MAC_ADDR_LENGTH];
	WiFi.macAddress(mac);
	device.setUniqueId(mac, sizeof(mac));
	device.setName(DEVICE_NAME);
	device.setSoftwareVersion(FW_VERSION);

	pinMode(PIN_LED_RED, OUTPUT);
	pinMode(PIN_LED_YELLOW, OUTPUT);
	//pinMode(PIN_LED_GREEN, OUTPUT);
	pinMode(PIN_FAN1, OUTPUT);
	pinMode(PIN_FAN2, OUTPUT);
	pinMode(PIN_FAN3, OUTPUT);
	pinMode(PIN_BUZZ, OUTPUT);
	pinMode(PIN_BTN_USR, INPUT_PULLUP);
 
	(void) usr_btn_cb;
	// attachInterrupt(digitalPinToInterrupt(PIN_BTN_USR), usr_btn_cb, FALLING);
	
	digitalWrite(LED_POWER, LOW);
	digitalWrite(LED_GENERAL, LOW);
	//digitalWrite(PIN_LED_GREEN, LOW);

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

	general_light.setName(GENERAL_LIGHT_NAME);
	general_light.onStateCommand(light_state_cmd_cb);

	mqtt.begin(MQTT_IP);
}

void loop()
{
	mqtt.loop();
#if SHT4X_PRESENT == 1
	if ((millis() - last_time) > TEMP_PERIOD) {
		sensors_event_t sht_humd, sht_temp;
		sht4.getEvent(&sht_humd, &sht_temp);
		temp_sensor.setValue(sht_temp.temperature);
		humd_sensor.setValue(sht_humd.relative_humidity);
		last_time = millis();
	}
#endif
}