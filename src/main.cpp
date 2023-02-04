/********************************************************
*
* Volvo Penta MD2030 Data Collector
*
********************************************************/

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
//#include <N2kMessages.h>
//#include <NMEA2000_esp32.h>

#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/analogvoltage.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/voltagedivider.h"
#include "sensesp/transforms/moving_average.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/system/lambda_consumer.h"

/* PIN definition :
*       - 1-Wire data pin on SH-ESP32. Using DS18B20 sensors
*       - Fuel Gauge
*       - Engine Coolant Sensor
*       - RPM value coming from alternator
*       - Oled display
*/

/***************************
* PIN Definition for DFRobot Firebeetle board
***************************/
/*
#define ONEWIRE_PIN 17
#define FUEL_GAUGE_PIN 34
#define ENGINE_COOLANT_PIN 36
#define RPM_PIN 16
#define SDA_PIN 21
#define SCL_PIN 22
*/

/***************************
* PIN Definition for DFRobot Beetle board
***************************/
#define ONEWIRE_PIN 13        // Pin D7
#define FUEL_GAUGE_PIN 34     // PIN A2
#define ENGINE_COOLANT_PIN 35 // PIN A3
#define RPM_PIN 27            // PIN D4
#define SDA_PIN 21
#define SCL_PIN 22


/// Screen definition
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// define temperature display units
#define TEMP_DISPLAY_FUNC KelvinToCelsius

using namespace sensesp;

// Initiate I2C : used for display and BMP280
TwoWire* i2c;
Adafruit_SSD1306* display;
Adafruit_BMP280 bmp280;
const float Vin = 3.3;

/// Read functions for BMP280
float read_temp_callback() { return (bmp280.readTemperature() + 273.15); }
float read_pressure_callback() { return (bmp280.readPressure()); }

/// Clear a text row on an Adafruit graphics display
void ClearRow(int row) { display->fillRect(0, 8 * row, SCREEN_WIDTH, 8, 0); }

float KelvinToCelsius(float temp) { return temp - 273.15; }

/// Display function
void displayData(int row, String title, float value) {
  ClearRow(row);
  display->setCursor(0, 8 * row);
  display->printf("%s: %.1f", title.c_str(), TEMP_DISPLAY_FUNC(value));
  display->display();
}

// Fuel Tank Capacity interpreter
class TankCapacityInterpreter : public CurveInterpolator {
 public:
  TankCapacityInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate measure into a %
    clear_samples();
    add_sample(CurveInterpolator::Sample(0, 0.0));
    add_sample(CurveInterpolator::Sample(36, 0.0));
    add_sample(CurveInterpolator::Sample(40, 0.0)); 
    add_sample(CurveInterpolator::Sample(44, 0.0));  // To that point : artificial values 
    add_sample(CurveInterpolator::Sample(200, 0.3448));
    add_sample(CurveInterpolator::Sample(355, 0.125));
    add_sample(CurveInterpolator::Sample(433, 0.25));
    add_sample(CurveInterpolator::Sample(466, 0.375));
    add_sample(CurveInterpolator::Sample(522, 0.5));
    add_sample(CurveInterpolator::Sample(568, 0.625));
    add_sample(CurveInterpolator::Sample(588, 0.75));
    add_sample(CurveInterpolator::Sample(637.0, 0.875));
    add_sample(CurveInterpolator::Sample(675.0, 1));
    add_sample(CurveInterpolator::Sample(680.0, 1)); // From that point : artificial values 
    add_sample(CurveInterpolator::Sample(685.0, 1));
    add_sample(CurveInterpolator::Sample(1000.0, 1));
  }
};

// Fuel Consomption interpreter based on engine RPM
// Provide fuel consumption related to RPM
// Source : D1-30 engine data from https://www.boat-fuel-economy.com/volvo-penta-diesel-d1-30-fuel-consumption-liters
class FuelInterpreter : public CurveInterpolator {
 public:
  FuelInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate RPM to LPH
    clear_samples();
    // addSample(CurveInterpolator::Sample(RPM, LPH));
    add_sample(CurveInterpolator::Sample(500, 0.4));
    add_sample(CurveInterpolator::Sample(1000, 0.7));
    add_sample(CurveInterpolator::Sample(1500, 1.1));
    add_sample(CurveInterpolator::Sample(1800, 1.5));
    add_sample(CurveInterpolator::Sample(2000, 1.9));
    add_sample(CurveInterpolator::Sample(2200, 2.4));
    add_sample(CurveInterpolator::Sample(2400, 2.85));
    add_sample(CurveInterpolator::Sample(2600, 3.5));
    add_sample(CurveInterpolator::Sample(2800, 4.45));
    add_sample(CurveInterpolator::Sample(3000, 5.5));
    add_sample(CurveInterpolator::Sample(3200, 6.6));
    add_sample(CurveInterpolator::Sample(3400, 7.6));
    add_sample(CurveInterpolator::Sample(3800, 10.0));  
  }
};

// Coolant Temperature interpreter based on sensor FAE 31020
class TemperatureInterpreter : public CurveInterpolator {
 public:
  TemperatureInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the ohm values returned by
    // our temperature sender to degrees Kelvin
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownOhmValue, knownKelvin));
    add_sample(CurveInterpolator::Sample(40, 393.15));
    add_sample(CurveInterpolator::Sample(70, 383.15));
    add_sample(CurveInterpolator::Sample(80, 373.15));
    add_sample(CurveInterpolator::Sample(92, 363.15));
    add_sample(CurveInterpolator::Sample(110, 353.15));
    add_sample(CurveInterpolator::Sample(140, 343.15));
    add_sample(CurveInterpolator::Sample(165, 333.15));
    add_sample(CurveInterpolator::Sample(210, 323.15));
    add_sample(CurveInterpolator::Sample(300, 313.15)); 
  }
};

#define SERIAL_DEBUG_DISABLED 1

reactesp::ReactESP app;

void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif


/***************************************************************************
* Build SensESP
***************************************************************************/
SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("StelamayaMD2030")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
//                    ->set_wifi("Stelamaya", "@Helios01!")
                    ->set_wifi("Livebox-Bignon", "0123456789ABCDEF9876543210")
//                    ->set_sk_server("stelamayarpi4.local", 3000)
//                    ->set_sk_server("themis-m93p.local", 3000)
                    ->set_sk_server("192.168.1.59", 3000)

                    ->enable_uptime_sensor()
                    ->get_app();


/***************************************************************************
* initialize the display
***************************************************************************/
  i2c = new TwoWire(0);
  i2c->begin(SDA_PIN, SCL_PIN);

  display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, i2c, -1);
  if (!display->begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 not found !"));
  } else {
    delay(100);
    display->clearDisplay();
    display->setTextSize(1);
    display->setTextColor(SSD1306_WHITE);
    display->setCursor(0, 0);
    display->printf("Host: %s", sensesp_app->get_hostname().c_str());
//    display->printf("IP: %s",WiFi.localIP().toString().c_str());
    display->display();
    delay(1000);
  }
/***************************************************************************
* BMP280 Sensor : to get environment : temperature, pressure
***************************************************************************/

/*
//if (!bmp280.begin(0x77)){
if (!bmp280.begin()){
    display->setCursor(0, 16);
    display->printf("%s", "BMP280 not found !");
    display->display();
} else {
auto* main_engine_bay_temperature = new RepeatSensor<float>(5000, read_temp_callback);
auto* main_engine_bay_pressure = new RepeatSensor<float>(60000, read_pressure_callback);

auto main_engine_bay_temperature_metadata =
    new SKMetadata("K",             // units
        "Engine Bay Temperature",   // display name
        "Engine Bay Temperature",   // description
        "Bay Temperature",          // short name
        10.                         // timeout, in seconds
    );
main_engine_bay_temperature
    ->connect_to(new SKOutputFloat("propulsion.engineBay.temperature", "/mainEngineBayTemperature/skPath", main_engine_bay_temperature_metadata))
    ->connect_to(new LambdaConsumer<float>([](float temperature) { displayData(6, "Bay T: ", temperature); }));

auto main_engine_bay_pressure_metadata =
    new SKMetadata("Pa",            // units
        "Engine Bay Pressure",      // display name
        "Engine Bay Pressure",      // description
        "Bay Pressure",             // short name
        10.                         // timeout, in seconds
    );

main_engine_bay_pressure
    ->connect_to(new SKOutputFloat("propulsion.engineBay.pressure", "/mainEngineBayPressure/skPath", main_engine_bay_pressure_metadata))
    ->connect_to(new LambdaConsumer<float>([](float pressure) { displayData(8, "Bay P: ", pressure); }));
}
*/

/***************************************************************************
* FAE31020 Sensor : Engine Coolant Temp Config
***************************************************************************/

/*
auto* main_engine_coolant_temperature = new AnalogInput(ENGINE_COOLANT_PIN, 5000);
const float Rt1 = 1000.0;

// Function to get the temperature based on the FAE31020 thermistor 
auto getTemperatureFromResistance_function = [](int res) -> float{
    //float Temp=-23.6*log(res)+204.7; // Got with Arduino but never worked here
    float Temp=-37.7*log(res)+547.35;
    return Temp;
};

auto getTemperatureFromResistance_transform = new LambdaTransform<int, float>(getTemperatureFromResistance_function);

auto main_engine_coolant_temperature_metadata =
    new SKMetadata("K",                 // units
                "Coolant Temperature",  // display name
                "Coolant Temperature",  // description
                "Coolant Temperature",  // short name
                10.                     // timeout, in seconds
    );

main_engine_coolant_temperature
    //->connect_to(new AnalogVoltage()) // Never succeed in getting valuable values using these transformations
    //->connect_to(new VoltageDividerR2(Rt1, Vin, "/EngineTemperature/sender"))
    ->connect_to(getTemperatureFromResistance_transform)
    ->connect_to(new SKOutputFloat("propulsion.engine.coolantTemperature", "/mainEngineCoolantTemperature/sk_path", main_engine_coolant_temperature_metadata)) // send to SignalK
    ->connect_to(new LambdaConsumer<float>([](float temperature) { displayData(7, "Coolant", temperature); })); // send to display
*/


/***************************************************************************
* DS18B20 Temperatures : exhaust gaz and various
***************************************************************************/

DallasTemperatureSensors* dts = new DallasTemperatureSensors(ONEWIRE_PIN);

// define three 1-Wire temperature sensors that update every 1000 ms
// and have specific web UI configuration paths
// define metadata for sensors

auto main_engine_exhaust_temperature =
      new OneWireTemperature(dts, 5000, "/mainEngineWetExhaustTemp/oneWire");
auto main_engine_exhaust_temperature_metadata =
    new SKMetadata("K",                     // units
                "Wet Exhaust Temperature",  // display name
                "Wet Exhaust Temperature",  // description
                "Exhaust Temperature",      // short name
                10.                         // timeout, in seconds
    );
// propulsion.*.wetExhaustTemperature is a non-standard path
main_engine_exhaust_temperature
    ->connect_to(new SKOutputFloat("propulsion.engine.exhaustTemperature", "/mainEngineExhaustTemperature/skPath", main_engine_exhaust_temperature_metadata))
    ->connect_to(new LambdaConsumer<float>([](float temperature) { displayData(5, "Exhaust", temperature); }));


/*=============================================================*/
/* Alternative to BMP280 sensor using DS18B20
auto main_engine_bay_temperature =
      new OneWireTemperature(dts, 5000, "/mainEngineBayTemp/oneWire");
auto main_engine_bay_temperature_metadata =
    new SKMetadata("K",                     // units
                "Engine Bay Temperature",   // display name
                "Engine Bay Temperature",   // description
                "Bay Temperature",          // short name
                10.                         // timeout, in seconds
    );
main_engine_bay_temperature
    ->connect_to(new SKOutputFloat("propulsion.engineBay.temperature", "/mainEngineBayTemperature/skPath", main_engine_bay_temperature_metadata))
    ->connect_to(new LambdaConsumer<float>([](float temperature) { displayData(6, "Engine Bay", temperature); }));
*/

/*=============================================================*/
/* Alternative to FAE31020 sensor using DS18B20
auto main_engine_coolant_temperature =
      new OneWireTemperature(dts, 5000, "/mainCoolantTemp/oneWire");
auto main_engine_coolant_temperature_metadata =
    new SKMetadata("K",                         // units
                "Coolant liquid Temperature",   // display name
                "Coolant liquid Temperature",   // description
                "Coolant Temperature",          // short name
                10.                             // timeout, in seconds
    );
main_engine_coolant_temperature
    ->connect_to(new SKOutputFloat("propulsion.engine.coolantTemperature", "/mainEngineCoolantTemperature/skPath", main_engine_coolant_temperature_metadata))
    ->connect_to(new LambdaConsumer<float>([](float temperature) { displayData(7, "Coolant", temperature); }));
*/


/***************************************************************************
* Tank remaining capacity using fuel gauge
****************************************************************************/

auto* main_engine_tank_currentVolume = new AnalogInput(FUEL_GAUGE_PIN, 500);
const float Rc1 = 100;
const float CAPACITY = 90.0; // Capacity in 1000xLiters
//auto* Capacity=new Configurable("Diesel Tank Capacity", "Capacity of Diesel tank in Liters");

auto main_engine_tank_level_metadata =
    new SKMetadata("L",                   // units
                "Diesel Tank remaining Level",  // display name
                "Diesel Tank remaining Level",  // description
                "Tank Level",         // short name
                10.                    // timeout, in seconds
    );


main_engine_tank_currentVolume
    ->connect_to(new TankCapacityInterpreter("/FuelTank/capacity/curve"))
    ->connect_to(new Linear(CAPACITY,0))
    ->connect_to(new SKOutputFloat("self.tanks.fuel.main.currentVolume", "/mainEngineTankCapacity/skPath", main_engine_tank_level_metadata))
    ->connect_to(new LambdaConsumer<float>([](float capacity) { displayData(2, "Tank Capacity", capacity+273.15); })); // add manually 273.15 because it is not a temperature.



/***************************************************************************
* Engine RPM : get value from alternator rotation speed
***************************************************************************/

auto* main_engine_rpm = new DigitalInputCounter(RPM_PIN, INPUT_PULLUP, RISING, 500);
float multiplier = 1.0;

auto main_engine_rpm_metadata =
    new SKMetadata("rpm",                   // units
                "Engine rotation speed",    // display name
                "Engine rotation speed",    // description
                "Engine Speed",             // short name
                10.                         // timeout, in seconds
    );
auto main_engine_consumption_metadata =
    new SKMetadata("L/H",                   // units
                "Engine fuel consumption",  // display name
                "Engine fuel consumption",  // description
                "Fuel consumption",         // short name
                10.                         // timeout, in seconds
    );

// Calculate engine speed (RPM)
main_engine_rpm
    ->connect_to(new Frequency(multiplier, "/Engine RPM/calibrate"))  
    ->connect_to(new SKOutputFloat("propulsion.engine.revolutions", "/mainEngineRPM/skPath", main_engine_rpm_metadata))
    ->connect_to(new LambdaConsumer<int>([](float speed) { displayData(3, "Engine RPM", speed+273.15); })); // add manually 273.15 because it is not a temperature.
    
// Calculate fuel consumption
main_engine_rpm
    ->connect_to(new Frequency(6))
    // times by 6 to go from Hz to RPM
    ->connect_to(new FuelInterpreter("/Engine Fuel/curve"))
    ->connect_to(new SKOutputFloat("propulsion.engine.fuel.rate", "/mainEngineConsumption/sk_path", main_engine_consumption_metadata))
    ->connect_to(new LambdaConsumer<int>([](float consumption) { displayData(4, "Consumption", consumption+273.15); })); // add manually 273.15 because it is not a temperature.



/***************************************************************************
* Starting Application
***************************************************************************/

sensesp_app->start();
}

// main program loop
void loop() {
    app.tick(); 
}

