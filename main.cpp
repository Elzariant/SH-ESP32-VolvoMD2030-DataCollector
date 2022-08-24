/********************************************************
*
* Volvo Penta MD2030 Data Collector
*
********************************************************/


#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
//#include <N2kMessages.h>
#include <NMEA2000_esp32.h>

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

using namespace sensesp;

class TankCapacityInterpreter : public CurveInterpolator {
 public:
  TankCapacityInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the ohm values returned by
    // our temperature sender to degrees Kelvin
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownOhmValue, knownCapacity%));
    add_sample(CurveInterpolator::Sample(9.6, 1));
    add_sample(CurveInterpolator::Sample(11.5, 0.75));
    add_sample(CurveInterpolator::Sample(14.1, 0.50));
    add_sample(CurveInterpolator::Sample(18.2, 0.25));
    add_sample(CurveInterpolator::Sample(32.2, 0.0));
  }
};

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
    add_sample(CurveInterpolator::Sample(3400, 7.2));
    add_sample(CurveInterpolator::Sample(3800, 7.4));  
  }
};

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

// 1-Wire data pin on SH-ESP32
#define ONEWIRE_PIN 17
#define FUEL_GAUGE_PIN 36
#define ENGINE_COOLANT_PIN 34
#define RPM_PIN 16

// define temperature display units
#define TEMP_DISPLAY_FUNC KelvinToCelsius
//#define TEMP_DISPLAY_FUNC KelvinToFahrenheit

ReactESP app;

void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

 // SensESPAppBuilder builder;

  //sensesp_app = builder.set_hostname("temperatures")->get_app();


SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("StelamayaMD2030")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    ->set_wifi("Stelamaya", "@Helios01!")
                    ->set_sk_server("stelamayarpi4.local", 3000)
                    ->enable_uptime_sensor()
                    ->get_app();


// DS18B20 Temperatures

DallasTemperatureSensors* dts = new DallasTemperatureSensors(ONEWIRE_PIN);

// define three 1-Wire temperature sensors that update every 1000 ms
// and have specific web UI configuration paths

auto main_engine_exhaust_temperature =
      new OneWireTemperature(dts, 10000, "/mainEngineWetExhaustTemp/oneWire");
auto main_engine_bay_temperature =
      new OneWireTemperature(dts, 10000, "/mainEngineBayTemp/oneWire");


// define metadata for sensors

auto main_engine_bay_temperature_metadata =
    new SKMetadata("K",                   // units
                "Engine Bay Temperature",  // display name
                "Engine Bay Temperature",  // description
                "Bay Temperature",         // short name
                10.                    // timeout, in seconds
    );
auto main_engine_exhaust_temperature_metadata =
    new SKMetadata("K",                        // units
                "Wet Exhaust Temperature",  // display name
                "Wet Exhaust Temperature",  // description
                "Exhaust Temperature",      // short name
                10.                         // timeout, in seconds
    );

// connect the sensors to Signal K output paths

main_engine_bay_temperature->connect_to(
    new SKOutputFloat(
        "propulsion.main.bay.temperature",
        "/mainEngineTemp/skPath",
        main_engine_bay_temperature_metadata
    )
);
  
// propulsion.*.wetExhaustTemperature is a non-standard path
main_engine_exhaust_temperature->connect_to(
    new SKOutputFloat(
        "propulsion.main.wetExhaustTemperature",
        "/mainEngineWetExhaustTemp/skPath",
        main_engine_exhaust_temperature_metadata
    )
);
  

/// Engine Coolant Temp Config ////

auto* main_engine_coolant_temperature = new AnalogInput(ENGINE_COOLANT_PIN, 2000);
const float Vin = 3.3;
const float R1 = 1000.0;

auto main_engine_coolant_temperature_metadata =
    new SKMetadata("K",                        // units
                "Coolant Temperature",  // display name
                "Coolant Temperature",  // description
                "Coolant Temperature",      // short name
                10.                         // timeout, in seconds
    );
main_engine_coolant_temperature
    ->connect_to(new AnalogVoltage())
    ->connect_to(new VoltageDividerR2(R1, Vin, "/Engine Temp/sender"))
    //->connect_to(new TemperatureInterpreter("/Engine Temp/curve"))
    //->connect_to(new Linear(1.0, 0.0, "/Engine Temp/calibrate"))
    ->connect_to(new SKOutputFloat(
        "propulsion.main.coolant.temperature",
        "/Engine Temp/sk_path",
        main_engine_coolant_temperature_metadata
    )
);



/// Fuel Gauge ////

auto* main_engine_tank_capacity = new AnalogInput(FUEL_GAUGE_PIN, 1000);
const float R2 = 100;
const float CAPACITY = 90; // Capacity in 1000xLiters

auto main_engine_tank_capacity_metadata =
    new SKMetadata("L",                   // units
                "Diesel Tank Capacity",  // display name
                "Diesel Tank Capacity",  // description
                "Tank Capacity",         // short name
                10.                    // timeout, in seconds
    );

main_engine_tank_capacity
    ->connect_to(new AnalogVoltage())
    ->connect_to(new VoltageDividerR2(R2, Vin, "/FuelTank/capacity/sender"))
    ->connect_to(new TankCapacityInterpreter("/FuelTank/capacity/curve"))
    ->connect_to(new Linear(CAPACITY, 0.0, "/FuelTank/capacity/calibrate")) // 90L
    ->connect_to(new SKOutputInt(
            "propulsion.main.fuelTankCapacity.currentLevel",
            "/mainEngineTankCapacity/skPath",
            main_engine_tank_capacity_metadata
        )
    );


//RPM Application/////

auto* main_engine_rpm = new DigitalInputCounter(RPM_PIN, INPUT_PULLUP, RISING, 500);
const char* config_path_calibrate = "/Engine RPM/calibrate";
const char* config_path_skpath = "/mainEngineRPM/sk_path";
const float multiplier = 1.0;

auto main_engine_rpm_metadata =
    new SKMetadata("rpm",                   // units
                "Engine rotation speed",  // display name
                "Engine rotation speed",  // description
                "Engine Speed",         // short name
                10.                    // timeout, in seconds
    );
auto main_engine_consumption_metadata =
    new SKMetadata("L/H",                   // units
                "Engine fuel consumption",  // display name
                "Engine fuel consumption",  // description
                "Fuel consumption",         // short name
                10.                    // timeout, in seconds
    );

// Calculate engine speed (RPM)
main_engine_rpm
    ->connect_to(new Frequency(multiplier, config_path_calibrate))  
    // connect the output of sensor to the input of Frequency()
    ->connect_to(
        new SKOutputFloat(
           "propulsion.main.revolutions",
            "/mainEngineRPM/skPath",
            main_engine_rpm_metadata
        )
    );  
    
// Calculate fuel consumption
main_engine_rpm
    ->connect_to(new Frequency(6))
    // times by 6 to go from Hz to RPM
    ->connect_to(new FuelInterpreter("/Engine Fuel/curve"))
    ->connect_to(
        new SKOutputFloat(
            "propulsion.engine.fuelconsumption",
            "/Engine_consumption/sk_path",
            main_engine_consumption_metadata
        )
    );                                       



sensesp_app->start();
}

// main program loop
void loop() { app.tick(); }