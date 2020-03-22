// Problem: Outdoor and astro photogrpahy is dew a problem.
// An uncontrolled Heater is a waste of (portable=rare) energy.
// Therefor we developed a simple two-point controller.
// The target setting is just 10°C above outdoor temperature.
// This should prevent dew on the objective.
//
// Version 1: slow PID Regler I-2019
// Version 2: output on 8004 Display III-2019
// Version 3: display of input voltage IV-2010
// Version 4: replacement of PID control by 2-step control, based on practical tests 1-2020
//
//  Frank Mersch - Norbert Beyer
//
// thanks for code:
// https://github.com/milesburton/Arduino-Temperature-Control-Library/blob/master/examples/Multiple/Multiple.pde
// DS18b20 readout
//
// hint to myself
// Use old Bootloader!
//

#include <OneWire.h>
#include <DallasTemperature.h>  // temperature sensors
#include <Wire.h>               // Wire = I2C interface
#include <LiquidCrystal_I2C.h>  // Display

//If true logging is active
#define StatusPrint true

// LED-assignment
// Led "Error", red LED
// intensityred defines intensity = attention to night setup! R=6K resistor is ok
# define ErrorLED 4
# define intensityred 100

// LED "no heating", yellow LED
// intensityyellow defines Intensity
# define NoHeatLED 5
# define intensityyellow 100

// heater is at Pin 6
# define HEATER 6

// Factor to calculate digits to Volt, depends on voltage devider
# define digitV 59.27

// Offset+Hysterese for control, OutputVal limits output for smooth heating
// IF T-Heater > ErrorOT+Offset then Heater off and ErrorLED starts blinking
#define Hysterese 1.1
#define Offset 9.9
#define ErrorOT 3.0

// Norbert: change value here ;-)
#define OutputVal 75 //%

// cycle time
#define TEMP_READ_DELAY 1000 //can only read digital temp sensor every ~750ms

// I2C-Bus is Pin 2  Arduino nano
#define ONE_WIRE_BUS 2
#define DS18B20_Resolution 12

// define disply Type: 20 4 lines - 20 colums, coded to 0x27
// more details in
// http://homepage.o2mail.de/acw2011/downloads/anleitungen/lcd/lcd_1602_2004_i2c.pdf
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Outside is outside temperature
// Inside is close to objective
double insidetemperature, outsidetemperature, setPoint;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature temperatureSensors(&oneWire);
// Heating = Inside, hard coded to prevent mistakes.
// inside of objective, outside = climate
DeviceAddress outsideThermometer = { 0x28, 0xFF, 0xD1, 0x58, 0xA2, 0x15, 0x03, 0x57 };
// 28 FF D1 58 A2 15 03 57
DeviceAddress insideThermometer = { 0x28, 0xFF, 0xD1, 0x58, 0xA2, 0x15, 0x03, 0x57 };
// time stamp of measurement
unsigned long lastTempUpdate;


// Voltmeter definition
int analogPin = A3; // pin connected to Vcc by votlage divider
int val = 0;        // Variable for reading that value

// internal LED
bool on = HIGH;

// number of linked sensors, should be 2
int number = 0;

// some variables
float Voltage;
bool Heating = false;
bool HeaterModus = false;
int i;

void error(void)
  {
  // Error Overtemperature handling
  // during runtime
  // every second a flash
  analogWrite(HEATER,0);
  digitalWrite(ErrorLED, HIGH);
  delay(100);
  digitalWrite(ErrorLED, LOW);
  }


// Init-Routine

void setup(void)
{
  // LEDs Error and HEATER
  pinMode(ErrorLED, OUTPUT);
  pinMode(HEATER, OUTPUT);
  pinMode(NoHeatLED, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Reference for voltage
  analogReference(EXTERNAL);

  // LCD start
  // Header
  lcd.noBacklight(); // backlight off - astro application otherwise lcd.backlight()
  lcd.setCursor(0, 0);
  lcd.print("Astropower 4.0");

  // Start of sensor library
  temperatureSensors.begin();
  //less than two sensors = no Heating
  number          = temperatureSensors.getDeviceCount();
  temperatureSensors.setResolution(insideThermometer,DS18B20_Resolution);
  temperatureSensors.setResolution(outsideThermometer,DS18B20_Resolution);

  // not two sensors
  if (number!=2)
      {
        lcd.setCursor(3, 2);
        lcd.print("kein Heizen");
        // lcd.print("no heating")
        HeaterModus = false;
        digitalWrite(NoHeatLED,  intensityyellow);
      }
  else
      {
        HeaterModus = true;
        digitalWrite(NoHeatLED,0);
      }

  if (StatusPrint)
      {
        // start serial port USB
        Serial.begin(9600);
        Serial.println("Norberts Objektiv-Heizer IV ");
        // Serial.println("Norberts Objektiv-Heating IV ");
        Serial.print ("Sensoren ");
        // Serial.print ("Sensors ");
        Serial.println(number);
        if (!temperatureSensors.getAddress(insideThermometer, 0)) Serial.println("Sensor Heizung nicht gefunden");
        //   if (!temperatureSensors.getAddress(insideThermometer, 0)) Serial.println("Sensor Heater not found");
        if (!temperatureSensors.getAddress(outsideThermometer, 1)) Serial.println("Sensor Raum nicht gefunden");
        //   if (!temperatureSensors.getAddress(outsideThermometer, 1)) Serial.println("Sensor Outside not found");
        Serial.println("HeatP HeaterT Surrounding");
      }
} // end init


//call repeatedly in loop, only updates after a certain time interval
//returns true if update happened
bool updateTemperature()
  {
        if ((millis() - lastTempUpdate) > TEMP_READ_DELAY)
        {
          insidetemperature = temperatureSensors.getTempC(insideThermometer); //get temp reading
          outsidetemperature = temperatureSensors.getTempC(outsideThermometer); //get temp reading
          lastTempUpdate = millis();
          temperatureSensors.requestTemperatures(); //request reading for next time
          return true;
        }
        return false;
}
// end update Temperature

/*
 Start of main program
*/

void loop(void)
  { // two sensors found
      if (HeaterModus){
        if (updateTemperature())
        {
          // calculate setpoint above surrounding
          setPoint = Offset + outsidetemperature;

          if (insidetemperature < setPoint)
          {
            // Heating on
            Heating = true;
          }
          if (insidetemperature >(setPoint+Hysterese))
          {
            Heating = false;
          }
          if (insidetemperature>(setPoint+ErrorOT))
          {
            // to hot = emergency off
            Heating = false;
            error();
          }
          if (Heating)
          {
            analogWrite(HEATER, OutputVal*2.55);
          }
          else
          {
            analogWrite(HEATER, 0);
          }
          lcd.setCursor(0,3);
          lcd.print("Th= ");
          lcd.print(insidetemperature);
          lcd.setCursor(10,3);
          lcd.print("To= ");
          lcd.print(outsidetemperature);
          lcd.setCursor(0,2);
          lcd.print("P = ");
          lcd.print(OutputVal);
          lcd.print("% ");
          if (Heating)
            { lcd.print("an "); }
          else
            {lcd.print("off"); }

          if (StatusPrint)
            {
              Serial.print(" ");
              Serial.print(insidetemperature);
              Serial.print(" ");
              Serial.println(outsidetemperature);
            } // Ende Statusprint
        }
        else
        // nicht Heizen Modus
        {
          digitalWrite(NoHeatLED,  intensityyellow);
        }
        // Ende Heating

        // Internal LED blinks as ok indicator .
        if (on==HIGH)
          {on = LOW; }
        else
          {on=HIGH; }
        digitalWrite(LED_BUILTIN, on);
       // end Blink-LED
       // waitung for next value
       // measuring voltage during that time
       val =0;
       // average by 20 values
       for (i=1;i<=20;i++)
       {
         val = val + analogRead(analogPin); // Pin einlesen
         delay(TEMP_READ_DELAY/20);
       }
       val = val/20;
       lcd.setCursor(0,1);
       lcd.print("Ub= ");
       Voltage = (val/digitV);
       lcd.print(Voltage);
       lcd.print(" Volt ");
      }
   }
