/******************************************************************
*******************************************************************
** Pool Solar Controller
**
** The prupose of this controller is primarily to watch the pool solary array and switch on the
** pump when the array is capable of geenrating heat.
** The controller also controls a float valve and fill solinoid to control water level in the pool
** The metrics are available via Pachube so we don't have to keep going to the pool shed
** the check the temperature
*/

#include <stdlib.h>


/******************************************************************
*******************************************************************
** Define the constants used
*/

/*
** Definitions around max temperautres for the pump shed
*/
#define MAX_SHED_TEMP  25      // temperature at which the case fan comes on
#define OK_SHED_TEMP   23      // temperature at which the case fan turns off
#define BUTTON_DRIFT    5      // Drift in the value of the buttons

/*
** Define the I/O pins and what they control
*/
#define POOLFILLVALVE A2       // Pin for the pool fill valve
#define SOLARPUMP     A1       // Pin for the Solar Pump (A1)
#define BUTTONS       A3       // Pin for the control buttons
#define SHEDFAN       A0       // Pin for the fan control
#define LCDBACKLIGHT   3       // Pin for the LCD backlight switch


// Setup the reguired global varibles
boolean pump_on = true;        // Assume the pump is on
boolean fill_on = true;        // Assume fill valve is on
boolean fan_on = true;         // Assume fan is on
boolean pool_lights = true;    // Assume pool lights are on


// Button related variables
boolean U_Button = false;      // Up button pressed
boolean D_Button = false;      // Down button pressed
boolean L_Button = false;      // Left button pressed
boolean R_Button = false;      // Right button pressed
boolean S_Button = false;      // Select button pressed
boolean F_Switch = true;       // Fill Switch in pool active (note - reverse sense)
int button_pressed = 0;        // Used to hold the button value during the keyboard routine


// Temperature related variables
float shed_temp   = 99.0;      // Assume it's too hot to begin with
float array_temp  = 10.0;      // Assume array is cold
float pool_temp   = 99.0;      // Assume pool is hot
float flow_temp   = 30.0;      // Assume flow is cooler than pool
float return_temp = 10.0;      // Assume return is warmer than array
#define TEMP_ACCURACY 0.5      // Accuracy of the sensors in reading the temperature


// Display mode related variables
byte display_mode = 1;         // Controls what is being displayed at the moment
#define MIN_DISPLAY_MODE 1     // Lowest numbered display mode
#define DISP_TEMP   1          // DIsplays the current temperatures and the pump status
#define DISP_TIME   2          // Displays the current time/date from the RTC
#define DISP_OTHER  3          // Test diagnostics (for testing functions as they are added
#define DISP_PUMP   4          // Manual pump mode
#define DISP_ERROR  5          // A page for displaying errors
#define MAX_DISPLAY_MODE 5     // Highest numbered display mode

// Counters for running the circulation pump in manual mode
int man_pump_set = 0;
#define DEFAULT_RUN_TIME 420   // Default run time in minutes for the pump in manual mode
int man_pump_run = DEFAULT_RUN_TIME;        

//Pump hold-off timer to stop the pump from getting thrashed
#define PUMP_HOLD_OFF  5       // Minutes between the pump changing state
byte pump_hold_timer = 0;      // The pump can start straight away

// Counters for the pool fill function
byte pool_fill_smooth = 0;     // Used to smooth the pool level switch for when people are in the pool
int pool_fill_rem    = 0;      // Tracks how long we have left to fill for. This is a safety stop.
#define POOL_FILL_SMOOTHED 250 // Number of execution loops required to debounce the pool fill float valve
#define MAX_POOL_FILL_TIME 60  // Only try to fill the pool for 60 minutes before calling no joy.

// Display dimming
byte backlight_timer = 0;      // Counts the number of execution loops before the display dims
#define BACKLIGHT_TIME 200     // Execution loops before the display dims

// Error conditions
#define NO_ERR 0
byte error_no = NO_ERR;        // Used to hold the error code when an error is encountered
#define ERR_FILL    1          // The fill operation timed out.
#define ERR_PACHUBE 2          // The connection to Pachube wouldn't open
#define ERR_SENSORS 3          // Some of the 1-wire sensors couldn't be found

// General
byte old_minute = 0;           // To track the minutes going by for relative timings
byte old_hour = 0;             // To track the hours going by for relative timings

// 1-wire sensors
#define NUM_OF_SENSORS 5       // Number of sensors the system expects to see

// Pachube data
#define PACHUBE_FEED "41617"
#define PACHUBE_KEY  "y_eXWNhsWfsaedd6VhbA13e9qVYCa1_ck5VniQ-3uUw"
byte pachube[] = { 173, 203, 98, 29 };
char pachube_data[10];



/******************************************************************
*******************************************************************
** Ethernet interface
** ------------------
** Ethernet shield attached to pins 10, 11, 12, 13
*/

#include <SPI.h>
#include <Ethernet.h>

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };   // Our MAC address
byte ip[] = { 192, 168, 100, 20 };                     // Our IP address

EthernetClient client;                                 // We are a client meaning we call them, we don't look for connections


/******************************************************************
*******************************************************************
** LiquidCrystal Display
** ---------------------
**
** The circuit:
** LCD RS pin to digital pin 8
** LCD Enable pin to digital pin 9
** LCD D4 pin to digital pin 4
** LCD D5 pin to digital pin 5
** LCD D6 pin to digital pin 6
** LCD D7 pin to digital pin 7
** LCD Backlight to digital pin 3
** LCD R/W pin to ground
** 10K resistor:
** ends to +5V and ground wiper to LCD VO pin (pin 3)
*/

#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);



/******************************************************************
*******************************************************************
**
** Real Time Clock
** ---------------
** Here are the routines needed for handling the real time clock.
** The 2IC bus for this is on Analogue pins 4 & 5
*/
#include <Wire.h>
#include <RealTimeClockDS1307.h>

#define DS1307_I2C_ADDRESS 0x68  // This is the I2C address
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;



/******************************************************************
*******************************************************************
**
** 1-wire
** ------
** Here are the routines needed for handling the 1-wire devices
** The 1-wire bus for this is on Analogue pins 2
**
*/
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 10

DeviceAddress shed_sensor =   { 0x10, 0xFD, 0xAC, 0x05, 0x00, 0x08, 0x00, 0x26 };
DeviceAddress pool_sensor =   { 0x28, 0x5A, 0xDA, 0x85, 0x03, 0x00, 0x00, 0xA0 };
DeviceAddress flow_sensor =   { 0x28, 0x5B, 0xD9, 0x85, 0x03, 0x00, 0x00, 0xD9 };
DeviceAddress array_sensor =  { 0x28, 0xC0, 0xC7, 0x85, 0x03, 0x00, 0x00, 0x4E };
DeviceAddress return_sensor = { 0x28, 0x31, 0xC9, 0x85, 0x03, 0x00, 0x00, 0xA9 };


OneWire oneWire(ONE_WIRE_BUS);        // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature. 
int numberOfDevices;                  // Number of temperature devices found
DeviceAddress tempDeviceAddress;      // We'll use this variable to store a found device address
byte sensor_errors = 0;               // Counter to track the number fo sensor errors in each pass




/******************************************************************
*******************************************************************
** Now set up all the bits and pieces
*/

void setup()
{
  
  delay(200);  // Ensure the Ethernet chip has reset fully
  
  // start serial port if required for debugging
#ifdef DEBUGGING
  Serial.begin(9600);
#endif
  
  // set up the LCD
  pinMode(LCDBACKLIGHT, OUTPUT);           // LCD backlight
  digitalWrite(LCDBACKLIGHT, HIGH);        // Turn on the backlight
  
  lcd.begin(16,2);                         // number of columns and rows: 
  lcd.clear();                             // Clear the display
  lcd.print("> Solar Master <");           // Who are we?
  lcd.setCursor(0,1);
  lcd.print("- Initialising -");           // Display startup message
  

  // Turn on the fan in case we are starting already hot
  pinMode(SHEDFAN, OUTPUT);
  digitalWrite(SHEDFAN, HIGH);
  fan_on = true;
  

  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);

  
  // Start up the 2IC bus to talk to the Real Time Clock
  Wire.begin();
  
  
  // Startup the 1-Wire devices
  sensors.begin();
  
  // Grab a count of devices on the wire
  numberOfDevices = sensors.getDeviceCount();
  if ( numberOfDevices != NUM_OF_SENSORS )
  {
    error_no = ERR_SENSORS;              // Flag the error
    display_mode = DISP_ERROR;           // Go to the error display page
  }

  // Loop through each device, and set the resolution
  for(byte i=0;i<numberOfDevices; i++)
  {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i))
      sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION); // set the resolution to TEMPERATURE_PRECISION bits
  }
  
  // Ensure the pump is stopped is stopped
  pinMode (SOLARPUMP, OUTPUT);
  digitalWrite(SOLARPUMP, LOW);    // Turn pump off
  pump_on = false;
  
  // Make the output for the fill valve an output
  pinMode (POOLFILLVALVE, OUTPUT);
  digitalWrite(POOLFILLVALVE, LOW);  // Turn the fill valve off
  fill_on = false;
  
  // Make sure the bit for the buttons is readable
  pinMode (BUTTONS, INPUT);
  
  delay(100);    // Wait for the Ethernet controller to fully initialise
}





/******************************************************************
*******************************************************************
** The main loop
** This needs to execute around 5 times a second in order to keep the display
** from flickering and to make the buttons reasonably responsive.
*/

void loop()
{
  
/*
** Get the current time fro mthe RTC
*/
  getDateDs1307();
  
  
/*
** Gather Temperatures
** Gather the temperature from the following sensors. This really should use addressed ensors rather than
** discovered sensors as the order may change.
**
** ROOF_ARRAY    - The temperature sensor on the back of the roof array
** ARRAY_FLOW    - The water flowing to the array
** ARRAY_RETURN  - The water returning from the array
** POOL          - The pool
** SHED_TEMP     - The pool shed containing the control electronics (sounds proof = heat proof!)
**
*/

// Read 1-wire temperatures

  sensors.requestTemperatures();                     // Send the command to get temperatures to all devices on the bus
  
  shed_temp = sensors.getTempC(shed_sensor);         // Get the shed temparture (actually it's in the case, but close enough)
  if (shed_temp < 0) sensor_errors++;                // Check that we could read it
  
  array_temp = sensors.getTempC(array_sensor);       // Get the array temperature from the roof 
  if (array_temp < 0) sensor_errors++;               // Check that we could read it
  
  pool_temp = sensors.getTempC(pool_sensor);         // Get the pool temparture
  if (pool_temp < 0) sensor_errors++;                // Check that we could read it
  
  return_temp = sensors.getTempC(return_sensor);     // Get the temperature of the returning water
  if (return_temp < 0) sensor_errors++;              // Check that we could read it

  flow_temp = sensors.getTempC(flow_sensor);         // Get the temperature of the flow water
  if (flow_temp < 0) sensor_errors++;                // Check that we could read it

  
/*
** Gather the button state
** Needs to allow for a range of values to compensate for thermal drift
** The pool fill switch is in parallel with the buttons on the front panel. This measns
** the buttons on the front panel will return one of two values depending upon status
** of the pool switch. We need to check for both. Since they are both near each other I've simply
** increased the range for each button to include both values while making sure they
** don't overlap.
*/
  U_Button = false;
  D_Button = false;
  L_Button = false;
  R_Button = false;
  S_Button = false;
  F_Switch = true;       // Note - reverse sense, true means the water level is good.
  
  button_pressed = analogRead(BUTTONS);
  
  backlight_timer++;
  
  if((button_pressed <= 0 + BUTTON_DRIFT) && (button_pressed >= 0)) R_Button = true;
    
  if(((button_pressed <= 144 + BUTTON_DRIFT) && (button_pressed >= 144 - BUTTON_DRIFT)) ||
     ((button_pressed <= 140 + BUTTON_DRIFT) && (button_pressed >= 140 - BUTTON_DRIFT)) ) { U_Button = true; backlight_timer = 0; }
    
  if(((button_pressed <= 331 + BUTTON_DRIFT) && (button_pressed >= 331 - BUTTON_DRIFT)) ||
     ((button_pressed <= 310 + BUTTON_DRIFT) && (button_pressed >= 310 - BUTTON_DRIFT)) ) { D_Button = true; backlight_timer = 0; }
    
  if(((button_pressed <= 507 + BUTTON_DRIFT) && (button_pressed >= 507 - BUTTON_DRIFT)) ||
     ((button_pressed <= 461 + BUTTON_DRIFT) && (button_pressed >= 461 - BUTTON_DRIFT)) ) { L_Button = true; backlight_timer = 0; }
    
  if(((button_pressed <= 745 + BUTTON_DRIFT) && (button_pressed >= 745 - BUTTON_DRIFT)) ||
     ((button_pressed <= 650 + BUTTON_DRIFT) && (button_pressed >= 650 - BUTTON_DRIFT)) ) { S_Button = true; backlight_timer = 0; }
    
  if( (button_pressed <= 855 + BUTTON_DRIFT) && (button_pressed >= 855 - BUTTON_DRIFT)) F_Switch = false; 


  // Check if no buttons have been pressed for a while. If so, turn out the backlight
  if ( backlight_timer == BACKLIGHT_TIME ) {
    digitalWrite(LCDBACKLIGHT, LOW);           // Turn off the backlight
    backlight_timer--;
  } else {
    digitalWrite(LCDBACKLIGHT, HIGH);         // Turn on the backlight
  }
 

/**
** Regularly timed events
** Handle both relative and absolute times for starting things.
*/

/*
** Relative - Check every hour what we should do
*/
  if (old_hour != hour) {
    old_hour = hour;
    
    // For the moment turn the pump on every hour to see if there is heat
    if ((hour > 8) && (hour < 18)) {
      digitalWrite(SOLARPUMP, HIGH);
      pump_on = true;
      pump_hold_timer = PUMP_HOLD_OFF;
    }
  }  
  
  
/*
** Relative - Check every minute what we should do
*/
  if (old_minute != minute) {
    old_minute = minute;
    
    if (man_pump_set > 0) {                // If we are in manual mode for the pump, have we finished?
      man_pump_set--;
      if(man_pump_set == 0) {
        digitalWrite(SOLARPUMP, LOW);     // Turn pump off
        pump_on = false;
        man_pump_run = DEFAULT_RUN_TIME;  // Go back to default value for manual run minutes
        display_mode = DISP_TEMP;         // Go back to the default display
      }
    }
    
      
    if (pool_fill_rem > 0) {              // If we are filling the pool with water, have we timed out?
      pool_fill_rem--;
      if(pool_fill_rem == 0) {
        digitalWrite(POOLFILLVALVE, LOW); // turn the fill valve off
        fill_on = false;
        error_no = ERR_FILL;              // Flag the error
        display_mode = DISP_ERROR;        // Go to the error display page
        pool_fill_rem = -1;               // Block future filling till someone resets the error
      }
    }
    
    if (pump_hold_timer > 0) {            // Stop the pump from being flogged
      pump_hold_timer--;
    }
      
    sendData(0,pool_temp);
    sendData(1,array_temp);
    sendData(2,return_temp);
    sendData(3,flow_temp);
    sendData(6,shed_temp);
    if(fill_on == true) {
      sendData(5,1);
    } else {
      sendData(5,0);
    }
    if(F_Switch == true) {
      sendData(4,0);
    } else {
      sendData(4,1);
    }
    if(pump_on == true) {
      sendData(7,1);
    } else {
      sendData(7,0);
    }
    if(fan_on == true) {
      sendData(8,1);
    } else {
      sendData(8,0);
    }
    sendData(9,sensor_errors);
    sensor_errors = 0;
  }

    
/*
** Check for absolute time events
*/  

/*
** Solar Ciculation Pump
** Figure out what the pump should be doing
** We should turn the pump on when the roof array is hotter than the pool and off
** when the return temperature is the same as the pool (to within twice the error
** rating of the temperature sensors.
*/

  if (pump_hold_timer == 0) {
    if (pump_on == false) {
      if( array_temp > ( pool_temp + 2*TEMP_ACCURACY) ) {
        digitalWrite(SOLARPUMP, HIGH);   // Turn pump on
        pump_on = true;
        pump_hold_timer = PUMP_HOLD_OFF;
      }
    } else {
      if( return_temp < (flow_temp + TEMP_ACCURACY) ) {
        digitalWrite(SOLARPUMP, LOW);   // Turn pump off
        pump_on = false;
        pump_hold_timer = PUMP_HOLD_OFF;
      }
    }
  }



  
/*
** Shed Fan
** This figures out what the shed fan should do to keep the shed cool.
** There shouldn't be a need for any hysteresis on this as the thermal mass
** of the shed should do the job.
*/


  if(shed_temp > MAX_SHED_TEMP) {
    digitalWrite(SHEDFAN, HIGH);      // Turn fan on
    fan_on = true;
  }
  
  if(shed_temp < OK_SHED_TEMP) {
    digitalWrite(SHEDFAN, LOW);       // Turn pump off
    fan_on = false;
  }
  
  
/*
** Pool Filling
** This checks the pool level sensor and decides if the pool needs water added.
**
** 1. We need to effectively "debounce" the water level sensor to allow for people
**    playing in the pool. We do this by requiring the float valve to be held for a certain number
**    of execution loops.
** 2. We need a timer so failure of the pool sensor won't release too much water.
** 3. We should log how long the fill solinoid was on for to track water usage. (See Pachube section)
*/

  if( F_Switch == false ) {
    pool_fill_smooth++;
    if ((pool_fill_smooth == POOL_FILL_SMOOTHED) && (pool_fill_rem == 0)) {
      pool_fill_smooth = 0;
      digitalWrite(POOLFILLVALVE, HIGH);   // Turn pool fill valve on
      fill_on = true;                      // The valve is now on
      pool_fill_rem = MAX_POOL_FILL_TIME;  // Start a timrer to stop the fill if it goes on too long
    } 
  } else {
      pool_fill_smooth = 0;
      digitalWrite(POOLFILLVALVE, LOW);    // Turn pool fill valve off
      fill_on = false;                     // The valve is now off
      pool_fill_rem = 0;                   // We have no minutes of fill remaining
  }
  

  
/*
** If the user pressed up or down, update the display mode
*/

  if (U_Button == true) display_mode++;
  if (D_Button == true) display_mode--;
  
  if (display_mode > MAX_DISPLAY_MODE) display_mode = MIN_DISPLAY_MODE;  // The two impliment wrap around on the menu
  if (display_mode < MIN_DISPLAY_MODE) display_mode = MAX_DISPLAY_MODE;
  
/*
** If we are in the manual pump setting scree, let the user set a manual run time for the pump.
*/

  if( display_mode == DISP_PUMP ) {
    if(R_Button == true) man_pump_run = man_pump_run + 5;
    if(man_pump_run == 999) man_pump_run = 998;
    if(L_Button == true) man_pump_run = man_pump_run - 5;
    if(man_pump_run == 0) man_pump_run =1;
    if(S_Button == true) {
      man_pump_set = man_pump_run;
      digitalWrite(SOLARPUMP, HIGH);   // Turn pump on
      pump_on = true;
    }
  }

/*
** Now update the display
*/
  lcd.clear();
  
  switch (display_mode) {
    
    case DISP_TEMP:
        // Display temperatures
        error_no = NO_ERR;
        pool_fill_rem = 0;
        lcd.setCursor(0,0);
        lcd.print("P=");
        if (pool_temp > 0) {
          lcd.print(pool_temp);
        } else {
          lcd.print("Error");
        }
  
        lcd.setCursor(9,0);
        lcd.print("A=");
        if (array_temp > 0) {
          lcd.print(array_temp);
        } else {
          lcd.print("Error");
        }
  
        lcd.setCursor(0,1);
        lcd.print("F=");
        if (flow_temp > 0) {
          lcd.print(flow_temp);
        } else {
          lcd.print("Error");
        }
  
        lcd.setCursor(9,1);
        lcd.print("R=");
        if (return_temp > 0) {
          lcd.print(return_temp);
        } else {
          lcd.print("Error");
        }

        break;
        
    case DISP_TIME:
        // Display the current time from the RTC
        error_no = NO_ERR;
        pool_fill_rem = 0;
        lcd.setCursor(0,0);
        lcd.print("Time: ");
        if (hour < 10) lcd.print("0");
        lcd.print(hour, DEC);
        lcd.print(":");
        if (minute < 10) lcd.print("0");
        lcd.print(minute, DEC);
        lcd.print(":");
        if (second < 10) lcd.print("0");
        lcd.print(second, DEC);
        
        lcd.setCursor(0,1);
        lcd.print("Date: ");
        if (dayOfMonth <10) lcd.print("0");
        lcd.print(dayOfMonth, DEC);
        lcd.print("/");
        if (month < 10) lcd.print("0");
        lcd.print(month, DEC);
        lcd.print("/");
        if (year <10) lcd.print("0");     // yeah, right. Like that's ever gonna happen!
        lcd.print(year, DEC);
        break;
        
        
    case DISP_OTHER:
        // Diagnostic display
        error_no = NO_ERR;
        pool_fill_rem = 0;
        lcd.setCursor(0,0);
        lcd.print("C=");
        if ( shed_temp > 0 ) {
          lcd.print(shed_temp);
        } else {
          lcd.print("Error");
        }
        
        if(F_Switch == true) {
          lcd.setCursor(0,1);
          lcd.print("H2O=OK ");
        } else {
          lcd.setCursor(0,1);
          lcd.print("H2O=Low");
        }
     
        if(pump_on == true) {
          if(R_Button == true) {
            digitalWrite(SOLARPUMP, LOW);    // Manually turn pump off
            pump_on = false;
          }
          lcd.setCursor(8,0);
          lcd.print("Pump=On ");
        } else {
          if(L_Button == true) {
            digitalWrite(SOLARPUMP, HIGH);   // Mannualy turn pump on
            pump_on = true;
          }
          lcd.setCursor(8,0);
          lcd.print("Pump=Off");
        }
               
        if(fill_on == true) {
          lcd.setCursor(8,1);
          lcd.print("Fill=On ");
        } else {
          lcd.setCursor(8,1);
          lcd.print("Fill=Off");
        }
        break;
        
     case DISP_PUMP:
        // Manual pump timer mode
        error_no = NO_ERR;
        pool_fill_rem = 0;
        lcd.setCursor(0,0);
        lcd.print("Timer: Pump ");
        if(pump_on == true) {
          lcd.print("On");
        } else {
          lcd.print("Off");
        }
        lcd.setCursor(0,1);
        lcd.print("Set ");
        lcd.print(man_pump_run);
        lcd.setCursor(8,1);
        lcd.print("Left ");
        lcd.print(man_pump_set);
        break;
        
        
     case DISP_ERROR:                        // Something went wrong....
        lcd.print("> Solar Master <");       // Who are we?
        lcd.setCursor(0,1);
        switch (error_no) {
          case ERR_FILL:
             // The fill operation failed
             lcd.print("  Fill Failed");
             break;
             
           case ERR_PACHUBE:
             lcd.print(" Pachube failed");
             break;
             
           case ERR_SENSORS:
             lcd.print(" Sensor failed");
             break;
             
          default:
             lcd.print("   System OK");
             break;
        }
        break;
             
        
     default:
        // Invalid display mode
        lcd.setCursor(0,0);
        lcd.print("Invalid Mode");
        break;
        
  }
}


/******************************************************************
*******************************************************************
** Pachube client
** --------------
** This routine takes a datastream number and a value and posts them to Pachube
*/

// This method makes a HTTP connection to the server and puts the argument thisData to the datastream thisFeed.
//
void sendData(int thisStream, float thisData) {

  // if there's a successful connection:

  if (client.connect(pachube, 80)) {

    sprintf(pachube_data,"%i,", thisStream);
    dtostrf(thisData,5,2,&pachube_data[2]);     // Write this data into the data string, starting at the 3rd character. Make it 5 chars in total with precission of 2 => XX.XX

 
    // Send the HTTP PUT request. This is a Pachube V2 format request. The .csv tells Pachube what format we are using 
    client.print("PUT /v2/feeds/");
    client.print(PACHUBE_FEED);
    client.println(".csv HTTP/1.1");
    
    // Send the host command
    client.println("Host: api.pachube.com");
    
    // Prove we are authorised to update this feed
    client.print("X-PachubeApiKey: ");
    client.println(PACHUBE_KEY);
    
    // Tell Pachube how many bytes we are sending. NOTE: This actually send the length of the buffer, not the number of characters.
    // As this device has fixed length numbers it's not a problem ... though it might be in winter. Also means you can't have more
    // than 9 data streams.
    client.print("Content-Length: ");
    client.println(strlen(pachube_data), DEC);  

    // last pieces of the HTTP PUT request:
    client.println("Connection: close");
    
    // Don't forgte the empty line between the header and the data!!
    client.println();

    // here's the actual content of the PUT request:
    client.println(pachube_data);
    
    // Don't forgte another empty line after the data.
    client.println();
    
    // Once it's all sent we can disconnect
    client.stop();
  }
  else {
    error_no = ERR_PACHUBE;
    display_mode = DISP_ERROR;        // Go to the error display page
    
  }
}


/******************************************************************
*******************************************************************
**
** Real Time Clock
** ---------------
** Here are the routines needed for handling the real time clock.
** The 2IC bus for this is on Analogue pins 4 & 5
*/

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}


// Gets the date and time from the ds1307 and stores the result for later use
void getDateDs1307()
{
  // Reset the register pointer
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.write((byte) 0x00);
  Wire.endTransmission();
 
  Wire.requestFrom(DS1307_I2C_ADDRESS, 7);
 
  // A few of these need masks because certain bits are control bits
  second     = bcdToDec(Wire.read() & 0x7f);
  minute     = bcdToDec(Wire.read());
  hour       = bcdToDec(Wire.read() & 0x3f);  // Need to change this if 12 hour am/pm
  dayOfWeek  = bcdToDec(Wire.read());
  dayOfMonth = bcdToDec(Wire.read());
  month      = bcdToDec(Wire.read());
  year       = bcdToDec(Wire.read());
}


