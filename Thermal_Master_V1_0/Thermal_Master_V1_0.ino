/******************************************************************
*******************************************************************
** Room Temperature Management
**
** The prupose of this controller is primarily to watch the temperatures in the hobby
** room and ensure they stay within bounds using as much free cooling as possible.
*/

#include <stdlib.h>
#include <Wire.h>

#define DEBUGGING false

/******************************************************************
*******************************************************************
** Define the constants used
*/


// Definitions for the temperautres
#define ROOM_TEMP_VHIGH  28.0    // temperature at which the HRAC comes on
#define ROOM_TEMP_HIGH   23.0    // temperature at which all fans come on
#define ROOM_TARGET_TEMP 21.0    // The ideal target temperature for the room
#define ROOM_TEMP_LOW    19.0    // temperature at which the all fans go off
#define ROOM_TEMP_VLOW   16.0    // temperature at which the HRAC come on

// Definitions for the fans
#define RUNNING          B0011010;  // The default state is for the fans to be running
#define STOPPED          B0011010;
#define FAST             B0000101;  // The default state os for the fans to be in fast mode
#define SLOW             B0000101;

#define FAN_INTERNAL     B0000011;
#define FAN_EXTERNAL     B0001100;

byte fan_mode = B00000000;

// Definitions for the HRAC
#define ON               true;   // The default is HRAC to be off
#define OFF              false;

/*
** Define the I/O pins and what they control
*/
// Analog pins
#define BUTTONS       A0       // Pin for 
#define UNUSED        A1       // Pin for 
#define UNUSED        A2       // Pin for 
#define UNUSED        A3       // Pin for 

// Digital pins
#define UNUSED         0       // Pin for
#define UNUSED         1       // Pin for
#define LCDBACKLIGHT   3       // Pin for the LCD backlight switch
#define ONEWIRE_RACK_A   2
#define ONEWIRE_RACK_B   99
#define ONEWIRE_RACK_C   99
#define ONEWIRE_ROOM     99
#define ONEWIRE_HOUSE    99

// Front panel button related variables
boolean U_Button = false;      // Up button pressed
boolean D_Button = false;      // Down button pressed
boolean L_Button = false;      // Left button pressed
boolean R_Button = false;      // Right button pressed
boolean S_Button = false;      // Select button pressed

boolean HRAC_Call = false;     // Call for HRAC only mode
boolean Full_Exhaust = false;  // Call for all fans on to exhaust room
boolean Printer = true;        // Assume the 3D printer is running
boolean All_Stop = false;      // Stop all ventilation

#define BUTTON_DRIFT    5      // Drift in the analog value of the buttons

int button_pressed = 0;        // Used to hold the button value during the keyboard routine

// Temperature related variables
float rack_a_upper_temp   = 99.0;    // Assume it's too hot to begin with
float rack_a_lower_temp   = 99.0;
float rack_b_upper_temp   = 99.0;
float rack_b_lower_temp   = 99.0;
float rack_c_upper_temp   = 99.0;
float rack_c_lower_temp   = 99.0;
;
float printer_temp     = ROOM_TARGET_TEMP; // Assume the printer is idle
;
float room_exhaust_temp   = 99.0;
float outside_air_temp    = ROOM_TARGET_TEMP; // Assume the outside air is the right temp
float room_temp           = ROOM_TEMP_VHIGH;
;
float house_temp          = ROOM_TARGET_TEMP;

// Announciator pannel LED definitions
#define AP_RACK1_UPPER  1
#define AP_RACK2_UPPER  2
#define AP_RACK3_UPPER  3
#define AP_ROOM         4
;
#define AP_RACK1_LOWER  5
#define AP_RACK2_LOWER  6
#define AP_RACK3_LOWER  7
#define AP_HOUSE        8
;
#define AP_HRAC_MODE    9
#define AP_FAN_IN_MODE  10
#define AP_FAN_OUT_MODE 11


// Display mode related variables
byte display_mode = 1;         // Controls what is being displayed at the moment
#define MIN_DISPLAY_MODE 1     // Lowest numbered display mode
#define DISP_TEMP   1          // DIsplays the current temperatures and the pump status
#define DISP_TIME   2          // Displays the current time/date from the RTC
#define DISP_ERROR  3          // A page for displaying errors
#define MAX_DISPLAY_MODE 3     // Highest numbered display mode

// Display dimming
byte backlight_timer = 0;      // Counts the number of execution loops before the display dims
#define BACKLIGHT_ON_TIME 200  // Execution loops before the display dims

// Possible error conditions
#define NO_ERR 0
#define ERR_FILL    1          // The fill operation timed out.
#define ERR_PACHUBE 2          // The connection to Pachube wouldn't open
#define ERR_SENSORS 3          // Some of the 1-wire sensors couldn't be found
byte error_no = NO_ERR;        // Used to hold the error code when an error is encountered

// General
byte old_minute = 0;           // To track the minutes going by for relative timings
byte old_hour = 0;             // To track the hours going by for relative timings

// Pachube data
#define PACHUBE_FEED "43570"
#define PACHUBE_KEY  "y_eXWNhsWfsaedd6VhbA13e9qVYCa1_ck5VniQ-3uUw"
/* byte pachube[] = { 173, 203, 98, 29 }; */
byte pachube[] = { 64,  94,  18, 121 };
//byte pachube[] = { 216, 52, 233, 122 };   // Seems to have been replaced.
char pachube_data[10];
// char array for constructing the COSM data strings so we can report it
char COSM_rack_a_temp[24];
char temp_str[20];



/******************************************************************
*******************************************************************
** Ethernet interface
** ------------------
** Ethernet shield attached to pins 10, 11, 12, 13
*/

#include <SPI.h>
#include <Ethernet.h>

// Ethernet parameters for the main network
byte my_mac[]     = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x24 };   // Our MAC address
byte my_ip[]      = { 192, 168, 100, 24 };                    // Our IP address
byte net_router[] = { 192, 168, 100, 1  };                    // The network router facing the internet
byte net_dns[]    = { 192, 168, 100, 9  };                    // The network DNS server


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
** 10K variable resistor:
** ends to +5V and ground, wiper to LCD VO pin (pin 3)
*/

#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd_main(8, 9, 4, 5, 6, 7);



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
** 
**
*/
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define TEMPERATURE_PRECISION 10    // Number of bit of precission to use in conversion
#define NUM_OF_SENSORS 1            // Number of sensors the system expects to see

//DeviceAddress shed_sensor =   { 0x10, 0xFD, 0xAC, 0x05, 0x00, 0x08, 0x00, 0x26 };
//DeviceAddress pool_sensor =   { 0x28, 0x5A, 0xDA, 0x85, 0x03, 0x00, 0x00, 0xA0 };
//DeviceAddress flow_sensor =   { 0x28, 0x5B, 0xD9, 0x85, 0x03, 0x00, 0x00, 0xD9 };
//DeviceAddress array_sensor =  { 0x28, 0xC0, 0xC7, 0x85, 0x03, 0x00, 0x00, 0x4E };
//DeviceAddress return_sensor = { 0x28, 0x31, 0xC9, 0x85, 0x03, 0x00, 0x00, 0xA9 };


OneWire OneWire_Bus_Rack_A(ONEWIRE_RACK_A);        // Setup a oneWire instance for Rack A
//OneWire OneWire_Bus_Rack_B(ONEWIRE_RACK_B);        // Setup a onewire instance for Rack B
//OneWire OneWire_Bus_Rack_C(ONEWIRE_RACK_C);        // Setup a onewire instance for Rack C
//OneWire OneWire_Bus_Room(ONEWIRE_ROOM);            // Setup a onewire instance for the equipment room
//OneWire OneWire_Bus_House(ONEWIRE_HOUSE);          // Setup a onewire instance for the house

DallasTemperature sensors(&OneWire_Bus_Rack_A);  // Pass our oneWire reference to Dallas Temperature.

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
  
  lcd_main.begin(20,4);                         // number of columns and rows: 
  lcd_main.clear();                             // Clear the display
  lcd_main.print(" Thermal Master ");           // Who are we?
  lcd_main.setCursor(0,1);
  lcd_main.print(" -Initialising- ");           // Display startup message
  

 // start the Ethernet connection and the server:
  lcd_main.setCursor(0,2);
  lcd_main.print(" -  Network  - ");
  Ethernet.begin(my_mac, my_ip, net_dns, net_router);  
  
  
  // Start up the 2IC bus to talk to the Real Time Clock
  lcd_main.setCursor(0,2);
  lcd_main.print(" -  2IC  Bus  - ");
  
  Wire.begin();
  
  // Set the addressing style
  Wire.beginTransmission(0x20);
  Wire.write(0x12);
  Wire.write(0x20);
  Wire.endTransmission();
  
  // Set the I/O bank A to be outputs
  Wire.beginTransmission(0x20);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();  
  
  // Set the fans tp full extract
  Wire.beginTransmission(0x20);
  Wire.write(0x12);
  Wire.write(B00000000);
  Wire.endTransmission();  
  
  // Startup the 1-Wire devices
  lcd_main.setCursor(0,2);
  lcd_main.print(" - 1Wire Bus  - ");

  sensors.begin();


#ifdef DEBUGGING
  // locate devices on the bus
  Serial.print(F("Locating devices..."));
  Serial.print(F("Found "));
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(F(" devices."));

  // report parasite power requirements
  Serial.print(F("Parasite power is: ")); 
  if (sensors.isParasitePowerMode()) Serial.println(F("ON"));
  else Serial.println(F("OFF"));
#endif

  
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
  

  
  // Make sure the bit for the buttons is readable
  pinMode (BUTTONS, INPUT);


  lcd_main.setCursor(0,2);
  lcd_main.print(" -    Done    - ");

  delay(100);    // Wait for the Ethernet controller to fully initialise

  lcd_main.clear();

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
** Gather Temperatures
**
*/

// Read 1-wire temperatures

  sensors.requestTemperatures();                     // Send the command to get temperatures to all devices on the bus
  
  sensors.getAddress(tempDeviceAddress, 1);
  rack_a_upper_temp = sensors.getTempC(tempDeviceAddress);   // Get the rack temparture
  
  if (rack_a_upper_temp < 0)
     { sensor_errors++; } // Check that we could read it
  else
  {
    // Figure out what to do with the fans
    fan_mode = B00001010;                               //All stop
    if (rack_a_upper_temp > 23) fan_mode = B00001001;
    if (rack_a_upper_temp > 25) fan_mode = B00001000;
    if (rack_a_upper_temp > 30) fan_mode = B00000100;
    if (rack_a_upper_temp > 35) fan_mode = B00000000;
    // Set the fans
    Wire.beginTransmission(0x20);
    Wire.write(0x12);
    Wire.write(fan_mode);
    Wire.endTransmission();
    
    // Format the current temp for reporting on COSM
    dtostrf(rack_a_upper_temp,5,2,temp_str);       // and is a floating in the format of XX.XX
    sprintf(COSM_rack_a_temp,"0,%s",temp_str);     // This data goes into stream 0 on COSM

  }
  
/*
** Send the data to COSM
*/

//  sendData(0,rack_a_upper_temp);

  //Report it all via COSM
  if (client.connect("api.cosm.com", 80)) {
    
    delay(200);      // Take this out and it stops working - why?
    
    client.println("PUT /v2/feeds/43570.csv HTTP/1.1");                        // Send the HTTP PUT request.
    client.println("Host: api.cosm.com");                                      // Send the host command        
    client.println("X-ApiKey: y_eXWNhsWfsaedd6VhbA13e9qVYCa1_ck5VniQ-3uUw");   // Prove we are authorised to update this feed        
    client.println("User-Agent: Energy-Master");                               // Explain who we are     
    client.print  ("Content-Length: ");
       client.println(strlen(COSM_rack_a_temp));                               // Send the length of the data being sent (remember the CR/LF)
    client.println("Content-Type: text/csv");                                  // Content type      
    client.println("Connection: close");                                       // Explain we will close the connection 
    client.println();                                                          // Don't forgte the empty line between the header and the data!!
    client.println(COSM_rack_a_temp);
    client.println();

    if (client.available()) {   // keep reading the return data till COSM stops sending it
      char c = client.read();   // Just throw it away. If it worked good, if not, too bad.
    }
    
    client.flush();
    client.stop();
  }    
  

   lcd_main.print("Temp = ");
   if (rack_a_upper_temp > 0) {
      lcd_main.println(rack_a_upper_temp);
   } else {
      lcd_main.println("Error");
   }
   lcd_main.println(COSM_rack_a_temp);

   delay(60000);

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


