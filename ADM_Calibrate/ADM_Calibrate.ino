              
//      *******  ADM Calibrator for MDCfVS Version 0.1
//      Project code written by Frank Appio
//      Unless otherwise noted
        
//      For Arduino Mega, Adafruit Ultimate GPS logging shield, 
//      Adafruit 10dof sensor, Adafruit Neopixel Jewel
        
//      Connect GPS shield to Arduino Mega via headers        
//      Connect TX on GPS shield to RX pin 19 
//      Connect RX on GPS shield to TX pin 18
//      Connect 3v on 10DOF breakout to 3v on shield
//      Connect GND on 10DOF breakout to GND on shield
//      Connect SCL on 10DOF breakout to SCL pin 21
//      Connect SDA on 10DOF breakout to SDA pin 20
//      Connect 5V on Neopixel to 5V on shield
//      Connect GND on Neopixel to GND on Mega or shield
//      Connect Data input on Neopixel to pin 25 on Mega
//      Select Soft Serial on shield's physical switch
        
//      A complete build tutorial including example video can be found at 
//      URL to instructable
                
        
// Import libraries
	// 10 DOF
	#include <Adafruit_10DOF.h>
	#include <Adafruit_LSM303_U.h>
	#include <Adafruit_BMP085_U.h>
	#include <Adafruit_L3GD20_U.h>
	#include <Adafruit_Sensor.h>
	// GPS - SD
	#include <Adafruit_GPS.h>
	#include <SoftwareSerial.h>
	#include <SD.h>
	#include <SPI.h>
	#include <avr/sleep.h>
	// Other
	#include <Adafruit_NeoPixel.h>
	#include <Wire.h>

// Declare sensor variables
	Adafruit_10DOF                dof  = Adafruit_10DOF();
	Adafruit_LSM303_Accel_Unified acc  = Adafruit_LSM303_Accel_Unified(30301);
	Adafruit_LSM303_Mag_Unified   mag  = Adafruit_LSM303_Mag_Unified(30302);
        Adafruit_BMP085_Unified       bmp  = Adafruit_BMP085_Unified(18001);
	Adafruit_L3GD20_Unified       gyr  = Adafruit_L3GD20_Unified(20);

	HardwareSerial mySerial = Serial1;
	Adafruit_GPS GPS(&Serial1);    

// Set the pins used
	#define chipSelect 10
	#define ledPin 25
	#define numPixels 7
	Adafruit_NeoPixel pixels = Adafruit_NeoPixel(numPixels, ledPin, NEO_GRB + NEO_KHZ800);

// Create our log file
	File logFile;

// Initialize variables
	float magHeading = 0;
	float temperature = 0;
	boolean runOnce = false;
        float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;       
        float alt=0, vio=0;
        float magMinX = 0;
        float magMinY = 0;
        float magMinZ = 0;
        float magMaxX = 0;
        float magMaxY = 0;
        float magMaxZ = 0;
        float accMinX = 0;
        float accMinY = 0;
        float accMinZ = 0;
        float accMaxX = 0;
        float accMaxY = 0;
        float accMaxZ = 0;
        float gyrMinX = 0;
        float gyrMinY = 0;
        float gyrMinZ = 0;
        float gyrMaxX = 0;
        float gyrMaxY = 0;
        float gyrMaxZ = 0;
        
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// This and other pieces from https://github.com/adafruit/Adafruit-GPS-Library/tree/master/examples/shield_sdlog
	// Set to 'true' if you want to debug and listen to the raw GPS sentences
	#define GPSECHO  false
	/* set to true to only log to SD when GPS has a fix, for debugging, keep it false */
	#define LOG_FIXONLY false  

// Do not touch this, use instance in setup.
	boolean usingInterrupt = false;
	void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

// Read a Hex value and return the decimal equivalent
uint8_t parseHex(char c) {
	if (c < '0')
	return 0;
	if (c <= '9')
	return c - '0';
	if (c < 'A')
	return 0;
	if (c <= 'F')
	return (c - 'A')+10;
	}

// Blink out an error code
void error(uint8_t errno) {
	while(1) {
	uint8_t i;
	pixels.begin(); 
	for (i=0; i<errno; i++) {
	pixels.setPixelColor(0, pixels.Color(100,0,0)); 
	pixels.show();
	delay(100);
	pixels.setPixelColor(0, pixels.Color(0,0,0)); 
	pixels.show();
	delay(100);
	}
	for (i=errno; i<10; i++) {
	  delay(200);
	}}}

// Sensor initialization errors
void initSensors()
	{
	if(!acc.begin())
	{
	/* There was a problem detecting the LSM303 ... check your connections */
	Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
	while(1);
	}
	if(!mag.begin())
	{
	/* There was a problem detecting the LSM303 ... check your connections */
	Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
	while(1);
	}
	if(!bmp.begin())
	{
	/* There was a problem detecting the BMP180 ... check your connections */
	Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
	while(1);
	}
	if(!gyr.begin())
	{
	/* There was a problem detecting the L3GD20 ... check your connections */
	Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
	while(1);
	}}


void setup() {
  
// Connect at 115200 
	Serial.begin(115200);
	Serial.println("Arduino Mega Motion Datalogger Calibrator Module");
	Serial.println("Initializing");
  
// Initialize
	analogReference(EXTERNAL);  //use external reference voltage (3,3V)
	delay(2000);
	Wire.begin();
	initSensors();
	pixels.begin(); // This initializes the NeoPixel library. 
        gyr.begin(); 
      
// Enable auto-range
        mag.enableAutoRange(true);
        gyr.enableAutoRange(true);
        acc.enableAutoRange(true);

// Make sure that the default chip select pin is set to output.
	pinMode(10, OUTPUT);
  
// See if the card is present and can be initialized:
	if (!SD.begin(chipSelect,11,12,13)) {
	Serial.println("Card init. failed!");
	error(2);
	}
	char filename[15];
	strcpy(filename, "CLBLOG00.CSV");
	for (uint8_t i = 0; i < 100; i++) {
	filename[6] = '0' + i/10;
	filename[7] = '0' + i%10;
// Create if does not exist, do not open existing, write, sync after write
	if (! SD.exists(filename)) {
	  break;
	}
	}
	logFile = SD.open(filename, FILE_WRITE);
	if( ! logFile ) {
	Serial.print("Couldnt create "); 
	Serial.println(filename);
	error(3);
	}
	Serial.print("Writing to "); 
	Serial.println(filename);
  
// Write heading in file
	logFile.println("GPS Time,GPS Date,Satellites,Fix Quality,Latitude,Longitude,GPS Altitude,GPS Speed,GPS Course,AccX,AccY,AccZ,Temperature,Heading,BMP Altitude,Variometer,MagX,MagY,MagZ,GyroX,GyroY,GyroZ,MinAccX,MinAccY,MinAccZ,MaxAccX,MaxAccY,MaxAccZ,MinMagX,MinMagY,MinMagZ,MaxMagX,MaxMagY,MaxMagZ,MinGyrX,MinGyrY,MinGyrZ,MaxGyrX,MaxGyrY,MaxGyrZ");
	logFile.flush();
  
// Connect to the GPS at the desired rate, set options
	GPS.begin(9600);
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
        // Set the update rate
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);   // 100 millihertz (once every 10 seconds), 1Hz or 5Hz update rate
        // Turn off updates on antenna status, if the firmware permits it
	GPS.sendCommand(PGCMD_NOANTENNA);
	useInterrupt(true);

  	Serial.println("Setup Complete");       
// End setup
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
	SIGNAL(TIMER0_COMPA_vect) {
	char c = GPS.read();
	#ifdef UDR0
	  if (GPSECHO)
		if (c) UDR0 = c;  
	#endif
	}

void useInterrupt(boolean v) {
	if (v) {
        // Interrupt and call the "Compare A" function above
	OCR0A = 0xAF;
	TIMSK0 |= _BV(OCIE0A);
	usingInterrupt = true;
	} 
	else {
        // Do not call the interrupt function COMPA anymore
	TIMSK0 &= ~_BV(OCIE0A);
	usingInterrupt = false;
	}
	}

void pixelDance() {
        for (int i = 0; i<8; i++){
	pixels.setPixelColor(i, pixels.Color(0,0,0)); 
	pixels.show(); 
	pixels.setPixelColor(i, pixels.Color(random(0,1),random(0,20),random(0,20))); 
	pixels.show(); 
        }
        }       
        
void pixelCalibrate() {
        for (int i = 0; i<8; i++){
	pixels.setPixelColor(i, pixels.Color(0,0,0)); 
	pixels.show(); 
	pixels.setPixelColor(i, pixels.Color(random(0,50),random(0,40),random(0,5))); 
	pixels.show(); 
        }        
        }
        
void loop() {    
	sensors_event_t acc_event;
	sensors_event_t mag_event;
	sensors_event_t bmp_event;
	sensors_event_t gyr_event; 
	sensors_vec_t orientation;
        sensor_t acc_sensor;
        sensor_t mag_sensor;
        sensor_t gyr_sensor;
        sensor_t bmp_sensor;

// Read the bmp, accelerometer, gyro, and magnetometer 
        acc.getSensor(&acc_sensor);
        acc.getEvent(&acc_event);
        mag.getSensor(&mag_sensor);
        mag.getEvent(&mag_event);
        gyr.getSensor(&gyr_sensor);
        gyr.getEvent(&gyr_event);
        bmp.getSensor(&bmp_sensor);
        bmp.getEvent(&bmp_event);

// Calculate the altitude using the barometric pressure sensor 
        if (bmp_event.pressure)
        {
        // Get ambient temperature in C, convert to F
        bmp.getTemperature(&temperature);
        temperature = temperature*1.8+32;
        // Convert atmospheric pressure, SLP and temp to altitude 
        float altNew=bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature);
        vio = (altNew-alt);
        alt = altNew;
        }

// Compass heading from mag
        if (dof.fusionGetOrientation(&acc_event, &mag_event, &orientation))
        {
        magHeading = 360-orientation.heading;
        if (magHeading < 0)
        magHeading += 360;
        if (magHeading > 360)
        magHeading -= 360;
        }
        
// Minimums        
        if (magMinX>mag_event.magnetic.x){
          magMinX=mag_event.magnetic.x;
        }
        if (magMinY>mag_event.magnetic.y){
          magMinY=mag_event.magnetic.y;
        }
        if (magMinZ>mag_event.magnetic.z){
          magMinZ=mag_event.magnetic.z;
        }
        if (accMinX>acc_event.acceleration.x){
          accMinX=acc_event.acceleration.x;
        }
        if (accMinY>acc_event.acceleration.y){
          accMinY=acc_event.acceleration.y;
        }
        if (accMinZ>acc_event.acceleration.z){
          accMinZ=acc_event.acceleration.z;
        }
        if (gyrMinX>gyr_event.gyro.x){
          gyrMinX=gyr_event.gyro.x;
        }
        if (gyrMinY>gyr_event.gyro.y){
          gyrMinY=gyr_event.gyro.y;
        }
        if (gyrMinZ>gyr_event.gyro.z){
          gyrMinZ=gyr_event.gyro.z;
        }
// Maximums
        if (magMaxX<mag_event.magnetic.x){
          magMaxX=mag_event.magnetic.x;
        }
        if (magMaxY<mag_event.magnetic.y){
          magMaxY=mag_event.magnetic.y;
        }
        if (magMaxZ<mag_event.magnetic.z){
          magMaxZ=mag_event.magnetic.z;
        }
        if (accMaxX<acc_event.acceleration.x){
          accMaxX=acc_event.acceleration.x;
        }
        if (accMaxY<acc_event.acceleration.y){
          accMinY=acc_event.acceleration.y;
        }
        if (accMaxZ<acc_event.acceleration.z){
          accMaxZ=acc_event.acceleration.z;
        }
        if (gyrMaxX<gyr_event.gyro.x){
          gyrMaxX=gyr_event.gyro.x;
        }
        if (gyrMaxY<gyr_event.gyro.y){
          gyrMaxY=gyr_event.gyro.y;
        }
        if (gyrMaxZ<gyr_event.gyro.z){
          gyrMaxZ=gyr_event.gyro.z;
        }
 
// Parse the GPS data
	if (GPS.newNMEAreceived()) {   
	GPS.parse(GPS.lastNMEA());
	if (!GPS.parse(GPS.lastNMEA()))   // This also sets the newNMEAreceived() flag to false
	return;  // We can fail to parse a sentence in which case we should just wait for another
	}           

// **** Light control
        if (GPS.speed>0.7){
        pixelCalibrate();
        }
        if (GPS.speed<0.6){
        pixelDance();
        }
        
        //Flash bright white light once at beginning for an easy video sync point.
  	if (!runOnce){
  	 for (int i = 0; i<8; i++){
  	  pixels.setPixelColor(i, pixels.Color(200,200,200)); 
  	  pixels.show();
  	 }
    	 Serial.println("Video Sync Point");
  	 delay(1000);
  	 for (int i = 0; i<8; i++){
  	  pixels.setPixelColor(i, pixels.Color(0,0,0)); 
  	  pixels.show(); 
  	 }
         runOnce = true;
  	}
          
//Write the data!
	logFile.print(GPS.hour, DEC); // UTC
	logFile.print(':');
	logFile.print(GPS.minute, DEC);
	logFile.print(':');
	logFile.print(GPS.seconds, DEC);
	logFile.print('.');
	logFile.print(GPS.milliseconds);
	logFile.print(",");   
        logFile.print(GPS.month, DEC); 
        logFile.print('/');
        logFile.print(GPS.day, DEC); 
        logFile.print("/20");
        logFile.print(GPS.year, DEC);        
	logFile.print(","); 
        logFile.print((int)GPS.satellites);
	logFile.print(","); 
        logFile.print((int)GPS.fixquality); 
	logFile.print(","); 
	logFile.print(GPS.latitude, 4);
	logFile.print(GPS.lat);
	logFile.print(",");
	logFile.print(GPS.longitude, 4);
	logFile.print(GPS.lon);
	logFile.print(",");
	logFile.print(GPS.altitude);
	logFile.print(",");
	logFile.print(GPS.speed*1.15078); // Converting knots to mph (1.15078) 
	logFile.print(",");
	logFile.print(GPS.angle);
	logFile.print(",");
	logFile.print(acc_event.acceleration.x);
	logFile.print(",");
	logFile.print(acc_event.acceleration.y);
	logFile.print(",");
	logFile.print(acc_event.acceleration.z);
	logFile.print(",");
	logFile.print(temperature);
	logFile.print(",");
	logFile.print(magHeading);
	logFile.print(",");
	logFile.print(alt);
	logFile.print(",");
	logFile.print(vio);
	logFile.print(",");
	logFile.print(mag_event.magnetic.x);
	logFile.print(",");
	logFile.print(mag_event.magnetic.y);
	logFile.print(",");
	logFile.print(mag_event.magnetic.z);
	logFile.print(",");
	logFile.print(gyr_event.gyro.x);
	logFile.print(",");
	logFile.print(gyr_event.gyro.y);
	logFile.print(",");
	logFile.print(gyr_event.gyro.z);
	logFile.print(",");
	logFile.print(accMinX);
	logFile.print(",");
	logFile.print(accMinY);
	logFile.print(",");
	logFile.print(accMinZ);
	logFile.print(",");
	logFile.print(accMaxX);
	logFile.print(",");
	logFile.print(accMaxY);
	logFile.print(",");
	logFile.print(accMaxZ);
	logFile.print(magMinX);
	logFile.print(",");
	logFile.print(magMinY);
	logFile.print(",");
	logFile.print(magMinZ);
	logFile.print(",");
	logFile.print(magMaxX);
	logFile.print(",");
	logFile.print(magMaxY);
	logFile.print(",");
	logFile.print(magMaxZ);
	logFile.print(gyrMinX);
	logFile.print(",");
	logFile.print(gyrMinY);
	logFile.print(",");
	logFile.print(gyrMinZ);
	logFile.print(",");
	logFile.print(gyrMaxX);
	logFile.print(",");
	logFile.print(gyrMaxY);
	logFile.print(",");
	logFile.println(gyrMaxZ);
	logFile.flush();    
              
        Serial.print("Time: ");
        Serial.print(GPS.hour, DEC); // UTC
	Serial.print(':');
	Serial.print(GPS.minute, DEC);
	Serial.print(':');
	Serial.print(GPS.seconds, DEC);
	Serial.print('.');
	Serial.print(GPS.milliseconds);
	Serial.print(" || "); 
        Serial.print("Date: ");
        Serial.print(GPS.month, DEC); 
        Serial.print('/');
        Serial.print(GPS.day, DEC); 
        Serial.print("/20");
        Serial.print(GPS.year, DEC);        
	Serial.print(" || "); 
        Serial.print("Satellites: "); 
        Serial.print((int)GPS.satellites);
	Serial.print(" || "); 
        Serial.print("Quality: "); 
        Serial.print((int)GPS.fixquality); 
	Serial.print(" || ");         
        Serial.print("Position: ");     
	Serial.print(GPS.latitude, 4);
	Serial.print(GPS.lat);
	Serial.print(" x ");
	Serial.print(GPS.longitude, 4);
	Serial.print(GPS.lon);
	Serial.print(" || ");   
        Serial.print("GPS Altitude: ");    
	Serial.print(GPS.altitude);
	Serial.print(" || ");    
	Serial.print("Speed: ");    
        Serial.print(GPS.speed*1.15078);
	Serial.print(" || ");      
	Serial.print("Course: ");    
	Serial.print(GPS.angle);
	Serial.print(" || "); 
	Serial.print("Accelerometer: X:");    
	Serial.print(acc_event.acceleration.x);
	Serial.print(" Y:");    
	Serial.print(acc_event.acceleration.y);
	Serial.print(" Z:");    
	Serial.print(acc_event.acceleration.z);
	Serial.print(" || ");   
	Serial.print("Gyro: X:");    
	Serial.print(gyr_event.gyro.x);
	Serial.print(" Y:");    
	Serial.print(gyr_event.gyro.y);
	Serial.print(" Z:");    
	Serial.print(gyr_event.gyro.z);
	Serial.print(" || ");   
	Serial.print("Mag: X:");    
	Serial.print(mag_event.magnetic.x);
	Serial.print(" Y:");    
	Serial.print(mag_event.magnetic.y);
	Serial.print(" Z:");    
	Serial.print(mag_event.magnetic.z);
	Serial.print(" || "); 
	Serial.print("Temp: ");    
	Serial.print(temperature);
	Serial.print(" || ");    
	Serial.print("Heading: ");    
	Serial.print(magHeading);
	Serial.print(" || ");      
	Serial.print("Barometric Altitude: ");    
	Serial.print(alt);
	Serial.print(" || ");    
	Serial.print("Variometer: ");    
	Serial.print(vio);
	Serial.print(" || ");    
	Serial.print("Accel Minimums: X:");    
	Serial.print(accMinX);
	Serial.print(" Y:");  
	Serial.print(accMinY);
	Serial.print(" Z:");  
	Serial.print(accMinZ);
	Serial.print(" Maximums: X:");    
	Serial.print(accMaxX);
	Serial.print(" Y:");  
	Serial.print(accMaxY);
	Serial.print(" Z:");  
	Serial.print(accMaxZ);
	Serial.print(" || ");    
	Serial.print("Mag Minimums: X:");    
	Serial.print(magMinX);
	Serial.print(" Y:");  
	Serial.print(magMinY);
	Serial.print(" Z:");  
	Serial.print(magMinZ);
	Serial.print(" Maximums: X:");    
	Serial.print(magMaxX);
	Serial.print(" Y:");  
	Serial.print(magMaxY);
	Serial.print(" Z:");  
	Serial.print(magMaxZ);
	Serial.print(" || ");    
	Serial.print("Gyro Minimums: X:");    
	Serial.print(gyrMinX);
	Serial.print(" Y:");  
	Serial.print(gyrMinY);
	Serial.print(" Z:");  
	Serial.print(gyrMinZ);
	Serial.print(" Maximums: X:");    
	Serial.print(gyrMaxX);
	Serial.print(" Y:");  
	Serial.print(gyrMaxY);
	Serial.print(" Z:");  
	Serial.println(gyrMaxZ);
// End Loop	
  }
/* End code */
