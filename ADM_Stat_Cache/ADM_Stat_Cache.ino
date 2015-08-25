              
//      *******  Stat Cache Beta 0.1
//      Motion Data Logging for Video Sync
//      Project code written by Frank Appio
//      Unless otherwise noted
        
//      For Arduino Mega, Adafruit Ultimate GPS logging shield, 
//      Adafruit 10dof sensor, Analog potentiometer, Analog button, Adafruit Neopixel Jewel
        
//      Connect GPS shield to Arduino Mega via headers        
//      Connect TX on GPS shield to RX pin 19 
//      Connect RX on GPS shield to TX pin 18
//      Connect 3V on 10DOF breakout to 3V on shield
//      Connect GND on 10DOF breakout to GND on shield
//      Connect SCL on 10DOF breakout to SCL pin 21
//      Connect SDA on 10DOF breakout to SDA pin 20
//      Connect GND on POT to GND on shield
//      Connect 5V on POT to 5V on shield
//      Connect ANALOG OUT on POT to pin 14 on Mega
//      Connect GND on Button to GND on shield
//      Connect 5V on Button to 5V on shield
//      Connect ANALOG OUT on Button to pin 12 on Mega
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
        #include "calibration.h"
	// GPS - SD
	#include <Adafruit_GPS.h>
	#include <SoftwareSerial.h>
	#include <SD.h>
	#include <SPI.h>
	#include <avr/sleep.h>
	// Other
	#include <Adafruit_NeoPixel.h>
	#include <Wire.h>
        #include "Kalman.h"
        #include "math.h"

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
        #define potPin 14
        #define buttonPin 12
	#define numPixels 7

// Create our log file
	File logFile;

// Initialize variables
	double camber = 0; 
	double grade = 0;
        double averageGrade = 0;
	double magHeading = 0;
	float temperature = 0;
	float bmpAltitude = 0;
        double steerAngle = 0;  
        double maxSpeed = 0;
        float lastLat = 0;
        float lastLon = 0;
        float origLat = 0;
        float origLon = 0;
        float origAlt = 0;
        double tripDistance = 0;
        double speedoCal = .18;
        double potVal = 0;
        int buttonState = 0;
        boolean brakesOn = false;
	boolean runOnce = false;
        boolean firstMove = false;             
        double accX, accY, accZ;
        double gyroX, gyroY, gyroZ;
        int tempRaw;
        long timer, timer2;      
        double gyroXangle, gyroYangle; 
        double kalRoll, kalPitch; 
        float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;       
        float alt=0, vio=0;
        long previousMillis=0;
        const float accelAlpha = 0.2;
        const float gyroAlpha = 0.2;
        const float magAlpha = 0.3;
        double fXa = 0;
        double fYa = 0;
        double fZa = 0;
        double fXg = 0;
        double fYg = 0;
        double fZg = 0;  
        double fXm = 0;
        double fYm = 0;
        double fZm = 0;
	Adafruit_NeoPixel pixels = Adafruit_NeoPixel(numPixels, ledPin, NEO_GRB + NEO_KHZ800);
        
// Create the Kalman instances
// From http://www.tkjelectronics.com
        Kalman kalmanX; 
        Kalman kalmanY;
        
// Variables for smoothing. Raising numReadings makes smoothing more aggressive. 
	const int numGaReadings = 3;
	const int numBaReadings = 3;
	const int numGdReadings = 3;
	const int numCaReadings = 3;
	const int numMhReadings = 3;
	const int numTmReadings = 9;
	int gaReadings[numGaReadings];      // the readings from the GPS.angle
	int baReadings[numBaReadings];      // the readings from the bmpAltitude
	int gdReadings[numGdReadings];      // the readings from the grade
	int caReadings[numCaReadings];      // the readings from the camber
	int mhReadings[numMhReadings];      // the readings from the magHeading
	int tmReadings[numTmReadings];      // the readings from the temperature
	int gaReadIndex = 0;              // the index of the current reading
	int baReadIndex = 0;              // the index of the current reading
	int gdReadIndex = 0;              // the index of the current reading
	int caReadIndex = 0;              // the index of the current reading
	int mhReadIndex = 0;              // the index of the current reading
	int tmReadIndex = 0;              // the index of the current reading
	int gaTotal = 0;                  // the running total
	int baTotal = 0;                  // the running total
	int gdTotal = 0;                  // the running total
	int caTotal = 0;                  // the running total
	int mhTotal = 0;                  // the running total
	int tmTotal = 0;                  // the running total
	int gaAverage = 0;                // the average
	int baAverage = 0;                // the average
	int gdAverage = 0;                // the average
	int caAverage = 0;                // the average
	int mhAverage = 0;                // the average
	int tmAverage = 0;                // the average

// Accel calibration for calculating G Force - using simple calibration method       
	int xRawMin = -121.85;
	int xRawMax = 121.85;
	int yRawMin = -119.75;
	int yRawMax = 119.75;
	int zRawMin = -11.1;
	int zRawMax = 11.1;

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// From https://github.com/adafruit/Adafruit-GPS-Library/tree/master/examples/shield_sdlog
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
	Serial.println("Arduino Mega Motion Datalogger");
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

// Set pin modes
	pinMode(chipSelect, OUTPUT);
        pinMode(potPin, INPUT);     
        pinMode(buttonPin, INPUT);     

//Initialize smoothing variables
          for (int thisReading = 0; thisReading < numGaReadings; thisReading++)
        gaReadings[thisReading] = 0;
          for (int thisReading = 0; thisReading < numBaReadings; thisReading++)
        baReadings[thisReading] = 0;
          for (int thisReading = 0; thisReading < numGdReadings; thisReading++)
        gdReadings[thisReading] = 0;
          for (int thisReading = 0; thisReading < numCaReadings; thisReading++)
        caReadings[thisReading] = 0;
          for (int thisReading = 0; thisReading < numMhReadings; thisReading++)
        mhReadings[thisReading] = 0;    
          for (int thisReading = 0; thisReading < numTmReadings; thisReading++)
        tmReadings[thisReading] = 0; 
        
// See if the card is present and can be initialized:
	if (!SD.begin(chipSelect,11,12,13)) {
	Serial.println("Card init. failed!");
	error(2);
	}
	char filename[15];
	strcpy(filename, "AMDLOG00.CSV");
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
	logFile.println("GPS Time,GPS Date,Satellites,Fix Quality,Latitude,Longitude,GPS Altitude,GPS Speed,GPS Course,Xaccel,Yaccel,Zaccel,Temperature,Average Grade,Grade,Camber,Heading,Steering Angle,BMP Altitude,Brakes,Max Speed,Distance Traveled,Variometer,GyroX,GyroY,GyroZ");
	logFile.flush();
  
// Connect to the GPS at the desired rate, set options
	GPS.begin(9600);
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
        // Set the update rate
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);   // 100 millihertz (once every 10 seconds), 1Hz or 5Hz update rate
        // Turn off updates on antenna status, if the firmware permits it
	GPS.sendCommand(PGCMD_NOANTENNA);
	useInterrupt(true);

// Initialize Kalman sensor fusion
        sensors_event_t acc_event;
        sensors_event_t mag_event;
        sensors_event_t bmp_event;
        sensors_event_t gyr_event;
        accX = acc_event.acceleration.x;
        accY = acc_event.acceleration.y;
        accZ = acc_event.acceleration.z;
        
        delay(100); // Wait for sensor to stabilize
        double roll  = atan2(-accX, accZ) * RAD_TO_DEG;
        double pitch = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    
        kalmanX.setAngle(roll); // Set starting angle - make sure the device is level on startup!
        kalmanY.setAngle(pitch);
        gyroXangle = roll;
        gyroYangle = pitch;          

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

// Converts lat/long from Adafruit degree-minute format to decimal-degrees
// From http://arduinodev.woofex.net/2013/02/06/adafruit_gps_forma/
double convertDegMinToDecDeg (float degMin) {
        double min = 0.0;
        double decDeg = 0.0;
        //get the minutes, fmod() requires double
        min = fmod((double)degMin, 100.0);
        //rebuild coordinates in decimal degrees
        degMin = (int) ( degMin / 100 );
        decDeg = degMin + ( min / 60 );
        return decDeg;
        }

// Distance Calculation
float calc_dist(float flat1, float flon1, float flat2, float flon2){
        flat1=convertDegMinToDecDeg(flat1);
        flon1=convertDegMinToDecDeg(flon1);
        flat2=convertDegMinToDecDeg(flat2);
        flon2=convertDegMinToDecDeg(flon2);
        float dist_calc=0;
        float dist_calc2=0;
        float diflat=0;
        float diflon=0;
        diflat=radians(flat2-flat1);
        flat1=radians(flat1);
        flat2=radians(flat2);
        diflon=radians((flon2)-(flon1));
        dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
        dist_calc2= cos(flat1);
        dist_calc2*=cos(flat2);
        dist_calc2*=sin(diflon/2.0);
        dist_calc2*=sin(diflon/2.0);
        dist_calc +=dist_calc2;
        dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
        dist_calc*=3958.74827; //Converting to fractions of a mile
        //Serial.println(dist_calc);
        return dist_calc;
        }

void pixelCompass(float headingValue){
        // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
        // Data from visual calibration = pixel 0 = center, pixel 1 = SE, pixel 2 = SW, pixel 3 = W, pixel 4 = NW, pixel 5 = NE, pixel 6 = E
	if (headingValue>0 && headingValue<60){
	for (int i = 0; i<8; i++){
	pixels.setPixelColor(i, pixels.Color(0,0,0)); 
	pixels.show(); }
	pixels.setPixelColor(5, pixels.Color(0,0,50)); 
	pixels.show(); }
	if (headingValue>60 && headingValue<120){
	for (int i = 0; i<8; i++){
	pixels.setPixelColor(i, pixels.Color(0,0,0)); 
	pixels.show(); }  
	pixels.setPixelColor(4, pixels.Color(0,0,50)); 
	pixels.show(); }    
	if (headingValue>120 && headingValue<180){
	for (int i = 0; i<8; i++){
	pixels.setPixelColor(i, pixels.Color(0,0,0)); 
	pixels.show(); }  
	pixels.setPixelColor(3, pixels.Color(0,0,50)); 
	pixels.show(); }
	if (headingValue>180 && headingValue<240){
	for (int i = 0; i<8; i++){
	pixels.setPixelColor(i, pixels.Color(0,0,0)); 
	pixels.show(); }  
	pixels.setPixelColor(2, pixels.Color(0,0,50)); 
	pixels.show(); }
	if (headingValue>240 && headingValue<300){
	for (int i = 0; i<8; i++){
	pixels.setPixelColor(i, pixels.Color(0,0,0)); 
	pixels.show(); }  
	pixels.setPixelColor(1, pixels.Color(0,0,50)); 
	pixels.show(); }
	if (headingValue>300 && headingValue<360){
	for (int i = 0; i<8; i++){
	pixels.setPixelColor(i, pixels.Color(0,0,0)); 
	pixels.show(); }  
	pixels.setPixelColor(6, pixels.Color(0,0,50)); 
	pixels.show(); } 
        // Center blinks green
        pixels.setPixelColor(0, pixels.Color(0,0,0)); 
	pixels.show();
        pixels.setPixelColor(0, pixels.Color(0,50,0)); 
	pixels.show();  
}
    
void pixelPowerUp(float speedValue){
        float interval = (5000/speedValue);
        unsigned long currentMillis = millis();
    	for (int i = 1; i<8; i++){
        pixels.setPixelColor(i, pixels.Color(0,0,0)); 
	pixels.show();
        if(currentMillis - previousMillis > interval) {
        previousMillis = currentMillis;       
        }
	pixels.setPixelColor(i, pixels.Color(0,0,50)); 
	pixels.show(); }
}

void pixelDance() {
        for (int i = 0; i<8; i++){
	pixels.setPixelColor(i, pixels.Color(0,0,0)); 
	pixels.show(); 
	pixels.setPixelColor(i, pixels.Color(random(0,1),random(0,20),random(0,20))); 
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
        
// Read from the potentiometer and map 10 bit values to degrees        
        potVal = analogRead(potPin);
        potVal = map(potVal, 0, 1023, 0, 190); // Potentiometer has range of +- 95 degrees
        
// Low pass filter
        fXa = acc_event.acceleration.x * accelAlpha + (fXa * (1.0 - accelAlpha));
        fYa = acc_event.acceleration.y * accelAlpha + (fYa * (1.0 - accelAlpha));
        fZa = acc_event.acceleration.z * accelAlpha + (fZa * (1.0 - accelAlpha));
        fXg = gyr_event.gyro.x * gyroAlpha + (fXg * (1.0 - gyroAlpha));
        fYg = gyr_event.gyro.y * gyroAlpha + (fYg * (1.0 - gyroAlpha));
        fZg = gyr_event.gyro.z * gyroAlpha + (fZg * (1.0 - gyroAlpha));
        fXm = mag_event.magnetic.x * magAlpha + (fXm * (1.0 - magAlpha));
        fYm = mag_event.magnetic.y * magAlpha + (fYm * (1.0 - magAlpha));
        fZm = mag_event.magnetic.z * magAlpha + (fZm * (1.0 - magAlpha));
 
 // Calculate G Force - convert raw values to 'milli-Gs"
	long xScaled = map(fXa, xRawMin, xRawMax, -1000, 1000);
	long yScaled = map(fYa, yRawMin, yRawMax, -1000, 1000);
	long zScaled = map(fZa, zRawMin, zRawMax, -1000, 1000);
        // Re-scale to fractional Gs
	float xAccel = xScaled / 1000.0;
	float yAccel = yScaled / 1000.0;
	float zAccel = zScaled / 10000.0;

 // Calculate Pitch and Roll using Kalman filter 
        accX = xAccel;
        accY = yAccel;
        accZ = zAccel;
        gyroX = fXg;
        gyroY = fYg;
        gyroZ = fZg;
        double dt = (double)(micros() - timer) / 250000; // Calculate delta time
        timer = micros();

        double roll  = atan2(-accX, accZ) * RAD_TO_DEG;
        double pitch = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
      
        double gyroXrate = gyroX * 57.2957795; // Convert to deg/s
        double gyroYrate = gyroY * 57.2957795; // Convert to deg/s
      
        kalRoll = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
        
        if (abs(kalRoll) > 90)
        gyroYrate = -gyroYrate; 
        kalPitch = kalmanY.getAngle(pitch, gyroYrate, dt);
    
        if ((pitch < -90 && kalPitch > 90) || (pitch > 90 && kalPitch < -90)) {
        kalmanY.setAngle(pitch);
        kalPitch = pitch;
        gyroYangle = pitch;
        } 
        else
        kalPitch = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
        
        if (abs(kalPitch) > 90)
        gyroXrate = -gyroXrate; 
        kalRoll = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
        
        //gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
        //gyroYangle += gyroYrate * dt;
        gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
        gyroYangle += kalmanY.getRate() * dt;
        
        // Reset the gyro angle when it has drifted too much
        if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalRoll;
        if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalPitch; 

// Calculate the altitude using the barometric pressure sensor 
        if (bmp_event.pressure)
        {
        // Get ambient temperature in C, convert to F
        bmp.getTemperature(&temperature);
        temperature = temperature*1.8+32;
        // Convert atmospheric pressure, SLP and temp to altitude 
        float altNew=bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature);
        altNew*=3.28084; // Convert meters to feet
        vio = (altNew-alt);
        alt = altNew;
        }

// Compass heading from magnetometer
        if (dof.fusionGetOrientation(&acc_event, &mag_event, &orientation))
        {
        magHeading = 360-orientation.heading;
        if (magHeading < 0)
        magHeading += 360;
        if (magHeading > 360)
        magHeading -= 360;
        }
 
// Parse the GPS data
	if (GPS.newNMEAreceived()) {   
	GPS.parse(GPS.lastNMEA());
	if (!GPS.parse(GPS.lastNMEA()))   // This also sets the newNMEAreceived() flag to false
	return;  // We can fail to parse a sentence in which case we should just wait for another
	}           
          
// **** Data smoothing 
        // Subtract the last reading:
	gaTotal = gaTotal - gaReadings[gaReadIndex];
	baTotal = baTotal - baReadings[baReadIndex];
	gdTotal = gdTotal - gdReadings[gdReadIndex];
	caTotal = caTotal - caReadings[caReadIndex];
	mhTotal = mhTotal - mhReadings[mhReadIndex];  
	tmTotal = tmTotal - tmReadings[tmReadIndex];
        // Read from the sensor:
	gaReadings[gaReadIndex] = GPS.angle;
	baReadings[baReadIndex] = alt;
	gdReadings[gdReadIndex] = kalPitch; //grade;
	caReadings[caReadIndex] = kalRoll; //camber;
	mhReadings[mhReadIndex] = magHeading;  
	tmReadings[tmReadIndex] = temperature;
	gaTotal = gaTotal + gaReadings[gaReadIndex];
	baTotal = baTotal + baReadings[baReadIndex];
	gdTotal = gdTotal + gdReadings[gdReadIndex];
	caTotal = caTotal + caReadings[caReadIndex];
	mhTotal = mhTotal + mhReadings[mhReadIndex];  
	tmTotal = tmTotal + tmReadings[tmReadIndex];
        // Advance to the next position in the array:
	gaReadIndex = gaReadIndex + 1;
	baReadIndex = baReadIndex + 1;
	gdReadIndex = gdReadIndex + 1;
	caReadIndex = caReadIndex + 1;
	mhReadIndex = mhReadIndex + 1;  
	tmReadIndex = tmReadIndex + 1;
        // At the end of the array wrap around to the beginning
	if (gaReadIndex >= numGaReadings)
	gaReadIndex = 0;
	if (baReadIndex >= numBaReadings)
	baReadIndex = 0;   
	if (gdReadIndex >= numGdReadings)
	gdReadIndex = 0;    
	if (caReadIndex >= numCaReadings)
	caReadIndex = 0;
	if (mhReadIndex >= numMhReadings)
	mhReadIndex = 0;    
	if (tmReadIndex >= numTmReadings)
	tmReadIndex = 0;    
        // Calculate the average:
	gaAverage = gaTotal / numGaReadings;
	baAverage = baTotal / numBaReadings;
	gdAverage = gdTotal / numGdReadings;
	caAverage = caTotal / numCaReadings;
	mhAverage = mhTotal / numMhReadings;  
	tmAverage = tmTotal / numTmReadings;
        
// Calculate steering angle (-95 to 95, because the pot has 190 degrees of sweep after pulley ratio)
        // Pot centered on steering angle of 0
        steerAngle = (potVal-95);

// Calculate max speed
        if (maxSpeed<(GPS.speed*(1.15078+speedoCal))){
        maxSpeed = GPS.speed*(1.15078+speedoCal);
        }

// Calculate distance traveled in miles
        if (lastLat!=0 && GPS.speed>1){ 
        tripDistance+=calc_dist(lastLat,lastLon,GPS.latitude,GPS.longitude);
        }
        lastLat=GPS.latitude;
        lastLon=GPS.longitude;
        
// Calculate average grade
        if (origLat!=0 && origAlt!=0){         
        float tempDistance = calc_dist(origLat,origLon,GPS.latitude,GPS.longitude);
        float tempVertical = baAverage-origAlt; 
        tempDistance*=5280; // Converting fractions of miles back into feet
        averageGrade = (tempVertical/tempDistance)*100; // Slope to percent
        }

// Set our origin the first time we move
        if (GPS.speed>5 && firstMove==false){
        origLat=GPS.latitude;
        origLon=GPS.longitude;
        origAlt=baAverage;
        firstMove = true;
        }
        
// **** Light control
        // Brake light on center pixel. 
        brakesOn = false;
        
        // Switch if statements when using brake lever return button 
        //buttonState = digitalRead(buttonPin);
        //if (buttonState != HIGH){
        if (yAccel<-.375){ 
          
        brakesOn = true;
        }
        if (brakesOn){
        for (int i = 0; i<8; i++){
	pixels.setPixelColor(i, pixels.Color(0,0,0)); 
	pixels.show(); 
	pixels.setPixelColor(i, pixels.Color(100,0,0)); 
	pixels.show(); 
        }  
        }
        // Active pixel tells us the compass heading by only lighting the northmost pixel.                 
        if (!brakesOn){
        if (GPS.speed>0.7 && steerAngle<15){
        pixelCompass(mhAverage);
        // If we are drifting lets spin the light
        if (steerAngle>15){
        pixelPowerUp(GPS.speed);  
        }
        }
        if (GPS.speed<0.6){
        // Or if we're stopped just randomize pixels and look nice.          
        pixelDance();
        }
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
	logFile.print(GPS.hour-4, DEC); // UTC - 4 = EST
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
	logFile.print(GPS.speed*(1.15078+speedoCal)); // Converting knots to mph (1.15078) plus calibration coefficient
	logFile.print(",");
	logFile.print(gaAverage);
	logFile.print(",");
	logFile.print(xAccel*-1);
	logFile.print(",");
	logFile.print(yAccel*-1);
	logFile.print(",");
	logFile.print(zAccel);
	logFile.print(",");
	logFile.print(tmAverage);
	logFile.print(",");
	logFile.print(averageGrade);
	logFile.print(",");
	logFile.print(gdAverage);
	logFile.print(",");
	logFile.print(caAverage);
	logFile.print(",");
	logFile.print(mhAverage);
	logFile.print(",");
	logFile.print(steerAngle);
	logFile.print(",");
	logFile.print(baAverage);
	logFile.print(",");
	logFile.print(brakesOn);
	logFile.print(",");
	logFile.print(maxSpeed);
	logFile.print(",");
	logFile.print(tripDistance);
	logFile.print(",");
	logFile.print(vio);
	logFile.print(",");
	logFile.print(gyroXrate);
	logFile.print(",");
	logFile.print(gyroYrate);
	logFile.print(",");
	logFile.println(gyroZ);
	logFile.flush();    
    
//Uncomment this section to debug with serial monitor
        /*       
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
        Serial.print(GPS.speed*(1.15078+speedoCal));
	Serial.print(" || ");      
	Serial.print("Course: ");    
	Serial.print(gaAverage);
	Serial.print(" || "); 
	Serial.print("G Force: X:");    
	Serial.print(xAccel*-1);
	Serial.print(" Y:");    
	Serial.print(yAccel*-1);
	Serial.print(" Z:");    
	Serial.print(zAccel);
	Serial.print(" || ");   
	Serial.print("Gyro: X:");    
	Serial.print(gyroXrate);
	Serial.print(" Y:");    
	Serial.print(gyroYrate);
	Serial.print(" Z:");    
	Serial.print(gyroZ);
	Serial.print(" || "); 
	Serial.print("Temp: ");    
	Serial.print(tmAverage);
	Serial.print(" || ");    
	Serial.print("Average Grade: ");    
	Serial.print(averageGrade);
	Serial.print(" || ");    
	Serial.print("Grade: ");    
	Serial.print(gdAverage);
	Serial.print(" || ");    
	Serial.print("Camber: ");    
	Serial.print(caAverage);
	Serial.print(" || ");    
	Serial.print("Heading: ");    
	Serial.print(mhAverage);
	Serial.print(" || ");    
	Serial.print("Steering Angle: ");    
	Serial.print(steerAngle);
	Serial.print(" || ");    
	Serial.print("Barometric Altitude: ");    
	Serial.print(baAverage);
	Serial.print(" || ");    
	Serial.print("Brakes: ");    
	Serial.print(brakesOn);
	Serial.print(" || ");    
	Serial.print("Max Speed: ");    
	Serial.print(maxSpeed);
	Serial.print(" || ");    
	Serial.print("Distance: ");    
	Serial.print(tripDistance);
	Serial.print(" || ");
	Serial.print("Variometer: ");    
	Serial.println(vio);
        */       
// End Loop	
  }
/* End code */
