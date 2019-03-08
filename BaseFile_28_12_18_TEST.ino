// Auto Configuration

//changed PREPARENEXTDATA()........
// TRY  Check_If_No_LAN();


//--------------------------------------------------------------------------
// ************** Things to Define before compiling ************************
// Define if RTC or UDP Time
// Define GSM
// Define Depth or -1
// Define Tank Offset
// Define ID
// Define Pumps
// Define Depthmeter
// Define LCD
// Define Mac Address
// Define IP
// Define Hardware Watchdog AND Pin
//---------------------------------------------------------------------------

// ---------------   Global Includes
#include <Ethernet.h>   // For Ethernet Shiels
#include <SD.h>         // Sdcard Library
#include <Time.h>       // Time Library
#include <NewPing.h>    //  Echo Library
#include <EEPROM.h>     // EEPROM Library
#include <Wire.h>
#include <ICMPPing.h>


String Version="2.0.0.1";

//@@@-@ VARIABLES HERE

#define SDFILE      "data.txt"
#define ID            22147
#define POSITION      "MOYSIOTITSA"
#define PUMP1         38
#define PUMP2         36
#define PUMP3         5//34
#define PUMP4         5//32
#define PUMP5         5//30
#define EEPROMGUARD 1

#define DEBUG 1
//#define TIME_DEB 1
//#define HTTP_DEB 1
#define ESSENTIALS 1
//#define PRINT_SD


#define LED_BAR 1
#ifdef LED_BAR
	int LED_BLINK = 500;
	byte _led_value = 0; //-----------------------------------------------------------------------
	char led_string[] = "00000001";
	static unsigned int ledStatus = 1;
	static unsigned long ledBlinkTime = 0;
#endif


//#define GSM_ 1
#ifdef GSM_
	int ch = 0;
	String val = "";
	boolean SimCard = true;
	#define GSM_Master_Ring_No  "6937807458"
	#define GSM_Response_NO  "6937807458"
	#define MASTER_RING 1
#endif

#define RTCTIME       1
#ifdef RTCTIME
	#include <DS3231.h>
	#define DS3231_I2C_ADDRESS 0x68
	DS3231  rtc(SDA, SCL);
	Time  t;
#endif

#define WATCHDOG   1
#ifdef WATCHDOG
	static const uint8_t analog_pins[] = {A0, A1, A2, A3, A4};
	int Watchdog_Pin = 48;
	float DISC_TIME = 20000; // time in millis
	unsigned long _StartTime = millis();
	unsigned long _CurrentTime = millis();
	unsigned long _ElapsedTime = _CurrentTime - _StartTime;
#endif

#ifdef LIQUIDCRYSTAL
	#include <LiquidCrystal_I2C.h>
	LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7);
	byte btn = 0; // button flag
	const byte buttonPin = 0;
	byte buttonState = 0;
	byte lastbuttonState = 0;
	unsigned long lcdOn;
#endif

//#define DEPTHMETER   1
#ifdef DEPTHMETER
	#define SAMPLING_TIME 200
	#define DEPTH         290
	#define TANKOFFSET    25
	byte Echo = 44;
	byte Trig = 42;
	NewPing sonar(Trig, Echo, 500);
	int depth_measure() {
		long distance = sonar.ping_cm();
		return distance;
	}
#endif
#ifndef DEPTHMETER
	#define DEPTH 0
#endif


const byte Pump_1_pin = 38;
const byte Pump_2_pin = 36;
const byte Pump_3_pin = 34;
const byte Pump_4_pin = 32;
const byte Pump_5_pin = 30;
byte Flag_Pump1 = 1; // PUMPS ARE ACTIVE HIGH WHEN OFF
byte Flag_Pump2 = 1;
byte Flag_Pump3 = 1;
byte Flag_Pump4 = 1;
byte Flag_Pump5 = 1;


File myFile;
char _reading[15];
byte lastHour = 0;
byte lastMin = 0;
byte pumpChange = 0;
byte i = 0;
bool legitRequest = false;

#define MAC "0x00,0x02,0x02,0x01,0x05,0x09"
#define IPADDRESS "10,160,117,137"
#define GATEWAY "10,160,117,7"

byte mac[] = {0x00, 0x02, 0x02, 0x01, 0x05, 0x09};
IPAddress ip(10, 160, 117, 137);
IPAddress gateway (10, 160, 117, 7);
IPAddress _dns(8, 8, 8, 8);
IPAddress subnet(255, 255, 254, 0);
EthernetServer server(80);
EthernetClient client;



// !@@@@! VARIABLES END


IPAddress pingAddr(8, 8, 8, 8);



// ************** Handle Incomming HTTP Connections *************
// ************** Search  incomming string for '?' character **************
// ************** If found check next one ************************
// ************** if next is '5' sent is allive answer ***********
// ************** If '9' then it is ack for received from server data and DELETE File ************
// ************** If it is '4' Then send DATA **********************
// ************** TODO  Make it Better ***********

bool readRequest(EthernetClient& client) {
	bool currentLineIsBlank = true;
	bool reading;
	legitRequest = false;
	i = 0;
	while (client.connected()) {
		if (client.available()) {
			char c = client.read();
			if (c == '?') reading = true;
			if (reading) {
				// If next character is legit prepare flag for answer
				if (c != '?') reading = false;
				// If request is just to see if alive
				if (c == '5') {
					client.println(F("HTTP/1.1 200 OK"));
				}
				// --- Ack for delete
				if (c == '9') { //--------------- Get Ack from Server
					#ifdef DEBUG
							  Serial.println("Ack Received .. Deleting Data...");
							  client.println(F("HTTP/1.1 200 OK"));
					#endif
					//deleteFile();//resetFunc();
					prepareNextData();  // Deletes file too.
				}
				// If request is for data
				if (c == '4') legitRequest = true;
			}
			#ifdef HTTP_DEB
				  Serial.print(c);
			#endif
			i++;
			if (i == 6 && c == 'f') // kill /favicon requests!!
				return false;
			if (c == '\n' && currentLineIsBlank) {
				return true;
			} else if (c == '\n') {
				currentLineIsBlank = true;
			} else if (c != '\r') {
				currentLineIsBlank = false;
			}
		}
	  }
	Serial.println(F(""));
	return false;
}




void(* _resetFunc) (void) = 0; //declare reset function at address 0

//**********  This One calls reset BUT *********

void resetFunc () {
	#ifdef EEPROMGUARD
		byte value = EEPROM.read(0);
		if (value == 10) value = 20;
		EEPROM.update(0, value);
	#endif
	 _resetFunc();
}



///  ----------------------  Setup ---------------------------

void setup() {

    //****************** Reset Watchdog Timer  ******************
    #ifdef WATCHDOG
      pinMode(Watchdog_Pin, OUTPUT);
      digitalWrite(Watchdog_Pin, LOW);
      delay(500);
      pinMode(Watchdog_Pin, INPUT);
    #endif
    
    // ****************** Initialize Ethernet ***************
    Ethernet.begin(mac, ip, _dns, gateway, subnet);
    Serial.begin(9600);
    server.begin();
    
    // ****************** Initialize GSM if exists ***************
    #ifdef GSM_
      Serial3.begin(115200);           // We have to set m590 baudrate to 9600 via comp rs232
      delay(5000);
    #ifdef DEBUG
      Serial.println("Trying GSM...");
    #endif
      Serial3.print ("AT+CCID\r");
      
    //  while (Serial3.available()){
    //    String smc_str=Serial3.readString();
    //    if (smc_str.indexOf("ERROR")){
    //      SimCard=false;
    //      Serial.println("No Sim Card...");
    //      Serial.println(smc_str);
    //    }
    //  }
    
    //if (SimCard) {
          
          Serial3.print("AT+CLIP=1\r");
        //  delay(2000);
          Serial3.print("AT+CMGF=1\r"); // SMS mode 1=text
         // delay(2000);
          Serial3.print("AT+CSCS=\"GSM\"\r");  //This command is to set TE character set
       //   delay(2000);
        
        #ifdef DEBUG
          Serial.println("SMS Indication format");
        #endif
          Serial3.print("AT+CNMI=2,2,0,0,0\r");
          delay(2500);
       
        #ifdef DEBUG
          Serial.println("Ready...");
        #endif
          Serial3.print("AT+CMGD=1,4\r"); // delete all SMS
        #ifdef DEBUG
          Serial.println("delete all SMS"); // delete all SMS
        #endif
        //  delay(2500);
        #ifdef DEBUG
          Serial.println ("Connected to");
        #endif
          Serial3.print("AT+COPS?\r");
         // delay(2000);
    //}
    #endif
    
    // ****************** Initialize RTC and Intenal Clock ***************
    #ifdef RTCTIME
      rtc.begin();
      delay(100);
      t = rtc.getTime();
      setTime(t.hour, t.min, t.sec, t.date, t.mon, t.year);
    #endif
    
    // ****************** Initialize LCD if existst ***************
    #ifdef LIQUIDCRYSTAL
      pinMode(buttonPin, INPUT);
      lcd.begin(16, 2);
      lcd.setBacklightPin(3, POSITIVE);
      lcd.setBacklight(LOW);
    #endif
    
    
    // ****************** Initialize Pump Inputs ***************
      if (PUMP1 != 5) pinMode (Pump_1_pin, INPUT_PULLUP);
      if (PUMP1 != 5) pinMode (Pump_2_pin, INPUT_PULLUP);
      if (PUMP1 != 5) pinMode (Pump_3_pin, INPUT_PULLUP);
      if (PUMP1 != 5) pinMode (Pump_4_pin, INPUT_PULLUP);
      if (PUMP1 != 5) pinMode (Pump_5_pin, INPUT_PULLUP);
    
      
    // ****************** Initialize SD Card ***************
    #ifdef ESSENTIALS
      Serial.println(F("Initializing SD card..."));
    #endif
    if (!SD.begin(4)) {
        #ifdef ESSENTIALS
            Serial.println(F("ERROR - SD card initialization failed!"));
        #endif
        Serial.println(F("RESETING - SD card initialization failed!"));
        delay(100);
        //resetFunc();  //-----------------------------------------------------------------------------------
        //return;    //  NO SD CARD OR BAD CARD init failed
    } else {
        #ifdef ESSENTIALS
          Serial.println(F("SUCCESS - SD card initialized."));
        #endif
    }
    
    // ****************** Initialize EEPROM ***************
    // ****************** WE HAVE TO SEE THIS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ***************
    #ifdef EEPROMGUARD
		byte value = EEPROM.read(0);
		Serial.print ("EEPROM value ");
		Serial.println(value);
		if (value == 20) {
			value = 10;
			deleteFile();
		} else value = 10;
		EEPROM.update(0, value);
    #else
		deleteFile();
    #endif
    
    
    #ifdef ESSENTIALS
		Serial.print(F("SRAM = "));
		Serial.println(freeRam());
		#endif
    
    // ****************** Initialize UDP ***************
    // ****************** DELETE THIS !!!!!!!!!!!!!!!!!!!! ***************
    #ifdef UDPTIME
		Serial.println(F("waiting for UDP sync"));
		Udp.begin(localPort);
		setSyncProvider(getNtpTime);
		timeSetup();
    #endif
    
    // ****************** IMPORTANT ***************
    // ****************** If Depthmeter is on will write data on card ***************
    // ****************** If not we hace to write data from pumps ***************
    // ****************** If no data on card it will BREAK CODE ***************
    #ifndef DEPTHMETER  // no need for data if depthmeter on will read on first cycle
		  if (!SD.exists(SDFILE)) prepareNextData();
    #endif
    
    // ****************** Initialize Depthmeter if exists ***************
    #ifdef DEPTHMETER
		pinMode(Echo, INPUT);
		pinMode(Trig, OUTPUT);
		// Int Depthmeter;
		depth_measure();
    #endif
    
    // ****************** Initialize LED BAR if exists ***************
    // ****************** !!!  TODO..... ***************
    // ****************** DEFINE LED_BAR ***************
    #ifdef LED_BAR
		Wire.begin(); // wake up I2C bus
		// set I/O pins to outputs
		Wire.beginTransmission(0x20);
		Wire.write(0x00); // IODIRA register
		Wire.write(0x00); // set all of port A to outputs
		Wire.write(0x12); // GPIOA
		Wire.endTransmission();
    #endif
}

///  ----------------------  Setup ---------------------------




#ifdef GSM_
	void sms(String text, String phone) {
		#ifdef DEBUG
			Serial.println("SMS send started");
		#endif
		Serial3.println("AT+CMGS=\"" + phone + "\"");
		delay(500);
		Serial3.print(text);
		delay(500);
		Serial3.print((char)26);
		Serial3.print((char)13);
		Serial3.print((char)10);
		delay(500);
		#ifdef DEBUG
			Serial.println("SMS send complete");
		#endif
		delay(2000);
	}
#endif



// ****************** SERIAL USER INTERFACE ***************
void printHelpMsg() {
  Serial.println("------Serial Command Manual -------------------");
  Serial.println("H  >> Display This Message");
  Serial.println("I  >> Display station Info");
  Serial.println("T  >> Display Time info");
  Serial.println("G  >> Display GSM info");
  Serial.println("R  >> WipeOut Stored data !!!! (Will Reboot) ");
  Serial.println("F  >> Print File Data  !!! (May be a Looooong String");
  Serial.println("C  >> Check Internet Connection");
}

void communicateWithGSM (String command){
  
#ifdef GSM_
  if (Serial3.available())
  Serial3.print(command);
  
  while (Serial3.available()) Serial.println(Serial3.read());
#endif

}

void printGSMInfo() {
#ifdef GSM_
  Serial.println ("GSM INFO ---------");
  //Serial3.println("ATI");
  //Serial.print ("Connected to : ");
  //Serial3.println("AT+COPS?");
  //Serial3.write("Sim Card CCID\r");
  Serial3.write("AT+CCID\r");
//  Serial3.write("AT+CREG\r");
  //  communicateWithGSM ("AT+CCID\r\n");
   // communicateWithGSM ("AT+CREG\r\n");
   
#else
  Serial.println ("--------- NO GSM AVAILABLE  ------------");
#endif

}

void printInfo() {

  Serial.println("-------------------INFO START------------------");
  Serial.print("ID : ");
  Serial.println (ID);
  Serial.print("Position : ");
  Serial.println (POSITION);
  Serial.print("Bios Version : ");
  Serial.println (Version);
  Serial.print("MAC address : ");
  Serial.println(MAC);
  Serial.print("IP Address: ");
  Serial.println(IPADDRESS);
  Serial.print("GATEWAY: ");
  Serial.println(GATEWAY);
  Serial.print("FREE RAM: ");
  Serial.println(freeRam());


#ifdef DEPTHMETER
  Serial.println ("Depthmeter online");
  Serial.print ("Samlpling time: ");
  Serial.println (SAMPLING_TIME);
  Serial.print ("Depth: " );
  Serial.println (DEPTH);
  Serial.print ("TankOffset: ");
  Serial.println (TANKOFFSET);
#endif

  if (PUMP1 != 5) Serial.println("PUMP 1 ONLINE");
  if (PUMP2 != 5) Serial.println("PUMP 2 ONLINE");
  if (PUMP3 != 5) Serial.println("PUMP 3 ONLINE");
  if (PUMP4 != 5) Serial.println("PUMP 4 ONLINE");
  if (PUMP5 != 5) Serial.println("PUMP 5 ONLINE");

#ifdef GSM_
  Serial.println ("GSM INFO ---------");
 
  Serial.print ("GSM Master No: ");
  Serial.println (GSM_Master_Ring_No);
  Serial.print ("GSM Response No: ");
  Serial.println (GSM_Response_NO);
 
#endif
  Serial.println("-------------------INFO END------------------");
}

void printTime() {
  Serial.println ("--------------- SYSTEM TIME ------------");
  Serial.print("Date: ");
  Serial.print(year());
  Serial.print("-");
  Serial.print(month());
  Serial.print("-");
  Serial.println(day());

  Serial.print("Time: ");
  Serial.print(hour());
  Serial.print(":");
  Serial.print(minute());
  Serial.print(":");
  Serial.println(second());
  Serial.println ("---------------------------------------");
}



void printIfConnected() {

  Serial.println("...... Trying Internet connection...");
  EthernetClient client;
  char server[] = "www.google.com";
  if (client.connect(server, 80)) {
    Serial.println("Internet connection available..");
  } else Serial.println ("Internet connection NOT Available");

}


void printFile() {
  printSD();
}

void rotateFile() { // For Serial Command
  deleteFile();//resetFunc();
  delay(100);
  prepareNextData();
}


// --------------- SERIAL USER INERFACE END -----------------------------


// ----- This is used as FAILSAFE for stuck LAN-------
// ----- try to connect to rourer. -------------------
// ----- If no connection RESET ----------------------

void Check_If_No_LAN() {
	
	#ifdef DEBUG
		Serial.println("...... Trying LAN connection...");
	#endif
	EthernetClient client;
	if (client.connect(gateway, 80)) {
		#ifdef DEBUG
			Serial.println("LAN connection available..");
		#endif
	} else {
		#ifdef DEBUG
			Serial.println ("LAN connection NOT Available");
			Serial.println ("Resseting");
		#endif
		delay (1000);
		 _resetFunc();
	}
}


// ----- This funnction is used with LCD and BUTTON
// ----- It Senses for button press on DELAY

#ifdef LIQUIDCRYSTAL
	void delayWithRelay(uint32_t del, int waitFor) {
		uint32_t timestamp = millis();
		while (millis() - timestamp < del) {
			if (waitFor == 1) {
				buttonState = digitalRead(buttonPin);
				// check if the pushbutton is pressed.
				// if it is, the buttonState is HIGH:
				if (buttonState == HIGH && buttonState != lastbuttonState) {
					lastbuttonState = buttonState;
					if (btn >= 5) btn = 0;
					lcdPrint();
					lcdOn = millis();
					Serial.println(F("Button Pressed"));
				} else if (buttonState == LOW) {
					if (millis() - lcdOn > 10000)
					{
						lcd.clear();
						lcd.setBacklight(LOW);
					}
					lastbuttonState = buttonState;
				}
			}
		}
	}
#endif


//------- Takes a ONE BYTE char string "00000000" to "11111111" ------
//--------- and returns a value from 0 to 255 --------------
#ifdef LED_BAR
	byte led_value (char* s) {
		int value = 0;
		for (int i = 0; i < strlen(s); i++) {
			value *= 2;
			if (s[i] == '1') value++;
		}
		return value;
	}
#endif


// ****************** Watchdog Timer DISCHARGE Function ***************
void WatchdogDischarge(){
	#ifdef WATCHDOG
		 _CurrentTime = millis();
		 _ElapsedTime = _CurrentTime - _StartTime;
		 if (_ElapsedTime > DISC_TIME) {
			pinMode(Watchdog_Pin, OUTPUT);
			digitalWrite(Watchdog_Pin, LOW);
			Serial.println(F("Watchdog DisCharge"));
			delay (800);
			//delayWithRelay(800, 1);
			pinMode(Watchdog_Pin, INPUT);
			_StartTime = millis();
		 }
	#endif
}

// -------------------------- MAIN LOOP --------------------------------------


// ------  Init Some Variables that used on loop -------
String a;
byte entries = 1;
long file_cursor_position = 0;
byte cheat = 0;

void loop() {

// ******************  Check For User input from serial and launch Response ***************
  while (Serial.available() > 0) {
	#ifdef DEBUG
		//Serial.println(F("Serial Command Entered"));
	#endif
    // Get Commands from serial
    char command = Serial.read();
    switch (command) {
      case 'I':
        printInfo();
        break;
      case 'T':
        printTime();
        break;
      case 'C':
        printIfConnected();
        break;
      case 'H':
        printHelpMsg();
        break;
      case 'G':
        printGSMInfo();
        break;
      case 'F':
        printFile();
        break;
      case 'R':
        rotateFile();
        break;

    }


  }


	// ****************** Start Web Server ***************
	// ****************** file_cursor_position: Holds Cursor position on file  ***************
	#ifdef DEBUG
		//Serial.println(F("Starting webserver"));
	#endif	 
	 webSrvr();
	  file_cursor_position = 0;


	  // ****************** Take Depth mesure ***************  
	#ifdef DEPTHMETER
		#ifdef DEBUG
		//	Serial.println(F("Reading from DEPTHMETER"));
		#endif
		if (timeToRead(SAMPLING_TIME))  writeIntoFile();
	#endif


	// ****************** Master ring From GSM Handle ***************
	#ifdef GSM_
		#ifdef MASTER_RING
			 // ------ discharge here because may be long task -------
			  WatchdogDischarge();
			  if (Serial3.available()>0) {
					 while (Serial3.available()) {
					   ch = Serial3.read();
					   val += char(ch);
					   WatchdogDischarge();
				   
					 }
					Serial.println(val);
					#ifdef DEPTHMETER
						if (val.indexOf("RING") > -1) {
						  if (val.indexOf(GSM_Master_Ring_No) > -1) {
							Serial.println("--- MASTER RING DETECTED ---");
							int distanceFromSurface = depth_measure();
							int waterLevel = DEPTH - distanceFromSurface;
							Serial3.println("ATH0");
							delay(500);
							sms(String(waterLevel), String(GSM_Response_NO));
							delay(500);
						  } else Serial.println(val);
						val = "";
						} else {
						  if (val.indexOf("sendsms") > -1) {
							sms(String("hello world"), String(GSM_Response_NO));
							val = "";
							delay(500);
						  }
						}
					#endif
			  }
		#endif
	#endif



	// ****************** Pump Read***************

  
	if ((digitalRead(Pump_1_pin) != Flag_Pump1 && PUMP1 != 5) || ((digitalRead(Pump_2_pin) != Flag_Pump2) && PUMP2 != 5) || ((digitalRead(Pump_3_pin) != Flag_Pump3) && PUMP3 != 5) || ((digitalRead(Pump_4_pin) != Flag_Pump4) && PUMP4 != 5) || ((digitalRead(Pump_5_pin) != Flag_Pump5) && PUMP5 != 5))
	{
		
		#ifdef DEBUG
			Serial.println(F("PUMP Read"));
		#endif
		
		
		if (PUMP1 != 5) Flag_Pump1 = digitalRead(Pump_1_pin);
		if (PUMP2 != 5) Flag_Pump2 = digitalRead(Pump_2_pin);
		if (PUMP3 != 5) Flag_Pump3 = digitalRead(Pump_3_pin);
		if (PUMP4 != 5) Flag_Pump4 = digitalRead(Pump_4_pin);
		if (PUMP5 != 5) Flag_Pump5 = digitalRead(Pump_5_pin);
		writeIntoFile();

		Serial.println (Flag_Pump1);
		Serial.println (Flag_Pump2);
		Serial.println (Flag_Pump3);
		Serial.println (Flag_Pump4);
		Serial.println (Flag_Pump5);

		#ifdef DEBUG
			Serial.println(F("Change on Pump Occured"));
		#endif

		// ----- Check if no lan -----
		// ----- In here in order to run all the time in LOOP ------
		// ----- discards the need for timer ----------
		Check_If_No_LAN();
	}
  
  
	// ****************** Update Led Bar and HEART BEAT***************
	#ifdef LED_BAR
		  //-------------------------INPUTS ACTIVE HIGH------------------------------------------------------
		#ifdef DEBUG
			//Serial.println(F("LED BAR Routine"));
		#endif
		if ((long)(millis() - ledBlinkTime) >= 0)
		{
			if (led_string[7] != '1')
			{
				led_string[7] = '1';
				LED_BLINK = 500;
			} else {
				led_string[7] = '0';
				LED_BLINK = 2000;
			}
			ledBlinkTime = millis() + LED_BLINK;
		  }
		  if (Flag_Pump1 != 1) led_string[6] = '1'; else led_string[6] = '0';
		  if (Flag_Pump2 != 1) led_string[5] = '1'; else led_string[5] = '0';
		  if (Flag_Pump3 != 1) led_string[4] = '1'; else led_string[4] = '0';
		  if (Flag_Pump4 != 1) led_string[3] = '1'; else led_string[3] = '0';
		  if (Flag_Pump5 != 1) led_string[2] = '1'; else led_string[2] = '0';
		  byte _led_value = led_value(led_string);
		  Wire.beginTransmission(0x20);
		  Wire.write(0x12); // GPIOA
		  Wire.write(_led_value); // port A
		  Wire.endTransmission();
	#endif




	// ************* check if the pushbutton is pressed. ***********
	// ************* if it is, the buttonState is HIGH:  ***********

	#ifdef LIQUIDCRYSTAL
		#ifdef DEBUG
			//Serial.println(F("LiquinCrystal Routine"));
		#endif
		buttonState = digitalRead(buttonPin);
		if (buttonState == HIGH && buttonState != lastbuttonState) {
			 lastbuttonState = buttonState;
			 if (btn >= 5) btn = 0;
			 lcdPrint();
			 lcdOn = millis();
			 Serial.println(F("Button Pressed"));
		} else if (buttonState == LOW) {
				if (millis() - lcdOn > 10000)
				{
					lcd.clear();
					lcd.setBacklight(LOW);
				}
			lastbuttonState = buttonState;
	   }
	#endif

	#ifdef PRINT_SD
	  printSD();
	#endif


	// ****************** Discharge watchdog timer ***************
	WatchdogDischarge();

}

// -------------------------- MAIN LOOP --------------------------------------


// ****************** Function to UPDATE LCD if exists***************
#ifdef LIQUIDCRYSTAL
	void lcdPrint() {
	  if (btn >= 5 ) btn = 0;
	  lcd.clear();
	  switch (btn) {

		case 0:
		  lcd.setBacklight(HIGH);
		  lcd.setCursor(0, 0); //Start at character 4 on line 0
		  lcd.print("Date: ");
		  lcd.print(year());
		  lcd.print("-");
		  lcd.print(month());
		  lcd.print("-");
		  lcd.print(day());

		  lcd.home();
		  lcd.setCursor(0, 1);
		  lcd.print("Time: ");
		  lcd.print(hour());
		  lcd.print(":");
		  lcd.print(minute());
		  lcd.print(":");
		  lcd.print(second());
		  lcd.print("/");
		  btn++;
		  break;

		case 1:
		  lcd.setBacklight(HIGH);
		  lcd.setCursor(6, 0); //Start at character 4 on line 0
		  lcd.print("IP : ");
		  lcd.setCursor(1, 1);
		  lcd.print(ip);
		  btn++;
		  break;
		case 2:
		  lcd.setBacklight(HIGH);
		  lcd.setCursor(0, 0); //Start at character 4 on line 0
		  lcd.print("ID : ");
		  lcd.print(ID);
		  btn++;
		  break;
		case 3:
		  lcd.setBacklight(HIGH);
		  lcd.setCursor(0, 0);
		  lcd.print("Pump1:");
		  lcd.print(!digitalRead(Pump_1_pin));

		  lcd.setCursor(8, 0);
		  lcd.print("Pump2:");
		  lcd.print(!digitalRead(Pump_2_pin));

		  lcd.setCursor(0, 1);
		  lcd.print("Pump3:");
		  lcd.print(!digitalRead(Pump_3_pin));

		  lcd.setCursor(8, 1);
		  lcd.print("Pump4:");
		  lcd.print(!digitalRead(Pump_4_pin));
		  btn++;
		  break;
		case 4:
		  btn++;
		  if (client.connect("google.com", 80) > 0) {
			lcd.setCursor(0, 0);
			lcd.print("Online");
			break;
		  } else {
			lcd.setCursor(0, 0);
			lcd.print("OFFLINE");
			break;
		  }
		  break;


	  }


	}

#endif


// ****************  WebServer Function *********************

void webSrvr() {
	client = server.available();
	file_cursor_position = 0; // file_cursor_position is used as a pointer in the current position of the file cursor
	if (client) {
		#ifdef DEBUG
			Serial.println(F("Server Call Detected ... Preparing Answer"));
		#endif
		bool success = readRequest(client);
		if (!legitRequest) success = false;
		if (success) {
			  //writeIntoFile();
			delay(50);
			file_cursor_position = 0;
			writeHeader(client);
			encodeStreamToJsonObject(client);
		}
		//delay(50);
		client.stop();
		#ifdef DEBUG
			Serial.println(F("Answer Send"));
		#endif
		// if (success) resetFunc();
	}
}


//************ Function that calls Write data to file *********** 

void writeIntoFile() {
	entries = findEntries() + 1; //get entries from the file in case Hell breaks loose
	writeID();
	writeDate();
	writeTime();
	writeDepth(DEPTH);

	writePump_OnOff(1);
	writePump_OnOff(2);
	writePump_OnOff(3);
	writePump_OnOff(4);
	writePump_OnOff(5);
}

void writeID() {
	myFile = SD.open(SDFILE, FILE_WRITE);
	if (myFile) {
		#ifdef DEBUG
			Serial.println(F("Writing ID"));
		#endif
		myFile.print("~");
		myFile.print(entries);
		myFile.print("/");
		myFile.print(ID);
		myFile.print("/");
		#ifdef DEBUG
			Serial.println(F("Done Writing ID"));
		#endif
	}
	else {
		Serial.println(F("1Error Writing ID"));
		delay(50);
		resetFunc();
	}

	myFile.close();
}



void writeDate() {
	myFile = SD.open(SDFILE, FILE_WRITE);

	if (myFile) {
		#ifdef DEBUG
			Serial.println(F("Writing Date"));
		#endif
		myFile.print(year());
		myFile.print("-");
		myFile.print(month());
		myFile.print("-");
		myFile.print(day());
		myFile.print("/");
		#ifdef DEBUG
			Serial.println(F("Done Writing Date"));
		#endif
	}else {
		Serial.println(F("2Error Writing Date"));
		resetFunc();
	}
	myFile.close();
}



void writeTime() {
	myFile = SD.open(SDFILE, FILE_WRITE);

	if (myFile) {
		#ifdef DEBUG
			Serial.println(F("Writing Time"));
		#endif
		myFile.print(hour());
		myFile.print(":");
		myFile.print(minute());
		myFile.print(":");
		myFile.print(second());
		myFile.print("/");
		#ifdef DEBUG
			Serial.println(F("Done Writing Time"));
		#endif
	} else {
		Serial.println(F("3Error Writing Time"));
		resetFunc();
	}
	  myFile.close();
}



// **************  TODO Define Allert SMS ************
// **************  TODO Define Safety Height *********
// ************** HAS resetfunc CHECK IT *************

void writeDepth(int Depth) {

	#ifdef DEPTHMETER
		int distanceFromSurface = depth_measure();
		int waterLevel = Depth - (distanceFromSurface - TANKOFFSET);

		#ifdef DEBUG
			Serial.print(F("Real ECHO mesurment: "));
			Serial.println(distanceFromSurface);
			Serial.print(F("calculated ECHO mesurment: "));
			Serial.println(waterLevel);
		#endif

		if (waterLevel < 0 || waterLevel > DEPTH)
			waterLevel = -1;

		#ifdef GSM_

		    Serial.println(waterLevel);

		//  if (distanceFromSurface < 50) { // --------------------- Should be depth - safety height
		//     sms(String("Tank ID 22146 WARNING tank LEVEL CRITICAL"), String("6937807458"));
		//     delay(5000);
		// }
		#endif
		myFile = SD.open(SDFILE, FILE_WRITE);
  	    if (myFile) {
			#ifdef DEBUG
				Serial.println(F("Writing depth"));
			#endif
			myFile.print(waterLevel);
			myFile.print("/");
			#ifdef DEBUG
				Serial.println(F("Done writing depth"));
			#endif
		} else {
			Serial.println(F("Error writing depth"));
			resetFunc();
		}
		myFile.close();
		#ifdef DEBUG
			Serial.println(F("No Tank"));
		#endif
	#else
		myFile = SD.open(SDFILE, FILE_WRITE);
		if (myFile) {
			myFile.print("-1");
			myFile.print("/");

		} else {
			resetFunc();
		}
		myFile.close();
	#endif
}



byte writePump_OnOff(byte pumpNo) {
	myFile = SD.open(SDFILE, FILE_WRITE);

	if (myFile) {
		#ifdef DEBUG
			Serial.println(F("Writing Pump ON/OFF"));
		#endif
		switch (pumpNo) {
			case 1:
				myFile.print(!digitalRead(Pump_1_pin));
				myFile.print("/");
				break;
			case 2:
				myFile.print(!digitalRead(Pump_2_pin));
				myFile.print("/");
				break;
			case 3:
				myFile.print(!digitalRead(Pump_3_pin));
				myFile.print("/");
				break;
			case 4:
				myFile.print(!digitalRead(Pump_4_pin));
				myFile.print("/");
				break;
			case 5:
				myFile.print(!digitalRead(Pump_5_pin));
				myFile.print("/");
			break;
		}
		#ifdef DEBUG
			// Serial.println(F("Done Writing Pump ON/OFF"));
		#endif
	} else {
		Serial.println(F("Error Writing ON/OFF"));
		resetFunc();
	}
	myFile.close();
}



void encodeStreamToJsonObject(EthernetClient client) {
	client.print("[");
	char reading;
	int _entries = findEntries();
	file_cursor_position = 0;
	int i = 0;

	do {
		client.print(F("{\"id\":\""));
		client.print(getReading(1, _reading));

		client.print(F("\",\"date\":\""));
		rewind();
		client.print(getReading(2, _reading));

		client.print(F("\",\"time\":\""));
		rewind();
		client.print(getReading(3, _reading));

		client.print(F("\",\"a\":\""));
		rewind();
		client.print(getReading(4, _reading));

		client.print(F("\",\"b\":\""));
		rewind();
		client.print(getReading(5, _reading));

		client.print(F("\",\"c\":\""));
		rewind();
		client.print(getReading(6, _reading));

		client.print(F("\",\"d\":\""));
		rewind();
		client.print(getReading(7, _reading));

		client.print(F("\",\"e\":\""));
		rewind();
		client.print(getReading(8, _reading));

		client.print(F("\",\"f\":\""));
		rewind();
		client.print(getReading(9, _reading));

		client.print(F("\"}"));
		i++;
		if (i != _entries) {
			client.print(",");
		}
	} while (i != _entries);
	client.print("]");
	#ifdef ESSENTIALS
		Serial.println(F("Done printing to Client"));
	#endif
	// *************** Update Ledbar with Transmit signal *************
	#ifdef LED_BAR
		led_string[1] = '0';
		_led_value = led_value(led_string);
		Wire.beginTransmission(0x20);
		Wire.write(0x12); // GPIOA
		Wire.write(_led_value); // port A
		Wire.endTransmission();
	#endif
	delay(50);
}


void writeHeader(EthernetClient& client) {
	#ifdef LED_BAR
		led_string[1] = '1';
		_led_value = led_value(led_string);
		Wire.beginTransmission(0x20);
		Wire.write(0x12); // GPIOA
		Wire.write(_led_value); // port A
		Wire.endTransmission();
	#endif
	client.println(F("HTTP/1.1 200 OK"));
	client.println(F("Content-Type: application/json"));
	client.println(F("Connection: Keep-Alive"));
	client.println();
}

// ****************   Returns one character from the file *************
// **************** according to the variable file_cursor_position ***************
char readFromFile() {
	myFile = SD.open(SDFILE);
	char incoming;
	if (myFile) {
		myFile.seek(file_cursor_position);
		incoming = myFile.read();
		file_cursor_position++;
		myFile.close();
		if (myFile.peek() == EOF)
			return ('*');
		return (incoming);
	} else {
		Serial.println(F("Error opening the file in readFromFile()"));
		resetFunc();
	}
	myFile.close();
}

/* ************************
   This function parses a line in the file
   and returns the entry according to the
   specified field from the variable num_reading
   ex : ~4/123/500/... num_reading = 2
   then reading = 500
   the character * indicates the EOF
 **************************  */

char* getReading(int num_reading, char* reading) {
	int i = 0;
	char inData;
	int j = 0;
	File myFile;
	myFile = SD.open(SDFILE, FILE_READ);
	do {
		if (myFile) {
			myFile.seek(file_cursor_position);
			inData = myFile.read();
			file_cursor_position++;
			if (myFile.peek() == EOF)
				inData = '*';
		} else {
			Serial.println(F("8Error opening the file in readFromFile()"));
			resetFunc();
		}
		if (inData == '/')
			i++;
		else if (inData == '*')
			break;
		else if (i == num_reading) {
			reading[j] = inData;
			j++;
			reading[j] = '\0';
		} else if (i > num_reading)
			break;
	} while (inData != '*');
	myFile.close();
	return reading;
}


/*
   It finds the number of entries inside
   the file. Inside the file the entries
   have fields that separate with the
   character /. The first field is the
   number of entry, so this function starts
   from the end of the file and searches for
   the first ~ that indicates the new line
   then the first number is the #ofEntries
*/

int findEntries() {
	myFile = SD.open(SDFILE);
	int _size = myFile.size();
	file_cursor_position = _size;
	char inData;
	int entries = 0;
	int i = 0;
	if (file_cursor_position == 0)
		return 0;

	do {
		file_cursor_position--;
		myFile.seek(file_cursor_position);
		inData = myFile.read();
	} while (inData != '~');

	char outData[5];
	do {
		inData = myFile.read();
		if (inData == '/')
		  break;
		outData[i] = inData;
		i++;
		outData[i] = '\0';
	} while (inData != '/');

	myFile.close();
	return (atoi(outData));
}


/*
   rewind is used to return the file cursor to
   the first position of the current line
   It reads the stream backwards and stops when
   it finds the character ~ that indicates a new entry
*/
void rewind() {
	File myFile = SD.open(SDFILE);
	char incoming;
	do {
		myFile.seek(file_cursor_position);
		incoming = myFile.read();
		if (incoming != '~')
		  file_cursor_position--;
	} while (incoming != '~');
	myFile.close();
}


/********************************************************************************************************************************************************/
/********************************************************************************************************************************************************/
/************************************************************* DELETE WHEN DONE *************************************************************************/
/********************************************************************************************************************************************************/
/********************************************************************************************************************************************************/

void printSD() {
	File myFile = SD.open(SDFILE);
	char inData;
	do {
		inData = myFile.read();
		Serial.print(inData);
	} while (myFile.peek() != EOF);
	myFile.close();
	Serial.println(F(""));
}


void dummyInit() {
	int k = 0;
	  // open the file. note that only one file can be open at a time,
	  // so you have to close this one before opening another.
	myFile = SD.open(SDFILE, FILE_WRITE);
	while (entries < 3) {
		// if the file opened okay, write to it:
		if (myFile) {
			Serial.print(F("Writing to data.txt..."));
			myFile.print("~");
			myFile.print(entries);
			myFile.print("/");
			#ifdef DEPTHMETER
				myFile.print(depth_measure());
			#else
				myFile.print("-1");
			#endif
			myFile.print("/");
			myFile.print(k);
			k = k + 100;
			myFile.println("!");
			Serial.println(F("done."));
			entries++;
		} else {
		  // if the file didn't open, print an error:
			Serial.println(F("9Error opening data.txt in dummyInit()"));
		}
	  }
	  myFile.close();
}



int freeRam() {
	extern int __heap_start, *__brkval;
	int v;
	return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int) __brkval);
}


#ifdef DEPTHMETER
	boolean timeToRead(int interval) {
		if (lastHour == hour()) {
			if (lastMin + interval == minute()) {
				lastHour = hour();
				lastMin = minute();
				return true;
			}
		} else if (lastHour < hour()) { // change of hour
			lastHour = hour();
			lastMin = minute();
			return true;
		} else if (lastHour > hour()) { // change of day
			lastHour = hour();
			lastMin = minute();
			return true;
		}
		if (entries == 1 && cheat == 0) {
			cheat = 1;
			return true;
		}
		return false;
	}
#endif



#ifdef UDPTIME

	/********************************************************************************************************************************************************/
	/********************************************************************************************************************************************************/
	/****************************************************************  TIME FUNCTIONS  **********************************************************************/
	/********************************************************************************************************************************************************/
	/********************************************************************************************************************************************************/


	/* Messages consist of the letter T followed by ten digit time (as seconds since Jan 1 1970)
	   you can send the text on the next line using Serial Monitor to set the clock to noon Jan 1 2013
	   T1357041600
	*/

	void timeSetup() {
	  if (timeStatus() != timeNotSet) {
		if (now() != prevDisplay) { //update the display only if time has changed
		  prevDisplay = now();
		}
	  }
	  lastHour = hour();
	  lastMin = minute();
	#ifdef ESSENTIALS
	  Serial.println(F("Done Syncing"));
	#endif

	}

	boolean processSyncMessage() {
	  unsigned long pctime;
	  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013
	  //1437132060  07/17/2015 @ 11:21am

	#ifdef TIME_DEB
	  setTime(1437132060);
	  return true;
	#else/*
		  while(Serial.find(TIME_HEADER)) {
			 pctime = Serial.parseInt();
			 if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
			   setTime(pctime); // Sync Arduino clock to the time received on the serial port
			   return true;
			 }
		  }
	*/
	#endif

	  return false;
	}

	time_t requestSync()
	{
	  Serial.write(TIME_REQUEST);
	  return 0; // the time will be sent later in response to serial mesg
	}


	/* Calculates if the time has passed by "interval" minutes and returns true in
	   such case so the next measurement can be made
	*/



	/*-------- NTP code ----------*/

	const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
	byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

	time_t getNtpTime()
	{
	  while (Udp.parsePacket() > 0) ; // discard any previously received packets
	#ifdef TIME_DEB
	  Serial.println(F("Transmit NTP Request"));
	#endif

	  sendNTPpacket(timeServer);

	  uint32_t beginWait = millis();
	  while (millis() - beginWait < 1500) {
		int _size = Udp.parsePacket();
		if (_size >= NTP_PACKET_SIZE) {
	#ifdef TIME_DEB
		  Serial.println(F("Receive NTP Response"));
	#endif
		  Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
		  unsigned long secsSince1900;
		  // convert four bytes starting at location 40 to a long integer
		  secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
		  secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
		  secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
		  secsSince1900 |= (unsigned long)packetBuffer[43];
		  return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
		}
	  }
	#ifdef TIME_DEB
	  Serial.println(F("No NTP Response"));
	#endif

	  return 0; // return 0 if unable to get the time
	}

	// send an NTP request to the time server at the given address
	void sendNTPpacket(IPAddress &address)
	{
	  // set all bytes in the buffer to 0
	  memset(packetBuffer, 0, NTP_PACKET_SIZE);
	  // Initialize values needed to form NTP request
	  // (see URL above for details on the packets)
	  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
	  packetBuffer[1] = 0;     // Stratum, or type of clock
	  packetBuffer[2] = 6;     // Polling Interval
	  packetBuffer[3] = 0xEC;  // Peer Clock Precision
	  // 8 bytes of zero for Root Delay & Root Dispersion
	  packetBuffer[12]  = 49;
	  packetBuffer[13]  = 0x4E;
	  packetBuffer[14]  = 49;
	  packetBuffer[15]  = 52;
	  // all NTP fields have been given values, now
	  // you can send a packet requesting a timestamp:
	  Udp.beginPacket(address, 123); //NTP requests are to port 123
	  Udp.write(packetBuffer, NTP_PACKET_SIZE);
	  Udp.endPacket();
	}

#endif

void deleteFile() {
	#ifdef ESSENTIALS
		Serial.println(F("Removing data.txt..."));
	#endif
	SD.remove(SDFILE);
}


void createFile() {
	myFile = SD.open(SDFILE, FILE_WRITE);
	myFile.close();
}


/* When the response is sent to the client
   this function is called to delete the file
   and create a new one so the next response
   won't be overwhelming
*/

void prepareNextData() {
	deleteFile();
	entries = 1;
	createFile();
	writeIntoFile();
}
