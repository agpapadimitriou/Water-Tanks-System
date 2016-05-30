
//--------------------------------------------------------------------------
// ************** Things to Define before compiling ************************
// Define if RTC or UDP Time
// Define Depth or -1
// Define ID
// Define Pumps 
// Define Depthmeter
// Define LCD
// Define Mac Address
// Define IP
//---------------------------------------------------------------------------

#include <SPI.h>
#include <Ethernet.h>
#include <SD.h>
#include <Time.h>

//#define UDPTIME       1
#ifdef UDPTIME
  #include <EthernetUdp.h>
#endif

#define RTCTIME       1
#ifdef RTCTIME
  #include "Wire.h"
#endif

#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 
#define SDFILE          "data.txt"
#define DEPTH        300 //400  //-1 For no Tank or Tank height 
#define ID            33124
#define PUMP1         5  // 1 for pump num reading @pin 2 or 5 for no reading
#define PUMP2         5  // 2 for pump num reading @pin 3 or 5 for no reading
#define PUMP3         5  // 3 for pump num reading @pin 5 or 5 for no reading
#define PUMP4         5  // 4 for pump num reading @pin 6 or 5 for no reading
#define DEPTHMETER    1  // 1 for height reading with echo @ pins 8 (echo) 9 (trigger) OR comment for no ECHO
//#define LIQUIDCRYSTAL 1

#ifdef LIQUIDCRYSTAL
   #include <LiquidCrystal_I2C.h>
#endif
#ifdef DEPTHMETER
  #define SAMPLING_TIME 10
#endif

//#define DEBUG 1
//#define TIME_DEB 1
//#define HTTP_DEB 1
//#define PRINT_SD
//#define ESSENTIALS 1


#ifdef LIQUIDCRYSTAL
  LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7); // 0x27 is the I2C bus address for an unmodified backpack
#endif

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEC};
IPAddress ip(192, 168, 209, 3);
EthernetServer server(80);
EthernetClient client;

#ifdef UDPTIME
  // NTP Servers:
  IPAddress timeServer(192, 168, 209, 1); // Router NTP SERVER
  const byte timeZone = 3;     // Central European Time
  EthernetUDP Udp;
  unsigned int localPort = 8888;  // local port to listen for UDP packets
#endif

time_t prevDisplay = 0; // when the digital clock was displayed

#ifdef RTCTIME
  #define DS3231_I2C_ADDRESS 0x68
  // Convert normal decimal numbers to binary coded decimal
  byte decToBcd(byte val)
  {
    return( (val/10*16) + (val%10) );
  }
  // Convert binary coded decimal to normal decimal numbers
  byte bcdToDec(byte val)
  {
    return( (val/16*10) + (val%16) );
  }
#endif

#ifdef RTCTIME
 // 
#endif

File myFile;

char _reading[15];

#ifdef DEPTHMETER
  byte Echo = 8;
  byte Trig = 9;
#endif



byte lastHour = 0;
byte lastMin = 0;
byte pumpChange = 0;
byte i = 0;
#ifdef LIQUIDCRYSTAL
  byte btn = 0; // button flag
  const byte buttonPin = 8;
  byte buttonState = 0;
  byte lastbuttonState = 0;
  unsigned long lcdOn;
#endif

const byte Pump_1_pin = 2;
const byte Pump_2_pin = 3;
const byte Pump_3_pin = 5;
const byte Pump_4_pin = 6;

byte Flag_Pump1 = 0;
byte Flag_Pump2 = 0;
byte Flag_Pump3 = 0;
byte Flag_Pump4 = 0;



bool legitRequest = false;


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
        if (c == '5'){
          client.println(F("HTTP/1.1 200 OK"));
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

#ifdef DEPTHMETER

  int depth_measure() {
    digitalWrite(Trig, LOW); // 2μs
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH); // 10μs
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);
    int distance = pulseIn(Echo, HIGH);
    distance = distance / 58;
    Serial.println(distance);
    delay(200);
    return distance;
  }

#endif

void(* resetFunc) (void) = 0; //declare reset function at address 0



///  ----------------------  Setup ---------------------------

void setup() {
  Ethernet.begin(mac, ip);
  Serial.begin(9600);
  server.begin();

#ifdef RTCTIME
  Wire.begin();
  byte RTCsecond, RTCminute, RTChour, dayOfWeek, dayOfMonth, RTCmonth, RTCyear;
  // retrieve data from DS3231
  readDS3231time(&RTCsecond, &RTCminute, &RTChour, &dayOfWeek, &dayOfMonth, &RTCmonth, &RTCyear);
  setTime(RTChour,RTCminute,RTCsecond,dayOfMonth,RTCmonth,RTCyear);
#endif

#ifdef LIQUIDCRYSTAL
  pinMode(buttonPin, INPUT);
  lcd.begin(16, 2);
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(LOW);
#endif

  
#ifdef DEPTHMETER
  pinMode (Pump_1_pin, INPUT);
  pinMode (Pump_2_pin, INPUT);
  pinMode (Pump_3_pin, INPUT);
  pinMode (Pump_4_pin, INPUT);
#endif

  //pinMode(4, OUTPUT);
  //digitalWrite(4, HIGH);
#ifdef DEPTHMETER
  pinMode(Echo, INPUT);
  pinMode(Trig, OUTPUT);
#endif

  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  // initialize SD card
#ifdef ESSENTIALS
  Serial.println(F("Initializing SD card..."));
#endif

  if (!SD.begin(4)) {
    #ifdef ESSENTIALS
        Serial.println(F("ERROR - SD card initialization failed!"));
    #endif
    resetFunc();
    return;    // init failed
  }

#ifdef ESSENTIALS
  Serial.println(F("SUCCESS - SD card initialized."));
#endif

  deleteFile();
 
 

#ifdef ESSENTIALS
  Serial.print(F("SRAM = "));
  Serial.println(freeRam());
  Serial.println(F("waiting for sync"));
#endif
 

#ifdef UDPTIME
  Udp.begin(localPort);
  setSyncProvider(getNtpTime);
  timeSetup();
#endif
  
  //  pumpChange = digitalRead(pump);

#ifndef DEPTHMETER  // no need for data if depthmeter on will read on first cycle
   prepareNextData();
#endif 

}

///  ----------------------  Setup ---------------------------

#ifdef RTCTIME
  void readDS3231time(byte *RTCsecond,byte *RTCminute,byte *RTChour,byte *dayOfWeek,byte *dayOfMonth,byte *RTCmonth,byte *RTCyear)
  {
    Wire.beginTransmission(DS3231_I2C_ADDRESS);
    Wire.write(0); // set DS3231 register pointer to 00h
    Wire.endTransmission();
    Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
    // request seven bytes of data from DS3231 starting from register 00h
    *RTCsecond = bcdToDec(Wire.read() & 0x7f);
    *RTCminute = bcdToDec(Wire.read());
    *RTChour = bcdToDec(Wire.read() & 0x3f);
    *dayOfWeek = bcdToDec(Wire.read());
    *dayOfMonth = bcdToDec(Wire.read());
    *RTCmonth = bcdToDec(Wire.read());
    *RTCyear = bcdToDec(Wire.read());
  
    #ifdef DEBUG
      Serial.println("RTC TIME BEGIN");
      Serial.println(*RTChour);
      Serial.println(*RTCminute);
      Serial.println(*RTCsecond);
      Serial.println(*dayOfMonth);
      Serial.println(*RTCmonth);
      Serial.println(*RTCyear);
      Serial.println("RTC TIME END");
    #endif
}
#endif


byte entries = 1;
long pos = 0;
byte cheat = 0;

// -------------------------- MAIN LOOP --------------------------------------

void loop() {
  webSrvr();
  pos = 0;

#ifdef DEPTHMETER
   if (timeToRead(SAMPLING_TIME))  writeIntoFile();
#endif
  
#ifndef DEPTHMETER
    Serial.println("NO DEPTHMETER");
    if (digitalRead(Pump_1_pin) != Flag_Pump1 || digitalRead(Pump_2_pin) != Flag_Pump2 || digitalRead(Pump_3_pin) != Flag_Pump3 || digitalRead(Pump_4_pin) != Flag_Pump4)
    {

      Flag_Pump1 = digitalRead(Pump_1_pin);
      Flag_Pump2 = digitalRead(Pump_2_pin);
      Flag_Pump3 = digitalRead(Pump_3_pin);
      Flag_Pump4 = digitalRead(Pump_4_pin);
      writeIntoFile();


      #ifdef DEBUG
        Serial.println(F("Change on Pump Occured"));
        if (Flag_Pump1) Serial.println(F("Pump1"));
        if (Flag_Pump2) Serial.println(F("Pump2"));
        if (Flag_Pump3) Serial.println(F("Pump3"));
        if (Flag_Pump4) Serial.println(F("Pump4"));
     #endif
        delay (200);

    }
#endif

#ifdef LIQUIDCRYSTAL

  buttonState = digitalRead(buttonPin);
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == HIGH && buttonState != lastbuttonState) {
    lastbuttonState = buttonState;
    if (btn >= 5) btn = 0;
    lcdPrint();
    lcdOn = millis();

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


delay(10);
}

// -------------------------- MAIN LOOP --------------------------------------

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
      lcd.print(digitalRead(Pump_1_pin));

      lcd.setCursor(8, 0);
      lcd.print("Pump2:");
      lcd.print(digitalRead(Pump_2_pin));

      lcd.setCursor(0, 1);
      lcd.print("Pump3:");
      lcd.print(digitalRead(Pump_3_pin));

      lcd.setCursor(8, 1);
      lcd.print("Pump4:");
      lcd.print(digitalRead(Pump_4_pin));
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

void webSrvr() {
  
  client = server.available();
  pos = 0; // pos is used as a pointer in the current position of the file cursor
  if (client) {
    #ifdef DEBUG
        Serial.println(F("Preparing Answer"));
    #endif
    bool success = readRequest(client);
    if (!legitRequest) success = false;
    if (success) {
      //writeIntoFile();
      delay(50);
      pos = 0;
      writeHeader(client);
      encodeStreamToJsonObject(client);

    }
    //delay(50);
    client.stop();

    #ifdef DEBUG
        Serial.println(F("Answer Send"));
    #endif
    if (success) resetFunc();
  }
}

void writeIntoFile() {
  entries = findEntries() + 1; //get entries from the file in case Hell breaks loose
  writeID();
  writeDate();
  writeTime();

  writeDepth(DEPTH);

  writePump_OnOff(PUMP1);
  writePump_OnOff(PUMP2);
  writePump_OnOff(PUMP3);
  writePump_OnOff(PUMP4);

  writeF();

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
  }
  else {
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
  }
  else {
    Serial.println(F("3Error Writing Time"));
    resetFunc();
  }
  myFile.close();
}


void writeDepth(int Depth) {

#ifdef DEPTHMETER

 
  byte distanceFromSurface = depth_measure();
  byte waterLevel = Depth - distanceFromSurface;

  if (waterLevel < 0 || waterLevel > DEPTH)
    waterLevel = -1;

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
  }
  else {
    Serial.println(F("4Error writing depth"));
    resetFunc();
  }
  myFile.close();

#else
#ifdef DEBUG
  Serial.println(F("No Tank"));
#endif
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
        myFile.print(digitalRead(Pump_1_pin));
        myFile.print("/");
        break;
      case 2:
        myFile.print(digitalRead(Pump_2_pin));
        myFile.print("/");
        break;
      case 3:
        myFile.print(digitalRead(Pump_3_pin));
        myFile.print("/");
        break;
      case 4:
        myFile.print(digitalRead(Pump_4_pin));
        myFile.print("/");
        break;
      // In Case no pump write -1
      case 5:
        myFile.print(-1);
        myFile.print("/");
        break;

    }

#ifdef DEBUG
    // Serial.println(F("Done Writing Pump ON/OFF"));
#endif
  }
  else {
    Serial.println(F("5Error Writing ON/OFF"));
    resetFunc();
  }
  myFile.close();


}




void writeF() {
  myFile = SD.open(SDFILE, FILE_WRITE);

  if (myFile) {
#ifdef DEBUG
    Serial.println(F("Writing F"));
#endif
    myFile.print("-1");
    myFile.println("!");
#ifdef DEBUG
    Serial.println(F("Done Writing F"));
#endif
  }
  else {
    Serial.println(F("6Error Writing F"));
    resetFunc();
  }

  myFile.close();
}

//byte times = 1;
void encodeStreamToJsonObject(EthernetClient client) {

  client.print("[");
  char reading;
  int _entries = findEntries();
  pos = 0;
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
  //-----------------------------------------------------------------------------------------------------------------------------
  delay(50);

 // prepareNextData();
}


void writeHeader(EthernetClient& client) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: application/json"));
  client.println(F("Connection: Keep-Alive"));
  client.println();

}

/*
   Returns one character from the file
   according to the variable pos
*/

char readFromFile() {

  myFile = SD.open(SDFILE);
  char incoming;
  if (myFile) {
    myFile.seek(pos);
    incoming = myFile.read();
    pos++;
    myFile.close();
    if (myFile.peek() == EOF)
      return ('*');
    return (incoming);
  }
  else {
    Serial.println(F("7Error opening the file in readFromFile()"));
    resetFunc();
  }

  myFile.close();
}

/*
   This function parses a line in the file
   and returns the entry according to the
   specified field from the variable num_reading
   ex : ~4/123/500/... num_reading = 2
   then reading = 500
   the character * indicates the EOF
*/

char* getReading(int num_reading, char* reading) {
  int i = 0;
  char inData;
  int j = 0;

  File myFile;
  myFile = SD.open(SDFILE, FILE_READ);

  do {
    //inData = readFromFile();

    if (myFile) {
      myFile.seek(pos);
      inData = myFile.read();
      pos++;
      if (myFile.peek() == EOF)
        inData = '*';
    }
    else {
      Serial.println(F("8Error opening the file in readFromFile()"));
      resetFunc();
    }



    if (inData == '/')
      i++;

    else if (inData == '!')
      break;

    else if (i == num_reading) {
      reading[j] = inData;
      j++;
      reading[j] = '\0';
    }
    else if (i > num_reading)
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
  pos = _size;
  char inData;
  int entries = 0;
  int i = 0;

  if (pos == 0)
    return 0;

  do {
    pos--;
    myFile.seek(pos);
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
    myFile.seek(pos);
    incoming = myFile.read();
    if (incoming != '~')
      pos--;
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
      }
      else if (lastHour < hour()) { // change of hour
        lastHour = hour();
        lastMin = minute();
        return true;
      }
      else if (lastHour > hour()) { // change of day
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
    #ifdef TIME_DEG
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
