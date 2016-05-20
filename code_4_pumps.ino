
#include <SPI.h>
#include <Ethernet.h>
#include <SD.h>
#include <Time.h>
#include <EthernetUdp.h>


//#define DEPTHMETER 1
//#define SAMPLING_TIME 10

#define LIQUIDCRYSTAL 1
#ifdef LIQUIDCRYSTAL
  #include <LiquidCrystal_I2C.h>
#endif


#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 
#define SDFILE          "data.txt"
#define DEPTH        -1 //400  //-1 For no Tank or Tank height 
#define ID            22145




//#define DEBUG 1
//#define TIME_DEB 1
//#define HTTP_DEB 1
//#define PRINT_SD
//#define ESSENTIALS 1

#ifdef LIQUIDCRYSTAL
  LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7); // 0x27 is the I2C bus address for an unmodified backpack
#endif

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 209, 2);
EthernetServer server(80);
EthernetClient client;
// NTP Servers:
IPAddress timeServer(192, 168, 209, 1); // Router NTP SERVER
const byte timeZone = 3;     // Central European Time
EthernetUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets
time_t prevDisplay = 0; // when the digital clock was displayed

File myFile;

char _reading[15];
//int pump = 7;
#ifdef DEPTHMETER
  byte Echo = 8;
#endif

//byte Trig = 9;

byte lastHour = 0;
byte lastMin = 0;
byte pumpChange = 0;
byte i = 0;
byte btn = 0; // button flag
const byte buttonPin = 8;
byte buttonState = 0;
byte lastbuttonState = 0;
unsigned long lcdOn;

const byte Pub_1_pin = 2;
const byte Pub_2_pin = 3;
const byte Pub_3_pin = 5;
const byte Pub_4_pin = 6;

byte Flag_Pump1 = 0;
byte Flag_Pump2 = 0;
byte Flag_Pump3 = 0;
byte Flag_Pump4 = 0;
bool legitRequest= false;


bool readRequest(EthernetClient& client) {
  bool currentLineIsBlank = true;
  bool reading;
  legitRequest=false;
  
  i = 0;
  while (client.connected()) {
    if (client.available()) {
      char c = client.read();
      if(c == '?') reading= true; 
        if(reading){
          // If next character is legit prepare flag for answer
         if (c !='?') reading = false;
         // If request is just to see if alive
         if (c=='5') writeHeader(client);
         // If request is for data
         if (c=='4') legitRequest = true;
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
      
        delay(200);
        return distance;
      }

#endif

void(* resetFunc) (void) = 0; //declare reset function at address 0


void setup() {
  Ethernet.begin(mac, ip);
  Serial.begin(9600);
  server.begin();

#ifdef LIQUIDCRYSTAL
  pinMode(buttonPin, INPUT);
  lcd.begin(16, 2);
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(LOW);
#endif


  //----------------------------------------
  pinMode (Pub_1_pin, INPUT);
  pinMode (Pub_2_pin, INPUT);
  pinMode (Pub_3_pin, INPUT);
  pinMode (Pub_4_pin, INPUT);


  //pinMode(4, OUTPUT);
  //digitalWrite(4, HIGH);

  // pinMode(Echo,INPUT);
  // pinMode(Trig,OUTPUT);
  // pinMode(pump,INPUT);

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
  Udp.begin(localPort);

#ifdef ESSENTIALS
  Serial.print(F("SRAM = "));
  Serial.println(freeRam());
  Serial.println(F("waiting for sync"));
#endif
  setSyncProvider(getNtpTime);

  timeSetup();
  //  pumpChange = digitalRead(pump);

  prepareNextData();

}

byte entries = 1;
long pos = 0;
byte cheat = 0;

void loop() {
  webSrvr();
  pos = 0;


  if (digitalRead(Pub_1_pin) != Flag_Pump1 || digitalRead(Pub_2_pin) != Flag_Pump2 || digitalRead(Pub_3_pin) != Flag_Pump3 || digitalRead(Pub_4_pin) != Flag_Pump4)
  {

    Flag_Pump1 = digitalRead(Pub_1_pin);
    Flag_Pump2 = digitalRead(Pub_2_pin);
    Flag_Pump3 = digitalRead(Pub_3_pin);
    Flag_Pump4 = digitalRead(Pub_4_pin);
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


  //if (timeToRead(SAMPLING_TIME))
  //  writeIntoFile();

  ///  TODO  Fix it   ----------------
  //  else if(pumpChange != digitalRead(pump)){ //if pumps change state between on/off : Create another entry
  //  pumpChange = digitalRead(pump);
  //  writeIntoFile();

  /// --------------------------


  // }

#ifdef PRINT_SD
  printSD();
#endif

  delay(10);
}

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
          lcd.print(digitalRead(Pub_1_pin));
    
          lcd.setCursor(8, 0);
          lcd.print("Pump2:");
          lcd.print(digitalRead(Pub_2_pin));
    
          lcd.setCursor(0, 1);
          lcd.print("Pump3:");
          lcd.print(digitalRead(Pub_3_pin));
    
          lcd.setCursor(8, 1);
          lcd.print("Pump4:");
          lcd.print(digitalRead(Pub_4_pin));
          btn++;
          break;
        case 4:
           btn++;
          if (client.connect("google.com",80)>0) {
            lcd.setCursor(0, 0);
            lcd.print("Online");
            break;
          } else{
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
    delay(10);
    client.stop();
    
#ifdef DEBUG
    Serial.println(F("Answer Send"));
#endif
resetFunc();
  }
}

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
      
        //int distanceFromSurface = depth_measure();
      
        byte distanceFromSurface = 100;//depth_measure();
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
    
    }else {
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
           myFile.print(digitalRead(Pub_1_pin));
           myFile.print("/");
           break;
        case 2:
           myFile.print(digitalRead(Pub_2_pin));
           myFile.print("/");
           break;
        case 3:
           myFile.print(digitalRead(Pub_3_pin));
           myFile.print("/");
           break;
        case 4:
           myFile.print(digitalRead(Pub_4_pin));
           myFile.print("/");
           break;
    }

#ifdef DEBUG
    Serial.println(F("Done Writing Pump ON/OFF"));
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
    
  prepareNextData();
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
  myFile = SD.open(SDFILE,FILE_READ);
  
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
