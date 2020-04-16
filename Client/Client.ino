#include <SPI.h>
#include <math.h>
#include "wiring_private.h"

#include "TinyGPS++.h"
//#include "WiFiNINA.h"
#include <WiFiNINA.h>
//#include <Arduino_LSM6DS3.h>
#include "Arduino_LSM6DS3.h"

#include "Wifi/Wifi_Settings.h";
int timeVector = 0;
int imuSampleRate; 

//const char *ssid     = "H1S2";
//const char *password = "pw1234pw1234";
char ssid[]     = SECRET_SSID;        // your network SSID (name)
char password[] = SECRET_PASS; 

IPAddress server(IPAddr[0],IPAddr[1],IPAddr[2],IPAddr[3]);  // numeric IP for Google (no DNS)



int status = WL_IDLE_STATUS;
//IPAddress server(192, 168, 43, 30); // Google laptop

// Initialize the client library
WiFiClient client;
float x, y, z;
int arrayPtr = 0;

typedef struct {
  float ax, ay, az, gx, gy, gz, tv; //possibly double?
  double lat, lng;
  int hour, min, sec, csec;
} movementType;
movementType data[20];//[9];

static const int RXPin = 5, TXPin = 6;
TinyGPSPlus gps;

Uart gpsSerial (&sercom0, RXPin, TXPin, SERCOM_RX_PAD_1, UART_TX_PAD_0);

void SERCOM0_Handler() {
  gpsSerial.IrqHandler();
  gps.encode(gpsSerial.read());
}

const unsigned char UBLOX_INIT[] PROGMEM = {
  // Disable NMEA
  //0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
  // 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, // GxGSA off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, // GxGSV off
  // 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, // GxVTG off
  // Rate (pick one)
    //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A, //(5Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39 //(1Hz)
  0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xd0, 0x08, 0x00, 0x00, 0x00, 0xc2, 0x01, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc4, 0x96, 0xb5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22 //baudrate 115200
};

void gpsInit() {
  for (unsigned int i = 0; i < sizeof(UBLOX_INIT); i++) {
    gpsSerial.write( pgm_read_byte(UBLOX_INIT + i) );
  }

  delay(100);
  gpsSerial.end();
  delay(100);
  gpsSerial.begin(115200);
}




int imuPoll() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    if (x > 0 || y > 0 || z > 0) {
      data[arrayPtr].ax = x;
      data[arrayPtr].ay = y;
      data[arrayPtr].az = z;
      //Serial.print("UPDATE @ ");
    }
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    data[arrayPtr].gx = x;
    data[arrayPtr].gy = y;
    data[arrayPtr].gz = z;
  }
  data[arrayPtr].tv = float(timeVector)/float(imuSampleRate);
  timeVector++;
}


void gpsPoll() {
  data[arrayPtr].lat  = gps.location.lat();
  data[arrayPtr].lng  = gps.location.lng();
  data[arrayPtr].hour = gps.time.hour();
  data[arrayPtr].min  = gps.time.minute();
  data[arrayPtr].sec  = gps.time.second();
  data[arrayPtr].csec = gps.time.centisecond();
}


void setup() {
  pinPeripheral(RXPin, PIO_SERCOM_ALT); //alt?
  pinPeripheral(TXPin, PIO_SERCOM_ALT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  

  Serial.begin(9600);
  gpsSerial.begin(9600);
  while (!gpsSerial);

  while (!IMU.begin()) 
    Serial.println("IMU init failed");
    
  imuSampleRate = IMU.accelerationSampleRate();
  
  Serial.print("Waiting for GPS Fix");

  //while  (!(gps.location.isValid() && gps.location.age() < 2000)) {
    //Serial.println(".");
    //Serial.println(gps.location.age());
    //Serial.println(gps.location.isValid());
    //Serial.println(gps.location.lat());
    //gpsInit();
  //}
  Serial.println("GPS Fix aquired, changing freq");
  gpsInit();
  wifiConnect();
  Serial.println(WiFi.SSID());

  
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);

  delay(5000);

  Serial.println("Connected to wifi");
  serverConnect();
}

void wifiConnect() {
  digitalWrite(LED_BUILTIN, LOW);
  while ( status != WL_CONNECTED ) {
    Serial.print("Attempt to connect to WPA SSID: ");
    Serial.println(ssid);
    Serial.println(status);

    status = WiFi.begin(ssid, password);
    //wifi_station_set_auto_connect(true);
    delay ( 10000 );
    Serial.print ( "." );
  }
  digitalWrite(LED_BUILTIN, HIGH);
}

void serverConnect() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  if (client.connect(server, 8080)) {
    Serial.println("connected");
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    // if you didn't get a connection to the server:
    Serial.println("connection failed");
  }
}
int disconnects = 0;
int servConnection = 0;
int dataReady = 0;
int inG = 0;

void loop() {
  while (!client.connected()) {
    if (servConnection) 
      disconnects++;
    if (disconnects > 3) {
      
      printf("4 disconnects detected; restarting wifi connection...");
      disconnects = 0;
      status = WiFi.begin(ssid, password);
      wifiConnect();
    }
    Serial.println();
    Serial.println("disconnecting.");
    client.stop();
    client.flush();
    serverConnect();
  }

  if (client.available()) {
    servConnection = 1;
    //char *c = (char *)client.read();
    char c = client.read();
    client.flush();
    Serial.print(c);
    //Serial.print(dataReady);
    if (c == 'A') {
      Serial.println("ACK received, server connected");
      

    } else if (c == 'G') {
      
      char *bytes = (char *) data;
      client.write(bytes, sizeof(data));
      client.flush();
      //char *cbytes = (char *) combData;
      Serial.print("Data:");
      Serial.println(sizeof(data));
      dataReady = 0;
      arrayPtr = 0;
    }
  } else if (!dataReady) {
    int start = millis();
    //Serial.println("DATA NOT READY");
    for (int i = 0; i < 20; i++) {
      imuPoll();
      gpsPoll();
      arrayPtr++;
      delay(31); //31ms 
    }
    Serial.print("Data ready in ms: ");
    Serial.println(millis() - start);
    dataReady = 1;
  } 
}
