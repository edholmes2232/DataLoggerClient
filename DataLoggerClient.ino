#include <SPI.h>
#include <math.h>
#include "wiring_private.h"

#include "TinyGPS++.h"
//#include "WiFiNINA.h"
#include <WiFiNINA.h>
//#include <Arduino_LSM6DS3.h>
#include "lib/Arduino_LSM6DS3.h"

const char *ssid     = "H1S2";
const char *password = "pw1234pw1234";

int status = WL_IDLE_STATUS;
IPAddress server(192, 168, 43, 30); // Google laptop

// Initialize the client library
WiFiClient client;
float x, y, z;
int arrayPtr = 0;

typedef struct {
  float ax, ay, az, gx, gy, gz;
  double lat, lng;
  int hour, min, sec, csec;
} movementType;
movementType data[20];//[9];

static const int RXPin = 5, TXPin = 6;
TinyGPSPlus gps;

Uart gpsSerial (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);

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
    0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  //0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A, //(5Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39 //(1Hz)
  0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xd0, 0x08, 0x00, 0x00, 0x00, 0xc2, 0x01, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc4, 0x96, 0xb5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22 //baudrate 115200
};

void gpsInit() {
  for (unsigned int i = 0; i < sizeof(UBLOX_INIT); i++) {
    gpsSerial.write( pgm_read_byte(UBLOX_INIT + i) );
  }


  gpsSerial.print("$PUBX,41,1,0003,0001,115200,0*1E\r\n");
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
}


void gpsPoll() {
  //Serial.println("0 returned");
  data[arrayPtr].lat = gps.location.lat();
  data[arrayPtr].lng = gps.location.lng();
  data[arrayPtr].hour = gps.time.hour();
  data[arrayPtr].min = gps.time.minute();
  data[arrayPtr].sec = gps.time.second();
  data[arrayPtr].csec = gps.time.centisecond();

}



void setup() {
  pinPeripheral(RXPin, PIO_SERCOM_ALT); //alt?
  pinPeripheral(TXPin, PIO_SERCOM_ALT);

  Serial.begin(9600);
  gpsSerial.begin(9600);
  while (!gpsSerial);
  //while(!Serial);
  while (!IMU.begin()) {
    Serial.println("IMU init failed");
  }
  gpsInit();


  Serial.println("GPS NOW");
  int gpsWait = 1;

  Serial.println(WiFi.SSID());

  while ( status != WL_CONNECTED ) {
    Serial.print("Attempt to connect to WPA SSID: ");
    Serial.println(ssid);
    Serial.println(status);

    status = WiFi.begin(ssid, password);
    //wifi_station_set_auto_connect(true);
    delay ( 10000 );
    Serial.print ( "." );
  }

  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);

  delay(5000);

  Serial.println("Connected to wifi");
  serverConnect();
}

void serverConnect() {
  if (client.connect(server, 8080)) {
    Serial.println("connected");
  } else {
    // if you didn't get a connection to the server:
    Serial.println("connection failed");
  }
}

int dataReady = 0;
int inG = 0;
void loop() {
  while (!client.connected()) {
    Serial.println();
    Serial.println("disconnecting.");
    client.stop();
    client.flush();
    serverConnect();
  }

  if (client.available()) {
    //char *c = (char *)client.read();
    char c = client.read();
    client.flush();
    Serial.print(c);
    //Serial.print(dataReady);
    if (c == 'A') {
      Serial.println("ACK received, server connected");
      /*else if (c == 'C') {
        //Serial.println("Tick received");
        imuPoll(); //Accel DATA
        Serial.print(",");
        gpsPoll();
        Serial.print(".");

        arrayPtr++;
        client.flush();*/

    } else if (c == 'G') {
      //Serial.print("G Delay ms: ");
      //Serial.println(millis() - inG);
      //inG = millis();
      //combine();
      char *bytes = (char *) data;
      client.write(bytes, sizeof(data));
      client.flush();
      //char *cbytes = (char *) combData;
      Serial.print("Data");
      Serial.println(sizeof(data));
      //Serial.print("GDATA");
      dataReady = 0;
      //Serial.println(sizeof(gdata));
      //client.write(cbytes, sizeof(combData));
      //char *gbytes = (char *) gdata;
      //client.write(gbytes, sizeof(gdata));
      arrayPtr = 0;
    }
  } else if (!dataReady) {
    int start = millis();
    //Serial.println("DATA NOT READY");
    for (int i = 0; i < 20; i++) {
      imuPoll();
      gpsPoll();
      arrayPtr++;
      //delay(100);
      delay(31);
      //delay(27);
    }
    Serial.print("Data ready in ms: ");
    Serial.println(millis() - start);
    dataReady = 1;
  }
}
