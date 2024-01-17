#include "DHTStable.h"              //DHT22
#include "DFRobot_OzoneSensor.h"

DHTStable DHT;
#define DHT22_PIN       6
struct
{
    uint32_t total;
    uint32_t ok;
    uint32_t crc_error;
    uint32_t time_out;
    uint32_t connect;
    uint32_t ack_l;
    uint32_t ack_h;
    uint32_t unknown;
} counter = { 0,0,0,0,0,0,0,0};

#define COLLECT_NUMBER   20              // collect number, the collection range is 1-100
#define Ozone_IICAddress OZONE_ADDRESS_3
DFRobot_OzoneSensor Ozone;
float temp, humi;
char CO_data[60];    //Hardware-designed co data archiving
char NO2_data[60];    //Hardware-designed no2 data archiving
char SO2_data[60];    //Hardware-designed so2 data archiving
void setup(){
  Serial.begin(9600);
  Serial3.begin(9600);        //SO2
  Serial2.begin(9600);        //NO2
  Serial1.begin(9600);        //CO
  Ozone.begin(Ozone_IICAddress);
  Ozone.setModes(MEASURE_MODE_PASSIVE);
}

void loop(){

    //SO2, NO2, CO
    Serial1.write('\r');
    Serial2.write('\r');
    Serial3.write('\r');
    delay(1000);

    int i = 0;

    //SO2, NO2 data get
    while (Serial1.available()) {
      CO_data[i++] = Serial1.read();
//      Serial.print(CO_data[i]);
    }
    delayMicroseconds(10);
    i = 0;
    while (Serial2.available()) {
      NO2_data[i] = Serial2.read();
//      Serial.print(NO2_data[i]);
      i++;
    }
    delayMicroseconds(10);
    i = 0;
    while (Serial3.available()) {
      SO2_data[i] = Serial3.read();
//      Serial.print(SO2_data[i]);
      i++;
    }
    int ozoneConcentration = Ozone.readOzoneData(COLLECT_NUMBER);
    int chk = DHT.read22(DHT22_PIN);
    switch (chk)
    {
    case DHTLIB_OK:
        counter.ok++;
        humi = DHT.getHumidity();
        temp = DHT.getTemperature();
        break;
    case DHTLIB_ERROR_CHECKSUM:
        counter.crc_error++;
        //Serial.print("Checksum error,\t");
        break;
    case DHTLIB_ERROR_TIMEOUT:
        counter.time_out++;
        //Serial.print("Time out error,\t");
        break;
    default:
        counter.unknown++;
        //Serial.print("Unknown error,\t");
        break;
    }
    delay(100);

    String str_co_data = "";
    String str_no2_data = "";
    String str_so2_data = "";

    for(int j = 12; j < 18; j++){
      str_co_data += CO_data[j];
      str_no2_data += NO2_data[j];
      str_so2_data += SO2_data[j];
    }

    float coPPM, no2PPM, so2PPM;
    float Ozone_ppm = (float)ozoneConcentration / 1000;
    coPPM = value_convert(str_co_data);
    no2PPM = value_convert(str_no2_data);
    so2PPM = value_convert(str_so2_data);

    Serial.print(coPPM, 3);
    Serial.print(",");
    Serial.print(no2PPM, 3);
    Serial.print(",");
    Serial.print(so2PPM, 3);
    Serial.print(",");
    Serial.print(temp);
    Serial.print(",");
    Serial.print(humi);
    Serial.print(",");
    Serial.println(Ozone_ppm, 3);
}

float value_convert(String value){
  int start = value.indexOf(',') + 1;
  int end = value.indexOf(',', start);

  float con_ppm = 0.0;
  String con_value = value.substring(start, end);
  int convalue = con_value.toInt();
  con_ppm = (float)convalue / 1000;

  return con_ppm;
}
