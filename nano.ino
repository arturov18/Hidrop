#include <ArduinoJson.h>//libreria para serializar valores de Presion  
//#include <SoftwareSerial.h>
//Libreria para expansores
#include <Wire.h>    // Required for I2C communication
#include "PCF8574.h" // Required for PCF8574
//Librerias para control Motores
#include "BTS7960.h"

//Librerias para sensores de temperatura
#include <OneWire.h>
#include <DallasTemperature.h>
//librerias BMP280
#include <Adafruit_BMP280.h>

//definicones Temperarura
#define pinSensoresTemp A2
#define TEMPERATURE_PRECISION 9


//Variables de sensores de temperatura
// Setear comunicacion OneWire
OneWire oneWire(pinSensoresTemp);
DallasTemperature sensors(&oneWire);
//Direccion de sensores de temperatura
//en las etiquetas el 4 y 2 estan cambiados
DeviceAddress senTemp1 = {0x28, 0x78, 0x0D, 0x76, 0xE0, 0x01, 0x3C, 0x74  };
DeviceAddress senTemp2 = {0x28, 0xBB, 0x01, 0x76, 0xE0, 0x01, 0x3C, 0x93  };
DeviceAddress senTemp3 = {0x28, 0x8B, 0x50, 0x76, 0xE0, 0x01, 0x3C, 0x26  };
DeviceAddress senTemp4 = {0x28, 0xAD, 0xDE, 0x75, 0xD0, 0x01, 0x3C, 0x1D  };
//Variables de temperatura
float temp1, temp2, temp3, temp4;

//Se debe alimentar con 5.0V para mejor rendimiento
//el valor de salida depende el voltaje de entrada
//Se debe usar una fuente de 5V
#define pinSenPH A1
float calibration_value = 21.34;
int phval = 0;
unsigned long int avgval;
int buffer_arr[10], temp;

//Variables sensores BMP
Adafruit_BMP280 bmp1; // Direccion 0x76
Adafruit_BMP280 bmp2; // Direccion 0x77
//Sensor de presion1
Adafruit_Sensor *bmp_temp1 = bmp1.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure1 = bmp1.getPressureSensor();
//Sensor de presion2
Adafruit_Sensor *bmp_temp2 = bmp2.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure2 = bmp2.getPressureSensor();
//eventos de captura de vallores de temperatura y presion
sensors_event_t temp_event1, pressure_event1, temp_event2, pressure_event2;
float tempAmbiente1, tempAmbiente2, presionAmbiente1, presionAmbiente2;

//Pines de Control Motores BTS7960
//Ojo que estos pines deben ser los mismos para tener el control de velocidad
#define EN1      7
#define L_PWM1   11
#define R_PWM1   10

#define EN2      8
#define L_PWM2   9
#define R_PWM2   6

#define EN3      4
#define L_PWM3   5
#define R_PWM3   3

#define TdsSensorPin A0
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;


//Instancias de control de Motores
BTS7960 motorController(EN1, L_PWM1, R_PWM1);
BTS7960 motorController2(EN2, L_PWM2, R_PWM2);
BTS7960 motorController3(EN3, L_PWM3, R_PWM3);

int velMotor1, velMotor2, velMotor3;

//Intancias de las expansore
PCF8574 expander, expander2;


DynamicJsonDocument docSensoresTx(256);
DynamicJsonDocument docRecibido(256);
//Comunicacion con ESP32
//SoftwareSerial mySerial(12, 13); // RX, TX
long lastMsg = 0;

/*void pinChanged(const char* message, bool pinstate) {
  Serial.print(message);
  Serial.println(pinstate ? "HIGH" : "LOW");
  }*/
void setup() {
  Serial.begin(19200);
  //Inicia comunicacion con esp32
  //mySerial.begin(19200);
  //Serial.println("comenzando");

  //Las direcciones de los expnasores debene estar con los jumpers adecuados para la direccion
  expander.begin(0x20);
  expander2.begin(0x24);
  //Inicialzar los pines del expnasor como salidas

  for (int i = 0; i < 8; i++) {
    expander.pinMode(i, OUTPUT);
    expander2.pinMode(i, OUTPUT);
  }
  for (int i = 0; i < 8; i++) {
    expander.digitalWrite(i, HIGH);
    expander2.digitalWrite(i, HIGH);
  }

  //Habilitar los motores
  motorController.Enable();
  motorController2.Enable();
  motorController3.Enable();

  //Iniicar lectura sensores temperatura
  iniciarTemperatura();
  inicioSensBMP();

}

void loop() {

  // if (mySerial.available() ) {
  if (Serial.available() ) {

  
          expander2.digitalWrite(1, 1);
          expander2.digitalWrite(2, 0);
          expander2.digitalWrite(3, 1);
          expander2.digitalWrite(4, 0);
          expander2.digitalWrite(5, 1);
          expander2.digitalWrite(6, 0);
          expander2.digitalWrite(7, 0);
    //deserializeJson(docRecibido, mySerial);

    Serial.println("recibo por serial");
    deserializeJson(docRecibido, Serial);
    //serializeJson(docRecibido, Serial);
    // Serial.println("");
    if (docRecibido["nombre"] == "modulo1" && docRecibido["salida"] == "out1") {
      expander.digitalWrite(0, docRecibido["valor"]);
    //  Serial.println("recibi salida 1 ");
    }
    if (docRecibido["nombre"] == "modulo1" && docRecibido["salida"] == "out2") {
      //Serial.println("Recibo modulo1");
      expander.digitalWrite(1, docRecibido["valor"]);
    }

    if (docRecibido["nombre"] == "modulo1" && docRecibido["salida"] == "out3") {
      //Serial.println("Recibo modulo1");
      expander.digitalWrite(2, docRecibido["valor"]);
    }
    if (docRecibido["nombre"] == "modulo1" && docRecibido["salida"] == "out4") {
      //Serial.println("Recibo modulo1");
      expander.digitalWrite(3, docRecibido["valor"]);
    }
    if (docRecibido["nombre"] == "modulo1" && docRecibido["salida"] == "out5") {
      //Serial.println("Recibo modulo1");
      expander.digitalWrite(4, docRecibido["valor"]);
    }
    if (docRecibido["nombre"] == "modulo1" && docRecibido["salida"] == "out6") {
      //Serial.println("Recibo modulo1");
      expander.digitalWrite(5, docRecibido["valor"]);
    }
    if (docRecibido["nombre"] == "modulo1" && docRecibido["salida"] == "out7") {
      //Serial.println("Recibo modulo1");
      expander.digitalWrite(6, docRecibido["valor"]);
    }
    if (docRecibido["nombre"] == "modulo1" && docRecibido["salida"] == "out8") {
      //Serial.println("Recibo modulo1");
      expander.digitalWrite(7, docRecibido["valor"]);
    }

    /*
        if (docRecibido["nombre"] == "modulo2") {
          expander2.digitalWrite(0, docRecibido["out1"]);
          expander2.digitalWrite(1, docRecibido["out2"]);
          expander2.digitalWrite(2, docRecibido["out3"]);
          expander2.digitalWrite(3, docRecibido["out4"]);
          expander2.digitalWrite(4, docRecibido["out5"]);
          expander2.digitalWrite(5, docRecibido["out6"]);
          expander2.digitalWrite(6, docRecibido["out7"]);
          expander2.digitalWrite(7, docRecibido["out8"]);

        }
    */
    if (docRecibido["nombre"] == "motores") {
      /*{ "nombre":"motores", "motor1":100, "motor2":200, "motor3":300,
        }*/
      // Serial.println("Recibo motores");
      //Actualizamos la velocidad de los motores.

      velMotor1 = docRecibido["motor1"];
      velMotor2 = docRecibido["motor2"];
      velMotor3 = docRecibido["motor3"];

    }
  }


  long now = millis();
  //Publicacion cada 5 seg
  if (now - lastMsg > 6000) {
    lastMsg = now;
    docSensoresTx["temp1"] = (int)sensors.getTempC(senTemp1);
    docSensoresTx["temp2"] = (int)sensors.getTempC(senTemp2);
    docSensoresTx["temp3"] = (int)sensors.getTempC(senTemp3);
    docSensoresTx["temp4"] = (int)sensors.getTempC(senTemp4);
    lecturasBMP();
    docSensoresTx["tempamb1"] = (int)tempAmbiente1;
    docSensoresTx["tempamb2"] = (int)tempAmbiente2;
    docSensoresTx["presamb1"] = (int)presionAmbiente1;
    docSensoresTx["presamb2"] = (int)presionAmbiente2;

    docSensoresTx["tdsagua"] = random(100, 1000);
    //docSensoresTx["tdsagua"] = (int)lecturaTDS();
    docSensoresTx["ph"] = (int)obtenerPH();

    //serializeJson(docSensoresTx, mySerial);
    serializeJson(docSensoresTx, Serial);
    //Serial.println();
    // Serial.flush();
  }
  //Serial.flush();
}

void iniciarTemperatura() {
  // Iniciamos sensores de temperatura
  sensors.begin();
  // report parasite power requirements
  /*
    Serial.print("Parasite power is: ");

    if (sensors.isParasitePowerMode()) Serial.println("ON");
    else Serial.println("OFF");
  */
  if (!sensors.getAddress(senTemp1, 0)) /*Serial.println("Unable to find address for Device 0")*/;
  if (!sensors.getAddress(senTemp2, 1)) /*Serial.println("Unable to find address for Device 1")*/;
  if (!sensors.getAddress(senTemp3, 2)) /*Serial.println("Unable to find address for Device 2")*/;
  if (!sensors.getAddress(senTemp4, 3)) /*Serial.println("Unable to find address for Device 3")*/;

  // Establecer resolucion de prescision
  sensors.setResolution(senTemp1, TEMPERATURE_PRECISION);
  sensors.setResolution(senTemp2, TEMPERATURE_PRECISION);
  sensors.setResolution(senTemp3, TEMPERATURE_PRECISION);
  sensors.setResolution(senTemp4, TEMPERATURE_PRECISION);

}

void inicioSensBMP() {
  //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
  if (!bmp1.begin(0x76)) {
    // Serial.println(F("Chequear sensor BMP 1"));
    while (1) delay(10);
  }
  if (!bmp2.begin(0x77)) {
    //  Serial.println(F("Chequear sensor BMP 2"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp1.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                   Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                   Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                   Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                   Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  bmp2.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                   Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                   Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                   Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                   Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  //bmp_temp1->printSensorDetails();
  //  bmp_temp2->printSensorDetails();
}

void lecturasBMP() {
  //captura de evento del sensor BMP1
  bmp_temp1->getEvent(&temp_event1);
  bmp_pressure1->getEvent(&pressure_event1);
  //captura de evento del sensor BMP2
  bmp_temp2->getEvent(&temp_event2);
  bmp_pressure2->getEvent(&pressure_event2);

  // Serial.print(F("Temperature1 = ")); Serial.print(temp_event1.temperature); Serial.println(" *C");
  tempAmbiente1 = temp_event1.temperature;
  // Serial.print(F("Pressure1 = ")); Serial.print(pressure_event1.pressure); Serial.println(" hPa");
  presionAmbiente1 = pressure_event1.pressure;
  // Serial.print(F("Temperature2 = ")); Serial.print(temp_event2.temperature); Serial.println(" *C");
  tempAmbiente2 = temp_event2.temperature;
  //Serial.print(F("Pressure2 = ")); Serial.print(pressure_event2.pressure); Serial.println(" hPa");
  presionAmbiente2 = pressure_event2.pressure;

}

float obtenerPH() {
  for (int i = 0; i < 10; i++)
  {
    buffer_arr[i] = analogRead(pinSenPH);
    delay(30);
  }
  for (int i = 0; i < 9; i++)
  {
    for (int j = i + 1; j < 10; j++)
    {
      if (buffer_arr[i] > buffer_arr[j])
      {
        temp = buffer_arr[i];
        buffer_arr[i] = buffer_arr[j];
        buffer_arr[j] = temp;
      }
    }
  }
  avgval = 0;
  for (int i = 2; i < 8; i++)
    avgval += buffer_arr[i];
  float volt = (float)avgval * 5.0 / 1024 / 6;
  float ph_act = -5.70 * volt + calibration_value;

  // Serial.print("pH Val:");
  // Serial.println(ph_act);
  return ph_act;
  //delay(1000);
}

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

float lecturaTDS() {
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)  //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U)
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient; //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value
    //Serial.print("voltage:");
    //Serial.print(averageVoltage,2);
    //Serial.print("V   ");
    //Serial.print("TDS Value:");
    // Serial.print(tdsValue, 0);
    // Serial.println("ppm");
    return tdsValue;
  }
}
