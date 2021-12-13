#include <ArduinoJson.h>//libreria para serializar valores de Presion
//Librerias para celdas decarga
#include <HX711_ADC.h>
#include <EEPROM.h>
//Librerias para WIfi y Mqtt
#include <WiFi.h>
#include <PubSubClient.h>

//Variables para declarar segunda COM serial
#define RxArdu 32
#define TxArdu 33
//#define disparadorNano 32

//33 25 26 27 14 12
//Pines de balanzas
#define pinDatPeso1 34
#define pinCkPeso1  35
#define pinDatPeso2 25
#define pinCkPeso2  26
#define pinDatPeso3 27
#define pinCkPeso3  14

int valTemp1, valTemp2, valTemp3, valTemp4;
int valPeso1, valPeso2, valPeso3, valPeso4;
int valTempAmb1, valTempAmb2, valPresAmb1, valPresAmb2;
int valTdsAgua;
int valPh;


//Variables para conexion Wifi y Mqtt
const char* ssid = "PCCONTROL.EC";
const char* password = "PCCONTROLWIFI!";
const char* mqtt_server = "pccontrol.ml";
const char* idClient = "nodo1prueba";
const char* userMqtt = "web_client";
const char* passMqtt = "121212";
int portMqtt = 1883;

//idClient, userMqtt, passMqtt
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;

//Documentos para los Objetos Json
DynamicJsonDocument docSensoresRx(256);
DynamicJsonDocument docTemp(512);
DynamicJsonDocument docPeso(512);
DynamicJsonDocument docBMP(512);
DynamicJsonDocument docTDS(48);
DynamicJsonDocument docPH(48);
//Documento Json para los mensajes recibidos
DynamicJsonDocument docDisparadores(256);

//docTemp, docPeso, docBMP, docTDS, docPH


//Objetos de celdas decarga
HX711_ADC LoadCell_1(pinDatPeso1, pinCkPeso1); //HX711 1
HX711_ADC LoadCell_2(pinDatPeso2, pinCkPeso2); //HX711 1
HX711_ADC LoadCell_3(pinDatPeso3, pinCkPeso3); //HX711 1

//Declaracion d espacios de memoria EEPROM para valores de celdas de carga
const int eepromAdress_1 = 0; // eeprom adress for calibration value load cell 1 (4 bytes)
const int eepromAdress_2 = 4; // eeprom adress for calibration value load cell 2 (4 bytes)

/*
  hidroponia/nodo1/disparadores/#
  hidroponia/nodo1/disparadores/modulosexpan/modulo1
  hidroponia/nodo1/disparadores/modulosexpan/modulo2
  hidroponia/nodo1/disparadores/motores
*/

void setup() {
  Serial.begin(115200);
  Serial2.begin(19200, SERIAL_8N1, RxArdu, TxArdu);// serial2 pines 16 y 17

  //Iniciar el wifi y conectar cliente a Broker
  setup_wifi();
  client.setServer(mqtt_server, portMqtt);
  client.setCallback(callback);


  /*
    float calValue_1; // Variable de calibracion celda 1
    float calValue_2; // Variable de calibracion celda 2

    calValue_1 = 761.80;//696.0; // calibrado el
    calValue_2 = 620.64;//733.0; // calibrado el
    calValue_3 = 620.64;//733.0; // calibrado el

    LoadCell_1.begin();
    LoadCell_2.begin();
    LoadCell_3.begin();

    long stabilisingtime = 2000; // Se neceita algunos segundos para realizar la tara
    byte loadcell_1_rdy = 0;
    byte loadcell_2_rdy = 0;
    byte loadcell_3_rdy = 0;
    while ((loadcell_1_rdy + loadcell_2_rdy+loadcell_3_rdy) < 3) { //corren ambos modulos simultanemente
      if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilisingtime);
      if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilisingtime);
      if (!loadcell_3_rdy) loadcell_3_rdy = LoadCell_3.startMultiple(stabilisingtime);

    }
    LoadCell_1.setCalFactor(calValue_1); // user set calibration value (float)
    LoadCell_2.setCalFactor(calValue_2); // user set calibration value (float)
    LoadCell_3.setCalFactor(calValue_3); // user set calibration value (float)

    Serial.println("Arranque de celdas y TARA compeltados");
  */
}

void loop() {

  if (Serial2.available() > 0) {
    deserializeJson(docSensoresRx, Serial2);
    //serializeJson(docSensoresRx, Serial);
    valTemp1 = docSensoresRx["temp1"];
    valTemp2 =    docSensoresRx["temp2"];
    valTemp3 = docSensoresRx["temp3"];
    valTemp4 = docSensoresRx["temp4"];
    valTempAmb1 = docSensoresRx["tempamb1"] ;
    valTempAmb2 = docSensoresRx["tempamb2"] ;
    valPresAmb1 = docSensoresRx["presamb1"] ;
    valPresAmb2 = docSensoresRx["presamb2"] ;
    valTdsAgua = docSensoresRx["tdsagua"];
    valPh = docSensoresRx["ph"];
  }


  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  //Serial2.write("hola");
  long now = millis();
  //Publicacion cada 5 seg
  if (now - lastMsg > 10000) {
    lastMsg = now;

    docTemp["temp1"] = valTemp1;
    docTemp["temp2"] = valTemp2;
    docTemp["temp3"] = valTemp3;
    docTemp["temp4"] = valTemp4;


    docPeso["peso1"] = random(100, 2000);
    docPeso["peso2"] = random(100, 2000);
    docPeso["peso3"] = random(100, 2000);
    docPeso["peso4"] = random(100, 2000);

    docBMP["tempamb1"] = valTempAmb1;
    docBMP["tempamb2"] = valTempAmb2;
    docBMP["presamb1"] = valPresAmb1;
    docBMP["presamb2"] = valPresAmb2;

    docTDS["tdsagua"] = valTdsAgua;

    docPH["ph"] = valPh;


    char buffer1[256];
    char buffer2[256];
    char buffer3[256];
    char buffer4[16];
    char buffer5[16];
    //docTemp, docPeso, docBMP, docTDS, docPH
    size_t n1 = serializeJson(docTemp, buffer1);
    size_t n2 = serializeJson(docPeso, buffer2);
    size_t n3 = serializeJson(docBMP, buffer3);
    size_t n4 = serializeJson(docTDS, buffer4);
    size_t n5 = serializeJson(docPH, buffer5);
    //client.publish("outTopic", buffer, n);
    client.publish("hidroponia/nodo1/sensores/temperatura", buffer1, n1);
    client.publish("hidroponia/nodo1/sensores/peso",        buffer2, n2);
    client.publish("hidroponia/nodo1/sensores/bmp",         buffer3, n3);
    client.publish("hidroponia/nodo1/sensores/tds",         buffer4, n4);
    client.publish("hidroponia/nodo1/sensores/ph",          buffer5, n5);
    Serial.println("Los valores a emviar :");
    serializeJson(docTemp, Serial);
    serializeJson(docPeso, Serial);
    serializeJson(docBMP, Serial);
    serializeJson(docTDS, Serial);
    serializeJson(docPH, Serial);
    //Serial.print(tempString1); Serial.print(" ");Serial.print(tempString2); Serial.print(" ");Serial.print(tempString3); Serial.print(" ");Serial.print(tempString4);
    Serial.println("");
  }

}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  deserializeJson(docDisparadores, (const byte*)message, length);
  //Serial.println(docDisparadores);

  Serial.println("");
  if (docDisparadores["nombre"] == "modulo1") {
    Serial.println("Recibo modulo1");
  }
  if (docDisparadores["nombre"] == "modulo2") {
    Serial.println("Recibo modulo2");
  }
  if (docDisparadores["nombre"] == "motores") {
    Serial.println("Recibo motores");
  }
  serializeJson(docDisparadores, Serial);
  Serial.println();
  //Serial2.write("hola");
  serializeJson(docDisparadores, Serial2);
  /* String messageTemp;

    for (int i = 0; i < length; i++) {
     Serial.print((char)message[i]);
     messageTemp += (char)message[i];
    }
    Serial.println();
  */

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
  // Changes the output state according to the message
  /*
    if (String(topic) == "hidroponia/nodo1/disparadores/#") {
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("on");
      // digitalWrite(ledPin, HIGH);
    }
    else if (messageTemp == "off") {
      Serial.println("off");
      // digitalWrite(ledPin, LOW);
    }
    }
  */
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(idClient, userMqtt, passMqtt)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("hidroponia/nodo1/disparadores/#");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
