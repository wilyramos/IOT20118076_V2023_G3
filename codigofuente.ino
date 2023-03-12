/*-------Librerias-------------------------*/

#include "DHTesp.h" //Librería para el sensor de humedad
#include <WiFi.h> 
#include <OneWire.h> ///Librería DS18B20 Dallas Temperatura
#include <DallasTemperature.h> ///Librería DS18B20 Dallas Temperatura
#include <ArduinoJson.h> // Libreria para convertir los datos en formato JSON
#include <PubSubClient.h> // Libreria para publicar y suscribir al MQTT

/*-------Sensor DHT11 Sensor de humedad-------------------------*/

int pinDHT = 15; //Decaramos el variable que almacena el pin a conectar el DHT11 pin 15
DHTesp dht;//Instanciamos el DHT

/*-------Sensor MQ4 Sensor de metano------------------------------*/

int LED1 = 32;//Pin LED definido conectado en D32
int Sensor_input = 34;//Pin digital 34 para la entrada de sensor en A0

/*-------Sensor DS18B20 sensor de Temperatura compost-------------*/

#define DS18B20 14  // Se conecta al pin 14 del esp32
OneWire oneWire(DS18B20); //Instacia el Objeto oneWire en el pin del Sensor para iniciar las lecturas
//Reenvia referencias de oneWire al sensor de Dallas(DS18B20)
DallasTemperature Sensor(&oneWire);
float lectura_temCompost; // Variable para almacenar datos

/*-------Sensor YL-69 sensor de Humedad compost-------*/

const int humsuelo = 33; //Conectado con el PIN A0 del sensor
int valHumsuelo; //Almacena el valor de la humedad en la variable

/*-------Sensor HC-SR04 sensor de Ultrasonido-------*/

int DISTANCIA =0;
int pinLed=26; // Declaramos Led en el pin 26
int pinEco=12; // Declaramos Led en el pin 12
int pinGatillo=13; // Declaramos Led en el pin 13

/*-------Variables para el loop-------*/

uint32_t timerUpdate = 0; //Para calcular la diferencia en el loop
const int DELAY_UPDATE = 1000; //Delay de 1 seg

/*-------Información de WiFi---------------*/

const char* ssid = "WIFI";
const char* password = "CONTRASEÑA";

/*-------Información del broker MQTT---------------*/

const char* mqtt_server = "broker.emqx.io";

/*-------Declaramos las variables para enviar los datos de los sensores.---------------*/
char data_temp[12]="";
char data_humi[12]="";
char data_humiCompost[12]="";
char data_meta[12]="";
char data_ds18b20[12]="";
char data_hcsr04[12]="";

/*----- Inicializacion del esp32 para el wifi mqtt---*/
WiFiClient esp32;
PubSubClient client(esp32);

/*-----Se declara la el long para calcular la distancia---*/
long leerDistanciaUltrasonido(int pinGatillo, int pinEco){
  pinMode(pinGatillo, OUTPUT);  //Iniciamos el pin emisor de ruido en salida
  digitalWrite(pinGatillo, LOW);  //Apagamos el emisor de sonido 
  delayMicroseconds(2);//Retrasamos la emision de sonido por 2 milesimas de segundo 
  digitalWrite(pinGatillo, HIGH);//Comenzamos a emitir sonido
  delayMicroseconds(10);//Retrasamos la emision de sonido por 10 milesimas de segundo
  digitalWrite(pinGatillo, LOW);//Apagamos el emisor de sonido
  pinMode(pinEco, HIGH);//Comenzamos a escuchar el sonido
  return pulseIn(pinEco, HIGH);//Calculamos el tiempo que tardo en regresar el sonido
}

// Conexión a nuestro WIFI
void config_wifi(){
  delay(10);
  WiFi.mode(WIFI_STA);
  Serial.println("");
  Serial.print("conectando ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while(WiFi.status() !=WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
 
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

/* ---Conexion al MQTT para reconectar-----*/

void reconnect() {
  // Bucle hasta que estemos reconectados
  while (!client.connected()) {
    Serial.print("Intentando la conexión MQTT...");
    // Crear una identificación de cliente aleatoriaID
    String clientId = "ESP32";
    // intento de conexión
    if (client.connect(clientId.c_str())) {
      Serial.println("conectado");
    }
    else {
      Serial.print("fallido, rc=");
      Serial.print(client.state());
      Serial.println("inténtalo de nuevo en 3 segundos");
      // Espere 5 segundos antes de volver a intentarlo
      delay(3000);
    }
  }
}

/*------------Sensor DHT11 Sensor de humedad--------------------------*/
void setup() {

  // monitor serial
  Serial.begin(115200);
  config_wifi(); // llamada a la funcion para conectar al wifi

  /*Sensor DHT11 Sensor de humedad--------------------------*/
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);


  /*Sensor MQ4 Sensor de metano------------------------------*/
  pinMode(pinLedRojo, OUTPUT);


  /*Sensor DS18B20 sensor de Temperatura compost-------------*/
  DS18B20.begin();


  /*Sensor YL-69 sensor de Humedad compost-------------*/
  pinMode(humsuelo, INPUT);

  /*Sensor HC-SR04 sensor de Ultrasonido---------------*/
  pinMode(pinLed, OUTPUT);
 
 
 /* Inicializacion a la conexion del servidor mqtt---------------*/*/
  client.setServer(mqtt_server,1883);

}


void loop() {

  /* Verificar la conexion al wifi */
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  /* Condicional para enviar periodicamente al nodered*/
  if (millis() - timerUpdate >= DELAY_UPDATE){
    timerUpdate = millis();  
        
    Serial.println("");
    Serial.println("\t****DATOS DEL COMPOST******");
    Serial.println("------------");
   
    /*Sensor DHT11 Sensor de humedad------------------------*/
    TempAndHumidity  data = dhtSensor.getTempAndHumidity();
    Serial.println("Temperatura: " +  String(data.temperature,2) + "°C");
    Serial.println("Humedad: " + String(data.humidity,1) + "%");
    Serial.println("------------");

    /*---------------Sensor MQ4 Sensor de metano------------------------------*/

    int  lectura_mq4 = analogRead(Sensor_input);
    Serial.print("Sensor de gas metano: ");  
    Serial.print(lectura_mq4);   /*Imprime el valor del sensor*/
    Serial.print("\t"); 

    
   
    Serial.println("------------");
   
    /*-----Sensor DS18B20 sensor de Temperatura compost-------------*/

    DS18B20.requestTemperatures(); //Se envía el comando para leer la temperatura
    float t_ds18b20= DS18B20.getTempCByIndex(0);
    Serial.print("Temperatura ds18b20: ");
    Serial.println(t_ds18b20);
    Serial.println("------------");
   
    /*Sensor YL-60 sensor de Humedad compost-------------*/

    valHumsuelo = map(analogRead(humsuelo), 4092, 0, 0, 100);
    Serial.print("Humedad del suelo: ");
    Serial.print(valHumsuelo);
    Serial.println(" %");  
    Serial.println("------------");
   
    /*Sensor HC-SR04 sensor de Ultrasonido---------------*/

    //calculamos la disctancia en cm
    DISTANCIA = 0.01723 * leerDistanciaUltrasonido(pinGatillo, pinEco);

    //Mostramos la distancia
    Serial.print("Distancia: ");
    Serial.println(DISTANCIA);

    //Si la distancia es menor a 20cm encendemos la led
    if(DISTANCIA < 20){
      digitalWrite(pinLed, HIGH);
    }
    //Si la distancia es mayor a 20 apagamos el led
    else{
      digitalWrite(pinLed, LOW);
    }
    Serial.println("------------");

/*---------- ALARMAS CON CONTROL DE LEDS-------*/

/* Encender el LED si la temperatura es menor a 30 grados*/
  if (temperaturaDHT > 30) {
    digitalWrite(LED1, HIGH);
  } else if{
    digitalWrite(LED1, LOW);
  }

/* Encender el LED si la humedad es menor 50 */
  if (valHumsuelo < 50) {
    digitalWrite(LED2, HIGH);
  } else {
    digitalWrite(LED2, LOW);
  }

/* Si el valor del metano es mayor a 1800, activar la alarma */

  if (lectura_mq4 > 1800) {    /*Si la lectura del sensor pasa el umbral 1800*/
      Serial.println("Hay presencia");  
      digitalWrite (pinLedRojo, HIGH) ; /*Enciende el LED en el sistema*/
    }
  else {
      Serial.println("No hay presencia");/*Si la lectura del sensor no pasa el umbral 1800*/
      digitalWrite (pinLedRojo, LOW) ;  /*Apaga el LED en el sistema*/
  }

/* Encender el LED si la distancia es menor a 5 cm  */
  if (valHumsuelo < 5) {
    digitalWrite(LED3, HIGH);
  } else {
    digitalWrite(LED3, LOW);
  }

/* Encender el LED si la temperatura del ds18b20 es menor a 29  */
  if (["DS18B20"]  > 30) {
    digitalWrite(LED4, HIGH);
  } else if{
    digitalWrite(LED4, LOW);
  }

/* CREACION DEL JSON CON LOS DATOS DE LOS SENSORES */

    StaticJsonDocument<128> doc;
    doc["TEMP_dht11"] = data.temperature;
    doc["HUM_dht11"] = data.humidity;
    doc["DS18B20"] = t_ds18b20;
    doc["METANO"] = lectura_mq4;    
    doc["YL_69"] = valHumsuelo;
    doc["HC_SR04"] = DISTANCIA;
     
    String output; //Variable de salida

    //Serializacion de los datos del JSON
    serializeJson(doc, output);
   
    Serial.print("Publish message: ");
    Serial.println(output);

    //Se envian los datos serializados al servidor mosquito en el topico Sensores
    client.publish("sensores",output.c_str());
  }
  delay(10000);
}
