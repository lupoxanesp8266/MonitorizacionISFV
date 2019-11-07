/********************************************* Módulo Proyecto para TFC *****************************************/
/****************************************************************************************************************/
/********************************** MONITORIZACIÓN DE ISFV EN EL IES CIUDAD JARDÍN ******************************/
/****************************************************************************************************************/
/***************************** Inicio: 21 - 11 - 2017 *** Finalización: 13 - 06 - 2018 **************************/
/****************************************************************************************************************/
/***************************************** Hecho por --> Carlos Sánchez Prudencio *******************************/
/****************************************************************************************************************/
/* http://www.esploradores.com/thinger-io-configuracion-almacenamiento-y-monitorizacion-de-datos-primera-parte/ */
/****************************************************************************************************************/
/* Incluir bibliotecas de funciones */
#include <ESP8266WiFi.h>//Biblioteca de conexión WiFi del módulo ESP8266
#include <ThingerESP8266.h>//Biblioteca de la plataforma thinger.io
#include <Wire.h>//Biblioteca para I2C
#include <RTClib.h>//Para RTC
#include <LiquidCrystal_I2C.h>//Para la lcd
#include <SD.h>//Para la SD
//
/* Definir constantes */
#define SSID1 "SSID"//SSID Wifi
#define PASS1 "PWD"//Password Wifi
//
#define usuario "userName"//Nombre de usuario de thinger
#define device_Id "deviceName"//Nombre del dispositivo al que nos conectaremos
#define device_credentials "deviceCredentials"//Credenciales generadas por Thinger
//
#define TIME 1000//Tiempo para medir valores
#define MAX 75//Máximas muestras para hacer media (muestreo)
#define MAXVAL 60//Maximo de valores que guarda por minuto
#define MAXSENS 6//Maximo de sensores que hay
//
#define timer0_preload 40161290//Timer que controla los cortes en la LCD
#define my_delay 10//No menos de 5
#define ADC 1023//Resolución del convertidor ADC
//
ThingerESP8266 thing(usuario, device_Id, device_credentials);//Variable para acceder a Thinger
//
//Parametros variables de tension
float offset_v = 3.39;//Punto que corta en el eje de ordenadas (y)
float voltage_v = 3.27;//Tension que da el ESP8266
float relation_v = -12.987;//Relacion tension/sensor, en este caso 77mV por voltio de entrada que equivale a 1V por 12.987v de entrada
//Parametros variables de corriente
float offset_i = 1.625;//Punto que corta en el eje de ordenadas (y)
float voltage_i = 3.27;//Tension que da el ESP8266
float relation_i = 0.149;//Relacion tension/sensor, en este caso 150mV por voltio de entrada
//
//Variable de tiempo de llamada para endpoints en minutos
int calltime = 720;//Lo que viene siendo cada doce horas
//
//Variables lcd, RTC y SD
LiquidCrystal_I2C lcd(0x27, 16, 2);//Variable para lcd0x3F
//RTC_DS3231 RTC;//Variable de RTC tipo DS3231 o DS1307 ¡OJO!
RTC_DS1307 RTC;//Descomentar para utilizar
File dataFile;//Variable para SD
//
//Variables de conexión WiFi
const char WiFi_ssid[] = SSID1;//Nombre de red
const char WiFi_password[] = PASS1;//Clave de red
//
//Variables para utilizar el multiplexor
const int muxSIG = A0;//A0
const int muxS0 = D0;//D0
const int muxS1 = D3;//D3
const int muxS2 = D4;//D4
const int muxS3 = D9;//Rx
//
/* Estructuras de datos */
typedef struct {//Por futuras modificaciones
  float valores[MAXVAL];//Valor de cada sensor
} mediciones;//Para guardar los valores de las mediciones
//
mediciones vec[MAXSENS];//Vector de registro tipo mediciones
//
//Variables globales
unsigned long presentime, ahora;//Para contar el tiempo
unsigned long previoustime = 0, antes = 0;//Para contar el  tiempo
int k = 0, i = 0;//Índice para seleccionar los valores de cada sensor
char buffer[128];//Añadir a un String
String strValues;//String que guarda valores
int entero, decimal;//Para separar los valores enteros de los decimales para añadirlos a strValues
int num;//Número de sensor para visualizar en la lcd
char tipo, ud; //tipo: si es V o I; ud: unidades de v, A.
//
/* Cambiar timer para visualizar los valores en la lcd */
void inline cambio (void) {
  switch (k) {
    case 0:// Valor ---> I2
      k = 1;
      tipo = 'I';
      ud = 'A';
      num = 2;
      break;
    case 1:// Valor ---> I3
      k = 2;
      tipo = 'I';
      ud = 'A';
      num = 3;
      break;
    case 2:// Valor ---> V1
      k = 3;
      tipo = 'V';
      ud = 'v';
      num = 1;
      break;
    case 3:// Valor ---> V2
      k = 4;
      tipo = 'V';
      ud = 'v';
      num = 2;
      break;
    case 4:// Valor ---> V3
      k = 5;
      tipo = 'V';
      ud = 'v';
      num = 3;
      break;
    case 5:// Valor ---> I1
      k = 0;
      tipo = 'I';
      ud = 'A';
      num = 1;
      break;
  }
  timer0_write(ESP.getCycleCount() + timer0_preload * my_delay); //Escribir en el timer
}
//
/** SETUP() **/
void setup() {
  Serial.begin(115200);//Iniciar la transmisión Serial a 115200 bauds

  RTC.begin();//Iniciar RTC
  if (!RTC.begin()) {//Comprobar si RTC está conectada
    Serial.println(F("Couldn't find RTC"));
    while (1);
  }
  //RTC.adjust(DateTime(__DATE__, __TIME__));//Para ajustar fecha y hora

  lcd.begin();//Iniciar pantalla lcd
  lcd.backlight();//Iniciar luz de fondo

  leer_tarjeta();//Para iniciar tarjeta y leer archivo

  Output_pins();//Iniciar los pines a OUTPUT

  WiFi.begin(WiFi_ssid, WiFi_password);// Inicialización de la WiFi para comunicarse con la API
  lcd.clear();//Borrar pantala
  lcd.setCursor(0, 0);
  lcd.print("Conectando WiFi");
  Serial.print("Conectandose a red : ");
  Serial.print(WiFi_ssid);
  while (WiFi.status() != WL_CONNECTED)//Para saber si está conectado o no a la red WiFi
  {
    delay(500);
    Serial.print(". ");
  }
  Serial.println("WiFi conectado");
  lcd.setCursor(0, 1);
  lcd.print("WiFi conectado");
  // Inicialización de la lectura de datos desde la API
  elementos_clave();
  delay(1000);//Esperar 1S para estabilizar
  lcd.clear();//Limpiar la lcd
  noInterrupts();//No permitir interrupciones
  timer0_isr_init();//Inicializar el counter interno
  timer0_attachInterrupt(cambio);//Llamada al módulo de cambio para la lcd
  timer0_write(ESP.getCycleCount() + timer0_preload * my_delay);//Escribir en el counter
  interrupts();//Permitir interrupciones
}

/** LOOP() **/
void loop() {
  DateTime now = RTC.now();//Variable para RTC
  thing.handle();//Objeto que llama a la conexión con el servidor Thinger
  presentime = millis();
  ahora = millis();
  if (presentime - previoustime >= TIME) {
    previoustime = presentime;
    if (i > MAXVAL - 1 ) {//Si excede de 60 valores resetarlo a cero
      i = 0;
    }
    sprintf(buffer, "%02d/%02d/%04d %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
    strValues = buffer;
    lcd.setCursor(0, 1);
    lcd.print(strValues);
    for (int j = 0; j <= MAXSENS - 1; j++) {
      if (j >= 0 && j <= 3 - 1) {
        vec[j].valores[i] = medicion(true, j);
        entero = vec[j].valores[i];
        decimal = (vec[j].valores[i] - entero) * 100;
        sprintf(buffer, "\t%d,%02d", entero, abs(decimal));
        strValues += buffer;
      }
      else {
        vec[j].valores[i] = medicion(false, j);
        entero = vec[j].valores[i];
        decimal = (vec[j].valores[i] - entero) * 100;
        sprintf(buffer, "\t%d,%02d", entero, abs(decimal));
        strValues += buffer;
      }
    }
    escribir_tarjeta(strValues);
    Serial.println(strValues);
    entero = vec[k].valores[0];//Separar entero de decimal
    decimal = (vec[k].valores[0] - entero) * 100;//Separar entero de decimal
    sprintf(buffer, "%c%d: %d,%02d", tipo, num, entero, abs(decimal));
    lcd.setCursor(0, 0);
    lcd.print(buffer);
    i++;
  }
  if (ahora - antes >= calltime * 60000) {
    antes = ahora;
    call_corrientes_endpoint();
    call_tensiones_endpoint();
    call_potencias_endpoint();
  }
}

/** PINES DE SALIDA "OUTPUT" **/
void Output_pins() {
  pinMode(muxS0, OUTPUT);//D0
  pinMode(muxS1, OUTPUT);//D3
  pinMode(muxS2, OUTPUT);//D4
  pinMode(muxS3, OUTPUT);//D9 --> Rx
  //pinMode(A0,INPUT);//No hace falta descomentar
}

/** FUNCIÓN PARA UTILIZAR EL MULTIPLEXOR **/
int SetMuxChannel(byte channel)
{
  digitalWrite(muxS0, bitRead(channel, 0));
  digitalWrite(muxS1, bitRead(channel, 1));
  digitalWrite(muxS2, bitRead(channel, 2));
  digitalWrite(muxS3, bitRead(channel, 3));
}
/** FUNCIÓN PARA LEER LA TARJETA SD **/
void leer_tarjeta() {
  Serial.print(F("Iniciando SD ... "));
  lcd.setCursor(0, 0);
  lcd.print("Iniciando SD ... ");
  if (!SD.begin(D8)) {
    Serial.println(F("Error al iniciar"));
    lcd.setCursor(0, 1);
    lcd.print("Error :(");
    return;
  }
  Serial.println(F("Correcto"));
  lcd.setCursor(0, 1);
  lcd.print("Correcto :)");
  if (dataFile = SD.open("datalog.csv")) {
    /*while (dataFile.available()){
      Serial.write(dataFile.read());
      } *///Descomentar para leer toda la SD
    dataFile.close();
  }
  else {
    Serial.println(F("Error al abrir el archivo"));
  }
  delay(1000);//Esperar 1S para estabillizar
}
/** FUNCIÓN PARA ESCRIBIR EN LA TARJETA SD **/
void escribir_tarjeta(String strValues) {
  File logFile;
  DateTime now = RTC.now();
  logFile = SD.open("datalog.csv", FILE_WRITE);
  if (logFile) {
    logFile.println(strValues);
    logFile.close();
  }
  else {
    Serial.println("Error al abrir el archivo");
  }
}
/* Hacer cálculos para tener los valores de tensión y corriente */
float medicion(boolean tipo, int indice) {
  float value;
  float media = 0.00;
  if (tipo == true) { //Corriente
    for (int k = 0; k <= MAX - 1; k++) {
      SetMuxChannel(indice);
      value = analogRead(muxSIG) * voltage_i / ADC;
      //value = (value - 0.8) / 0.100;//0.185 for 5 A 2ºForma de calcular la corriente
      value = (value - offset_i) / relation_i;
      media = media + value;
    }
  }
  else { //Tensión
    for (int k = 0; k <= MAX - 1; k++) {
      SetMuxChannel(indice);
      value = analogRead(muxSIG) * voltage_v / ADC;
      //value = mapeo(value, 0, 1024, 50.0, 0.0);//2º forma de calcular la tensión
      value = (value - offset_v) * relation_v;
      media = media + value;
    }
  }
  media = media / MAX;
  return media;

}
void elementos_clave() {
  //Parametros para las mediciones
  thing["Corrientes"] >> [](pson & out) {
    out["I1"] = medicion(true, 0);//Corriente paneles
    out["I2"] = medicion(true, 1);//Corriente baterias
    out["I3"] = medicion(true, 2);//Corriente carga
  };
  thing["Tensiones"] >> [](pson & out) {
    out["V1"] = medicion(false, 3);//Tension paneles
    out["V2"] = medicion(false, 4);//Tension baterias
    out["V3"] = medicion(false, 5);//Tension carga
  };
  thing["Potencias"] >> [](pson & out) {
    out["P1"] = medicion(true, 0) * medicion(false, 3);//Potencia paneles
    out["P2"] = medicion(true, 1) * medicion(false, 4);//Potencia baterias
    out["P3"] = medicion(true, 2) * medicion(false, 5);//Potencia carga
  };
  //Parametros para calibrar la tension
  thing["Offset_v"] << inputValue(offset_v);
  thing["Voltage_v"] << inputValue(voltage_v);
  thing["Relation_v"] << inputValue(relation_v);
  //Parametros para calibrar la corriente
  thing["Offset_i"] << inputValue(offset_i);
  thing["Voltage_i"] << inputValue(voltage_i);
  thing["Relation_i"] << inputValue(relation_i);
  //Para cambiar el tiempo de espera de los endpoints
  thing["Calltime"] << inputValue(calltime);//Esperemos que funcione
  //Probar los endpoints cada vez que se quiera desde Thinger.io
  thing["Potencias_test"] = []() {
    call_potencias_endpoint();
  };
  thing["Corrientes_test"] = []() {
    call_corrientes_endpoint();
  };
  thing["Tensiones_test"] = []() {
    call_tensiones_endpoint();
  };
}
/* Para llamar a los endpoints se crean variables tipo pson (json) para integrarlos en IFTTT */
void call_tensiones_endpoint() {//Se añadirá una nueva línea en un Google Sheet
  pson tensiones;//Esta en concreto lleva asociada una cuenta de Twitter para subir un tweet cada vez que se añada una nueva línea
  tensiones["value1"] = medicion(true, 0);//Tensión paneles
  tensiones["value2"] = medicion(true, 1);//Tensión baterias
  tensiones["value3"] = medicion(true, 2);//Tensión carga
  thing.call_endpoint("Tensiones", tensiones);
}
void call_corrientes_endpoint() {//Se añadirá una nueva línea en un Google Sheet
  pson corrientes;
  int m, entero, decimal;
  String value1, value2, value3;
  char buffer1[128];
  float valor;
  for (m = 3; m <= 5; m++) {
    valor = medicion(false, m);
    entero = valor;
    decimal = (valor - entero) * 100;
    sprintf(buffer1, "%02d.%02d", entero, decimal);
    switch(m){
      case 3:
      value1 = buffer1;
      break;
      case 4:
      value2 = buffer1;
      break;
      case 5:
      value3 = buffer1;
      break;
    }
  }
  corrientes["value1"] = value1;//Corriente paneles
  corrientes["value2"] = value2;//Corriente baterias
  corrientes["value3"] = value3;//Corriente carga
  thing.call_endpoint("Corrientes", corrientes);
}
void call_potencias_endpoint() {//Se añadirá una nueva línea en un Google Sheet
  pson potencias;
  potencias["value1"] = medicion(true, 0) * medicion(false, 3);//Potencia paneles
  potencias["value2"] = medicion(true, 1) * medicion(false, 4);//Potencia baterias
  potencias["value3"] = medicion(true, 2) * medicion(false, 5);//Potencia carga
  thing.call_endpoint("Potencias", potencias);
}
