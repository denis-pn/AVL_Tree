/*
   Мастер создаёт сетевую таблицу с номерами узлов и их состоянием.
   nodeID записывается на позицию соответсвующую его номеру с 0 по 9 (можно расширить).
   Во втором столбце таблицы указывается активность: 0 - узел не активен, 1 - активен.
   Если в обоих столбцах 0, значит такого узла в сети ещё не было.
   Если произошла смена мастера, то новый мастер ставит на свою бывшую позицию в первом столбце 0.
   При этом во втором столбце остаётся 1, это позволяет не терять информацю.

   Префиксы сообщений:
   H - сетевая таблица состояния
   S - данные от узлов
   M - служебная информация (добавим различные коды)
   C - пустые сообщения для проверки подключения к сети
   Максимальная длина сообщения 120 байт (uint8_t до 255)
*/
#include <AESLib.h>
#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <printf.h>
#include "nRF24L01.h"
#include <EEPROM.h>


RF24 radio(9, 10); // RF24 radio(1, 0); // для 32U4 9 10
RF24Network network(radio);
RF24Mesh mesh(radio, network);

//#define MESH_NOMASTER

//переменные для работы radio
int servise_channel = 70;
bool p0 = 0; //перключатель radio/mesh
//перменные для работы mesh

uint8_t nodeID = 3;


//статический ip - задел на будущее
uint8_t static_IP[4] = {192, 168, 10, nodeID}; //последний октет совпадает с nodeID присвоенным изначально

uint8_t netTable[10][2]; // сетевая таблица для учёта смены узла
uint8_t channel = 81;// несущая mesh по умолчанию 81
uint8_t key_0[] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}; //ключ для шифрования частоты

uint8_t data[] = {255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0};
uint8_t dataR[32];
uint32_t del;//время задержки отправки сообщений
bool p = 0;//переключатель передачи тестовых сообщений
uint8_t k;
uint8_t o; //мсчётчик для переключения узла обратно в радиорежим
uint8_t n; // nodeID узла для отправки сообщения
int ID; //nodeID на приём
uint32_t pdr;
uint32_t t[10]; // массив времён передачи для рассчёта IPTD
uint32_t IPTD[9];// массив для рассчёта IPTD
uint32_t a_IPTD;// средний IPTD
uint32_t m_IPTD;// минимальный IPTD
uint32_t IPDV;
uint32_t dataCF[3];
uint32_t fails;
uint32_t success;
uint16_t d_fails;
uint16_t d_success;

uint32_t displayTimer = 0;
uint32_t displayTimer1 = 0;
uint32_t displayTimer2 = 0;
uint32_t mesh_timer; // таймер для check_connection
//переменные для шифрования
//длина блока должна быть 128 бит
uint8_t key[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};//ключ шифрования сообщений
uint8_t dataE[16];//блок зашифрованных данных
//uint8_t data1[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}; //блок данных для передачи
char route[16] = {'A', 'B', 'C', 'D', 'E'}; //блок данных для передачи

float zarad_akk = 0;
int low_Energy_NodeAdress = 0;

void setup() {
  //delay(2000);
  /*radio.begin();
    radio.setChannel(servise_channel);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_HIGH);
    radio.openReadingPipe(1, 0xAABBCCDD11LL);
    radio.startListening();*/
  Serial.begin(9600);
}


void loop() {
  if (p0 == 0) {
    /*if (radio.available()) {
      radio.read(&dataE, sizeof(dataE));
      aes128_dec_single(key_0, dataE);//дешифруем сообщение
      channel = dataE[0];
      radio.begin();
      radio.setPALevel(RF24_PA_MIN);
      mesh.setNodeID(nodeID);
      mesh.begin(channel, RF24_250KBPS, MESH_RENEWAL_TIMEOUT); // mesh.begin(channel, RF24_1MBPS, MESH_RENEWAL_TIMEOUT); The data rate (RF24_250KBPS,RF24_1MBPS,RF24_2MBPS) default:RF24_1MBPS
      mesh.update();
      if ( ! mesh.checkConnection() ) {
        mesh.renewAddress();
      }
      p0 = 1;
      Serial.print("запуск mesh на канале ");
      Serial.println(channel);
      }*/
    radio.begin();
    radio.setPALevel(RF24_PA_MIN);
    mesh.setNodeID(nodeID);
    mesh.begin(channel, RF24_250KBPS, MESH_RENEWAL_TIMEOUT); // mesh.begin(channel, RF24_1MBPS, MESH_RENEWAL_TIMEOUT); The data rate (RF24_250KBPS,RF24_1MBPS,RF24_2MBPS) default:RF24_1MBPS
    mesh.update();
    if ( ! mesh.checkConnection() ) {
      mesh.renewAddress();
    }
    p0 = 1;
    Serial.print("запуск mesh на канале ");
    Serial.println(channel);
  }
  else {

    mesh.update();
    network.failures(&fails, &success);
    
    if (network.available()) {
      RF24NetworkHeader header;
      network.peek(header);
      ID = mesh.getNodeID(header.from_node);

      //---------------Приём---------------------------------------------------

      if (header.type == 'K') {   //считывание таблицы сети
        network.read(header, &low_Energy_NodeAdress, sizeof(low_Energy_NodeAdress));
        mesh.renewAddress();
        Serial.println("get_comand_Refresh");
        Serial.println();
      }

      if (header.type == 'H') {   //считывание таблицы сети
        network.read(header, &netTable, sizeof(netTable));
        Serial.println("----------------");
        for (int i = 0; i < 10; i++) {
          Serial.print(netTable[i][0]);
          Serial.println(netTable[i][1]);
        }
      }

      if (header.type == 'S') { // рассчёт характеристик при получении тестового сообщения
        network.read(header, &dataR, sizeof(dataR));
        if (pdr < 10) {
          t[pdr] = millis();
        }
        pdr++;
      }

      else if (header.type == 'N') { //команда старта измерений
        network.read(header, &del, sizeof(del));
        pdr = 0;
        for (uint8_t i = 0; i < 10; i++) { // обнуляем массивы данных
          t[i] = 0;
        }
        for (uint8_t i = 0; i < 9; i++) {
          IPTD[i];
        }
        a_IPTD = 0;
        m_IPTD = 0;
        IPDV = 0;
        d_fails = fails;
        d_success = success;
        k = 0;
        p = 1;
      }
      else if (header.type == 'M') {
        network.read(header, &dataR, sizeof(dataR));
        if (pdr > 10) {
          pdr = 10;
        }
        for (uint8_t i = 1; i < 10; i++) { //бежим по точкам времени
          if ((t[i] != 0) && (t[i - 1] != 0)) {
            IPTD[i - 1] = t[i] - t[i - 1]; //-del
          }
        }
        for (uint8_t i = 0; i < 9; i++) {
          a_IPTD = ((a_IPTD + IPTD[i]) / (pdr - 1)); // средний IPTD
        }
        m_IPTD = a_IPTD;// принимаем за минимальный IPTD средний
        for (uint8_t i = 0; i < 9; i++) { // сравниваем средний IPTD с IPTD для iго узла
          if (IPTD[i] > 0) { // обязатльно учитываем только не нулевые значения
            m_IPTD = min(m_IPTD, IPTD[i]); // минимальный IPTD
          }
        }
        for (uint8_t i = 0; i < 9; i++) {
          if (IPTD[i] > 0) {
            IPDV = (IPDV + (IPTD[i] - m_IPTD)) / (pdr - 1); //средний IPDV(джиттер) - из каждого IPTD вычитаем минимальный IPTD
          }
        }
        dataCF[0] = pdr;
        dataCF[1] = a_IPTD; //IPTD
        dataCF[2] = IPDV; //IPDV

        if (!mesh.write(&dataCF, 'M', sizeof(dataCF))) {  //посылка на мастер данных эксперимента
          if ( ! mesh.checkConnection() ) {
            mesh.renewAddress();
          }
        }
      }
      /*else if (header.type == 'M') { // информация о смене канала
        //Serial.println(header.type);
        network.read(header, &channel, sizeof(channel));
        mesh.setChannel(channel);
        Serial.println(channel);
        }*/

    }
    //------------Передача------------------------------------------------
    if ((p == 1) && (millis() - displayTimer >  del)) { // отправка тестовых сообщений типа S
      displayTimer = millis();
      if (!mesh.write(&data, 'S', sizeof(data))) {
        if ( ! mesh.checkConnection() ) {
          mesh.renewAddress();
        }
      }
      else {
        k++;
      }
      if (k == 10) {
        if (!mesh.write(&data, 'N', sizeof(data))) {  //посылка на мастера об окончании отправки данных
          if ( ! mesh.checkConnection() ) {
            mesh.renewAddress();
          }
        }
        p = 0;
      }
    }

    if ((p == 0) && (millis() - displayTimer1 >  30000)) { // периодическая отправка шифрованных сообщений на рандомный узел для имитации нагрузки сети
      displayTimer1 = millis();

      n = 0;//на мастер
      if (netTable[n][1] != 0&&(n!=nodeID)) { //если этот узел активен и выбран не сам же отправитель, то посылаем сообщение
        //uint16_t batteryLevel = readBatteryMillivoltsLV4();
        int sensorValue = analogRead(A11);
        float voltage = sensorValue * (4.539 / 1024.0);
        
        Serial.println(voltage);
        
//        if (!mesh.write(&voltage, 'T', sizeof(voltage), n)) { // отправили зашифрованное сообщение
//          if ( ! mesh.checkConnection() ) {
//            mesh.renewAddress();
//          }
//        }
      }
      }
    if ((p == 0) && (millis() - mesh_timer > 10000)) { //каждые 10 секунд проверять подключение к сети, если не проходит тестирование
      mesh_timer = millis();
      if (!mesh.write(&dataE, 'С', sizeof(dataE))) {// отправили зашифрованное сообщение
        o++;
        Serial.println(o);
        if ( ! mesh.checkConnection() ) {
          mesh.renewAddress();
        }
      }
    }
    /* if (o >= 3) { //возврат в режим слушания служебной частоты
        o = 0;
        p0 = 0;
        Serial.println("return to radio");
        radio.begin();
        radio.setChannel(servise_channel);
        radio.setDataRate(RF24_1MBPS);
        radio.setPALevel(RF24_PA_HIGH);
        radio.openReadingPipe(1, 0xAABBCCDD11LL);
        radio.startListening();
      }
      if ((p == 0) && (millis() - displayTimer2 > 60000)) { // отправка информации о канале на служебной частоте
        displayTimer2 = millis();
        dataE[0] = channel; //информация о канале содержится в 0 позиции
        for (uint8_t i = 1; i < 16; i++) {
          dataE[i] = random(81); //заполняем остальные позиции случайными числами в пределах до 81 (как количество каналов)
        }
        aes128_enc_single(key_0, dataE);//шифруем с помощью key_0
        mesh.releaseAddress();
        radio.begin();
        radio.setChannel(servise_channel);
        radio.setDataRate(RF24_1MBPS);
        radio.setPALevel(RF24_PA_HIGH);
        radio.openWritingPipe(0xAABBCCDD11LL);
        radio.stopListening();
        radio.setAutoAck(true);
        radio.write(&dataE, sizeof(dataE));
        delay(200);
        radio.write(&dataE, sizeof(dataE));
        delay(200);
        radio.write(&dataE, sizeof(dataE));
        Serial.println("channel send succesfully");
        radio.begin();
        radio.setPALevel(RF24_PA_MIN);
        mesh.setChannel(channel);
        mesh.setNodeID(nodeID);
        mesh.begin(channel, RF24_250KBPS, 1000);
        if ( ! mesh.checkConnection() ) {
          if (!mesh.renewAddress()) {
            Serial.println("return to radio");
            radio.begin();
            radio.setChannel(servise_channel);
            radio.setDataRate(RF24_1MBPS);
            radio.setPALevel(RF24_PA_HIGH);
            radio.openReadingPipe(1, 0xAABBCCDD11LL);
            radio.startListening();
            p0 = 0;
          }
        }
      }*/
  }
}
