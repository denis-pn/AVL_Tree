/*
   Мастер создаёт сетевую таблицу с номерами узлов и их состоянием.
   nodeID записывается на позицию соответсвующую его номеру с 0 по 9 (можно расширить).
   Во втором столбце таблицы указывается активность: 0 - узел не активен, 1 - активен.
   Если в обоих столбцах 0, значит такого узла в сети ещё не было.
   Если произошла смена мастера, то новый мастер ставит на свою бывшую позицию в первом столбце 0.
   При этом во втором столбце остаётся 1, это позволяет не терять информацю.

   Префиксы сообщений:
   H - сетевая таблица состояния
   V - служебная информация (номер канала для смены)
   T - Информация
   S - тестовые для подсчёта метрик
   N - старт измерений
   M - запрос данных эксперимента
   С - для проверки связи
   Максимальная длина сообщения 120 байт (uint8_t до 255)

   Частоты: 2425, 2450, 2475, 2478 и 2481 МГц, мощность не более 100 мВт
*/

#include <AESLib.h>
#include <Arduino.h>
//#include <unistd.h>
#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <EEPROM.h>
#include "nRF24L01.h"

RF24 radio(1, 0);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

// переменные для функции выбора канала
uint8_t servise_channel = 70;// канал для радио
uint8_t values[83];//массив для функции сканера
uint8_t key_0[] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}; //ключ для шифрования частоты

//общие переменные
uint8_t nodeID = 0;
uint8_t static_IP[4] = {192, 168, 10, nodeID}; //последний октет совпадает с nodeID присвоенным изначально
uint8_t channel = 81; // несущая для сети по умолчанию 81
uint8_t netTable[10][2]; // сетевая таблица для учёта смены узла
uint32_t fails;
uint32_t success;
uint16_t d_fails;
uint16_t d_success;

//переменные при работе в качестве мастера
uint8_t nodeAmount = 6; // общее количество активных узлов в сети включая мастера
uint16_t failID = 0;
uint32_t del = 200;//время задержки отправки тестовых сообщений
uint32_t dataTable[10][6];// [0] - pdr up, [1] - pdr down, iptd up, iptd down, ipdv up, ipdv down
uint32_t dataCF[3]; //данные эксперимента от узлов
uint8_t n; //nodeID для передачи
uint8_t n1; //количество активных узлов в сети
char symb;
uint8_t data[] = {255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0};
uint8_t dataR[32];
uint8_t k; // счётчик количества посылаемых тестовых сообщений (в программе 10)
uint8_t ID; //nodeID на приём
uint32_t pdr;
uint32_t t[10]; // массив времён передачи для рассчёта IPTD
uint32_t IPTD[9];// массив для рассчёта IPTD
uint32_t a_IPTD;// средний IPTD
uint32_t m_IPTD;// минимальный IPTD
uint32_t IPDV;

bool p0 = 0; // переключатель radio/mesh
bool p1 = 0;//переклюатель на передачу тестовых сообщений S типа
bool p2; // активатор тестирования pdr iptd ipdv
bool p3; //переключатель для активации таймаута
bool p4 = 1; // активатор функции пингования
bool xn = false; //сигнал об окончании передачи узлом
bool x0 = false; //сигнал об окончании передачи мастером

// различные переменные
uint32_t displayTimer0 = 0;// таймер пингования (master)/отправки сообщений(node)
uint32_t displayTimer1 = 0; //время задержки отправки сообщений S типа
uint32_t displayTimer2 = 0; //таймаут при тестировании узлов (master)
uint32_t displayTimer3 = 0; // для отправки по radio текущего канала (node)
uint32_t displayTimer4 = 0; //таймер проверки наличия хотя бы одного активного узла в сети
//переменные для шифрования
//длина блока должна быть 128 бит, размер символа 8 бит
char route[16];
uint8_t key[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
uint8_t dataE[16];

float voltage = 0;
uint16_t batteryLevel;

int low_Energy_NodeID = 0;
int low_Energy_NodeAdress = 0;

void setup() {
  delay(5000);
  Serial.begin(9600);
  Serial.print(static_IP[0]); Serial.print("."); Serial.print(static_IP[1]); Serial.print("."); Serial.print(static_IP[2]); Serial.print("."); Serial.println(static_IP[3]);
}
//-------------функция пингования--------------------------------------------
void pingNode(uint8_t listNo) {
  RF24NetworkHeader headers(mesh.addrList[listNo].address, NETWORK_PING);
  uint32_t pingtime = millis();
  bool ok;
  if (headers.to_node) {
    ok = network.write(headers, 0, 0);
    if (ok && failID == mesh.addrList[listNo].nodeID) {
      failID = 0;
    }
    if (!ok) {
      failID = mesh.addrList[listNo].nodeID;
    }
  }
  pingtime = millis() - pingtime;
  Serial.print(" Time: "); Serial.print(pingtime); Serial.print(" ms; ");
  netTable[0][1] = 1;//мастер подразумеваем включённым
  if (ok || !headers.to_node) {
    Serial.println("status ONLINE");
    netTable[mesh.addrList[listNo].nodeID][0] = mesh.addrList[listNo].nodeID;
    netTable[mesh.addrList[listNo].nodeID][1] = 1;
  } else {
    Serial.println("status OFFLINE");
    // netTable[mesh.addrList[listNo].nodeID][0] = 0;// если узел отключен замени в таблице на 0
    netTable[mesh.addrList[listNo].nodeID][1] = 0;
  }
}
//-------------------------------------------------------------------------------------------
//------------------Функция сканирования--------------------------------------------------
/*uint8_t scanner() {
  uint8_t k = 0;// счётчик количества сканирований диапазона
  uint8_t chan = 0;
  for (uint8_t i; i < 83; i++) {
    values[i] = 0;
  }
  Serial.println("Запуск сканирования диапазона");
  radio.begin();
  while (k <= 100) { // сканирование диапазона
    radio.setAutoAck(false);
    for (int i = 0; i <= 82; i++) {
      radio.setChannel(i);
      radio.startListening();
      delayMicroseconds(200);
      radio.stopListening();
      if ( radio.testCarrier() ) {
        values[i]++;//если частота занята пишем
      }
    }
    k++;
    if (k == 20) {
      Serial.print(k); Serial.println("%");
    }
    else if (k == 40) {
      Serial.print(k); Serial.println("%");
    }
    else if (k == 60) {
      Serial.print(k); Serial.println("%");
    }
    else if (k == 80) {
      Serial.print(k); Serial.println("%");
    }
    else if (k == 100) {
      Serial.print(k); Serial.println("%");
    }
  }
  // каналы 2424-2426, 2449-2451, 2474-2483 МГц, 2477-2479 МГц, 2480-2482 МГц.
  // частоты: 2425, 2450, 2475, 2478 и 2481 МГц. Максимальная частота ограничена согласно рекомендации ГКРЧ
  if ((values[80] == 0) && (values[81] == 0) && (values[82] == 0)) {
    Serial.println("Установлен канал 2481 МГц");
    chan = 81;
    return chan;
  }

  else if ((values[77] == 0) && (values[78] == 0) && (values[79] == 0)) {
    Serial.println("Установлен канал 2478 МГц");
    chan = 78;
    return chan;
  }

  else if ((values[74] == 0) && (values[75] == 0) && (values[76] == 0)) {
    Serial.println("Установлен канал 2475 МГц");
    chan = 75;
    return chan;
  }

  else if ((values[49] == 0) && (values[50] == 0) && (values[51] == 0)) {
    Serial.println("Установлен канал 2450 МГц");
    chan = 50;
    return chan;
  }

  else if ((values[24] == 0) && (values[25] == 0) && (values[26] == 0)) {
    Serial.println("Установлен канал 2426 МГц");
    chan = 26;
    return chan;
  }

  else {
    Serial.println("Нет свободных каналов, выбран канал по умолчанию 81");
    chan = 81
    return chan;
  }
  }*/
//-------------------------------------------------------------------------------------------------------
void loop() {
  if (p0 == 0) {
    /*channel = scanner();// запуск функции сканера частотного диапазона
      Serial.print("channel "); Serial.println(channel);
      dataE[0] = channel; //информация о канале содержится в 0 позиции
      for (uint8_t i = 1; i < 16; i++) {
      dataE[i] = random(81); //заполняем остальные позиции случайными числами в пределах до 81 (как количество каналов)
      }
      aes128_enc_single(key_0, dataE);//шифруем с помощью key_0
      radio.setChannel(servise_channel);
      radio.setDataRate(RF24_1MBPS);
      radio.setPALevel(RF24_PA_HIGH);
      radio.openWritingPipe(0xAABBCCDD11LL);
      radio.stopListening();
      radio.setAutoAck(true);
      radio.write(&dataE, sizeof(dataE));
      delay(400);
      radio.write(&dataE, sizeof(dataE));
      delay(400);
      radio.write(&dataE, sizeof(dataE));
      delay(400);
      Serial.println("Несущая отправлена");*/

    //------запускаем MESH----------//----------------------------
    radio.begin();
    radio.setPALevel(RF24_PA_MIN);
    mesh.setNodeID(0);
    mesh.begin(channel, RF24_250KBPS, MESH_RENEWAL_TIMEOUT);
    mesh.update();
    mesh.DHCP();
    Serial.println("канал установлен, mesh запущен");
    p0 = 1;
  }
  //-----------------------------------Работа mesh---------------------------------------------
  else {
    mesh.update();
    mesh.DHCP();
    //-----------------------------Чтение команд с клавиатуры------------------------------------------------
    if (Serial.available()) {
      symb = Serial.read();
      if (symb == 't') { // сигнал о начале тестирования
        p4 = 0;// отключаем функцию пингования
        p2 = 1;// включаем функцию отслеживания
        n = 1; // переключатель номера запрашиваемого узла (начинаем с 1)
        x0 = false; // сигнал об окончании передачи со стороны мастера
        xn = false; // сигнал об окончании передачи со стороны узла
        for (uint8_t i = 0; i < 6; i++) {
          for (uint8_t j = 0; j < nodeAmount; j++) { // очистка таблицы данных
            dataTable[i][j] = 0;
          }
        }
        Serial.println("Tests start");
      }
    }
    //----------------------------------------------------------------------------------------------------------
    //-------------------------Рассчёт характеристик PDR IPTD IPDV ---------------------------------------------
    if ((p1 == 0) && (p2 == 1)) {    // функция приёма/передачи данных для pdr itd ipdv
      if (netTable[n][1] == 1) {    //если узел активен
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
        x0 = 0;
        xn = 0;
        if (!mesh.write(&del, 'N', sizeof(del), n)) {
          mesh.write(&del, 'N', sizeof(del), n);
        }
        d_fails = fails;
        d_success = success;
        k = 0; // счётчик количества отправленных сообщений
        p1 = 1; //разрешаем передачу тестовых сообщений
        p2 = 0; // переключатель
        Serial.print("Test node ");
        Serial.print(n);
        Serial.println(" has started");
        displayTimer1 = millis();
        displayTimer2 = millis();
        p3 = 1;// включаем таймаут
      }
      else { // если узел не активен переключись на следующий
        n++;
        if (n > nodeAmount) { // если прошли по всем узлам - закончить выполнение расчётов
          n = 0;
          p1 = 0;
          p2 = 0;
          p3 = 0;
          x0 = false;
          xn = false;
          Serial.println();
          for (int i = 1; i < nodeAmount; i++) { // показ составленной таблицы данных
            Serial.print("NodeID "); Serial.print(i); Serial.print(" PDR down: "); Serial.print(dataTable[0][i]); Serial.print(" IPTD down: "); Serial.print(dataTable[1][i]); Serial.print(" IPDV down: "); Serial.print(dataTable[2][i]);
            Serial.print(" PDR up: "); Serial.print(dataTable[3][i]); Serial.print(" IPTD up: "); Serial.print(dataTable[4][i]); Serial.print(" IPDV up: "); Serial.println(dataTable[5][i]);
            p4 = 1; // включаем функцю пингования
          }

        }
      }
    }

    if ((xn == true) && (x0 == true)) { //для того чтобы начать рассчёт характеристик проверка окончания отправки тестовых сообщений на мастере и узле
      if (pdr > 10) {
        pdr = 10;
      }
      for (uint8_t i = 1; i < 10; i++) { //бежим по точкам времени
        if (t[i] != 0) {
          IPTD[i - 1] = t[i] - t[i - 1]; // -del
        }
      }
      for (uint8_t i = 0; i < 9; i++) {
        a_IPTD = a_IPTD + IPTD[i]; // средний IPTD
      }
      a_IPTD = a_IPTD / (pdr - 1);
      m_IPTD = a_IPTD;// принимаем за минимальный IPTD средний
      for (uint8_t i = 0; i < 9; i++) { // сравниваем средний IPTD с IPTD для iго узла
        if (IPTD[i] > 0) { // обязатльно учитываем только не нулевые значения
          m_IPTD = min(m_IPTD, IPTD[i]); // минимальный IPTD
        }
      }
      for (uint8_t i = 0; i < 9; i++) {
        if (IPTD[i] > 0) {
          IPDV = IPDV + (IPTD[i] - m_IPTD); //средний IPDV(джиттер) - из каждого IPTD вычитаем минимальный IPTD
        }
      }
      IPDV = IPDV / (pdr - 1);
      dataTable[0][n] = pdr;
      dataTable[1][n] = a_IPTD;
      dataTable[2][n] = IPDV;

      if (!mesh.write(&del, 'M', sizeof(del), n)) { //отправляем на узел запрос о данных
        mesh.write(&del, 'M', sizeof(del), n);
      }
      x0 = false;
      xn = false;
    }

    if (p3 == 1 && (millis() - displayTimer2 > 20000)) { // таймаут функции тестирования
      p3 = 0;
      x0 = true;
      xn = true;
      Serial.println("Timeout");
    }

    if ((p1 == 1) && (millis() - displayTimer1 > del)) { // отправка тестовых сообщений S типа
      displayTimer1 = millis();
      if (!mesh.write(&data, 'S', sizeof(data), n)) {
        mesh.write(&data, 'S', sizeof(data), n);
      }
      else {
        k++;
      }
      if (k == 10) {
        p1 = 0;
        x0 = true;
      }
    }
    //---------------------------------------------------------------------------------------------
    //-------------------Передача сообщений--------------------------------------------------------
    /*if ((p1 == 0) && (millis() - displayTimer1 >  1000)) { // периодическая отправка шифрованных сообщений на рандомный узел
      displayTimer1 = millis();
      n = random(9);//выбираем рандомный узел
      if ((netTable[n][1] != 0) && (n != nodeID)) { //если этот узел активен и выбран не сам же отправитель, то посылаем сообщение
        for (uint8_t i = 0; i < 16; i++) { //переписали данные в массив для зашифровки
          dataE[i] = route[i];
        }
        aes128_enc_single(key, dataE); // зашифровали
        if (!mesh.write(&dataE, 'T', sizeof(dataE), n)) { // отправили зашифрованное сообщение
          if ( ! mesh.checkConnection() ) {
            mesh.renewAddress();
          }
        }
      }
    }*/
    //------отправка информации о смене несущей на узлы-------------------
    /* if (millis() - displayTimer1 > 40000) {
       displayTimer1 = millis();
       channel = 80;
       for (int i = 0; i < 10; i++) {
         if (netTable[i][1] == 1 && netTable[i][0] != 0) {
           mesh.write(&channel, 'M', sizeof(channel), netTable[i][0]);
         }
       }
       mesh.setChannel(channel);
       Serial.println("смена канала");
      }*/
    //---------------отправка несущей на служебной частоте---------------------------------------------------------
    /*if (millis() - displayTimer4 > 60000) { //отправка несущей на служебной частоте в случае если отвалились/не подключены ноды
      n1 = 0;
      displayTimer4 = millis();
      for (uint8_t i = 1; i < nodeAmount; i++) {
        if (netTable[i][1] != 0) {
          n1++;
        }
      }
      if (n1 == 0) {
        dataE[0] = channel; //информация о канале содержится в 0 позиции
        for (uint8_t i = 1; i < 16; i++) {
          dataE[i] = random(81); //заполняем остальные позиции случайными числами в пределах до 81 (как количество каналов)
        }
        aes128_enc_single(key_0, dataE);//шифруем с помощью key_0
        radio.begin();
        radio.setChannel(servise_channel);
        radio.setDataRate(RF24_1MBPS);
        radio.setPALevel(RF24_PA_HIGH);
        radio.openWritingPipe(0xAABBCCDD11LL);
        radio.stopListening();
        radio.setAutoAck(true);
        radio.write(&dataE, sizeof(dataE));
        delay(400);
        radio.write(&dataE, sizeof(dataE));
        delay(400);
        radio.write(&dataE, sizeof(dataE));
        delay(400);
        Serial.println("Несущая отправлена");
        radio.begin();
        radio.setPALevel(RF24_PA_MIN);
        mesh.setNodeID(0);
        mesh.begin(channel, RF24_250KBPS, MESH_RENEWAL_TIMEOUT);
        mesh.update();
        mesh.DHCP();
        Serial.println("канал установлен, mesh запущен снова");
      }
      }*/


    //-----------------------------------------------------------------------------------------------
    //----------------------Приём сообщений----------------------------------------------------------
    if (network.available()) {
      RF24NetworkHeader header;
      network.peek(header);
      ID = mesh.getNodeID(header.from_node);

      if (header.type == 'S') { // рассчёт характеристик при получении тестового сообщения
        network.read(header, &dataR, sizeof(dataR));
        if (pdr < 10) {
          t[pdr] = millis();
        }
        pdr++;
      }
      else if (header.type == 'N') { // сообщение об окончании отправки тестовых сообщений узлом
        network.read(header, &dataR, sizeof(dataR));
        xn = true;
        Serial.print("node "); Serial.print(ID); Serial.println(" has finished test");
      }
      else if (header.type == 'M') { // запись характеристик pdr iptd ipdv от узлов в таблицу
        network.read(header, &dataCF, sizeof(dataCF));
        dataTable[3][ID] = dataCF[0];
        dataTable[4][ID] = dataCF[1];
        dataTable[5][ID] = dataCF[2];
        n++;
        p2 = 1;
        if (n > nodeAmount) { //когда пробежали по всей netTable и получили послений ответ заканчваем процедуру
          n = 0;
          p2 = 0;
          p3 = 0;
        }
        else {
          p2 = 1;
          p3 = 0;
        }
      }
      else if (header.type == 'T') {
        network.read(header, &voltage, sizeof(voltage)); //приняли зашифрованное сообщение

        Serial.print("From NodeID "); Serial.println(ID);
        Serial.print("Decripted message: ");

        Serial.print(voltage); Serial.print(" ");
        Serial.println();

        if (voltage < 2.6){
          Serial.print("LowEnergy NodeID "); Serial.println(ID);
          low_Energy_NodeID = int(ID);
          low_Energy_NodeAdress = 0;
        }
        
      }
      else if (header.type == 'C') {
        network.read(header, &dataE, sizeof(dataE)); //приняли зашифрованное сообщение для проверки связи
      }
    }
    //-----------------------------------------------------------


    //-----------вывод состояния сети---------------------------------
    if ((millis() - displayTimer0 > 5000) && (p4 == 1)) {
      mesh.update();
      mesh.DHCP();
      displayTimer0 = millis();
      Serial.println(" ");
      Serial.println(F("********Assigned Addresses********"));
      for (int i = 0; i < mesh.addrListTop; i++) {
        Serial.print("NodeID: ");
        Serial.print(mesh.addrList[i].nodeID);
        Serial.print("; ");
        Serial.print("RF24Network Address: ");
        Serial.print(mesh.addrList[i].address, OCT);
        Serial.print("0; ");

        pingNode(i);

        if ((low_Energy_NodeID != 0)  && (low_Energy_NodeAdress == 0)){

          String my_str_Address = String(mesh.addrList[i].address, OCT);
          low_Energy_NodeAdress = my_str_Address[0];
          Serial.println();
          Serial.println(F("**********************************"));
          Serial.print("LowEnergyNode Address [0] ");
          Serial.println(low_Energy_NodeAdress);
          Serial.println(F("**********************************"));
          Serial.println();
        }
        
        if ((low_Energy_NodeID != 0)  && (low_Energy_NodeAdress != 0)){

          int index = String(mesh.addrList[i].address, OCT).indexOf(String(low_Energy_NodeAdress)); // поиск символа 'o' в строке
          
          if (index != -1) {
                Serial.println();
                Serial.println(F("**********************************"));
                Serial.print("Node");
                Serial.print(mesh.addrList[i].nodeID);
                Serial.println(" send Refresh");
                Serial.println(F("**********************************"));
                Serial.println();

                mesh.write(&low_Energy_NodeAdress, 'K', sizeof(low_Energy_NodeAdress), mesh.addrList[i].nodeID);
          }
        }
      }
      for (int i = 0; i < nodeAmount; i++) { // показ составленной сетевой таблицы
        Serial.print(netTable[i][0]);
        Serial.println(netTable[i][1]);
      }
      Serial.println(F("**********************************"));
      for (int i = 0; i < nodeAmount; i++) { // отправка таблицы на узлы
        if (netTable[i][1] == 1 && netTable[i][0] != 0) {
          mesh.write(&netTable, 'H', sizeof(netTable), netTable[i][0]);
        }
      }
    }
    //-----------------------//-------------------------------------------
  }
}
