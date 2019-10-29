/*

Arduino PIN 12 (MAX485_WRITE_PIN) <===> MAX485 RE + MAX485 DE

Arduino PIN 10 (MAX485_TX_PIN) <===> MAX485 TX
Arduino PIN 11 (MAX485_RX_PIN) <===> MAX485 RX

Arduino PIN 5 (RED_PIN) <===> RED LED
Arduino PIN 6 (YELLOW_PIN) <===> YELLOW LED
Arduino PIN 7 (GREEN_PIN) <===> GREEN LED


Python code for testing : 

import random
from pymodbus.client.sync import ModbusSerialClient as ModbusClient

client = ModbusClient(method='rtu', port='/dev/ttyUSB0', timeout=1, baudrate=9600)
client.connect()
l1, l2, l3 = random.randint(0,1), random.randint(0,1), random.randint(0,1)
rq = client.write_registers(1, [l1,l2,l3], unit=0x01)
print("Set led 1 as %s, led 2 as %s, led 3 as %s" % (l1, l2, l3))
print("Error :(" if rq.isError() else "Success")

 */

#define SLAVE_ID 1

#define RED_PIN 5
#define YELLOW_PIN 6
#define GREEN_PIN 7

#define MAX485_TX_PIN 10
#define MAX485_RX_PIN 11

#define MAX485_WRITE_PIN 12

#define MAX485_BAUDRATE 9600

#define SERIAL_DEBUG

#define REQUEST_TIMEOUT 100

#define MEMORY_SIZE 3

#define REQUEST_MAX_SIZE 255


#include <SoftwareSerial.h>
#include <Crc16.h>


SoftwareSerial Max485Serial(MAX485_TX_PIN, MAX485_RX_PIN);

unsigned short memory[MEMORY_SIZE];


void setup() {
  memoryInit();
  ledsInit();
  pinMode(MAX485_WRITE_PIN, OUTPUT);
  digitalWrite(MAX485_WRITE_PIN, LOW);  
#ifdef SERIAL_DEBUG
  Serial.begin(57600);
  while (!Serial) { ; }
#endif
  Max485Serial.begin(MAX485_BAUDRATE);
}

void loop() { 
  Max485Serial.setTimeout(REQUEST_TIMEOUT);
  byte buf[REQUEST_MAX_SIZE];
  int len = Max485Serial.readBytes(buf, 255);
  if (len >= 3) {
    for(int i=len; i<REQUEST_MAX_SIZE;i++) buf[i] = '\0';
    parseRequest(buf, len);
  }
  ledsRefresh();
}


void memoryInit(){
  for(int i=0;i<MEMORY_SIZE;i++) {
    memory[i] = 0;
  }
}

void ledsInit() {
  pinMode(RED_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
}


void ledsRefresh() {
  digitalWrite(RED_PIN, memory[0] > 0 ? HIGH : LOW);  
  digitalWrite(YELLOW_PIN, memory[1] > 0 ? HIGH : LOW);  
  digitalWrite(GREEN_PIN, memory[2] > 0 ? HIGH : LOW);  
}

byte getSlaveId() {
  return SLAVE_ID;
}


void parseRequest(byte * data, int len) {
  Crc16 crc;
  
#ifdef SERIAL_DEBUG
  Serial.print("Parsing request with len "); Serial.println(len);
#endif

  byte slaveId = data[0];
  unsigned short slaveCrc = (data[len-1] << 8) + data[len-2];
  crc.clearCrc();
  unsigned short calculatedCrc = crc.Modbus(data, 0, len - 2);
  byte fcode = data[1];
  if (slaveId != getSlaveId()) {
#ifdef SERIAL_DEBUG
    Serial.println("Bad slave id !");
#endif
    return;
  }
  if (calculatedCrc != slaveCrc) {
#ifdef SERIAL_DEBUG
    Serial.println("Bad CRC !");
#endif
    return;
  }
  byte requestData[REQUEST_MAX_SIZE - 4];
  for(int i = 0;i<REQUEST_MAX_SIZE - 4;i++) {
    requestData[i] = i < len - 4 ? data[i + 2] : '\0';
  }
  handleRequest(fcode, requestData, len - 4);
}


void handleRequest(byte fcode, byte * data, int len)
{
#ifdef SERIAL_DEBUG
  Serial.print("Handle request with function code "); Serial.println(fcode); 
#endif
  if (fcode == 1) {
    handleReadCoilStatus(data, len);
  } else if (fcode == 2) {
    handleReadInputStatus(data, len);
  } else if (fcode == 3) {
    handleReadHoldingRegisters(data, len);
  } else if (fcode == 4) {
    handleReadInputRegisters(data, len);
  } else if (fcode == 5) {
    handleWriteSingleCoil(data, len);
  } else if (fcode == 6) {
    handleWriteSingleRegister(data, len);
  } else if (fcode == 15) {
    handleWriteMultipleCoil(data, len);
  } else if (fcode == 16) {
    handleWriteMultipleRegister(data, len);
  } else {
    handleUnknownFunctionCode(data, len, fcode);
  }
}


void send(byte * data, int len, bool addCrc = true) {
  Crc16 crc; 
  if (addCrc) {
    crc.clearCrc();
    unsigned short calculatedCrc = crc.Modbus(data, 0, len);
    data[len] = lowByte(calculatedCrc);
    data[len + 1] = highByte(calculatedCrc);
    len = len + 2;
  }
  digitalWrite(MAX485_WRITE_PIN, HIGH); 
  for(int i=0;i<len;i++) {
    Max485Serial.write((byte) data[i]);
  }
  digitalWrite(MAX485_WRITE_PIN, LOW); 
}


void handleUnknownFunctionCode(byte * data, int len, byte fcode) { // Unknown FC
  bitWrite(fcode, 7, 1);
  byte response[REQUEST_MAX_SIZE] = { getSlaveId(), fcode, 1 };
  send(response, 3, true);
}


void handleReadCoilStatus(byte * data, int len) { // FC01: see http://www.simplymodbus.ca/FC01.htm
  byte response[REQUEST_MAX_SIZE] = { getSlaveId(), 129, 1 };
  send(response, 3, true);
}


void handleReadInputStatus(byte * data, int len) { // FC02: see http://www.simplymodbus.ca/FC02.htm
  byte response[REQUEST_MAX_SIZE] = { getSlaveId(), 130, 1 };
  send(response, 3, true);
}


void handleReadHoldingRegisters(byte * data, int len) { // FC03: see http://www.simplymodbus.ca/FC03.htm
  short address = (data[0] << 8) + data[1];
  short quantity = (data[2] << 8) + data[3];
  short value = 0;
  
  if (quantity > 100) { 
    quantity = 100;
  }
  
  byte response[REQUEST_MAX_SIZE] = { getSlaveId(), 03, (byte) (quantity * 2) };
  
  for(int i=0;i<quantity;i++) {
    if (!isValidMemoryAddress(address + i)) {
#ifdef SERIAL_DEBUG
    Serial.print("Bad memory address "); Serial.println(address + (i / 2));
#endif
      byte response[REQUEST_MAX_SIZE] = { getSlaveId(), 131, 2 };
      send(response, 3, true);
      return;
    }
    value = readMemory(address + i);
    response[3 + (i * 2)] = highByte(value);
    response[4 + (i * 2)] = lowByte(value);
  }
  send(response, 3 + (quantity * 2), true);
}


void handleReadInputRegisters(byte * data, int len) { // FC04: see http://www.simplymodbus.ca/FC04.htm
  byte response[REQUEST_MAX_SIZE] = { getSlaveId(), 132, 1 };
  send(response, 3, true);
}


void handleWriteSingleCoil(byte * data, int len) { // FC05: see http://www.simplymodbus.ca/FC05.htm
  byte response[REQUEST_MAX_SIZE] = { getSlaveId(), 133, 1 };
  send(response, 3, true);
}


void handleWriteSingleRegister(byte * data, int len) { // FC06: see http://www.simplymodbus.ca/FC06.htm
  short address = (data[0] << 8) + data[1];
  short value = (data[2] << 8) + data[3];
  if (!writeMemory(address, value)) {
#ifdef SERIAL_DEBUG
    Serial.print("Bad memory address "); Serial.println(address);
#endif
    byte response[REQUEST_MAX_SIZE] = { getSlaveId(), 134, 2 };
    send(response, 3, true);
    return;
  }
  byte response[REQUEST_MAX_SIZE] = { getSlaveId(), 6, data[0], data[1], data[2], data[3] };
  send(response, 6, true);
}


void handleWriteMultipleCoil(byte * data, int len) { // FC15: see http://www.simplymodbus.ca/FC15.htm
  byte response[REQUEST_MAX_SIZE] = { getSlaveId(), 143, 1 };
  send(response, 3, true);
}


void handleWriteMultipleRegister(byte * data, int len) { // FC16: see http://www.simplymodbus.ca/FC16.htm
  short address = (data[0] << 8) + data[1];
  short quantity = (data[2] << 8) + data[3];
  byte bytesCount = data[4];
  for(int i=0;i<bytesCount;i=i+2) {
    if (!writeMemory(address + (i / 2), (data[5 + i] << 8) + data[6 + i])) {
#ifdef SERIAL_DEBUG
    Serial.print("Bad memory address "); Serial.println(address + (i / 2));
#endif
      byte response[REQUEST_MAX_SIZE] = { getSlaveId(), 144, 2 };
      send(response, 3, true);
      return;
    }
  }
  byte response[REQUEST_MAX_SIZE] = { getSlaveId(), 16, data[0], data[1], data[2], data[3] };
  send(response, 6, true);
}


bool isValidMemoryAddress(short address) {
  return address >= 0 and address < MEMORY_SIZE;
}

bool writeMemory(short address, short value) {
  if (isValidMemoryAddress(address)) {
#ifdef SERIAL_DEBUG
    Serial.print("Write in memory address "); Serial.print(address); Serial.print(" value "); Serial.println(value);
#endif
    memory[address] = value;
    return true;
  } else return false;
}


unsigned short readMemory(short address) {
  if (isValidMemoryAddress(address)) {
    return memory[address];
  } else return 0;
}

