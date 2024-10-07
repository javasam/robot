#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
  while (!Serial);  // Ожидаем подключения к Serial Monitor
  Serial.println("\nI2C Scanner");
}

void loop() {
  byte error, address;
  int nDevices = 0;
  
  Serial.println("Scanning...");

  // Проходим по всем возможным адресам (от 1 до 127)
  for (address = 1; address < 127; address++) {
    // Проверяем, есть ли устройство по этому адресу
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C устройство найдено по адресу 0x");
      if (address < 16) 
        Serial.print("0");  // Для красоты выводим ведущий 0
      Serial.println(address, HEX);

      nDevices++;
    } 
  }

  if (nDevices == 0) {
    Serial.println("Нет I2C устройств найдено\n");
  } else {
    Serial.println("Сканирование завершено\n");
  }

  delay(5000);  // Повторяем сканирование каждые 5 секунд
}
