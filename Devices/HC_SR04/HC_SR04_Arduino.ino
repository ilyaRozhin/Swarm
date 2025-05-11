// Укажем, что к каким пинам подключено
int trigPin = 10; 
int echoPin = 11;  
 
void setup() { 
  Serial.begin(9600); 
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
} 
 
void loop() { 
  int duration, distance;
  if (Serial.available() > 0) {
    if (Serial.read() == 1) { // В режиме генерации сигнала
      // для большей точности установим значение LOW на пине Trig
      digitalWrite(trigPin, LOW); 
      delayMicroseconds(2); 
      // Теперь установим высокий уровень на пине Trig
      digitalWrite(trigPin, HIGH);
      // Подождем 10 μs 
      delayMicroseconds(10); 
      digitalWrite(trigPin, LOW); 
    } 
    else if (Serial.read() == 2) { // В режиме ожидания сгенерированного сигнала
      // Узнаем длительность высокого сигнала на пине Echo
      duration = pulseIn(echoPin, HIGH); 
      // Рассчитаем расстояние
      distance = duration / 29;
      Serial.write(distance);
    }
  }
  delay(100);
}