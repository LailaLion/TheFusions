// Definições dos pinos dos sensores
const int sensorPins[5] = {A0, A1, A2, A3, A4};
int sensorValues[5];

// Definições dos pinos do DRV8833
const int motorA1 = 3;
const int motorA2 = 5;
const int motorB1 = 6;
const int motorB2 = 9;


// Parâmetros PID
float Kp = 0.5, Ki = 0.0, Kd = 0.1;
float integral = 0, lastError = 0;

// Função para ler os valores dos sensores
void readSensors() {
  for (int i = 0; i < 5; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
}

// Função para calcular o erro baseado nos sensores
int calculateError() {
  int weightedSum = 0;
  int totalWeight = 0;
  for (int i = 0; i < 5; i++) {
     int weight = (i - 2) * 1000; // Peso baseado na posição do sensor
     weightedSum += sensorValues[i] * weight;
     totalWeight += sensorValues[i];
  }
  if (totalWeight == 0) return 0; // Nenhum sensor detectando a linha
  return weightedSum / totalWeight;
}

// Função PID
int pidControl(int error) {
  integral += error;
  float derivative = error - lastError;
  int output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  return output;
}

// Função para controlar os motores
void setMotorSpeeds(int speedA, int speedB) {
    analogWrite(motorA1, speedA);
    analogWrite(motorA2, 0);
    analogWrite(motorA1, 0);
    analogWrite(motorA2, speedB);
}

void setup() {
  // Inicializa os pinos dos sensores
  for (int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  // Inicializa os pinos do DRV8833
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  // Inicializa a comunicação serial
  Serial.begin(9600);
}

void loop() {
  readSensors();
  int error = calculateError();
  int correction = pidControl(error);
  int baseSpeed = 150;
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;
  setMotorSpeeds(leftSpeed, rightSpeed);
  delay(10); // Aguarda um pouco antes de ler novamente
}
