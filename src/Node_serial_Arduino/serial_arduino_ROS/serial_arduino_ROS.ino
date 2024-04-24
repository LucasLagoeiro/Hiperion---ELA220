//#include "ESP32Encoder.h"
#include <ArduinoJson.h>
#include "Pins.h"

#define BUFFER_SIZE_RECEIVER 300
#define END_OF_JSON_CHAR '}'

#define ENCODER_PERIOD 5000




char buffer[BUFFER_SIZE_RECEIVER]; // Buffer para armazenar a mensagem
int buffer_index = 0; // Indicador de posição no buffer

long encoder_timer = 0;

int velMotors[2] = {0,0};  // {Right , Left}


//ESP32Encoder Encoder1;
//ESP32Encoder Encoder2;

//Usado para as velocidades dos motores
struct Vel {
  struct Linear {
    float x;
    float y;
    float z;
  } linear;
  struct Angular {
    float x;
    float y;
    float z;
  } angular;
};

// Cria uma variável do tipo Vel
Vel vel;

// Esta função será chamada sempre que dados estiverem disponíveis na porta serial
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    buffer[buffer_index] = inChar;
    buffer_index++;
    // Verifica se a mensagem está completa
    if (inChar == END_OF_JSON_CHAR) {
      buffer[buffer_index] = '\0'; // Adiciona o caractere de terminação nulo para tornar o buffer uma string válida
      processMessage();
      buffer_index = 0; // Reseta o buffer
    }
  }
}

// Esta função deserializa a mensagem JSON e processa os dados
void processMessage() {
  StaticJsonDocument<BUFFER_SIZE_RECEIVER> doc_subscription_cmd_vel;
  DeserializationError error = deserializeJson(doc_subscription_cmd_vel, buffer);
  if (error) {

  } else {
    if (doc_subscription_cmd_vel.containsKey("linear_x") && doc_subscription_cmd_vel.containsKey("angular_z")){
      float linear_x = doc_subscription_cmd_vel["linear_x"];
      float angular_z = doc_subscription_cmd_vel["angular_z"];

      if(linear_x < 20.0 && linear_x > -20.0 && angular_z < 20.0 && angular_z > -20.0) {
        // Define os valores
        vel.linear.x = linear_x;
//        Serial.println(vel.linear.x);
        vel.linear.y = 0.0;
        vel.linear.z = 0.0;
        vel.angular.x = 0.0;
        vel.angular.y = 0.0;
        vel.angular.z = angular_z;

        
        
      }
    }
  }
}

void clearEncoders() {
  // clear the encoder's raw count and set the tracked count to zero
//  Encoder1.clearCount();
//  Encoder2.clearCount();
}

void setup() {
  Serial.begin(115200);
  definePins();
  

  // use pin 19 and 18 for the first encoder
//  Encoder1.attachHalfQuad(19, 18);
//  // use pin 17 and 16 for the second encoder
//  Encoder2.attachHalfQuad(17, 16);

  clearEncoders();

    // Define os valores
  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;

}

void loop() {

   // Verifica se existem novos dados na porta serial
  if (Serial.available() > 0) {
      serialEvent();
  }

  if(millis() - encoder_timer >=  ENCODER_PERIOD){
    encoder_timer = millis();
    // put your main code here, to run repeatedly:
    // Serial.print("Encoder 01: ");
    // Serial.println(Encoder1.getCount());
    // Serial.print("Encoder 02: ");
    // Serial.println(Encoder2.getCount());

    StaticJsonDocument<300> doc; // Cria um documento JSON

      // --- Leitura dos Encoders------------------------------------------------------
    JsonArray array_encoders = doc.createNestedArray("encoders"); // Cria um array JSON
    // Adiciona valores ao array
//    array_encoders.add( Encoder1.getCount());
//    array_encoders.add( Encoder2.getCount());
    //-------------------------------------------------------------------------------
    
//
    array_encoders.add(100*vel.linear.x);
    array_encoders.add(100*vel.angular.z);

    
  
//    diffRobot(vel.linear.x,vel.angular.z,velMotors);
    if(vel.linear.x > 0.4){
      goFoward();


    }
    else if(vel.linear.x < 0){
      goBackward();
    }

    else if(vel.angular.z < 0){
      goRight();
    }
    else if(vel.angular.z > 0.9){
      goLeft();
    }
    else{
      stopRobot();
    }

      // Serializa e envia o documento JSON
    serializeJson(doc, Serial);
    Serial.println(); // Adiciona uma nova linha
    
    
  }

}
