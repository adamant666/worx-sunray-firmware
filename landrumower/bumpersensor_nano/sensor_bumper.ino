

#include <SoftwareSerial.h>

#define SENSOR1_RX 8  
#define SENSOR1_TX 9  
#define SENSOR2_RX 10 
#define SENSOR2_TX 11 

#define BUMPER_X 2    
#define BUMPER_Y 3    

#define DISTANCE_THRESHOLD 30
#define MEASUREMENT_INTERVAL 200  // 5 mérés másodpercenként (1000ms/5 = 200ms)
#define SENSOR_WAIT_TIME 100     // 100ms várakozás a megbízható mérésért (bőven több mint a minimum 60ms)

SoftwareSerial sensor1Serial(SENSOR1_RX, SENSOR1_TX);
SoftwareSerial sensor2Serial(SENSOR2_RX, SENSOR2_TX);

const byte MEASURE_CMD[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};

unsigned long lastMeasurementTime = 0;

void setup() {
    Serial.begin(9600);
    sensor1Serial.begin(9600);
    sensor2Serial.begin(9600);
    
    pinMode(BUMPER_X, OUTPUT);
    pinMode(BUMPER_Y, OUTPUT);
    digitalWrite(BUMPER_X, LOW);
    digitalWrite(BUMPER_Y, LOW);
    
    delay(500); // Inicializálási idő
}

uint16_t readSensor(SoftwareSerial &sensorSerial) {
    // Mérés parancs küldése
    sensorSerial.write(MEASURE_CMD, sizeof(MEASURE_CMD));
    
    // Várunk 100ms-ot a megbízható mérésért
    delay(SENSOR_WAIT_TIME);
    
    // Válasz olvasása
    if (sensorSerial.available() >= 8) {
        byte response[8];
        sensorSerial.readBytes(response, 8);
        
        // Távolság kiszámítása (mm -> cm)
        uint16_t distance = ((uint16_t)response[1] << 8) | response[2];
        return distance / 10;  // mm to cm
    }
    return 0xFFFF; // Hiba esetén
}

void loop() {
    unsigned long currentTime = millis();
    
    // Csak akkor mérünk, ha eltelt a MEASUREMENT_INTERVAL
    if (currentTime - lastMeasurementTime >= MEASUREMENT_INTERVAL) {
        // Első szenzor olvasása
        uint16_t distance1 = readSensor(sensor1Serial);
        if (distance1 != 0xFFFF) {
            if (distance1 < DISTANCE_THRESHOLD && distance1 >= 3) {
                digitalWrite(BUMPER_X, HIGH);
            } else {
                digitalWrite(BUMPER_X, LOW);
            }
            Serial.print("Sensor1: ");
            Serial.print(distance1);
            Serial.print("cm ");
        }
        
        // Második szenzor olvasása
        uint16_t distance2 = readSensor(sensor2Serial);
        if (distance2 != 0xFFFF) {
            if (distance2 < DISTANCE_THRESHOLD && distance2 >= 3) {
                digitalWrite(BUMPER_Y, HIGH);
            } else {
                digitalWrite(BUMPER_Y, LOW);
            }
            Serial.print("Sensor2: ");
            Serial.print(distance2);
            Serial.println("cm");
        }
        
        lastMeasurementTime = currentTime;
    }
}