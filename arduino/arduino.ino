// version 0.0.1

const int STEP = 0.025; //mm; just for information
const int STEP_DELAY = 500; //microseconds

const int STEP_PIN = 2;
const int DIRECTION_PIN = 5;

const int DEFAULT_POS = 0;
long current_pos = 0;

void setup() {
    Serial.begin(115200);
    
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIRECTION_PIN, OUTPUT);

    digitalWrite(DIRECTION_PIN, HIGH);

}

void slide() {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY);
}

void loop() {
    if (Serial.available()) {
        long target_pos;

        String line = Serial.readStringUntil('\n');
        if (line.length() > 0) {
            target_pos = line.toInt();
        } else {
            return;
        }

        long diff = 0;

        if (target_pos < current_pos) {
            diff = current_pos - target_pos;
        } else if (target_pos > current_pos) {
            diff = target_pos - current_pos;
        }

        for (long i = 0; i < diff; i++) {
            if (target_pos < current_pos) {
                digitalWrite(DIRECTION_PIN, LOW);
            } else if (target_pos > current_pos) {
                digitalWrite(DIRECTION_PIN, HIGH);
            }
            slide();
        }

        current_pos = target_pos;
    }
}