// AAOS DISPLAY OCI PROGRAM

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>

/*
    PINS
*/

#define HALL        2           // pin for hall sensor (A0)
#define SERVO_PIN   D7          // servo pin
#define LED_DATA    D10         // LED data out pin


/*
    LEDs
*/

#define NUMPIXELS           240          // 60 LED/meter 4 metre strip
#define DELAY               0           // 10ms delay
#define LED_BOUNCE_SPEED    3           // controls speed of LEDBounce()
Adafruit_NeoPixel strip(NUMPIXELS, LED_DATA, NEO_GRB + NEO_KHZ800);
int brightness = 0;
int add = 1;
// Enztec purple 215, 0, 118

/*
    SERVO
*/

#define SERVO_SMOOTHING 0.0001  // smoothing value for servo, smaller = smoother
Servo servo = Servo();
float servoTarget = 0;
float servoPrevious = 0;
float servoSmoothed = 0;

/* 
        ANIMATION
*/

// animation framerate
#define FRAMES_PER_SECOND 120
// number of animation stages
#define NUM_STAGES 10
// total run time of animation (seconds)
#define ANIMATION_RUNTIME 10
// total number of frames in animation
#define MAX_FRAMES (FRAMES_PER_SECOND * ANIMATION_RUNTIME)
// frames in each animation stage
#define FRAMES_PER_STAGE (MAX_FRAMES / NUM_STAGES)
// frame time in microseconds
#define FRAME_TIME_US ((1000 * 1000) / FRAMES_PER_SECOND)
// ADC level for magnet detection
#define HALL_THRESHOLD 2900
// maxmimum pacer value for hall sensor analogRead
#define HALL_PACER_MAX 5000

// stage beginning and end frames
#define STAGE_1_START   (FRAMES_PER_STAGE * 0)
#define STAGE_1_END     (FRAMES_PER_STAGE * 1)
#define STAGE_2_START   (FRAMES_PER_STAGE * 1)
#define STAGE_2_END     (FRAMES_PER_STAGE * 2)
#define STAGE_3_START   (FRAMES_PER_STAGE * 2)
#define STAGE_3_END     (FRAMES_PER_STAGE * 3)
#define STAGE_4_START   (FRAMES_PER_STAGE * 3)
#define STAGE_4_END     (FRAMES_PER_STAGE * 4)
#define STAGE_5_START   (FRAMES_PER_STAGE * 4)
#define STAGE_5_END     (FRAMES_PER_STAGE * 5)
#define STAGE_6_START   (FRAMES_PER_STAGE * 5)
#define STAGE_6_END     (FRAMES_PER_STAGE * 6)
#define STAGE_7_START   (FRAMES_PER_STAGE * 6)
#define STAGE_7_END     (FRAMES_PER_STAGE * 7)
#define STAGE_8_START   (FRAMES_PER_STAGE * 7)
#define STAGE_8_END     (FRAMES_PER_STAGE * 8)
#define STAGE_9_START   (FRAMES_PER_STAGE * 8)
#define STAGE_9_END     (FRAMES_PER_STAGE * 9)
#define STAGE_10_START   (FRAMES_PER_STAGE * 9)
#define STAGE_10_END    (FRAMES_PER_STAGE * 10)

long frame = 0;
long prevFrame = 0;
long frameStartTime = 0;

// animation functions
void runLever(long frame);
void runNoseLighting(long frame);
void runLeverLighting(long frame);
float servoLinear(long frame, long startFrame, long endFrame, long startPos, long endPos);
void initialiseLEDs();
void LEDBounce();
/*
        FILTERING
*/

#define WINDOW_SIZE 10

int idx = 0;
long sum = 0;
long readings[WINDOW_SIZE];
long averaged = 0;


void setup() {
    // put your setup code here, to run once:
    pinMode(HALL, INPUT_PULLDOWN);

    // initialise servo
    pinMode(SERVO_PIN, OUTPUT);
    servo.write(SERVO_PIN, 0);

    // initialse LEDs
    pinMode(LED_DATA, OUTPUT);
    strip.begin();

    // initialiseLEDs();

    // initialise serial
    Serial.begin(460800);
}

void loop() {
    // animation timing and repeat management
    if (frame <= MAX_FRAMES) {
        if ((micros() - frameStartTime) >= FRAME_TIME_US) {
            frame++;
            frameStartTime = micros();
        }
        
    } else {
        static int pacer = 0; 
        if (pacer++ >= HALL_PACER_MAX) {
            long reading = analogRead(HALL);
            // Serial.println(reading);

            sum = sum - readings[idx];
            readings[idx] = reading;
            sum = sum + reading;
            idx = (idx + 1) % WINDOW_SIZE;
            averaged = sum / WINDOW_SIZE;

            // Serial.println(reading);
            Serial.println(averaged);
            pacer = 0;

            if (averaged >= HALL_THRESHOLD) { // if hall reset switch is deemed close enough
                // reset animation
                frame = 1;
                frameStartTime = micros();
                prevFrame = 0;

                // reset filter to prevent instantly triggering the next time the animation finishes
                averaged = 0;
                sum = 0;
                for (int i = 0; i<WINDOW_SIZE; i++)
                {
                    readings[i] = 0;
                }
                
            }
        }
    }

    /* ANIMATION */
    if (frame > prevFrame) { // only update animation on new frames
        Serial.println("new frame, updating animation");
        runLever(frame);
        runNoseLighting(frame);
    // runLeverLighting(frame);
    }

    // THIS MUST REMAIN AT THE END OF THE MAIN LOOP
    if (frame > prevFrame) {
            Serial.println(frame);
            prevFrame = frame;
    }
}

void runLever(long frame) {
    if (frame <= STAGE_1_END) {
        Serial.println("Stage 1");
        // slow linear to 10 deg
        // servo.write(SERVO_PIN, 10, 10, 0.0);
        float target = servoLinear(frame, 0, STAGE_1_END, 0, 180);
        servo.write(SERVO_PIN, target);
        Serial.println(target);

    // } else if (frame > STAGE_2_START && frame <= STAGE_2_END) {
    //     // snap open then ease out to fully open
    //     if (frame == STAGE_2_START + 1) {
    //         // snap to 40 deg
    //         Serial.println("Stage 2 frame 1");
    //         servo.write(SERVO_PIN, 40);
    //         servoPrevious = 40;
    //     } else if (frame >= STAGE_2_START + (FRAMES_PER_STAGE/2)) {
    //         // // ease to fully open
    //         Serial.println("Stage 2");
    //         servo.write(SERVO_PIN, 180);
    //         // servoTarget = 180;
    //         // servoSmoothed = servoTarget * SERVO_SMOOTHING + servoPrevious * (1-SERVO_SMOOTHING);
    //         // servoPrevious = servoSmoothed;
    //         // // servo.write(SERVO_PIN, 180, 40.0, 0.0);
    //         // servo.write(SERVO_PIN, servoSmoothed);
    //     }

    // } else if (frame > STAGE_7_START && frame <= STAGE_7_END) {
    //     // ease back to 40 degrees
    //     Serial.println("Stage 7");
    //     // servo.write(SERVO_PIN, 40, 40.0, 0.66);
    //     servo.write(SERVO_PIN, 40);

    // } else if (frame > STAGE_8_START && frame <= STAGE_8_END) {
    //     if (frame == STAGE_8_START + 1) {
    //         Serial.println("Stage 8 frame 1");
    //         // snap back to 10 deg
    //         servo.write(SERVO_PIN, 10);

    //     } else if (frame >= STAGE_8_START + (FRAMES_PER_STAGE/2)) {
    //         // linear to 0 deg
    //         Serial.println("Stage 8");
    //         // servo.write(SERVO_PIN, 0, 10, 0.0);
    //         servo.write(SERVO_PIN, 0);
    //     }

    } else if (frame >= STAGE_9_START) {
        float target = servoLinear(frame, STAGE_9_START, STAGE_9_END, 180, 0);
        // Serial.println(target);
        servo.write(SERVO_PIN, target);
        // servoSmoothed = 0;
        // servoPrevious = 0;
    }
}

void runNoseLighting(long frame) {
    // if (frame > prevFrame) { // only update strip on new frames
        // strip.clear();

        // for (int i = 0; i<NUMPIXELS; i++) {
        //     strip.setPixelColor(i, strip.Color(215,0,118));

        //     // strip.show();
        // }
        // strip.setBrightness(frame % 255);
        // strip.show();
    // }
    if (frame < STAGE_10_END) {
        LEDBounce();
    } else {
        strip.clear();
        strip.show();
    }
}

void initialiseLEDs() {
    strip.clear();
    for (int i = 0; i<NUMPIXELS; i++) {
        strip.setPixelColor(i, strip.Color(215,0,118));
    }
    strip.show();
}

void LEDBounce() {
    strip.clear();
    for (int i = 0; i<NUMPIXELS; i++) {
        strip.setPixelColor(i, strip.Color(215,0,118));
    }

    brightness += add;
    if (brightness >= 255) {
        add = -1 * LED_BOUNCE_SPEED;
    } else if (brightness <= 0) {
        add = 1 * LED_BOUNCE_SPEED;
    }
    strip.setBrightness(brightness);
    strip.show();
}

float servoLinear(long frame, long startFrame, long endFrame, long startPos, long endPos) {
    float dPos = (endPos - startPos)/(float(endFrame) - float(startFrame));
    return startPos + (float(frame - startFrame) * dPos);
}