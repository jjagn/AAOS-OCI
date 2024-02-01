// AAOS DISPLAY OCI PROGRAM

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>

/*
    PINS
*/

#define HALL        3           // pin for hall sensor (A0)
#define SERVO_PIN   D7          // servo pin
#define LED_DATA    D10         // LED data out pin


/*
    LEDs
*/

#define NUMPIXELS           14          // 60 LED/meter 4 metre strip
#define DELAY               0           // 10ms delay
#define LED_BOUNCE_SPEED    3           // controls speed of LEDBounce()
Adafruit_NeoPixel strip(NUMPIXELS, LED_DATA, NEO_GRB + NEO_KHZ800);
int brightness = 0;
int add = 1;
// Enztec purple 215, 0, 118

/*
    SERVO
*/

#define SERVO_SMOOTHING 0.02  // smoothing value for servo, smaller = smoother
#define LEVER_OPEN 0
#define LEVER_CLOSED 90
Servo servo = Servo();
float servoTarget = 0;
float servoPrevious = 0;
float servoSmoothed = 0;

#define CLOSE_DELAY_MS  500
#define OPEN_DELAY_MS   10000
long tContact = 0;
bool latch = false;

/* 
        ANIMATION
*/

// animation framerate
#define FRAMES_PER_SECOND 120
// number of animation stages
#define NUM_STAGES 10
// total run time of animation (seconds)
#define ANIMATION_RUNTIME 45
// total number of frames in animation
#define MAX_FRAMES (FRAMES_PER_SECOND * ANIMATION_RUNTIME)
// frames in each animation stage
#define FRAMES_PER_STAGE (MAX_FRAMES / NUM_STAGES)
// frame time in microseconds
#define FRAME_TIME_US ((1000 * 1000) / FRAMES_PER_SECOND)
// ADC level for magnet detection
#define HALL_THRESHOLD 2600
// maxmimum pacer value for hall sensor analogRead
#define HALL_PACER_MAX 5000


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
void LEDrotate();
void runLever2();

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
    servo.write(SERVO_PIN, LEVER_CLOSED);
    delay(1000);

    // initialse LEDs
    pinMode(LED_DATA, OUTPUT);
    strip.begin();

    // initialiseLEDs();

    // initialise serial
    // Serial.begin(9600);
}

void loop() {
    // animation timing and repeat management
    // if (frame <= MAX_FRAMES) {
        if ((micros() - frameStartTime) >= FRAME_TIME_US) {
            frame++;
            frameStartTime = micros();
        }
        
    // if (frame >= MAX_FRAMES) {
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
            // Serial.println(averaged);
            pacer = 0;

            // if (averaged >= HALL_THRESHOLD) { // if hall reset switch is deemed close enough
                // reset animation
                // frame = 1;
                // frameStartTime = micros();
                // prevFrame = 0;

                // reset filter to prevent instantly triggering the next time the animation finishes
                // averaged = 0;
                // sum = 0;
                // for (int i = 0; i<WINDOW_SIZE; i++)
                // {
                //     readings[i] = 0;
                // }
                
            // }
        }
    // }

    /* ANIMATION */
    if (frame > prevFrame) {
        prevFrame = frame;
        runNoseLighting(frame);

        runLever2();

        // Serial.print("frame: ");
        // Serial.println(frame);
        // Serial.print("time: ");
        // Serial.println(millis());

        // int animationStage = frame / FRAMES_PER_STAGE;

        // Serial.print("animation stage: ");
        // Serial.println(animationStage)
        // servoSmoothed = SERVO_SMOOTHING * servoTarget + (1.0-SERVO_SMOOTHING) * servoPrevious;
        // servo.write(SERVO_PIN, servoSmoothed);
        // servoPrevious = servoSmoothed;
        // Serial.println(servoSmoothed);
    }
}

void runLever(long frame) {
    int animationStage = frame / FRAMES_PER_STAGE;

    switch (animationStage)
    {
    case 0:
        servoTarget = LEVER_CLOSED;
        break;

    case 1:
        servoTarget = LEVER_OPEN;
        break;
    
    case 10:
        servoTarget = LEVER_CLOSED;
        break;

    default:
        break;
    }

}

void runLever2() {
    unsigned long timeNow = millis();
    if (averaged >= HALL_THRESHOLD && !latch) {
        tContact = timeNow;
        latch = true;
    }


    if (latch) {
        if ((timeNow >= tContact + CLOSE_DELAY_MS) && timeNow < tContact + OPEN_DELAY_MS) {
            servoTarget = LEVER_CLOSED;
        } else if (timeNow >= tContact + OPEN_DELAY_MS) {
            servoTarget = LEVER_OPEN;
            if (averaged < HALL_THRESHOLD) {
                // i.e. cup has left
                latch = false; // reset circuit
            }
        }
    }

    servoSmoothed = SERVO_SMOOTHING * servoTarget + (1.0-SERVO_SMOOTHING) * servoPrevious;
    servo.write(SERVO_PIN, servoSmoothed);
    servoPrevious = servoSmoothed;
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
    // if (frame < STAGE_10_END) {
        // LEDBounce();
        LEDrotate();
    // } else {
    //     strip.clear();
    //     strip.show();
    // }
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

void LEDrotate() {
    strip.clear();

    for (int i = 0; i<NUMPIXELS; i++) {
        int brightnessMod = (frame/15 - i) % 30;
        int brightnessMod2 = (frame/9 - i) % 30;
        int brightnessMod3 = (frame/30 - i) % 30;
        int brightnessMod4 = (frame/5 - i) % 30;
        int s = brightnessMod + brightnessMod2 + brightnessMod3 + brightnessMod4;

        strip.setPixelColor(i, strip.Color((215.0-(s)),0,(118-(s))));
    }
    strip.show();
}

float servoLinear(long frame, long startFrame, long endFrame, long startPos, long endPos) {
    float dPos = (endPos - startPos)/(float(endFrame) - float(startFrame));
    return startPos + (float(frame - startFrame) * dPos);
}