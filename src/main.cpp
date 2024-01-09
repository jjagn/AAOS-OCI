// AAOS DISPLAY OCI PROGRAM

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_DATA    D0 // LED data out pin
#define NUMPIXELS   240 // 60 LED/meter 4 metre strip
#define DELAY       0 // 10ms delay

/*
    ANIMATION VARIABLES
*/

// fps of program states
#define FRAMES_PER_SECOND 120

// number of different animation states
#define NUM_STAGES 10

// time to complete one full animation, e.g. cycle through all states IN SECONDS
#define CYCLE_TIME 10

// cycle time in milliseconds
#define CYCLE_TIME_MS (CYCLE_TIME * 1000)

// time per frame IN MILLISECONDS
#define FRAME_TIME_MS (1000 / FRAMES_PER_SECOND)

// total number of frames in animation
#define TOTAL_FRAMES (FRAMES_PER_SECOND * CYCLE_TIME)

// frames per stage
#define FRAMES_PER_STAGE (TOTAL_FRAMES / NUM_STAGES)

int state; // controls program state
long masterFrame; // counts up program frames
long prevFrame = 0; // previous frame
long frameOffset = 0;

// int totalFrames = FRAMES_PER_SECOND * CYCLE_TIME;
// int framesPerStage = totalFrames / NUM_STAGES;

long tCurrent; // current program time in milliseconds
long tOffset = 0; // used to offset the program by a certain time and maintain sync with the lower half


/*
    PIN DEFINITIONS
*/

#define HALL 2


// R 125, G 0, B 118

// Adafruit_NeoPixel pixels(NUMPIXELS, LED_DATA, NEO_GRB + NEO_KHZ800);

// void colorWipe(uint32_t color, int wait);
// void theaterChase(uint32_t color, int wait);
// void rainbow(int wait);
// void theaterChaseRainbow(int wait);

void setup() {
//   pixels.begin(); // INITIALIZE NeoPixel strip object
//   pixels.clear();
//   pixels.show();
//   pixels.setBrightness(255);
    Serial.begin(460800); 
    pinMode(HALL, INPUT_PULLDOWN);      // hall sensor output active high
}

void loop() {
    // get time
    tCurrent = millis();
    masterFrame = tCurrent / (FRAME_TIME_MS);
    int frame = (masterFrame - frameOffset) % TOTAL_FRAMES;

    // decide on state from frame number    
    if (frame < FRAMES_PER_STAGE) state = 1;
    else if (frame >= FRAMES_PER_STAGE && frame < 2*FRAMES_PER_STAGE) state = 2;
    else if (frame >= 2*FRAMES_PER_STAGE && frame < 3*FRAMES_PER_STAGE) state = 3;
    else if (frame >= 3*FRAMES_PER_STAGE && frame < 4*FRAMES_PER_STAGE) state = 4;
    else if (frame >= 4*FRAMES_PER_STAGE && frame < 5*FRAMES_PER_STAGE) state = 5;
    else if (frame >= 5*FRAMES_PER_STAGE && frame < 6*FRAMES_PER_STAGE) state = 6;
    else if (frame >= 6*FRAMES_PER_STAGE && frame < 7*FRAMES_PER_STAGE) state = 7;
    else if (frame >= 7*FRAMES_PER_STAGE && frame < 8*FRAMES_PER_STAGE) state = 8;
    else if (frame >= 8*FRAMES_PER_STAGE && frame < 9*FRAMES_PER_STAGE) state = 9;
    else if (frame >= 9*FRAMES_PER_STAGE) state = 10;

    // Serial.println("state:");
    // Serial.println(state);


    switch (state)
    {
    // state 1
    case 1:
        // servo open slowly
        /* code */
        break;

    // state 2
    case 2:
        // STEPPER:
        // set target to open

        break;

    // state 3
    case 3:

        break;

    case 4:
    
        break;

    case 5:
        // STEPPER:
        // set target to closed

        break;

    case 6:

        break;

    case 7:

        break;

    case 8:

        break;

    case 9:

        break;

    case 10:
        Serial.print("monitoring hall: ");
        Serial.println(analogRead(HALL));
        if (analogRead(HALL) >= 3500) { // 4095 max
            state = 1; // reset animation to beginning
            frameOffset = millis() / FRAME_TIME_MS; // take final tOffset reading
        }
        break;

    default:
        break;
    }
    
    if (masterFrame > prevFrame) {
        Serial.print("current time: ");
        Serial.println(tCurrent);
        Serial.print("absolute frame: ");
        Serial.println(masterFrame);
        Serial.print("animation frame: ");
        Serial.println(frame);
        Serial.print("state: ");
        Serial.println(state);
        prevFrame = masterFrame;
        Serial.print("Hall sensor: ");
        Serial.println(analogRead(HALL));
    }

//   pixels.clear(); // Set all pixel colors to 'off'
//   static float brightness = 0;
//   static int brightnessChange = 1;

//   if (brightness == 255) {
//     brightnessChange = -1;
//   } else if (brightness == 0) {
//     brightnessChange = 1;
//   }

//   brightness += brightnessChange;

//   float scaler = brightness/255;
//   colorWipe(pixels.Color(125*scaler,   0,   118*scaler), DELAY); // Red
//   delay(10);
}

// void colorWipe(uint32_t color, int wait) {
//   for(int i=0; i<pixels.numPixels(); i++) { // For each pixel in strip...
//     pixels.setPixelColor(i, color);         //  Set pixel's color (in RAM)
//     delay(wait);                           //  Pause for a moment
//   }
//     pixels.show();                          //  Update strip to match
// }