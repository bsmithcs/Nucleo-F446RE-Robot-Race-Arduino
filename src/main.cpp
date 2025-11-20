
#include "stm32f4xx.h"
#include <Arduino.h>

#define BAUDRATE 115200
#define CLOCK_SPEED 16000000

/***** SSD Definitions *****/
#ifndef SSD_ARRAY_H
#define SSD_ARRAY_H

#ifdef __cplusplus
extern "C" {
    #endif
    void SSD_init(void);
    void SSD_update(int digitSelect, int value, int decimalPoint);
    #ifdef __cplusplus
}
#endif
#endif

/***** Servo Definitions *****/
#include <Servo.h>

Servo myservo; // create Servo object to control a servo

/***** NeoPixel Definitions *****/
#include <Adafruit_NeoPixel.h>

#define LED_PIN PA0
#define LED_COUNT 4

#define RED pixels.Color(150, 0, 0)
#define PURPLE pixels.Color(150, 0, 150)
#define BLUE pixels.Color(0, 150, 150)
#define GREEN pixels.Color(0, 150, 0)

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

/***** Button Definitions *****/
#define BTN_PIN     PC13

/***** Timer Definitions *****/
HardwareTimer *timer2;

/***** Function Declarations *****/
void TIM2_Handler(void);
void BTN_Handler(void);

void update_reading(void);
void get_SSD_reading(void);

/***** Setup/Loop *****/
void setup() {

    /***** Servo Config *****/
    myservo.attach(9); // attaches the servo on pin 9 to the Servo object
    myservo.write();              // sets the servo position according to the scaled value

    /***** NeoPixel Config *****/
    pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
    pixels.setBrightness(50);


}

void loop() {

}

/***** Function Definitions *****/
