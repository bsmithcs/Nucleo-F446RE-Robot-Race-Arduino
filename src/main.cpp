
#include "stm32f4xx.h"
#include <Arduino.h>

#define CLOCK_SPEED 16000000UL

/***** SSD Definitions *****/
#ifndef SSD_ARRAY_H
#define SSD_ARRAY_H

#ifdef __cplusplus
extern "C"
{
#endif
    void SSD_init(void);
    void SSD_update(int digitSelect, int value, int decimalPoint);
#ifdef __cplusplus
}
#endif
#endif

#define SSD_RATE 200

/***** Servo Definitions *****/
#include <Servo.h>
#define L_SERVO_PIN PC8
#define R_SERVO_PIN PC9

#define STOP_SIGNAL 1500
// Possibly wrong
#define L_MAX_FORWARD 1720
#define R_MAX_FORWARD 1280
#define L_MAX_BACKWARD 1280
#define R_MAX_BACKWARD 1720

#define L_MIN_FORWARD 1550
#define R_MIN_FORWARD 1450
#define L_MIN_BACKWARD 1450
#define R_MIN_BACKWARD 1550

typedef enum movement_commands
{
    stop_motors,
    backward,
    forward,
    left,
    right,
    pivot_left,
    pivot_right
} movement_e;

Servo left_servo;
Servo right_servo;

/***** IR Definitions *****/
#define IR_SENSOR_PORT GPIOC
volatile int IR_reading;

/***** Sonar Defintions *****/
#define TX_PIN PA4
#define RX_PIN PB0

volatile uint32_t rise;
volatile uint32_t fall;
volatile float sonar_distance;

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
#define BTN_PIN PC13

/***** Timer Definitions *****/
HardwareTimer *timer2;
HardwareTimer *timer5;

/***** State *****/
typedef enum state_machine
{
    stopped,
    following_line,
    seeking_opening,
    parking,
    dancing
} state_e;
volatile state_e state;

volatile uint32_t start_time;
volatile uint32_t time;
volatile uint32_t finish_time;

volatile int digit;
volatile int decimal;

/***** Flags *****/
volatile bool dancing_flag;
volatile bool neo_blinking;

/***** Function Declarations *****/
void TIM2_Handler(void);
void BTN_Handler(void);
void sonar_handler_rise(void);
void sonar_handler_fall(void);

void sonar_config(void);
void IR_config(void);

void read_IR(void);
void read_sonar(void);
void update_motors(movement_e movement);

void stop(void);
void follow_line(void);
void seek_opening(void);
void park(void);
void dance(void);

void neo_blink(uint32_t color);

/***** Setup/Loop *****/
void setup()
{
    /***** Timer Config *****/
    timer2 = new HardwareTimer(TIM2);
    timer5 = new HardwareTimer(TIM5);

    timer2->setOverflow(SSD_RATE, HERTZ_FORMAT);
    timer5->setOverflow(0xFFFFFFFF);

    timer2->attachInterrupt(TIM2_Handler);

    /***** Button Config *****/
    pinMode(BTN_PIN, INPUT_PULLUP);
    attachInterrupt(BTN_PIN, BTN_Handler, FALLING);

    /***** SSD Config *****/
    SSD_init();

    /***** Servo Config *****/
    left_servo.attach(L_SERVO_PIN);
    right_servo.attach(R_SERVO_PIN);

    /***** Sonar Config *****/
    sonar_config();

    /***** IR Config *****/
    IR_config();

    /***** NeoPixel Config *****/
    pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
    pixels.setBrightness(50);

    /***** Initialize State *****/
    state = stopped;
    start_time = -1;
    time = -1;
    finish_time = -1;
    digit = 0;
    decimal = 2;
    sonar_distance = 0.0;
}

void loop()
{
    switch (state)
    {
    case stopped:
        stop();
        break;
    case following_line:
        follow_line();
        break;
    case seeking_opening:
        seek_opening();
        break;
    case parking:
        park();
        break;
    case dancing:
        dance();
        break;
    default:
        break;
    }
}

/***** Function Definitions *****/

void TIM2_Handler(void)
{
    digit = (digit + 1) % 4;

    if (finish_time != -1)
    {
        SSD_update(digit, finish_time / 10, decimal);
    }
    else if (start_time != -1)
    {
        time = timer5->getCount();
        SSD_update(digit, (time - start_time) / 10, decimal);
    }
    else
    {
        SSD_update(digit, 0, decimal);
    }
}

void BTN_Handler(void)
{
    if (state == stopped)
    {
        state = following_line;
        start_time = timer5->getCount();
    }
    else
    {
        dancing_flag = false;
        state = stopped;
    }
}

void stop(void)
{
    neo_blink(RED);
    update_motors(stop_motors);
}

void follow_line(void)
{
    read_IR();
    switch (IR_reading)
    {
    case 0b0110:
        update_motors(forward);
        break;
    case 0b0001:
    case 0b0010:
    case 0b0011:
    case 0b0111:
        update_motors(right);
        break;
    case 0b1000:
    case 0b0100:
    case 0b1100:
    case 0b1110:
        update_motors(left);
        break;
    case 0b0000:
        update_motors(stop_motors);
        break;
    default:
        update_motors(forward);
        break;
    }
}

void seek_opening(void)
{
    update_motors(pivot_left);
    uint32_t scan_start = timer5->getCount();
    // Adjust while loop to set time pivoting one direction
    while (((timer5->getCount() - scan_start) < 3000) || sonar_distance > 60.0)
    {
        read_sonar();
    }
    if (sonar_distance > 60.0)
    {
        state = parking;
        return;
    }
    update_motors(pivot_right);
    uint32_t scan_start = timer5->getCount();
    // Adjust while loop to set time pivoting one direction
    while (((timer5->getCount() - scan_start) < 3000) || sonar_distance > 60.0)
    {
        read_sonar();
    }
    if (sonar_distance > 60.0)
    {
        state = parking;
        return;
    }
}

void park(void)
{
    neo_blink(BLUE);
    update_motors(forward);
    read_IR();
    if (IR_reading == 0b1111)
    {
        update_motors(stop_motors);
        state = dancing;
        delay(3000);
    }
}

void dance(void)
{
    dancing_flag = true;
    while (dancing_flag)
    {
        update_motors(forward);
        delay(250);
        update_motors(backward);
        delay(250);
    }
    state = stopped;
}

void update_motors(movement_e movement)
{
    switch (movement)
    {
    case stop_motors:
        left_servo.write(STOP_SIGNAL);
        right_servo.write(STOP_SIGNAL);
        break;
    case forward:
        left_servo.write(L_MAX_FORWARD);
        right_servo.write(R_MAX_FORWARD);
        break;
    case left:
        left_servo.write(L_MIN_FORWARD);
        right_servo.write(R_MAX_FORWARD);
        break;
    case right:
        left_servo.write(L_MAX_FORWARD);
        right_servo.write(R_MIN_FORWARD);
        break;
    case pivot_left:
        left_servo.write(L_MIN_BACKWARD);
        right_servo.write(R_MIN_FORWARD);
        break;
    case pivot_right:
        left_servo.write(L_MIN_FORWARD);
        right_servo.write(R_MIN_BACKWARD);
        break;
    default:
        break;
    }
}

void neo_blink(uint32_t color)
{
    pixels.fill(color);
    delay(1000);
    pixels.clear();
    delay(500);
}

void sonar_config(void)
{
    pinMode(TX_PIN, OUTPUT);
    pinMode(RX_PIN, INPUT_PULLDOWN);
    attachInterrupt(RX_PIN, sonar_handler_rise, RISING);
    attachInterrupt(RX_PIN, sonar_handler_fall, FALLING);
}

void read_sonar(void)
{
    digitalWrite(TX_PIN, HIGH);
    uint32_t pwm_rise = timer5->getCount();
    while ((timer5->getCount() - pwm_rise) < 10)
        ;
    digitalWrite(TX_PIN, LOW);
}

void sonar_handler_rise(void)
{
    rise = timer5->getCount();
}

void sonar_handler_fall(void)
{
    fall = timer5->getCount();
    uint32_t pulse_width = fall - rise;
    sonar_distance = (float)pulse_width / 148.1f;
}

void IR_config(void)
{
    pinMode(PC0, INPUT);
    pinMode(PC1, INPUT);
    pinMode(PC2, INPUT);
    pinMode(PC3, INPUT);
}

void read_IR(void)
{
    IR_reading = (~IR_SENSOR_PORT->IDR) & 0b1111;
}
