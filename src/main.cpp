
#include "stm32f4xx.h"
#include <Arduino.h>

#define CLOCK_SPEED 16000000UL // Core Clock Speed

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
#include "STM32_ISR_Servo.h"

#define L_SERVO_PIN PC8
#define R_SERVO_PIN PC9

#define MIN_MICROS 1280
#define MAX_MICROS 1720

#define STOP_SIGNAL 90
// Possibly wrong
#define L_MAX_FORWARD 180
#define R_MAX_FORWARD 0
#define L_MAX_BACKWARD 0
#define R_MAX_BACKWARD 180

#define L_MIN_FORWARD 100
#define R_MIN_FORWARD 80
#define L_MIN_BACKWARD 80
#define R_MIN_BACKWARD 100

int left_servo = -1;
int right_servo = -1;

/// @brief Possible movement commands to send to the servos
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

volatile uint32_t neo_last_update = 0;
volatile bool neo_flag = false;
volatile uint32_t neo_current_color = RED;

/***** Button Definitions *****/
#define BTN_PIN PC13

/***** Timer Definitions *****/
HardwareTimer *timer2;
HardwareTimer *timer5;

/***** State *****/

/// @brief Global state that indicates the robot's current process
typedef enum state_machine
{
    stopped,
    following_line,
    seeking_opening,
    parking,
    dancing
} state_e;
volatile state_e state;

// Stopwatch timer variables
volatile uint32_t start_time;
volatile uint32_t stopwatch_time;
volatile uint32_t finish_time;

volatile int digit;
volatile int decimal;

/***** Flags *****/
volatile bool dancing_flag;

/***** Function Declarations *****/
void tim2_handler(void);
void btn_handler(void);
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
    timer5->setPrescaleFactor(15);
    timer5->setOverflow(0xFFFFFFFF);

    timer2->attachInterrupt(tim2_handler);

    /***** Button Config *****/
    pinMode(BTN_PIN, INPUT_PULLUP);
    attachInterrupt(BTN_PIN, btn_handler, FALLING);

    /***** SSD Config *****/
    SSD_init();

    /***** Servo Config *****/
    STM32_ISR_Servos.useTimer(TIM7);

    left_servo = STM32_ISR_Servos.setupServo(L_SERVO_PIN, MIN_MICROS, MAX_MICROS);
    right_servo = STM32_ISR_Servos.setupServo(R_SERVO_PIN, MIN_MICROS, MAX_MICROS);

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
    stopwatch_time = -1;
    finish_time = -1;
    digit = 0;
    decimal = 2;
    sonar_distance = 0.0;

    /***** Start Timers *****/
    timer2->resume();
    timer5->resume();
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

/// @brief Interrupt handler for TIM2, updates SSD with current stopwatch time
void tim2_handler(void)
{
    digit = (digit + 1) % 4;

    if (finish_time != -1)
    {
        SSD_update(digit, finish_time / 10, decimal);
    }
    else if (start_time != -1)
    {
        stopwatch_time = timer5->getCount();
        SSD_update(digit, (stopwatch_time - start_time) / 10000, decimal);
    }
    else
    {
        SSD_update(digit, 0, decimal);
    }
}

/// @brief Interrupt handler for BTN, updates state from stopped to following_line
///        or if not stopped, updates to stopped state, resetting flags
void btn_handler(void)
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

/// @brief Executed continuously under the "stopped" state
///        Move to following_line state when button is pressed
void stop(void)
{
    neo_blink(RED);
    update_motors(stop_motors);
}

/// @brief Executed continuously under the "following_line" state
///        Move to seeking_opening state when IR reads 0b0000
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
        state = seeking_opening;
        break;
    default:
        update_motors(forward);
        break;
    }
}

/// @brief Executed continuously under the "seeking_opening" state
///        Move to parking state when sonar reads long distance
void seek_opening(void)
{
    uint32_t scan_start;
    update_motors(pivot_left);
    scan_start = timer5->getCount();
    // Adjust while loop to set time pivoting one direction (currently 3000ms)
    while (((timer5->getCount() - scan_start) < 3000000) && (sonar_distance <= 60.0))
    {
        read_sonar();
    }
    if (sonar_distance > 60.0)
    {
        state = parking;
        return;
    }
    update_motors(pivot_right);
    scan_start = timer5->getCount();
    // Adjust while loop to set time pivoting one direction (currently 3000ms)
    while (((timer5->getCount() - scan_start) < 3000000) && (sonar_distance <= 60.0))
    {
        read_sonar();
    }
    if (sonar_distance > 60.0)
    {
        state = parking;
        return;
    }
}

/// @brief Executed continuously under the "parking" state
///        Move to dancing state when IR reads 0b1111
void park(void)
{
    static uint32_t wait_timer = 0;
    static bool waiting = false;
    static bool initialized = false;

    // Initialize on first entry to parking state
    if (!initialized)
    {
        waiting = false;
        initialized = true;
    }

    neo_blink(BLUE);

    if (!waiting)
    {
        // Normal parking behavior
        update_motors(forward);
        read_IR();

        if (IR_reading == 0b1111)
        {
            update_motors(stop_motors);
            waiting = true;
            wait_timer = timer5->getCount();
        }
    }
    else
    {
        // Waiting 3 seconds before transitioning to dancing
        if ((timer5->getCount() - wait_timer) >= 3000000)
        { // 3 seconds
            state = dancing;
            waiting = false;
            initialized = false; // Reset for next time
        }
    }
}

/// @brief Executed continuously under the "dancing" state
///        Move to stopped state when button is pressed
void dance(void)
{
    static uint32_t dance_timer = 0;
    static bool moving_forward = true;
    static bool initialized = false;

    // Initialize on first entry to dancing state
    if (!initialized)
    {
        dancing_flag = true;
        dance_timer = timer5->getCount();
        moving_forward = true;
        initialized = true;
    }

    // Check if it's time to switch direction
    if ((timer5->getCount() - dance_timer) >= 250000)
    { // 250ms
        if (moving_forward)
        {
            update_motors(forward);
            moving_forward = false;
        }
        else
        {
            update_motors(backward);
            moving_forward = true;
        }
        dance_timer = timer5->getCount();
    }

    // Check if button was pressed to stop dancing
    if (!dancing_flag)
    {
        state = stopped;
        initialized = false; // Reset for next time
    }
}

/// @brief Updates servo motors with the given movement, sending needs PWMs
/// @param movement Instruction to execute for the robot
void update_motors(movement_e movement)
{
    switch (movement)
    {
    case stop_motors:
        STM32_ISR_Servos.setPosition(left_servo, STOP_SIGNAL);
        STM32_ISR_Servos.setPosition(right_servo, STOP_SIGNAL);
        break;
    case backward:
        STM32_ISR_Servos.setPosition(left_servo, L_MAX_BACKWARD);
        STM32_ISR_Servos.setPosition(right_servo, R_MAX_BACKWARD);
        break;
    case forward:
        STM32_ISR_Servos.setPosition(left_servo, L_MAX_FORWARD);
        STM32_ISR_Servos.setPosition(right_servo, R_MAX_FORWARD);
        break;
    case left:
        STM32_ISR_Servos.setPosition(left_servo, L_MIN_FORWARD);
        STM32_ISR_Servos.setPosition(right_servo, R_MAX_FORWARD);
        break;
    case right:
        STM32_ISR_Servos.setPosition(left_servo, L_MAX_FORWARD);
        STM32_ISR_Servos.setPosition(right_servo, R_MIN_FORWARD);
        break;
    case pivot_left:
        STM32_ISR_Servos.setPosition(left_servo, L_MIN_BACKWARD);
        STM32_ISR_Servos.setPosition(right_servo, R_MIN_FORWARD);
        break;
    case pivot_right:
        STM32_ISR_Servos.setPosition(left_servo, L_MIN_FORWARD);
        STM32_ISR_Servos.setPosition(right_servo, R_MIN_BACKWARD);
        break;
    default:
        break;
    }
}

/// @brief Blinks NeoPixels the given color
/// @param color Color to blink the NeoPixels with
void neo_blink(uint32_t color)
{
    neo_current_color = color;
    uint32_t current_time = timer5->getCount();
    uint32_t elapsed = current_time - neo_last_update;

    if (neo_flag && elapsed >= 1000000)
    { // 1 second ON
        pixels.clear();
        pixels.show();
        neo_flag = false;
        neo_last_update = current_time;
    }
    else if (!neo_flag && elapsed >= 500000)
    { // 0.5 second OFF
        pixels.fill(neo_current_color);
        pixels.show();
        neo_flag = true;
        neo_last_update = current_time;
    }
}

/// @brief Configures TX and RX Pins for sonar sensor
void sonar_config(void)
{
    pinMode(TX_PIN, OUTPUT);
    pinMode(RX_PIN, INPUT_PULLDOWN);
    attachInterrupt(RX_PIN, sonar_handler_rise, RISING);
    attachInterrupt(RX_PIN, sonar_handler_fall, FALLING);
}

/// @brief Initiates sonar read by sending signal to TX pin
void read_sonar(void)
{
    digitalWrite(TX_PIN, HIGH);
    uint32_t pwm_rise = timer5->getCount();
    while ((timer5->getCount() - pwm_rise) < 10)
        ;
    digitalWrite(TX_PIN, LOW);
}

/// @brief Reads in rise on RX pin
void sonar_handler_rise(void)
{
    rise = timer5->getCount();
}

/// @brief Reads in fall on RX pin, calculating distance via the pulse width
void sonar_handler_fall(void)
{
    fall = timer5->getCount();
    uint32_t pulse_width = fall - rise;
    sonar_distance = (float)pulse_width / 148.1f; // Convert w/ speed of sound
}

/// @brief Configures InfraRed Sensors
void IR_config(void)
{
    pinMode(PC0, INPUT);
    pinMode(PC1, INPUT);
    pinMode(PC2, INPUT);
    pinMode(PC3, INPUT);
}

/// @brief Updates IR_reading with a new reading,
///        where (1 = line) and (0 = no line)
void read_IR(void)
{
    IR_reading = (~IR_SENSOR_PORT->IDR) & 0b1111;
}
