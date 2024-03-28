/*
 * Team Id: 1792
 * Author List: Akash Potti
 * Filename: rover.ino
 * Theme: GeoGuide
 * Functions: void initializeSensor(), void initializeBuzzer(), void initializeLed(), void initializeMotor(), void led_control(int ledid, int ledstate), void buzzer_control(int buzzerstate), void forward(), void left(), void left_uturn(), void right(), int stop(int opt_delay), void LeftAndCalibrateOnlineAlign(), void LeftAndCalibrateOnline(), void RightAndCalibrateOnlineAlign(), void RightAndCalibrateOnline(), int JunctionRightAndCalibrateOnline(int node_skip_delay, int node_turn_delay, int time_to_stop, int left_turn_delay, int right_turn_delay), int JunctionLeftAndCalibrateOnline(int node_skip_delay, int node_turn_delay, int time_to_stop, int left_turn_delay, int right_turn_delay), int NavigateAndCalibrateOnline_WithStop(int node_skip_delay, int node_turn_delay, int time_to_stop, int left_turn_delay, int right_turn_delay), int NavigateAndCalibrateOnline(int node_skip_delay, int node_turn_delay, int time_to_stop, int left_turn_delay, int right_turn_delay), int junction_uturn(int node_skip_delay, int node_turn_delay, int time_to_stop, int left_turn_delay, int right_turn_delay), int turn_90(int node_skip_delay, int node_turn_delay, int time_to_stop, int left_turn_delay, int right_turn_delay), int JunctionStop(int node_skip_delay, int node_turn_delay, int left_turn_delay, int right_turn_delay), int readsensors(), unsigned long prev_junction_time, int navigate(int node_skip_delay, int time_to_stop, int left_turn_delay, int right_turn_delay), void initialise_network(), void setup(), void loop()
 * Global Variables: const char *ssid, const char *password, const uint16_t port, const char *host, char incomingPacket[80], WiFiClient client
 */
#include <string.h>
#include <WiFi.h>
// GPIO assignments
// Motor A
#define EnableA 15
#define in1 2
#define in2 4

// Motor B
#define EnableB 5
#define in3 18
#define in4 19

#define buzzerPin 21

// LED Pins
#define ledGreenPin 22
#define ledRedPin 23

// IR Sensor Pins
#define irPinEdgeLeft 27  // to detect road left edge
#define irPinEdgeRight 14 // to detect road right edge
#define irPinFrontCenter 13
#define irPinFrontLeft 26  // to keep rover on center line, detect left offset.
#define irPinFrontRight 25 // to keep rover on center line, detect right offset.

#define freq 30000              // Specifies the frequency (in Hertz) of the PWM signal used for controlling the motors A higher frequency provides smoother motor operation and reduces audible noise.
#define pwmchannelARightWheel 0 // Assigns the PWM channel number for the right wheel motor.
#define pwmchannelBLeftWheel 1  // Assigns the PWM channel number for the left wheel motor.
#define resolution 8            // Specifies the resolution (in bits) of the PWM signal.Higher resolution allows for finer control over the duty cycle, resulting in smoother motor speed control.
#define dutycycle 220           // Specifies the duty cycle (in bits) for the PWM signal. A higher duty cycle value corresponds to a higher motor speed.

#define DELAY 1 //  Specifies the delay (in milliseconds) between sensor readings.

#define GREEN_LED 1 //  Specifies the ID of the green LED.
#define RED_LED 2   //  Specifies the ID of the red LED.

const char *ssid = "";             // Enter your wifi hotspot ssid
const char *password = ""; // Enter your wifi hotspot password
const uint16_t port = 8002;                // Enter the port number of your laptop after connecting it to the wifi hotspot
const char *host = "";        // Enter the ip address of your laptop after connecting it to the wifi hotspot
char incomingPacket[80];
WiFiClient client;
/*
 * Function Name: void initializeSensor()
 * Input: None
 * Output: None
 * Description: Initializes the sensor pins for reading infrared (IR) sensor data.
 *              Specifically, configures the pins for input mode to read sensor values.
 * Example Call: initializeSensor();
 */
void initializeSensor()
{
    pinMode(irPinEdgeLeft, INPUT);
    pinMode(irPinEdgeRight, INPUT);
    pinMode(irPinFrontCenter, INPUT);
    pinMode(irPinFrontLeft, INPUT);
    pinMode(irPinFrontRight, INPUT);
}
/*
 * Function Name: void initializeBuzzer()
 * Input: None
 * Output: None
 * Description: Initializes the buzzer pin for controlling the buzzer.
 * Example Call: initializeBuzzer();
 */
void initializeBuzzer()
{
    buzzer_control(HIGH);
}
/*
 * Function Name: void initializeLed()
 * Input: None
 * Output: None
 * Description: Initializes the LED pins for controlling the green and red LEDs.
 * Example Call: initializeLed();
 */
void initializeLed()
{
    pinMode(ledGreenPin, OUTPUT);
    pinMode(ledRedPin, OUTPUT);

    led_control(GREEN_LED, LOW);
    led_control(RED_LED, LOW);
}
/*
 * Function Name: void initializeMotor()
 * Input: None
 * Output: None
 * Description: Initializes the motor pins for controlling the motors.
 * Example Call: initializeMotor();
 */
void initializeMotor()
{
    // Motor A
    pinMode(EnableA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    // Motor B
    pinMode(EnableB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    // Set the motor direction pins to move the robot forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    // Set up the PWM signal for motor speed control
    ledcSetup(pwmchannelARightWheel, freq, resolution);
    ledcSetup(pwmchannelBLeftWheel, freq, resolution);
    ledcAttachPin(EnableA, pwmchannelARightWheel);
    ledcAttachPin(EnableB, pwmchannelBLeftWheel);
    ledcWrite(pwmchannelARightWheel, dutycycle);
    ledcWrite(pwmchannelBLeftWheel, dutycycle);
}
/*
 * Function Name: void led_control(int ledid, int ledstate)
 * Input: ledid -> an integer representing the ID of the LED ( GREEN_LED, RED_LED)
 *        ledstate -> an integer representing the desired state of the LED (e.g., HIGH, LOW)
 * Output: None
 * Description: Controls the state of the LED based on the specified ledid and ledstate.
 * Example Call: led_control(GREEN_LED, HIGH);
 */

void led_control(int ledid, int ledstate)
{
    if (GREEN_LED == ledid)
        digitalWrite(ledGreenPin, ledstate);
    else if (RED_LED == ledid)
        digitalWrite(ledRedPin, ledstate);
}
/*
 * Function Name: void buzzer_control(int buzzerstate)
 * Input: buzzerstate -> an integer representing the desired state of the buzzer (e.g., HIGH, LOW)
 * Output: None
 * Description: Controls the state of the buzzer based on the specified buzzerstate.
 *              If buzzerstate is HIGH, it configures the buzzerPin as INPUT and sets its state to HIGH.
 *              If buzzerstate is LOW, it configures the buzzerPin as OUTPUT and sets its state to LOW.
 *              (e.g., buzzerPin). It uses pinMode and digitalWrite functions to control the buzzer state.
 * Example Call: buzzer_control(HIGH);
 */

void buzzer_control(int buzzerstate)
{
    if (buzzerstate) // buzzerstate == HIGH
    {
        pinMode(buzzerPin, INPUT);
        digitalWrite(buzzerPin, buzzerstate);
    }
    else if (!buzzerstate) // buzzerstate == LOW
    {
        pinMode(buzzerPin, OUTPUT);
        digitalWrite(buzzerPin, buzzerstate);
    }
}

#define BIAS_ADJUST 10 // Specifies the bias adjustment value for motor control.
/*
 * Function Name: void forward()
 * Input: None
 * Output: None
 * Description: Moves the robot forward by setting the PWM signals to both motors
 *              to drive them forward at the specified duty cycle.
 *              This function requires global variables defining
 *              PWM channels and duty cycles for both motors (e.g., pwmchannelARightWheel,
 *              pwmchannelBLeftWheel, dutycycle). It also requires the presence of
 *              a constant BIAS_ADJUST for adjusting motor bias.
 * Example Call: forward();
 */

void forward()
{
    // Set the motor direction pins to move the robot forward
    ledcWrite(pwmchannelARightWheel, dutycycle);
    ledcWrite(pwmchannelBLeftWheel, dutycycle + BIAS_ADJUST);
}
/*
 * Function Name: void left()
 * Input: None
 * Output: None
 * Description: Turns the robot left by setting the PWM signals to the right motor
 *              to drive it forward at the specified duty cycle and the left motor
 *              at a reduced duty cycle to induce a left turn.
 *              This function requires global variables defining
 *              PWM channels and duty cycles for both motors (e.g., pwmchannelARightWheel,
 *              pwmchannelBLeftWheel, dutycycle). It also requires the presence of
 *              a constant BIAS_ADJUST for adjusting motor bias.
 * Example Call: left();
 */
void left()
{
    // Set the motor direction pins to move the robot left
    ledcWrite(pwmchannelARightWheel, dutycycle);
    ledcWrite(pwmchannelBLeftWheel, (dutycycle / 3) + BIAS_ADJUST);
}
/*
 * Function Name: void left_uturn()
 * Input: None
 * Output: None
 * Description: Performs a left U-turn by setting the direction pins accordingly
 *              and setting the PWM signals to both motors to drive them forward
 *              at the specified duty cycle.
 *              This function requires global variables defining
 *              direction control pins, PWM channels, and duty cycles for both motors
 *              (e.g., in1, in2, in3, in4, pwmchannelARightWheel, pwmchannelBLeftWheel,
 *              dutycycle). It also requires the presence of a constant BIAS_ADJUST
 *              for adjusting motor bias.
 * Example Call: left_uturn();
 */
void left_uturn()
{
    // Set the motor direction pins to move the robot left
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    // Set the PWM signals to move the robot forward
    ledcWrite(pwmchannelARightWheel, dutycycle);
    ledcWrite(pwmchannelBLeftWheel, (dutycycle + BIAS_ADJUST));
}
/*
 * Function Name: void right()
 * Input: None
 * Output: None
 * Description: Turns the robot right by setting the PWM signals to the left motor
 *              to drive it forward at a reduced duty cycle and the right motor
 *              at the specified duty cycle.
 *              This function requires global variables defining
 *              PWM channels and duty cycles for both motors (e.g., pwmchannelARightWheel,
 *              pwmchannelBLeftWheel, dutycycle). It also requires the presence of
 *              a constant BIAS_ADJUST for adjusting motor bias.
 * Example Call: right();
 */
void right()
{
    // Set the motor direction pins to move the robot right
    ledcWrite(pwmchannelARightWheel, (dutycycle / 3));
    ledcWrite(pwmchannelBLeftWheel, dutycycle + BIAS_ADJUST);
}
/*
 * Function Name: int stop(int opt_delay)
 * Input: opt_delay -> an integer representing an optional delay in milliseconds
 * Output: Returns 0 after stopping the motors
 * Description: Stops the robot by setting the PWM signals to both motors to 0,
 *              optionally delaying execution by the specified amount of time.
 *              This function requires global variables defining
 *              PWM channels for both motors (e.g., pwmchannelARightWheel,
 *              pwmchannelBLeftWheel). It also uses the delay function to introduce
 *              an optional delay.
 * Example Call: stop(100);
 */
int stop(int opt_delay)
{
    // Set the PWM signals to stop the motors
    delay(opt_delay);
    // Set the PWM signals to stop the motors
    ledcWrite(pwmchannelARightWheel, 0);
    ledcWrite(pwmchannelBLeftWheel, 0);
    return 0;
}
/*
 * Function Name: void LeftAndCalibrateOnlineAlign()
 * Input: None
 * Output: None
 * Description: Performs a left turn while calibrating and aligning the robot online.
 *              This function continuously reads sensor values and compares them to detect any deviation.
 *              If there is no deviation in sensor readings, it initiates a left turn using the left() function.
 *              If there is a deviation, it continues moving forward until it detects the required alignment,
 *              then it stops the motion.
 *              It also relies on the left() and forward() functions for robot movement.
 *              Additionally, it uses a constant DELAY for introducing delays between sensor readings.
 * Example Call: LeftAndCalibrateOnlineAlign();
 */
void LeftAndCalibrateOnlineAlign()
{
    volatile int sensorValu5 = 0; //
    volatile int sensorValue2 = 0;
    do
    {
        sensorValu5 = readsensors();
        delay(DELAY);
        sensorValue2 = readsensors();
        if (sensorValu5 == sensorValue2)
            left();
        else
        {
            forward();
            break;
        }
    } while ((sensorValue2 & 0x7) != 0x2);
}
/*
 * Function Name: void LeftAndCalibrateOnline()
 * Input: None
 * Output: None
 * Description: Performs a left turn while calibrating the robot online.
 *              This function continuously reads sensor values and compares them to detect any deviation.
 *              If there is no deviation in sensor readings, it initiates a left turn using the left() function.
 *              If there is a deviation, it continues moving forward until it detects the required alignment,
 *              then it stops the motion.
 *              It also relies on the left() and forward() functions for robot movement.
 *              Additionally, it uses a constant DELAY for introducing delays between sensor readings.
 * Example Call: LeftAndCalibrateOnline();
 */

void LeftAndCalibrateOnline()
{
    volatile int sensorValu5 = 0;
    volatile int sensorValue2 = 0;
    do
    {
        sensorValu5 = readsensors();
        delay(DELAY);
        sensorValue2 = readsensors();
        if (sensorValu5 == sensorValue2)
            left();
        else
        {
            forward();
            break;
        }
    } while ((sensorValue2 & 0x2) != 0x2);
}
/*
 * Function Name: void RightAndCalibrateOnlineAlign()
 * Input: None
 * Output: None
 * Description: Performs a right turn while calibrating and aligning the robot online.
 *              This function continuously reads sensor values and compares them to detect any deviation.
 *              If there is no deviation in sensor readings, it initiates a right turn using the right() function.
 *              If there is a deviation, it continues moving forward until it detects the required alignment,
 *              then it stops the motion.
 *              It also relies on the right() and forward() functions for robot movement.
 *              Additionally, it uses a constant DELAY for introducing delays between sensor readings.
 * Example Call: RightAndCalibrateOnlineAlign();
 */

void RightAndCalibrateOnlineAlign()
{
    volatile int sensorValu5 = 0;
    volatile int sensorValue2 = 0;
    do
    {
        sensorValu5 = readsensors();
        delay(DELAY);
        sensorValue2 = readsensors();
        if (sensorValu5 == sensorValue2)
            right();
        else
        {
            forward();
            break;
        }
    } while ((sensorValue2 & 0x7) != 0x2);
}
/*
 * Function Name: void RightAndCalibrateOnline()
 * Input: None
 * Output: None
 * Description: Performs a right turn while calibrating the robot online.
 *              This function continuously reads sensor values and compares them to detect any deviation.
 *              If there is no deviation in sensor readings, it initiates a right turn using the right() function.
 *              If there is a deviation, it continues moving forward until it detects the required alignment,
 *              then it stops the motion.
 *              It also relies on the right() and forward() functions for robot movement.
 *              Additionally, it uses a constant DELAY for introducing delays between sensor readings.
 * Example Call: RightAndCalibrateOnline();
 */

void RightAndCalibrateOnline()
{
    volatile int sensorValu5 = 0;
    volatile int sensorValue2 = 0;
    do
    {
        sensorValu5 = readsensors();
        delay(DELAY);
        sensorValue2 = readsensors();
        if (sensorValu5 == sensorValue2)
            right();
        else
        {
            forward();
            break;
        }
    } while ((sensorValue2 & 0x2) != 0x2);
}

#define TURN_DEMINISH 0.95
/*
 * Function Name: int JunctionRightAndCalibrateOnline(int node_skip_delay, int node_turn_delay, int time_to_stop, int left_turn_delay, int right_turn_delay)
 * Input: node_skip_delay -> an integer representing the delay to skip a node
 *        node_turn_delay -> an integer representing the delay for turning at a node
 *        time_to_stop -> an integer representing the time to stop at a node
 *        left_turn_delay -> an integer representing the delay for a left turn
 *        right_turn_delay -> an integer representing the delay for a right turn
 * Output: Returns an integer value returned by the navigate function
 * Description: Performs a right turn at a junction while calibrating the robot online.
 *              This function rotates the robot right at the junction, stops briefly,
 *              then continues moving forward while reading sensor values.
 *              If the robot encounters a junction (where the center and right sensors are active),
 *              it adjusts the turn delay based on the TURN_DEMINISH constant and repeats the process.
 *              Once the junction is navigated, it calls the navigate function to proceed further.
 *              and it relies on constants like TURN_DEMINISH for calibration.
 * Example Call: JunctionRightAndCalibrateOnline(100, 500, 2000, 300, 300);
 */
// edgeRightSensorValue << 4 | edgeLeftSensorValue << 3 | frontRightSensorValue << 2 | frontSensorValue << 1 | frontLeftSensorValue << 0)
int JunctionRightAndCalibrateOnline(int node_skip_delay, int node_turn_delay, int time_to_stop, int left_turn_delay, int right_turn_delay)
{
    volatile int sensorValue = 0;
    int cnt = 100;
    do
    {
        right();
        stop(node_turn_delay);
        forward();
        sensorValue = readsensors();
        if (node_turn_delay > 10)
            node_turn_delay = node_turn_delay * TURN_DEMINISH;
    } while (((sensorValue & 0x6) != 0x2) && (cnt-- > 0));
    return navigate(node_skip_delay, time_to_stop, left_turn_delay, right_turn_delay);
}
/*
 * Function Name: int JunctionLeftAndCalibrateOnline(int node_skip_delay, int node_turn_delay, int time_to_stop, int left_turn_delay, int right_turn_delay)
 * Input: node_skip_delay -> an integer representing the delay to skip a node
 *        node_turn_delay -> an integer representing the delay for turning at a node
 *        time_to_stop -> an integer representing the time to stop at a node
 *        left_turn_delay -> an integer representing the delay for a left turn
 *        right_turn_delay -> an integer representing the delay for a right turn
 * Output: Returns an integer value returned by the navigate function
 * Description: Performs a left turn at a junction while calibrating the robot online.
 *              This function rotates the robot left at the junction, stops briefly,
 *              then continues moving forward while reading sensor values.
 *              If the robot encounters a junction (where the center and left sensors are active),
 *              it adjusts the turn delay based on the TURN_DEMINISH constant and repeats the process.
 *              Once the junction is navigated, it calls the navigate function to proceed further.
 *              and it relies on constants like TURN_DEMINISH for calibration.
 * Example Call: JunctionLeftAndCalibrateOnline(100, 500, 2000, 300, 300);
 */
// edgeRightSensorValue << 4 | edgeLeftSensorValue << 3 | frontRightSensorValue << 2 | frontSensorValue << 1 | frontLeftSensorValue << 0)
int JunctionLeftAndCalibrateOnline(int node_skip_delay, int node_turn_delay, int time_to_stop, int left_turn_delay, int right_turn_delay)
{
    volatile int sensorValue = 0;
    int cnt = 100;
    do
    {
        left();
        stop(node_turn_delay);
        forward();
        sensorValue = readsensors();
        if (node_turn_delay > 10)
            node_turn_delay = node_turn_delay * TURN_DEMINISH;
    } while (((sensorValue & 0x3) != 0x2) && (cnt-- > 0));
    return navigate(node_skip_delay, time_to_stop, left_turn_delay, right_turn_delay);
}
/*
 * Function Name: int NavigateAndCalibrateOnline_WithStop(int node_skip_delay, int node_turn_delay, int time_to_stop, int left_turn_delay, int right_turn_delay)
 * Input: node_skip_delay -> an integer representing the delay to skip a node
 *        node_turn_delay -> an integer representing the delay for turning at a node
 *        time_to_stop -> an integer representing the time to stop at a node
 *        left_turn_delay -> an integer representing the delay for a left turn
 *        right_turn_delay -> an integer representing the delay for a right turn
 * Output: Returns an integer value returned by the navigate function
 * Description: Navigates while calibrating the robot online, with intermittent stops at nodes.
 *              This function moves forward, briefly stops at each node to adjust calibration,
 *              and then continues forward while reading sensor values.
 *              It adjusts the turn delay based on the TURN_DEMINISH constant for each node.
 *              Once the desired node configuration (center sensor active) is detected,
 *              it calls the navigate function to proceed further.
 * Example Call: NavigateAndCalibrateOnline_WithStop(100, 500, 2000, 300, 300);
 */

int NavigateAndCalibrateOnline_WithStop(int node_skip_delay, int node_turn_delay, int time_to_stop, int left_turn_delay, int right_turn_delay)
{
    volatile int sensorValue = 0;
    int cnt = 10;
    do
    {
        forward();
        stop(node_turn_delay);
        forward();
        sensorValue = readsensors();
        if (node_turn_delay > 10)
            node_turn_delay = node_turn_delay * TURN_DEMINISH;
    } while (((sensorValue & 0x7) != 0x2) && (cnt-- > 0));
    return navigate(node_skip_delay, time_to_stop, left_turn_delay, right_turn_delay);
}
/*
 * Function Name: int NavigateAndCalibrateOnline(int node_skip_delay, int node_turn_delay, int time_to_stop, int left_turn_delay, int right_turn_delay)
 * Input: node_skip_delay -> an integer representing the delay to skip a node
 *        node_turn_delay -> an integer representing the delay for turning at a node
 *        time_to_stop -> an integer representing the time to stop at a node
 *        left_turn_delay -> an integer representing the delay for a left turn
 *        right_turn_delay -> an integer representing the delay for a right turn
 * Output: Returns an integer value returned by the navigate function
 * Description: Navigates while calibrating the robot online.
 *              This function moves forward, briefly stops at each node to adjust calibration,
 *              and then continues forward while reading sensor values.
 *              It adjusts the turn delay based on the TURN_DEMINISH constant for each node.
 *              Once the desired node configuration (center sensor active) is detected,
 *              it calls the navigate function to proceed further.
 * Example Call: NavigateAndCalibrateOnline(100, 500, 2000, 300, 300);
 */
// edgeRightSensorValue << 4 | edgeLeftSensorValue << 3 | frontRightSensorValue << 2 | frontSensorValue << 1 | frontLeftSensorValue << 0)
int NavigateAndCalibrateOnline(int node_skip_delay, int node_turn_delay, int time_to_stop, int left_turn_delay, int right_turn_delay)
{
    volatile int sensorValue = 0;
    int cnt = 10;
    do
    {
        forward();
        stop(node_turn_delay);
        forward();
        sensorValue = readsensors();
        if (node_turn_delay > 10)
            node_turn_delay = node_turn_delay * TURN_DEMINISH;
    } while (((sensorValue & 0x7) != 0x2) && (cnt-- > 0));
    return navigate(node_skip_delay, time_to_stop, left_turn_delay, right_turn_delay);
}
/*
 * Function Name: int junction_uturn(int node_skip_delay, int node_turn_delay, int time_to_stop, int left_turn_delay, int right_turn_delay)
 * Input: node_skip_delay -> an integer representing the delay to skip a node
 *        node_turn_delay -> an integer representing the delay for turning at a node
 *        time_to_stop -> an integer representing the time to stop at a node
 *        left_turn_delay -> an integer representing the delay for a left turn
 *        right_turn_delay -> an integer representing the delay for a right turn
 * Output: Returns an integer value returned by the navigate function
 * Description: Performs a U-turn at a junction.
 *              This function initiates a left U-turn using the left_uturn() function,
 *              followed by a brief delay to complete the turn.
 *              It then sets the robot's direction to move forward and reads sensor values.
 * Example Call: junction_uturn(100, 500, 2000, 300, 300);
 */

int junction_uturn(int node_skip_delay, int node_turn_delay, int time_to_stop, int left_turn_delay, int right_turn_delay)
{
    volatile int sensorValue = 0;
    left_uturn();
    delay(node_turn_delay);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    forward();
    delay(300);
    sensorValue = readsensors();
    return navigate(node_skip_delay, time_to_stop, left_turn_delay, right_turn_delay);
}
/*
 * Function Name: int JunctionStop(int node_skip_delay, int node_turn_delay, int left_turn_delay, int right_turn_delay)
 * Input: node_skip_delay -> an integer representing the delay to skip a node
 *        node_turn_delay -> an integer representing the delay for turning at a node
 *        left_turn_delay -> an integer representing the delay for a left turn
 *        right_turn_delay -> an integer representing the delay for a right turn
 * Output: Returns 0 after performing junction stop actions
 * Description: Performs a stop action at a junction.
 *              This function stops the robot's motion, activates a red LED,
 *              and sounds a buzzer for a specified duration.
 *              After the specified delay, it deactivates the LED and buzzer.
 * Example Call: JunctionStop(100, 500, 300, 300);
 */
// edgeRightSensorValue << 4 | edgeLeftSensorValue << 3 | frontRightSensorValue << 2 | frontSensorValue << 1 | frontLeftSensorValue << 0)
int JunctionStop(int node_skip_delay, int node_turn_delay, int left_turn_delay, int right_turn_delay)
{
    stop(0);
    led_control(RED_LED, HIGH);
    buzzer_control(LOW);
    delay(3000);
    led_control(RED_LED, LOW);
    buzzer_control(HIGH);
    return 0;
}
/*
 * Function Name: int readsensors()
 * Input: None
 * Output: Returns an integer representing the combined sensor readings
 * Description: Reads sensor values from the infrared sensors placed on the robot.
 *              This function reads the digital values from the front left, front center,
 *              front right, left edge, and right edge sensors.
 *              It combines these values into a single integer representing the sensor readings.
 *              The bit positions in the returned integer represent sensor positions as follows:
 *              Bit 4: Right edge sensor
 *              Bit 3: Left edge sensor
 *              Bit 2: Front right sensor
 *              Bit 1: Front center sensor
 *              Bit 0: Front left sensor
 * Example Call: int sensorData = readsensors();
 */

int readsensors()
{
    volatile int frontLeftSensorValue = digitalRead(irPinFrontLeft);
    volatile int frontSensorValue = digitalRead(irPinFrontCenter);
    volatile int frontRightSensorValue = digitalRead(irPinFrontRight);

    volatile int edgeLeftSensorValue = digitalRead(irPinEdgeLeft);
    volatile int edgeRightSensorValue = digitalRead(irPinEdgeRight);

    return (edgeRightSensorValue << 4 | edgeLeftSensorValue << 3 | frontRightSensorValue << 2 | frontSensorValue << 1 | frontLeftSensorValue << 0);
}

unsigned long prev_junction_time = 0;
/*
 * Function Name: int navigate(int node_skip_delay, int time_to_stop, int left_turn_delay, int right_turn_delay)
 * Input: node_skip_delay -> an integer representing the delay to skip a node
 *        time_to_stop -> an integer representing the time to stop at a node
 *        left_turn_delay -> an integer representing the delay for a left turn
 *        right_turn_delay -> an integer representing the delay for a right turn
 * Output: Returns 1 when a junction is triggered; otherwise, 0
 * Description: Navigates the robot based on sensor readings while calibrating online.
 *              This function continuously reads sensor values and adjusts robot movement accordingly.
 *              It implements different actions based on various sensor configurations to navigate through the environment.
 *              The function includes logic to handle junction triggers and stops.
 *              It also considers time constraints for stopping at nodes.
 * Example Call: int triggered = navigate(100, 2000, 300, 300);
 */

int navigate(int node_skip_delay, int time_to_stop, int left_turn_delay, int right_turn_delay)
{
    unsigned long current_time = 0;
    unsigned long start_time = millis();
    int junction_trigger = 0;
    volatile int sensorValue = readsensors();
    forward();

    while (!junction_trigger)
    {
        sensorValue = readsensors();

        switch (sensorValue & 0x1F)
        {        // edgeRightSensorValue	edgeLeftSensorValue	frontRightSensorValue	frontSensorValue	frontLeftSensorValue
                 // x x _ 1 _
        case 0:  // 0	0	0	0	0
        case 24: // 1	1	0	0	0
            forward();
            break;
        case 1:  // 0	0	0	0	1
        case 25: // 1	1	0	0	1
            LeftAndCalibrateOnline();
            forward();
            break;
        case 2:  // 0	0	0	1	0
        case 26: // 1	1	0	1	0
            forward();
            break;
        case 3:  // 0	0	0	1	1
        case 27: // 1	1	0	1	1
            LeftAndCalibrateOnlineAlign();
            forward();
            break;
        case 4:  // 0	0	1	0	0
        case 28: // 1	1	1	0	0
            RightAndCalibrateOnline();
            forward();
            break;
        case 5:  // 0	0	1	0	1 // align to the center line
        case 29: // 1	1	1	0	1 // align to the center line
            if ((current_time - prev_junction_time) > 1500)
            {
                junction_trigger = 1;
                prev_junction_time = millis();
            }
            break;
        case 6:  // 0	0	1	1	0
        case 30: // 1	1	1	1	0
            RightAndCalibrateOnlineAlign();
            forward();
            break;
        case 7:  // 0	0	1	1	1
        case 31: // 1	1	1	1	1
            if ((current_time - prev_junction_time) > 1500)
            {
                junction_trigger = 1;
                prev_junction_time = millis();
            }
            break;

        case 10: // 0	1	0	1	0
            forward();
            break;

        case 8:  // 0	1	0	0	0
        case 9:  // 0	1	0	0	1
        case 12: // 0	1	1	0	0
        case 13: // 0	1	1	0	1
            right();
            delay(right_turn_delay); // 10
            if (right_turn_delay > 500)
                right_turn_delay = 10;
            forward();
            break;
        case 11: // 0	1	0	1	1
        case 14: // 0	1	1	1	0
        case 15: // 0	1	1	1	1
            if ((current_time - prev_junction_time) > 1500)
            {
                junction_trigger = 1;
                prev_junction_time = millis();
            }
            break;

        case 18: // 1	0	0	1	0
            forward();
            break;

        case 16: // 1	0	0	0	0
        case 17: // 1	0	0	0	1
        case 20: // 1	0	1	0	0
        case 21: // 1	0	1	0	1
            left();
            delay(left_turn_delay); // 2
            if (left_turn_delay > 500)
                left_turn_delay = 10;
            forward();
            break;

        case 19: // 1	0	0	1	1
        case 22: // 1	0	1	1	0
        case 23: // 1	0	1	1	1
            if ((current_time - prev_junction_time) > 1500)
            {
                junction_trigger = 1;
                prev_junction_time = millis();
            }
            break;

        default:
            stop(0);
            break;
        }

        current_time = millis();
        if ((time_to_stop != 0) && ((current_time - start_time) > time_to_stop)) // time_to_stop is in milliseconds = 3000
        {
            prev_junction_time = millis();
            junction_trigger = 1;
            stop(0);
            buzzer_control(LOW);
            delay(1000);
            buzzer_control(HIGH);
        }
    }

    if (junction_trigger == 1)
    {

        stop(node_skip_delay);
        delay(100);
    }

    return junction_trigger;
}
/*
 * Function Name: void initialise_network()
 * Input: None
 * Output: None
 * Description: Initializes the network connection for the robot.
 *              This function attempts to connect to the specified WiFi network using provided credentials.
 *              It waits until the connection is established and then prints the local IP address.
 * Example Call: initialise_network();
 */

void initialise_network()
{
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.println("...");
    }

    Serial.print("WiFi connected with IP: ");
    Serial.println(WiFi.localIP());
}
/*
 * Function Name: void setup()
 * Input: None
 * Output: None
 * Description: Setup function for initializing the robot.
 *              This function calls various initialization functions to set up the sensors, motors, buzzer, LED, and network connection.
 *              It also stops the robot initially.
 *              Uncomment the Serial.begin(115200) line if serial communication is needed for debugging.
 * Example Call: setup();
 */
void setup()
{
    initializeSensor();
    initializeMotor();
    stop(0);
    initializeBuzzer();
    initializeLed();
    // Serial.begin(115200);
    initialise_network();
}
/*
 * Structure Name: arena_node
 * Description: Represents a node in the robot's arena with route information and related parameters.
 * Fields:
 *      - route_id: Identifier for the route associated with the node (e.g., AB, CD, EF).
 *      - prev_nid: Identifier for the previous node in the route.
 *      - cur_self: Identifier for the current node (junction id).
 *      - dst_nid: Identifier for the destination node in the route.
 *      - fn_nextpath: Pointer to a function that defines the next path/action for the robot based on parameters.
 *      - node_skip_delay: Delay duration for skipping a node.
 *      - node_turn_delay: Delay duration for turning at a node.
 *      - time_to_stop: Duration for stopping at a node.
 *      - left_turn_delay: Delay duration for left turn.
 *      - right_turn_delay: Delay duration for right turn.
 * Usage: This structure is used in defining the array of all_possible_path, which contains various paths and associated parameters for the robot to navigate the arena.
 */
typedef struct arena_node
{
    char *route_id; // e.g., AB, CD, EF
    char *prev_nid;
    char *cur_self; // junction id
    char *dst_nid;
    int (*fn_nextpath)(int node_skip_delay, int node_turn_delay, int time_to_stop, int left_turn_delay, int right_turn_delay);
    int node_skip_delay;
    int node_turn_delay;
    int time_to_stop;
    int left_turn_delay;
    int right_turn_delay;
} PATH;
// all possible path that the rover can take in the arena is defined here

PATH all_possible_path[] = {
    {.route_id = "_SA", .prev_nid = "_", .cur_self = "S", .dst_nid = "A", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 300, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "SA1", .prev_nid = "S", .cur_self = "A", .dst_nid = "1", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 2000, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "1AS", .prev_nid = "1", .cur_self = "A", .dst_nid = "S", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 1500, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "SAH", .prev_nid = "S", .cur_self = "A", .dst_nid = "H", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "HAS", .prev_nid = "H", .cur_self = "A", .dst_nid = "S", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 2000, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "SAB", .prev_nid = "S", .cur_self = "A", .dst_nid = "B", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 100, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "BAS", .prev_nid = "B", .cur_self = "A", .dst_nid = "S", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 30, .time_to_stop = 1500, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "HAB", .prev_nid = "H", .cur_self = "A", .dst_nid = "B", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "BAH", .prev_nid = "B", .cur_self = "A", .dst_nid = "H", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "1AB", .prev_nid = "1", .cur_self = "A", .dst_nid = "B", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "BA1", .prev_nid = "B", .cur_self = "A", .dst_nid = "1", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 1800, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "ABC", .prev_nid = "A", .cur_self = "B", .dst_nid = "C", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "CBA", .prev_nid = "C", .cur_self = "B", .dst_nid = "A", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "GBA", .prev_nid = "G", .cur_self = "B", .dst_nid = "A", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 80, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "ABG", .prev_nid = "A", .cur_self = "B", .dst_nid = "G", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "GBC", .prev_nid = "G", .cur_self = "B", .dst_nid = "C", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "CBG", .prev_nid = "C", .cur_self = "B", .dst_nid = "G", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "BC4", .prev_nid = "B", .cur_self = "C", .dst_nid = "4", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 1500, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "4CB", .prev_nid = "4", .cur_self = "C", .dst_nid = "B", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "BCD", .prev_nid = "B", .cur_self = "C", .dst_nid = "D", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "DCB", .prev_nid = "D", .cur_self = "C", .dst_nid = "B", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "DC4", .prev_nid = "D", .cur_self = "C", .dst_nid = "4", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 2000, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "4CD", .prev_nid = "4", .cur_self = "C", .dst_nid = "D", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "BCF", .prev_nid = "B", .cur_self = "C", .dst_nid = "F", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "FCB", .prev_nid = "F", .cur_self = "C", .dst_nid = "B", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "FCD", .prev_nid = "F", .cur_self = "C", .dst_nid = "D", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "DCF", .prev_nid = "D", .cur_self = "C", .dst_nid = "F", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "4FE", .prev_nid = "4", .cur_self = "F", .dst_nid = "E", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "EF4", .prev_nid = "E", .cur_self = "F", .dst_nid = "4", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 0, .node_turn_delay = 0, .time_to_stop = 4000, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "CDE", .prev_nid = "C", .cur_self = "D", .dst_nid = "E", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "EDC", .prev_nid = "E", .cur_self = "D", .dst_nid = "C", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "CD5", .prev_nid = "C", .cur_self = "D", .dst_nid = "5", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 4500, .left_turn_delay = 10, .right_turn_delay = 1100},
    {.route_id = "5DC", .prev_nid = "5", .cur_self = "D", .dst_nid = "C", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "CDK", .prev_nid = "C", .cur_self = "D", .dst_nid = "K", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "KDC", .prev_nid = "K", .cur_self = "D", .dst_nid = "C", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "5KE", .prev_nid = "5", .cur_self = "K", .dst_nid = "E", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "EK5", .prev_nid = "E", .cur_self = "K", .dst_nid = "5", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 0, .node_turn_delay = 0, .time_to_stop = 2000, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "DEK", .prev_nid = "D", .cur_self = "E", .dst_nid = "K", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "KED", .prev_nid = "K", .cur_self = "E", .dst_nid = "D", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "EDK", .prev_nid = "E", .cur_self = "D", .dst_nid = "K", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "KDE", .prev_nid = "K", .cur_self = "D", .dst_nid = "E", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "EKD", .prev_nid = "E", .cur_self = "K", .dst_nid = "D", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "DKE", .prev_nid = "D", .cur_self = "K", .dst_nid = "E", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "4F3", .prev_nid = "4", .cur_self = "F", .dst_nid = "3", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 50, .node_turn_delay = 50, .time_to_stop = 3400, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "3F4", .prev_nid = "3", .cur_self = "F", .dst_nid = "4", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 300, .node_turn_delay = 200, .time_to_stop = 4000, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "4FG", .prev_nid = "4", .cur_self = "F", .dst_nid = "G", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "GF4", .prev_nid = "G", .cur_self = "F", .dst_nid = "4", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 0, .node_turn_delay = 0, .time_to_stop = 4000, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "FEK", .prev_nid = "F", .cur_self = "E", .dst_nid = "K", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "KEF", .prev_nid = "K", .cur_self = "E", .dst_nid = "F", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "F3J", .prev_nid = "F", .cur_self = "3", .dst_nid = "J", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "J3F", .prev_nid = "J", .cur_self = "3", .dst_nid = "F", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "FGB", .prev_nid = "F", .cur_self = "G", .dst_nid = "B", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "BGF", .prev_nid = "B", .cur_self = "G", .dst_nid = "F", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "FG2", .prev_nid = "F", .cur_self = "G", .dst_nid = "2", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 3500, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "2GF", .prev_nid = "2", .cur_self = "G", .dst_nid = "F", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "FGH", .prev_nid = "F", .cur_self = "G", .dst_nid = "H", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "HGF", .prev_nid = "H", .cur_self = "G", .dst_nid = "F", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "FGI", .prev_nid = "F", .cur_self = "G", .dst_nid = "I", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "IGF", .prev_nid = "I", .cur_self = "G", .dst_nid = "F", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "FJK", .prev_nid = "F", .cur_self = "J", .dst_nid = "K", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "KJF", .prev_nid = "K", .cur_self = "J", .dst_nid = "F", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "FJI", .prev_nid = "F", .cur_self = "J", .dst_nid = "I", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "IJF", .prev_nid = "I", .cur_self = "J", .dst_nid = "F", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "3FG", .prev_nid = "3", .cur_self = "F", .dst_nid = "G", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 180, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "GF3", .prev_nid = "G", .cur_self = "F", .dst_nid = "3", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 3000, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "3FE", .prev_nid = "3", .cur_self = "F", .dst_nid = "E", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "EF3", .prev_nid = "E", .cur_self = "F", .dst_nid = "3", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 0, .node_turn_delay = 0, .time_to_stop = 4000, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "EFJ", .prev_nid = "E", .cur_self = "F", .dst_nid = "J", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "JFE", .prev_nid = "J", .cur_self = "F", .dst_nid = "E", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "EKJ", .prev_nid = "E", .cur_self = "K", .dst_nid = "J", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "JKE", .prev_nid = "J", .cur_self = "K", .dst_nid = "E", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "3JK", .prev_nid = "3", .cur_self = "J", .dst_nid = "K", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "KJ3", .prev_nid = "K", .cur_self = "J", .dst_nid = "3", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 0, .node_turn_delay = 0, .time_to_stop = 3000, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "3JI", .prev_nid = "3", .cur_self = "J", .dst_nid = "I", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "IJ3", .prev_nid = "I", .cur_self = "J", .dst_nid = "3", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 200, .time_to_stop = 3000, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "5DE", .prev_nid = "5", .cur_self = "D", .dst_nid = "E", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "ED5", .prev_nid = "E", .cur_self = "D", .dst_nid = "5", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 0, .node_turn_delay = 0, .time_to_stop = 2000, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "JFC", .prev_nid = "J", .cur_self = "F", .dst_nid = "C", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "CFJ", .prev_nid = "C", .cur_self = "F", .dst_nid = "J", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "JFG", .prev_nid = "J", .cur_self = "F", .dst_nid = "G", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "GFJ", .prev_nid = "G", .cur_self = "F", .dst_nid = "J", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "JK5", .prev_nid = "J", .cur_self = "K", .dst_nid = "5", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 0, .node_turn_delay = 0, .time_to_stop = 13000, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "5KJ", .prev_nid = "5", .cur_self = "K", .dst_nid = "J", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "JI2", .prev_nid = "J", .cur_self = "I", .dst_nid = "2", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 2400, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "2IJ", .prev_nid = "2", .cur_self = "I", .dst_nid = "J", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "JIG", .prev_nid = "J", .cur_self = "I", .dst_nid = "G", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "GIJ", .prev_nid = "G", .cur_self = "I", .dst_nid = "J", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "I2G", .prev_nid = "I", .cur_self = "2", .dst_nid = "G", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "G2I", .prev_nid = "G", .cur_self = "2", .dst_nid = "I", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "IGB", .prev_nid = "I", .cur_self = "G", .dst_nid = "B", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "BGI", .prev_nid = "B", .cur_self = "G", .dst_nid = "I", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "IGH", .prev_nid = "I", .cur_self = "G", .dst_nid = "H", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "HGI", .prev_nid = "H", .cur_self = "G", .dst_nid = "I", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "2GH", .prev_nid = "2", .cur_self = "G", .dst_nid = "H", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 250, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "HG2", .prev_nid = "H", .cur_self = "G", .dst_nid = "2", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 3000, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "2GB", .prev_nid = "2", .cur_self = "G", .dst_nid = "B", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "BG2", .prev_nid = "B", .cur_self = "G", .dst_nid = "2", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 0, .node_turn_delay = 0, .time_to_stop = 3900, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "2IH", .prev_nid = "2", .cur_self = "I", .dst_nid = "H", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "HI2", .prev_nid = "H", .cur_self = "I", .dst_nid = "2", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 0, .node_turn_delay = 0, .time_to_stop = 3000, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "FED", .prev_nid = "F", .cur_self = "E", .dst_nid = "D", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "DEF", .prev_nid = "D", .cur_self = "E", .dst_nid = "F", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "CFE", .prev_nid = "C", .cur_self = "F", .dst_nid = "E", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "EFC", .prev_nid = "E", .cur_self = "F", .dst_nid = "C", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "GFE", .prev_nid = "G", .cur_self = "F", .dst_nid = "E", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "EFG", .prev_nid = "E", .cur_self = "F", .dst_nid = "G", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "GFC", .prev_nid = "G", .cur_self = "F", .dst_nid = "C", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "CFG", .prev_nid = "C", .cur_self = "F", .dst_nid = "G", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "HGB", .prev_nid = "H", .cur_self = "G", .dst_nid = "B", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "BGH", .prev_nid = "B", .cur_self = "G", .dst_nid = "H", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "AHG", .prev_nid = "A", .cur_self = "H", .dst_nid = "G", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "GHA", .prev_nid = "G", .cur_self = "H", .dst_nid = "A", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "IHG", .prev_nid = "I", .cur_self = "H", .dst_nid = "G", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "GHI", .prev_nid = "G", .cur_self = "H", .dst_nid = "I", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "IHA", .prev_nid = "I", .cur_self = "H", .dst_nid = "A", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "AHI", .prev_nid = "A", .cur_self = "H", .dst_nid = "I", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "IJK", .prev_nid = "I", .cur_self = "J", .dst_nid = "K", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "KJI", .prev_nid = "K", .cur_self = "J", .dst_nid = "I", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 260, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "JKD", .prev_nid = "J", .cur_self = "K", .dst_nid = "D", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "DKJ", .prev_nid = "D", .cur_self = "K", .dst_nid = "J", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "HIJ", .prev_nid = "H", .cur_self = "I", .dst_nid = "J", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "JIH", .prev_nid = "J", .cur_self = "I", .dst_nid = "H", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "GIH", .prev_nid = "G", .cur_self = "I", .dst_nid = "H", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 0, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "HIG", .prev_nid = "H", .cur_self = "I", .dst_nid = "G", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "GH1", .prev_nid = "G", .cur_self = "H", .dst_nid = "1", .fn_nextpath = JunctionRightAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 3500, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "1HG", .prev_nid = "1", .cur_self = "H", .dst_nid = "G", .fn_nextpath = JunctionLeftAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 180, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "IH1", .prev_nid = "I", .cur_self = "H", .dst_nid = "1", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 0, .node_turn_delay = 0, .time_to_stop = 4000, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "1HI", .prev_nid = "1", .cur_self = "H", .dst_nid = "I", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "A1H", .prev_nid = "A", .cur_self = "1", .dst_nid = "H", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "H1A", .prev_nid = "H", .cur_self = "1", .dst_nid = "A", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "C4F", .prev_nid = "C", .cur_self = "4", .dst_nid = "F", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "F4C", .prev_nid = "F", .cur_self = "4", .dst_nid = "C", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "D5K", .prev_nid = "D", .cur_self = "5", .dst_nid = "K", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 600},
    {.route_id = "K5D", .prev_nid = "K", .cur_self = "5", .dst_nid = "D", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 0, .right_turn_delay = 10},

    {.route_id = "CF3", .prev_nid = "C", .cur_self = "F", .dst_nid = "3", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 3500, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "3FC", .prev_nid = "3", .cur_self = "F", .dst_nid = "C", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 160, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "4FJ", .prev_nid = "4", .cur_self = "F", .dst_nid = "J", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "JF4", .prev_nid = "J", .cur_self = "F", .dst_nid = "4", .fn_nextpath = NavigateAndCalibrateOnline, .node_skip_delay = 200, .node_turn_delay = 0, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "A1A", .prev_nid = "A", .cur_self = "1", .dst_nid = "A", .fn_nextpath = junction_uturn, .node_skip_delay = 0, .node_turn_delay = 1400, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "H1H", .prev_nid = "H", .cur_self = "1", .dst_nid = "H", .fn_nextpath = junction_uturn, .node_skip_delay = 0, .node_turn_delay = 1600, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "G2G", .prev_nid = "G", .cur_self = "2", .dst_nid = "G", .fn_nextpath = junction_uturn, .node_skip_delay = 200, .node_turn_delay = 1400, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "I2I", .prev_nid = "I", .cur_self = "2", .dst_nid = "I", .fn_nextpath = junction_uturn, .node_skip_delay = 0, .node_turn_delay = 1600, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "C4C", .prev_nid = "C", .cur_self = "4", .dst_nid = "C", .fn_nextpath = junction_uturn, .node_skip_delay = 0, .node_turn_delay = 1600, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "F4F", .prev_nid = "F", .cur_self = "4", .dst_nid = "F", .fn_nextpath = junction_uturn, .node_skip_delay = 0, .node_turn_delay = 1600, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "F3F", .prev_nid = "F", .cur_self = "3", .dst_nid = "F", .fn_nextpath = junction_uturn, .node_skip_delay = 0, .node_turn_delay = 1500, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},
    {.route_id = "J3J", .prev_nid = "J", .cur_self = "3", .dst_nid = "J", .fn_nextpath = junction_uturn, .node_skip_delay = 0, .node_turn_delay = 1600, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 10},

    {.route_id = "D5D", .prev_nid = "D", .cur_self = "5", .dst_nid = "D", .fn_nextpath = junction_uturn, .node_skip_delay = 0, .node_turn_delay = 1600, .time_to_stop = 0, .left_turn_delay = 300, .right_turn_delay = 10},
    {.route_id = "K5K", .prev_nid = "K", .cur_self = "5", .dst_nid = "K", .fn_nextpath = junction_uturn, .node_skip_delay = 200, .node_turn_delay = 1400, .time_to_stop = 0, .left_turn_delay = 10, .right_turn_delay = 300},

};
// final planned path is stored in the path_task_4b array for the rover to navigate
PATH path_task_4b[100] = {};

int processCharArray(const char arr[]);
/*
 * Function Name: int processCharArray(const char arr[])
 * Input: arr -> a constant character array (C-string)
 * Output: Returns the number of substrings processed
 * Description: Processes a character array to extract substrings and match them with predefined routes.
 *              This function takes a character array as input and extracts substrings of length 3.
 *              It then compares each substring with predefined route IDs stored in the 'all_possible_path' array.
 *              If a match is found, the corresponding route information is stored in the 'path_task_4b' array.
 *              If no match is found, the function triggers a buzzer alarm.
 *              Finally, it prints the extracted route IDs to Serial for debugging.
 * Example Call: int numSubstrings = processCharArray("ABCDEF");
 */

int processCharArray(const char arr[])
{
    int length = strlen(arr);
    int numSubstrings = length - 2;
    const int substringLength = 3;
    char substrings[numSubstrings][substringLength + 1];
    for (int i = 0; i < numSubstrings; i++)
    {
        for (int j = 0; j < substringLength; j++)
        {
            substrings[i][j] = arr[i + j];
        }

        substrings[i][substringLength] = '\0';
    }
    int path_steps = sizeof(all_possible_path) / sizeof(all_possible_path[0]);

    for (int i = 0; i < numSubstrings; i++)
    {
        for (int j = 0; j < path_steps; j++)
        {
            if (strcmp(substrings[i], all_possible_path[j].route_id) == 0)
            {
                path_task_4b[i] = all_possible_path[j];
                break;
            }
            if (j == path_steps)
            {
                for (int k = 0; k < 5; k++)
                {
                    buzzer_control(LOW);
                    delay(100);
                    buzzer_control(HIGH);
                    delay(100);
                }
            }
        }
    }
    for (int i = 0; i < numSubstrings; i++)
    {
        Serial.println(path_task_4b[i].route_id);
    }

    return numSubstrings;
}
String msg;
/*
 * Function Name: void loop()
 * Input: None
 * Output: None
 * Description: Main loop function for the robot's operation.
 *              This function attempts to establish a connection with a host.
 *              Upon successful connection, it reads a message from the host.
 *              The message is then processed to extract route information using the processCharArray function.
 *              The extracted route information is used to execute corresponding actions.
 *              After executing all actions, the function triggers LED and buzzer signals for a brief period.
 *              This function relies on the availability of network communication functionalities,
 *              a message received from the host, and defined route processing logic.
 * Example Call: loop();
 */

void loop()
{

    while (!client.connect(host, port))
    {
        // Serial.println("Connection to host failed");
        delay(200);
    }
    // Serial.println("Connected");

    msg = client.readStringUntil('\r');
    Serial.println(msg);

    int path_size_all_array = (sizeof(all_possible_path) / sizeof(PATH));
    char arr[msg.length() + 1];
    msg.toCharArray(arr, msg.length() + 1);

    int path_size_final_array = processCharArray(arr);

    delay(2000);
    // led_control(RED_LED, HIGH);
    delay(0);
    // led_control(RED_LED, LOW);
    // buzzer_control(HIGH);
    Serial.println(path_size_final_array);
    int i = 0;
    for (i = 0; i < path_size_final_array; i++)
    {
        PATH *path = &path_task_4b[i];
        path->fn_nextpath(path->node_skip_delay, path->node_turn_delay, path->time_to_stop, path->left_turn_delay, path->right_turn_delay);
    }
    led_control(RED_LED, HIGH);
    buzzer_control(LOW);
    delay(4000);
    led_control(RED_LED, LOW);
    buzzer_control(HIGH);
    delay(100000);
}
