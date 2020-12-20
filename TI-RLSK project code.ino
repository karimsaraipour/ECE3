#include <ECE3.h>

//constants, bools, and arrays
uint16_t sensorValues[8];                                         // right -> left, 0 -> 7
float offsetValues[8] = {713, 760, 666, 736, 574, 643, 666, 736}; // values depend on car
int weights[8] = {-8, -4, -2, -1, 1, 2, 4, 8};

//modifiable values
int referenceSpeed = 95;

int leftSpeed = referenceSpeed;
int rightSpeed = referenceSpeed;
int driftLeft = leftSpeed * 2;
int driftRight = rightSpeed * 2;
float k_p = 3.9;
float k_d = 22.5;

//other variables
bool lineFound{};
int error{}, average_error{}, error_derivative{}, lasterror{}, weightedSum{}, count{};

//pins
const int left_nslp_pin = 31;  // nslp ==> awake & ready for PWM
const int right_nslp_pin = 11; //motor on/off
const int left_dir_pin = 29;   //motor direction
const int right_dir_pin = 30;
const int left_pwm_pin = 40; //motor speed
const int right_pwm_pin = 39;
const int LED_RF = 41;

//prototypes
bool black(uint16_t sensorValues[]);
void turnAround();
void completion();
void findLine();

void setup()
{
    // put your setup code here, to run once:
    pinMode(left_nslp_pin, OUTPUT);
    pinMode(left_dir_pin, OUTPUT);
    pinMode(left_pwm_pin, OUTPUT);
    pinMode(right_nslp_pin, OUTPUT);
    pinMode(right_dir_pin, OUTPUT);
    pinMode(right_pwm_pin, OUTPUT);

    digitalWrite(left_dir_pin, LOW);
    digitalWrite(left_nslp_pin, HIGH);
    digitalWrite(right_dir_pin, LOW);
    digitalWrite(right_nslp_pin, HIGH);

    pinMode(LED_RF, OUTPUT);
    ECE3_Init();

    // set the data rate in bits/second for serial data transmission
    Serial.begin(9600);
}

void loop()
{
    processSensorData();
    if (lineFound)
        moveCar();
    else
        findLine();
}

bool black(uint16_t sensorValues[])
{
    for (int i = 1; i < 7; i++)
        if (sensorValues[i] < 970)
            return false;
    return true;
}

void turnAround()
{
    count++;

    digitalWrite(29, HIGH);

    analogWrite(40, 220);
    analogWrite(39, 220);
    delay(200);

    digitalWrite(29, LOW);
    analogWrite(40, leftSpeed);
    analogWrite(39, rightSpeed);
    delay(75);
}

void findLine()
{
    analogWrite(left_pwm_pin, 80);
    analogWrite(right_pwm_pin, 80);
    if (sensorValues[4] > 800)
    {
        lineFound = true;
        return;
    }
    else
        return;
}

void processSensorData()
{
    ECE3_read_IR(sensorValues); // read values
    weightedSum = 0;            // reset sensor fusion

    // calibrate raw data
    for (unsigned char i = 0; i < 8; i++)
    {
        int offset = sensorValues[i] - offsetValues[i];
        offset = (offset < 0) ? 0 : offset;

        float divisor = ((2500 - offsetValues[i]) / 1000);
        int scaledValue = (offset / divisor);
        sensorValues[i] = scaledValue;
    }

    //update sensor fusion (weighted sum)
    for (unsigned char i = 0; i < 8; i++)
        weightedSum += (sensorValues[i] * weights[i]);
}

void moveCar()
{
    if (black(sensorValues))
        turnAround();
    else
    {
        //PID

        error = weightedSum;
        error_derivative = error - lasterror;
        lasterror = error;
        int pid = (k_p * error) + (k_d * error_derivative);
        leftSpeed = referenceSpeed - pid / referenceSpeed;
        rightSpeed = referenceSpeed + pid / referenceSpeed;

        //other controllers

        //proportional control

        //error = weightedSum;
        //leftSpeed = referenceSpeed - k_p * error; // could be + or -
        //rightSpeed = referenceSpeed + k_p * error; // could be + or -

        //brute force

        //bool leftError = sensorValues[5] > 100 || sensorValues[6] > 100 || sensorValues[7] > 100;
        //bool rightError = sensorValues[2] > 100 || sensorValues[1] > 100 || sensorValues[0] > 100;
        //rightSpeed = (leftError) ? driftRight : referenceSpeed;
        //leftSpeed = (rightError) ? driftLeft : referenceSpeed;
    }

    if (count == 2)
    {
        analogWrite(left_pwm_pin, 0);
        analogWrite(right_pwm_pin, 0);
    }
    else
    {
        analogWrite(left_pwm_pin, leftSpeed);
        analogWrite(right_pwm_pin, rightSpeed);

        digitalWrite(LED_RF, HIGH);
    }
}
