# **ECE 4180 Final Project**
Developed by Aditya Ghatge, Kabir Jain, and Yash Vardhan

# mbed Sound Bot
## Introduction

This notebook explores the design of a sound activated robot which leverages the mbed LPC1768 microcontroller and bluetooth software capabilites. The robot's commercial use case would be for individuals with disabilities preventing them from walking. By using the robot, individuals can call out for and control; it improves their quality of life. Features implemented include the ability for the robot to move towards sound on command, a manual override mode, collision prevention technology, and viewing of peripheral data on an LCD screen. The robot is fully controlled using bluetooth via the Adafruit Bluefruit app.

### Design Block Diagram
 ![Picture of Diagram](https://github.com/adityaghatge/Sound-Bot/blob/main/Screenshot%202024-05-01%20194921.png?raw=true)

### Picture of Design

 ![Picture of Robot](https://github.com/adityaghatge/Sound-Bot/blob/main/IMG_2621.jpg?raw=true)


## Components

Our design made use of the following components:

- mbed Microcontroller (LPC1768) https://os.mbed.com/platforms/mbed-LPC1768/
- Adafruit Bluetooth Module https://os.mbed.com/users/4180_1/notebook/adafruit-bluefruit-le-uart-friend---bluetooth-low-/
- uLCD Screen https://os.mbed.com/users/4180_1/notebook/ulcd-144-g2-128-by-128-color-lcd/
- 5V Power Supply (and adapter jack)
- Adafruit MEMS Microphone Breakout (3) https://os.mbed.com/components/Adafruit-MEMS-Microphone-Breakout-SPW243/
- HC-SR04 Ultrasonic Sensor https://os.mbed.com/components/HC-SR04/
- ROB-13302 Hobby Gearmotor https://os.mbed.com/cookbook/Motor
- ROB-12629 Wheel Encoder Kit
- Shadow Chassis

### Pin Connection Table
LPC1768 Pin	| Component	 | Pin Name	| Misc.
| ----------- | ----------- | ----------- | ----------- |
| p15 | Adafruit MEMS Microphone(1) | DC | - |
| - | Adafruit MEMS Microphone(1) | Vin | 5V |
| GND | Adafruit MEMS Microphone(1) | GND | - |
| p16 | Adafruit MEMS Microphone(2) | DC | - |
| - | Adafruit MEMS Microphone(2) | Vin | 5V |
| GND | Adafruit MEMS Microphone(2) | GND | - |
| p17 | Adafruit MEMS Microphone(3) | DC | - |
| - | Adafruit MEMS Microphone(3) | Vin | 5V |
| GND | Adafruit MEMS Microphone(3) | GND | - |
| p4 | uLCD Screen | RX | - |
| p13 | uLCD Screen | TX | - |
| p12 | uLCD Screen | Reset | - |
| - | uLCD Screen | 5V | 5V Supply |
| GND | uLCD Screen | GND | - |
| p27 | Bluetooth Module | RXI | - |
| p28 | Bluetooth Module | TXO | - |
| - | Bluetooth Module | Vin | 5V Supply |
| GND | Bluetooth Module | CTS | - |
| GND | Bluetooth Module | GND | - |
| p9 | ROB-12629 Wheel Encoder Kit(1) | OUT | - |
| - | ROB-12629 Wheel Encoder Kit(1) | 3V | - |
| GND | ROB-12629 Wheel Encoder Kit(1) | GND | - |
| p10 | ROB-12629 Wheel Encoder Kit(2) | OUT | - |
| - | ROB-12629 Wheel Encoder Kit(2) | 3V | - |
| GND | ROB-12629 Wheel Encoder Kit(2) | GND | - |
| p21 | ROB-13302 Hobby Gearmotor | pwm | - |
| p22 | ROB-13302 Hobby Gearmotor | fwd | - |
| p23 | ROB-13302 Hobby Gearmotor | rev | - |
| - | ROB-13302 Hobby Gearmotor | 5V Supply | 5V |
| p6 | HC-SR04 Ultrasonic Sensor | Trig | - |
| p7 | HC-SR04 Ultrasonic Sensor | Echo | - |
| - | HC-SR04 Ultrasonic Sensor | 5V Supply | 5V |
| GND | HC-SR04 Ultrasonic Sensor | GND | - |


The HC-SR04 Ultrasonic Sensor detects objects at a distance and has a range between 2mm and 400mm. We used this sensor to detect objects ahead of the robot. If there was an object at 10mm away, we send a signal to stop the motors and display a warning on the LCD screen. Once stopped, a manual override is required to get it moving.

The Bluetooth control is accomplished using the Adafruit Bluetooth module and is controlled using the Bluefruit app. The interface we decided to use was the control pad. When '1' is pressed, it is switched into manual override mode. When in manual override mode, the D-pad can be used to control the robot directly. If there is an object in front of it, the robot will not move forward and will display a warning on the LCD screen. When '2' is pressed, it switches to sound mode. When in sound mode, it takes samples of the ambiance of its surroundings. Then upon '3' being pressed, it listens to surrounds and moves in the direction of sound.

The LCD screen is used to display the mode it is in, if it is in sampling mode, the warning for a close object, distance to closest object, and errors.


## Software Used

We make use of the “HALLFX_ENCODER” library https://os.mbed.com/users/electromotivated/code/HALLFX_ENCODER/ with modifications to better control the encoders. We also used the "hcsr04" library to use the ultrasonic sensor without interrupts within our threads.

The code for our sound bot is shown below:

```
{
    #include "mbed.h"
#include "ultrasonic.h"
#include "Motor.h"
#include "HALLFX_ENCODER.h"
#include "rtos.h"
#include "uLCD_4DGL.h"
#include "hcsr04.h"
#include <cstdio>

Motor m1(p21, p22, p23); // pwm, fwd, rev
Motor m2(p21, p24, p25);
HALLFX_ENCODER EncL(p9);
HALLFX_ENCODER EncR(p10);
Serial blue(p28, p27);
uLCD_4DGL lcd(p13, p14, p12);
Mutex lcd_mutex;
BusOut myleds(LED1, LED2, LED3, LED4);
HCSR04 mu(p6, p7); 


// Max function macro
#define max(a, b) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

// Microphone class, taken from the cookbook
class microphone
{
public:
    microphone(PinName pin);
    float read();
    operator float();

private:
    AnalogIn _pin;
};
microphone::microphone(PinName pin) : _pin(pin)
{
}
float microphone::read()
{
    return _pin.read();
}
inline microphone::operator float()
{
    return _pin.read();
}
microphone mic1(p15);
microphone mic2(p16);
microphone mic3(p17);

volatile int manual_mode = 4;
volatile bool button_ready = 0;
volatile int bnum = 0;
volatile int bhit;
volatile int global_no = 0;
float speed;
float k = .009;
// Mic constants for calibration purposes
volatile float mic1s;
volatile float mic2s;
volatile float mic3s;
// Used during the actual measuring
volatile float mic1_val;
volatile float mic2_val;
volatile float mic3_val;
volatile float mic1_max;
volatile float mic2_max;
volatile float mic3_max;

// Distances from the ultrasound, current and previous distance, updated each iteration
volatile float d, temp_d;

// Mutexes for mic access, serial device access, and the ultrasound sensor
Mutex mic;
Mutex serial;
Mutex d_mutex;

// Function to sample mics at the start of the program execution
void sampleMics(void const *args)
{
    while (1)
    {   
        // flag used to turn off all other threads using serial devices
        if (!global_no)
        {
            
            mic.lock(); // lock the mic mutex
            float mic1_roll = 0.0; // reset rolling sum variables
            float mic2_roll = 0.0;
            float mic3_roll = 0.0;
            for (int i = 0; i < 10000; i++)
            {
                // 10000 iterations of mic samples spaced out by 1 ms
                mic1_roll += mic1.read();
                mic2_roll += mic2.read();
                mic3_roll += mic3.read();
                Thread::wait(1);
            }

            mic1s = mic1_roll / 10000; // divide all variables by the number of iterations
            mic2s = mic2_roll / 10000;
            mic3s = mic3_roll / 10000;
            serial.lock();
            printf("mic :%f, %f, %f\n", mic1s, mic2s, mic3s); // print data to the console
            serial.unlock();
            mic.unlock(); // unlock mutexes and wait 100 seconds before the next iteration
            Thread::wait(100000);
        }
    }
}

// Thread to run the motors using the bluetooth control pad
void motorThread(void const *args)
{
    while (1)
    {
        // while the device is not readable or the global no is set continue
        while (global_no || !blue.readable())
        {
            continue;
        }
        // if readable lock the serial mutex and reset encoders in case of the encoder testing mode
        serial.lock();
        EncL.reset();
        EncR.reset();
        if (blue.getc() == '!')
        {
            if (blue.getc() == 'B')
            {
                bnum = blue.getc();
                bhit = blue.getc();
                if (blue.getc() == char(~('!' + 'B' + bnum + bhit)))
                {
                    switch (bnum) // switch on the bnum after we detected a usuable bluetooth hit
                    {
                    case '1': // number button 1
                        if (bhit == '1')
                        {
                            manual_mode = 1; // if we press number butotn 1 unlock the manual_mode
                        }
                        else
                        {
                            // add release code here
                        }
                        break;
                    case '2': // number button 2
                        if (bhit == '1')
                        {
                            manual_mode = 0; // turn off the manual_mode

                            // LCD print that Sampling Mode is activated in large BLUE text
                            lcd.cls();
                            lcd.locate(1, 3);
                            lcd.text_width(2);
                            lcd.text_height(2);
                            lcd.text_bold(1);
                            lcd.color(BLUE);
                            lcd.printf("SAMPLING MODE");
                        }
                        else
                        {
                            // add release code here
                        }
                        break;
                    case '3': // number button 3
                        if (bhit == '1')
                        {
                            // If we press number button 1 turn a specified amount of Encoder ticks (USED only for testing)
                            m1.speed(.5);
                            m2.speed(-.5);
                            while (EncL.read() < 500 && EncR.read() < 500)
                            {
                                continue;
                            }
                            m1.speed(0);
                            m2.speed(0);
                        }
                        else
                        {
                            // add release code here
                        }
                        break;
                    case '4': // number button 4
                        break;
                    case '5': // button 5 up arrow
                        if (bhit == '1')
                        {
                            d_mutex.lock();
                            if (manual_mode == 1 && d > 8) // If manual_mode is activated and the distance is not too close move forwards
                            {
                                // Move forward at a slow speed
                                m1.speed(-0.5);
                                m2.speed(-0.5);
                            }
                            else
                            {
                                // Otherwise don't move at all
                                m1.speed(0);
                                m2.speed(0);
                            }
                            d_mutex.unlock();
                        }
                        else
                        {
                            if (manual_mode == 1)
                            {
                                // Don't move at all
                                m1.speed(0);
                                m2.speed(0);
                            }
                        }
                        break;
                    case '6': // button 6 down arrow
                        if (bhit == '1')
                        {
                            if (manual_mode == 1)
                            {
                                // Move backward at a slow speed
                                m1.speed(0.5);
                                m2.speed(0.5);
                            }
                        }
                        else
                        {
                            if (manual_mode == 1)
                            {
                                // Don't move at all
                                m1.speed(0);
                                m2.speed(0);
                            }
                        }
                        break;
                    case '7': // button 7 left arrow
                        if (bhit == '1')
                        {
                            if (manual_mode == 1)
                            {
                                // Turn left at a slow speed
                                m2.speed(-0.45);
                                m1.speed(0.45);
                            }
                        }
                        else
                        {
                            if (manual_mode == 1)
                            {
                                // Don't move at all
                                m1.speed(0);
                                m2.speed(0);
                            }
                        }
                        break;
                    case '8': // button 8 right arrow
                        if (bhit == '1')
                        {
                            if (manual_mode == 1)
                            {
                                // Turn right at a slow speed
                                m2.speed(0.45);
                                m1.speed(-0.45);
                            }
                        }
                        else
                        {
                            if (manual_mode == 1)
                            {
                                // Don't move at all
                                m1.speed(0);
                                m2.speed(0);
                            }
                        }
                        break;
                    default:
                        break;
                    }
                    serial.unlock();

                }
            }
        }

        Thread::wait(250);
    }
}

// Thread to sample the mics
void sensorThread(void const *args)
{
    while (1)
    {
        if (manual_mode == 0) // Run only when manual_mode is off (sampling mode on)
        {
            // Reset mic max readings
            mic1_max = 0;
            mic2_max = 0;
            mic3_max = 0;
            serial.lock();
            printf("SAMPLING!!");
            mic.lock();

            // For sample of 1000 ms figure out the mic maximum value for each mic
            for (int i = 0; i < 1000; i++)
            {
                mic1_max = max(mic1_max, mic1.read());
                mic2_max = max(mic2_max, mic2.read());
                mic3_max = max(mic3_max, mic3.read());
                Thread::wait(1);
            }
            printf("DONE SAMPLING");

            /**
            * For all mics determine the usuable value by first checking if the difference between the sampled offset
            * is large enough (over a constant value determined through testing of .009. Extract the usable value by 
            * subtracting the sampled offset. If it doesn't meet this condition set the value to 0.
            **/
            if ((mic1_max - mic1s) > k)
            {
                mic1_val = (mic1_max - mic1s);
            }
            else
            {
                mic1_val = 0;
            }
            if ((mic2_max - mic2s) > k)
            {
                mic2_val = (mic2_max - mic2s);
            }
            else
            {
                mic2_val = 0;
            }
            if ((mic3_max - mic3s) > k)
            {
                mic3_val = (mic3_max - mic3s);
            }
            else
            {
                mic3_val = 0;
            }

            // If the value is not greater than 0 don't apply the log scaling. Multiply by 1000 and then take the log;
            // we do this so the values are more significantly larger than one another at a smaller scale, making the algorithm work better
            mic1_val = mic1_val > 0 ? log(mic1_val * 1000) : 0;
            mic2_val = mic2_val > 0 ? log(mic2_val * 1000) : 0;
            mic3_val = mic3_val > 0 ? log(mic3_val * 1000) : 0;

            // Print the values to the console
            printf("mic1: %f\n", mic1_val);
            printf("mic2: %f\n", mic2_val);
            printf("mic3: %f\n", mic3_val);

            // Declare variables for angle calculations
            float sg, out;
            float sg_idx, sg_deg;
            out = 0;

            // If we detect no values across all channels print the error mode to the screen
            if (mic1_val + mic2_val + mic3_val == 0) {
                printf("error");
                lcd.cls();
                lcd.text_width(2);
                lcd.text_height(2);
                lcd.textbackground_color(BLACK);
                lcd.color(RED);
                lcd.locate(2, 3);
                lcd.printf("ERROR");
                
                lcd.color(WHITE);
                lcd.locate(2 , 5);
                lcd.text_width(1);
                lcd.text_height(1);
                Thread::wait(500);
                lcd.printf("resetting...");
                Thread::wait(1000);
                lcd.cls();
                mic.unlock();
                serial.unlock();
                manual_mode = 4;
                continue;
            }

            /**
            * Triangulation algorithm:
            * We take the largest value of all the mics and we make that the center point of the triangle.
            * From there we take the difference in the other two mics and based on the sign of that we shift our
            * center point left or right based on the direction of the second loudest mic. For example, if the loudest
            * microphone is mic 1 (0 Deg @ 2) and the difference between mic 2 and 3 is -1 we would move our center point from
            * 0 degrees to 1/2 * -45 (-45 is half of the angular difference between mic 2 and 1). This would mean our output heading is -22.5 degrees
            **/

            // Determine the maximum mic and compute the out heading using the above algorithm.
            if (mic1_val > mic2_val && mic1_val > mic3_val)
            {
                sg_idx = (mic2_val > mic3_val) ? mic2_val - mic3_val : mic3_val - mic2_val;
                sg_deg = (mic2_val > mic3_val) ? -90 : 90;
                out = (sg_deg / 2.0) - ((mic1_val - sg_idx) / mic1_val) * sg_deg / 2.0;
                myleds = 1;
            }
            else if (mic2_val > mic1_val && mic2_val > mic3_val)
            {
                sg_idx = (mic1_val > mic3_val) ? mic1_val - mic3_val : mic3_val - mic1_val;
                sg_deg = (mic1_val > mic3_val) ? 90 : -180;
                out = (-90 + sg_deg / 2.0) - ((mic2_val - sg_idx) / mic2_val) * (sg_deg) / 2.0;
                myleds = 2;
            }
            else if (mic3_val > mic1_val && mic3_val > mic2_val)
            {
                sg_idx = (mic1_val > mic2_val) ? mic1_val - mic2_val : mic2_val - mic1_val;
                sg_deg = (mic1_val > mic2_val) ? -90 : 180;
                out = (90 + sg_deg / 2.0) - ((mic3_val - sg_idx) / mic3_val) * (sg_deg) / 2.0;
                myleds = 4;
            }

            // Print the output degree to console
            printf("output deg: %f", out);

            // Display the target heading to the LCD screen
            lcd.cls();
            lcd.locate(1, 4);
            lcd.text_height(2);
            lcd.text_width(2);
            lcd.color(GREEN);
            lcd.printf("Target: %.1f", out);

            // Determine the number of encoder ticks required to turn, uses 850 ticks of each wheel to make a full rotation of the bot
            sg = abs(850 * out / 360.0);
            printf("target: %.1f", sg);
            printf("\n\n");

            // Set the global no to shut off all other threads (used so that isolating the distance thread and the sample thread works well)
            global_no = 1;
            
            // Reset encoders
            EncL.reset();
            EncR.reset();

            // If the output degree is > 0 we turn right otherwise we turn left
            if (out > 0)
            {
                m1.speed(-.45);
                m2.speed(.45);
            }
            else
            {
                m1.speed(.45);
                m2.speed(-.45);
            }
            float calibrationConstant = 0.04;
            // We read the encoders until one of them has turned the right amount
            while (EncR.read() < sg && EncL.read() < sg)
            {
                float leftEnc = EncL.read();
                float rightEnc =  EncR.read();
                float encDiff = leftEnc - rightEnc;

                float fix = calibrationConstant * encDiff;

                m1.speed(-0.45 - fix);  
                m2.speed(-0.45 + fix);  
            }

            // Set speeds to 0 once reached target heading
            m1.speed(0);
            m2.speed(0);
            lcd.cls();
            serial.unlock();
            mic.unlock();

            Thread::wait(50);

            // After a brief wait move forwards in the heading direction
            EncL.reset();
            EncR.reset();

            speed = -.45; // set the move forward speed

            m1.speed(speed);
            m2.speed(speed);


            // While the distance is not too close and the sum of the encoders isn't less than 
            // 13600 (8 full revolutions of each wheel as a hard stop) run the encoder correction algorithm
            while (!(d < 8) || (EncL.read() + EncR.read()) < 13600)
            {

                // Generally, the left encoder always falls short of the right encoder. So we add in a correction algorithm.
                // We subtract the difference between encoder ticks and divide by a correction factor of 50. We add this to the speed.
                // We set the max newLspeed to .9 to avoid going too fast.
                float newLspeed = speed - (EncR.read() - EncL.read()) / 50.0;
                newLspeed = newLspeed > .9 ? .9 : newLspeed;
                m2.speed(newLspeed);

                // Add a 1 ms wait
                Thread::wait(1);

            }

            // If either break condition occurs stop driving
            m1.speed(0);
            m2.speed(0);

            // Wait 100 ms then allow other threads to reconnect
            Thread::wait(100);
            global_no = 0;

            // Set manual_mode back to an unused value
            manual_mode = 4;
        }
    }
}

// Thread to sample the ultrasound sensor
void distanceThread(void const *args)
{
    while (1)
    {   
        // We want this thread to always run as it computes the distance sensor
        mu.start();
        temp_d = mu.get_dist_cm(); // After the start of the sample period get the previous distance value
        d_mutex.lock();
        d = temp_d < 360 ? temp_d : d; // If not an invalid number set it as the distance value otherwise hold the old value
        while (d < 8 && d >= 1)
        {
            // If the distance is too small flash the danger message over the LCD
            d_mutex.unlock();
            serial.lock();
            lcd.cls();
            lcd.locate(0, 2);
            lcd.text_width(3);
            lcd.text_height(3);
            lcd.filled_rectangle(0, 0, 127, 127, RED);
            lcd.color(BLACK);
            lcd.textbackground_color(RED);
            lcd.printf("DANGER");
            Thread::wait(50);
            lcd.filled_rectangle(0, 0, 127, 127, BLACK);
            lcd.textbackground_color(BLACK);
            lcd.color(RED);
            lcd.locate(0, 2);
            lcd.printf("DANGER");
            serial.unlock();
            mu.start();
            temp_d = mu.get_dist_cm();
            d_mutex.lock();
            d = temp_d < 360 ? temp_d : d;
            lcd.cls();
        }
        serial.lock();
        
        // Otherwise clear the non-static portion of the screen
        lcd.filled_rectangle(0, 0, 127, 37, BLACK);
        lcd.filled_rectangle(65, 0, 127, 127, BLACK);
        lcd.locate(2, 3);
        lcd.textbackground_color(BLACK);
        lcd.color(WHITE);
        lcd.text_width(2);
        lcd.text_height(2);

        // Print out the current distance on the LCD
        lcd.printf("cm: %d", (int)d);
        d_mutex.unlock();
        serial.unlock();
        Thread::wait(50);
    }
}

int main()
{

    // Clear LCD and set baudrate
    lcd.cls();
    lcd.baudrate(96000);

    // Initialize all threads
    Thread motorT(motorThread);
    Thread sensorT(sensorThread);
    Thread sample(sampleMics);
    Thread dT(distanceThread);

    while (1)
    {
    }
}

```


## Video

This video demonstrates the main features of the sound bot. Click the picture below and get linked to Youtube.

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/8Jz2cLw9_kc/0.jpg)](https://www.youtube.com/watch?v=8Jz2cLw9_kc)

Our video highlights our features and starts with our Bluetooth connectivity, which is enabled through the bluefruit app.

We then showcase our ultrasound sensor which we use for distance measurement. The LCD flashes the distance to the closest object the sensor sees, and if the object is too close, flashes danger on the screen.

Next, we showcase our manual control, which is triggered by pressing the button 1 on the app keypad. This allows us to move the robot in any way we want and is possible to go to a full 360 degrees.

The use of our ultrasound sensor is highlighted as we put our hand in front of the sensor, and the robot cannot go forward even by pressing the forward button on our app. The robot can only go backwards, or to the side as showcased.

Next, we showcase our sound mode, which is triggered by the press of a button on our app, which lets the bot listen for sound, and then the bot goes towards wherever the sound is in a 360 deg radius. This first clip shows when the bot hears sound directly in front.

We then showcase when our bot hears sound very close to one of the side microphones, and the bot produces a 90 deg turn. We also highlight application of our sensor as the bot is approaching the wall but does not hit the wall.

Last, we showcase our triangulation algorithm, as we clap in between two microphones, and our program calculates a degree to go to and starts going towards the direction of sound.

## Conclusion

This project demonstrates the capability of the mbed to control an embedded system given the right hardware. Using threads from the RTOS library to control the various IO devices we were able to share hardware resources and limit the amount of IO devices added.

Some problems we encountered included library compatability issues with the ultrasonic sensor. The regular library we used in the lab relied on interrupts which cannot simultaneuously be used with the RTOS library. We found a workaround by using a different ultasonic sensor library which did not use interrupts. Another issue we faced was with accessing parts of the LCD as well as multiple parts accessing the motors at the same time. To get around this, we used the mbed RTOS. This allowed us to use threads and mutex lock some resources as they were being used to remove race conditions such as the ultrasonic sensor needing access to the motor. 

Another one of the main issues we faced was with the microphone outputs. The microphones we used produced inaccurate data because of the lack of op amp and sound isolation techniques. To compensate for this issue, we scaled the values to make the differences in sound more clear and we sampled for an ambiance to generate a baseline to base readings off of. The final big issue was wheel slippage and weight imbalances. This is solved through the implementation of wheel encoders. The encoder allows us to account for wheel slippage as it occurs and maintains the robots forward movement without unwanted rotations due to the wheel slipping and weight imbalances.

The inclusion of the bluetooth module allowed us to easily control the robot. For commercial purposes as well, this was noted as one of best ways to connect. It was a great way to prototype a final iteration of the product.

In future iterations, we would be sure to finetune the encoder calculations to fully eliminate drift, we would also use more intricate microphones to allow to better data readings, and finally we would implement a bigger chassis to allow for more microphone isolation and allow for there to be more practical uses.

