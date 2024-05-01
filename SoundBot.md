# **ECE 4180 Final Project**
Developed by Aditya Ghatge, Kabir Jain, and Yash Vardhan

# mbed Sound Bot
## Introduction

This notebook explores the design of a sound activated robot which leverages the mbed LPC1768 microcontroller and bluetooth software capabilites. The robot's commercial use case would be for individuals with disabilities preventing them from walking. By using the robot, individuals can call out for and control; it improves their quality of life. Features implemented include the ability for the robot to move towards sound on command, a manual override mode, collision prevention technology, and viewing of peripheral data on an LCD screen. The robot is fully controlled using bluetooth via the Adafruit Bluefruit app.

### Design Block Diagram
 ---- Include diagram

### Picture of Design

 ---- Include Picure of design
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
LPC1768 Pin	Component	Pin Name	Misc.
p5	uSD File System	SPI MOSI	-
p6	uSD File System	SPI MISO	-
p7	uSD File System	SPI SCLK	-
p8	uSD File System	CS	-
VOUT	uSD File System	VCC	-
GND	uSD File System	GND	-
p9	uLCD Screen	RX	-
p10	uLCD Screen	TX	-
p11	uLCD Screen	Reset	-
-	uLCD Screen	5V	5V Supply
GND	uLCD Screen	GND	-
p13	Bluetooth Module	RXI	-
p14	Bluetooth Module	TXO	-
-	Bluetooth Module	Vin	5V Supply
GND	Bluetooth Module	CTS	-
GND	Bluetooth Module	GND	-
p18	Class D Amplifier	in+	-
-	Class D Amplifier	pwr+	5V Supply
GND	Class D Amplifier	in-	-
GND	Class D Amplifier	pwr-	-
-	Class D Amplifier	out+	Speaker +
-	Class D Amplifier	out-	Speaker -

------ EXPLAIN DESIGN CHOICES

The Class D Amplifier amplifies the PWM signal that contains the music data. A potentiometer is used in combination with the amplifier to control the volume of the audio output. The amplifier sends the amplified music to a speaker.

amp speaker

The Bluetooth control of the music player is accomplished using the Adafruit Bluetooth module shown below. Music playback is controlled as follows: the left arrow skips backward 1 song, the right arrow skips forward 1 song, the up arrow increases volume, the down arrow decreases volume, the (1) button plays the song, the (2) button pauses the song, the (3) button mutes the song, and the (4) button jumps to a random song.

blue

app

The LCD screen displays the song title, progress (via a progress bar with time stamps), and status (playing/stopped).

lcd

The external 5V power supply is necessary due to all of the external components of the design; the mbed wouldn’t be able to supply enough power to everything in the design using its 5V output pin.

--------- EXPLAIN DESIGN CHOICES

## Software Used

We make use of the “HALLFX_ENCODER” library https://os.mbed.com/users/electromotivated/code/HALLFX_ENCODER/ with modifications to better control the encoders. We also used the "hcsr04" library to use the ultrasonic sensor without interrupts within our threads.

The code for our sound bot is shown below:

```
{
    // Sweep the motor speed from full-speed reverse (-1.0) to full speed forwards (1.0)

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

volatile int manual_mode = 4;


HCSR04 mu(p6, p7); // Set the trigger pin to p6 and the echo pin to p7
                   // have updates every .1 seconds and a timeout after 1
                   // second, and call dist when the distance changes


#define max(a, b) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

volatile bool button_ready = 0;
volatile int bnum = 0;
volatile int bhit;
volatile int global_no = 0;
// state used to remember previous characters read in a button message
enum statetype
{
    start = 0,
    got_exclm,
    got_B,
    got_num,
    got_hit
};
statetype state = start;

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

float speed;

volatile float mic1max = -1000.0;
volatile float mic2max = -1000.0;
volatile float mic3max = -1000.0;
volatile float mic1val;
volatile float mic2val;
volatile float mic3val;
volatile int mic1sample;
volatile int mic2sample;
volatile int mic3sample;

volatile float d, temp_d;

volatile float mic1s;
volatile float mic2s;
volatile float mic3s;
Mutex mic;
Mutex serial;
Mutex d_mutex;

void sampleMics(void const *args)
{
    while (1)
    {
        if (!global_no)
        {

            mic.lock();
            float mic1_roll = 0.0;
            float mic2_roll = 0.0;
            float mic3_roll = 0.0;
            for (int i = 0; i < 10000; i++)
            {
                mic1_roll += mic1.read();
                mic2_roll += mic2.read();
                mic3_roll += mic3.read();
            }

            mic1s = mic1_roll / 10000;
            mic2s = mic2_roll / 10000;
            mic3s = mic3_roll / 10000;
            serial.lock();
            printf("mic :%f, %f, %f\n", mic1s, mic2s, mic3s);
            serial.unlock();
            mic.unlock();
            Thread::wait(100000);
        }
    }
}

void motorThread(void const *args)
{
    while (1)
    {
        while (global_no || !blue.readable())
        {
            continue;
        }
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
                    switch (bnum)
                    {
                    case '1': // number button 1
                        if (bhit == '1')
                        {
                            manual_mode = 1;
                        }
                        else
                        {
                            // add release code here
                        }
                        break;
                    case '2': // number button 2
                        if (bhit == '1')
                        {
                            manual_mode = 0;
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
                        { // 1000
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
                            if (manual_mode == 1 && d > 8)
                            {
                                m1.speed(-0.5);
                                m2.speed(-0.5);
                            }
                            else
                            {
                                m1.speed(0);
                                m2.speed(0);
                            }
                            d_mutex.unlock();
                        }
                        else
                        {
                            if (manual_mode == 1)
                            {
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
                                m1.speed(0.5);
                                m2.speed(0.5);
                            }
                        }
                        else
                        {
                            if (manual_mode == 1)
                            {
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
                                m2.speed(-0.45);
                                m1.speed(0.45);
                            }
                        }
                        else
                        {
                            if (manual_mode == 1)
                            {
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
                                m2.speed(0.45);
                                m1.speed(-0.45);
                            }
                        }
                        else
                        {
                            if (manual_mode == 1)
                            {
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

volatile float mic1_val, mic2_val, mic3_val, mic1_max, mic2_max, mic3_max;

void sensorThread(void const *args)
{
    while (1)
    {
        if (manual_mode == 0)
        {

            // mic1_val =  mic1s < mic1.read() ? (mic1.read() - mic1s) : 0;
            // mic2_val = mic2s < mic2.read() ? (mic2.read() - mic2s) : 0;
            // mic3_val = mic3s < mic3.read() ? (mic3.read() - mic3s) : 0;
            int positive = 0;
            mic1_max = 0;
            mic2_max = 0;
            mic3_max = 0;
            serial.lock();
            printf("SAMPLING!!");
            mic.lock();

            for (int i = 0; i < 1000; i++)
            {
                mic1_max = max(mic1_max, mic1.read());
                mic2_max = max(mic2_max, mic2.read());
                mic3_max = max(mic3_max, mic3.read());
                Thread::wait(1);
            }
            printf("DONE SAMPLING");

            float mic1_init = mic1_max;
            float mic2_init = mic2_max;
            float mic3_init = mic3_max;
            if (mic1s < mic1_init && (mic1_init - mic1s) > 0.009)
            {
                mic1_val = (mic1_init - mic1s);
            }
            else
            {
                mic1_val = 0;
            }
            if (mic2s < mic2_init && (mic2_init - mic2s) > 0.009)
            {
                mic2_val = (mic2_init - mic2s);
            }
            else
            {
                mic2_val = 0;
            }
            if (mic3s < mic3_init && (mic3_init - mic3s) > 0.009)
            {
                mic3_val = (mic3_init - mic3s);
            }
            else
            {
                mic3_val = 0;
            }

            mic1_val = mic1_val > 0 ? log(mic1_val * 1000) : 0;
            mic2_val = mic2_val > 0 ? log(mic2_val * 1000) : 0;
            mic3_val = mic3_val > 0 ? log(mic3_val * 1000) : 0;

            printf("mic1: %f\n", mic1_val);
            printf("mic2: %f\n", mic2_val);
            printf("mic3: %f\n", mic3_val);
            float sg, out;
            float sg_idx, sg_deg;
            out = 0;

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

            if (mic1_val > mic2_val && mic1_val > mic3_val)
            {
                sg = (mic2_val > mic3_val) ? mic2_val : mic3_val;
                sg_idx = (mic2_val > mic3_val) ? mic2_val - mic3_val : mic3_val - mic2_val;
                sg_deg = (mic2_val > mic3_val) ? -90 : 90;
                out = (sg_deg / 2.0) - ((mic1_val - sg_idx) / mic1_val) * sg_deg / 2.0;
                myleds = 1;
            }
            else if (mic2_val > mic1_val && mic2_val > mic3_val)
            {
                sg = (mic1_val > mic3_val) ? mic1_val : mic3_val;
                sg_idx = (mic1_val > mic3_val) ? mic1_val - mic3_val : mic3_val - mic1_val;
                sg_deg = (mic1_val > mic3_val) ? 90 : -180;
                out = (-90 + sg_deg / 2.0) - ((mic2_val - sg_idx) / mic2_val) * (sg_deg) / 2.0;
                myleds = 2;
            }
            else if (mic3_val > mic1_val && mic3_val > mic2_val)
            {
                sg = (mic1_val > mic2_val) ? mic1_val : mic2_val;
                sg_idx = (mic1_val > mic2_val) ? mic1_val - mic2_val : mic2_val - mic1_val;
                sg_deg = (mic1_val > mic2_val) ? -90 : 180;
                out = (90 + sg_deg / 2.0) - ((mic3_val - sg_idx) / mic3_val) * (sg_deg) / 2.0;
                myleds = 4;
            }

            printf("output deg: %f", out);

            lcd.cls();
            lcd.locate(1, 4);
            lcd.text_height(2);
            lcd.text_width(2);
            lcd.color(GREEN);
            lcd.printf("Target: %.1f", out);

            sg = abs(850 * out / 360.0);
            printf("target: %.1f", sg);
            printf("\n\n");
            global_no = 1;

            EncL.reset();
            EncR.reset();

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

            while (EncR.read() < sg && EncL.read() < sg)
            {
                continue;
            }
            m1.speed(0);
            m2.speed(0);
            lcd.cls();
            serial.unlock();
            mic.unlock();

            Thread::wait(50);
            m1.speed(-.45);
            m2.speed(-.45);
            EncL.reset();
            EncR.reset();
            while (!(d < 8))
            {
            }

            m1.speed(0);
            m2.speed(0);

            Thread::wait(100);
            global_no = 0;
            wait(1.0 / 8000.0);

            manual_mode = 4;
        }
    }
}

void distanceThread(void const *args)
{
    while (1)
    {
        mu.start();
        temp_d = mu.get_dist_cm();
        d_mutex.lock();
        d = temp_d < 360 ? temp_d : d;
        while (d < 8 && d >= 1)
        {
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
        // lcd.filled_rectangle(0, 0, 127, 127, WHITE);
        lcd.filled_rectangle(0, 0, 127, 37, BLACK);
        lcd.filled_rectangle(65, 0, 127, 127, BLACK);
        // printf("dist: %d", (int) d);
        lcd.locate(2, 3);
        lcd.textbackground_color(BLACK);
        lcd.color(WHITE);
        lcd.text_width(2);
        lcd.text_height(2);
        lcd.printf("cm: %d", (int)d);
        d_mutex.unlock();
        serial.unlock();
        Thread::wait(50);
    }
}

int main()
{

    // mu.startUpdates();
    // sampleMics();
    // blue.attach(&parse_message,Serial::RxIrq);
    lcd.cls();
    // lcd.baudrate(96000);
    lcd.baudrate(96000);
    Thread motorT(motorThread);
    Thread sensorT(sensorThread);
    Thread sample(sampleMics);
    Thread dT(distanceThread);

    while (1)
    {
    }
}

}
```


## Video

This video demonstrates the main features of the sound bot.

-----ADD VIDEO

## Conclusions

--- ADD CONCLUSION

With this project, we demonstrated the capability of the mbed library to control a fairly sophisticated embedded system with the appropriate supported hardware. We took advantage of the mbed RTOS library to control various components of the design while sharing hardware resources via mutex locks.

We encountered problems with library compatibility between the various components we used but found some workarounds by using older libraries. Even with protecting shared resources via mutex locks, we found that the timing of each thread's while() loop could result in some bugs depending on which thread yielded first. We found it was best to make the main thread poll the Bluetooth app fastest (every 0.075 s) while the music playing thread was the second fastest (every 0.1 s), and the LCD thread was the slowest (every 0.25 s).

Additionally, to control music playback we had to explicitly modify the wave_player library so that we could exit the loop that played the music when necessary (toggling a flag that returns from the function and adding some cleanup code for safety when a song is skipped), control playback (pause and resume music playing by toggling a flag), and control volume (writing a scaled unsigned short value to the given AnalogOut pin).

-----ADD CONCLUSION