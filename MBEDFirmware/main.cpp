//ECE 4180 Project
//      4/26/22
#include "mbed.h"
#include "emic2.h"
#include <string>
#include <cstdio>
#include "rtos.h"
#include "WS2812.h"
#include "PixelArray.h"
#include "Servo.h"

#define WS2812_BUF 5
#define NUM_COLORS 6
#define NUM_LEDS_PER_COLOR 1

#define TOP_MAX     .9f
#define TOP_MIN     .1f
#define BOTTOM_MAX  1.0f
#define BOTTOM_MIN  0.0f

PixelArray px(WS2812_BUF);
WS2812 ws(p9, WS2812_BUF, 5, 10, 10, 15);

// set up the colours we want to draw with
int colorbuf[NUM_COLORS] = {0x000000,0xfc0000,0xfc7f00,0xfce800,0x80fc00,0x00fc00};
int battery_level = 0;

Ticker battery_reading;
AnalogIn   batteryin(p19);

Thread servomotion;
Servo top(p21);
Servo bottom(p22);
typedef enum {
    STOP,
    HAPPY,
    SAD
} servo_pos_t;

servo_pos_t tail_choice = STOP;

RawSerial  rpi(USBTX, USBRX);

Thread tazerthread;
DigitalOut tazer(p18);
bool taze_flag = false;

DigitalOut myled(LED1);
DigitalOut myled2(LED2);
RawSerial  dev(p28,p27); // bluetooth
emic2 myTTS(p13, p14); //serial RX,TX pins to emic
const int size = 64;
char msg[size];
string msgString = "";
bool messageReady = false;
int messageLength = 0;

Mutex serial_mutex;

typedef enum {
    OFF,
    SLOWDOWN,
    BOOTUP
} sounds_t;
sounds_t sound_flag = OFF;

void dev_recv()
{
    static int i = 0;
    static bool buttonCommand = false;
    char temp = dev.getc();
    if (!messageReady) {
        if (temp == '!' && !buttonCommand) {
            msg[i++] = '!';
            buttonCommand = true;
        } else if (buttonCommand) {
            if (i < 5) {
                msg[i++] = temp;
            } else {
                i = 0;
                buttonCommand = false;
                messageLength = 5;
                messageReady = true;
            }
        } else if (temp == '\n') {
            messageLength = i;
            i = 0;
            messageReady = true;
        } else {
            if (i < size - 1){
                msg[i++] = temp;
            }
        }
    }
}

void print_battery_level() {
    int battery_level = (int)10371*batteryin;
    if (battery_level < 6480) {
        for (int i = 0; i < WS2812_BUF; i++) {
            px.Set(i, colorbuf[0]);
        }
        for (int i = 0; i < 1; i++) {
            px.Set(i, colorbuf[1]);
        }
    } else if (battery_level < 6960) {
        for (int i = 0; i < WS2812_BUF; i++) {
            px.Set(i, colorbuf[0]);
        }
        for (int i = 0; i < 2; i++) {
            px.Set(i, colorbuf[2]);
        }
    } else if (battery_level < 7440) {
        for (int i = 0; i < WS2812_BUF; i++) {
            px.Set(i, colorbuf[0]);
        }
        for (int i = 0; i < 3; i++) {
            px.Set(i, colorbuf[3]);
        }
    } else if (battery_level < 7920) {
        for (int i = 0; i < WS2812_BUF; i++) {
            px.Set(i, colorbuf[0]);
        }
        for (int i = 0; i < 4; i++) {
            px.Set(i, colorbuf[4]);
        }
    } else {
        for (int i = 0; i < WS2812_BUF; i++) {
            px.Set(i, colorbuf[0]);
        }
        for (int i = 0; i < 5; i++) {
            px.Set(i, colorbuf[5]);
        }
    }
    px.SetAllI(64);
    ws.write(px.getBuf());
}

void servo_helper() {
    static float servo1Current = .5f;
    static float servo1New = servo1Current;
    static float ratio1 = .1f;
    static float servo2Current = .5f;
    static float servo2New = servo2Current;
    static float ratio2 = .1f;
    static float delay = 50;
    bool updateServos = false;
    for (;;) {
        switch (tail_choice) {
            case STOP:
            updateServos = false;
            break;
            case HAPPY:
            ratio1 = .2f;
            ratio2 = .2f;
            updateServos = true;
            break;
            case SAD:
            ratio1 = .1f;
            ratio2 = .05f;
            updateServos = true;
            break;
        }
        if (updateServos) {
            if (servo1Current >= servo1New -.05f && servo1Current <= servo1New +.05f) {
                if (servo1New == TOP_MAX) {
                    servo1New = TOP_MIN;
                } else {
                    servo1New = TOP_MAX;
                }
            }
            if (servo2Current >= servo2New -.05f && servo2Current <= servo2New +.05f) {
                if (servo2New == TOP_MAX) {
                    servo2New = TOP_MIN;
                } else {
                    servo2New = TOP_MAX;
                }
            }
            servo1Current = (1.0f - ratio1)*servo1Current + ratio1*servo1New;
            servo2Current = (1.0f - ratio2)*servo2Current + ratio2*servo2New;
            top = servo1Current;
            bottom = servo2Current;
        }
        Thread::wait(delay);
    }
}

void rpi_handle()
{
    char temp = 0;
    serial_mutex.lock();
    while(rpi.readable()) {
        temp = rpi.getc();
        rpi.putc(temp);
        switch (temp) {
            case '0':
            tail_choice = STOP;
            break;
            case '1':
            tail_choice = HAPPY;
            break;
            case '2':
            tail_choice = SAD;
            break;
            case '3':
            sound_flag = SLOWDOWN;
            break;
            case '4':
            sound_flag = BOOTUP;
            break;
            case '9':
            taze_flag = true;
            break;
        }
    }
    serial_mutex.unlock();
}

void tazer_helper() {
    for(;;) {
        if (taze_flag) {
            tazer = 1;
            taze_flag = false;
        } else {
            tazer = 0;
        }
        Thread::wait(1000);
    }
}

int main()
{
    int i = 10;
    int j = 1;
    
    battery_reading.attach(&print_battery_level, 2.0);
    ws.useII(WS2812::PER_PIXEL); // use per-pixel intensity scaling
    
    tazerthread.start(tazer_helper);
    
    servomotion.start(servo_helper);
    rpi.baud(115200);
    rpi.attach(&rpi_handle, Serial::RxIrq);
    
    dev.baud(9600);
    dev.attach(&dev_recv, Serial::RxIrq);
    myTTS.volume(10); //max volume
    //myTTS.speakf("S");//Speak command starts with "S" // Send the desired string to convert to speech
//    myTTS.speakf("Booting u");
//    myTTS.speakf("\r"); //marks end of speak command
    while(1){
        //pc.printf(msg);
        if (messageReady) {
            for(int k = 0; k < messageLength; k++){
                msgString += msg[k];
            }
            messageReady = false;
            
            serial_mutex.lock();
            myTTS.speakf("S");//Speak command starts with "S" // Send the desired string to convert to speech
            myTTS.speakf("\r"); //marks end of speak command
            myTTS.ready(); //ready waits for speech to finish from last command with a ":" response
            if (msgString=="!B516!B507" || msgString=="!B516" || msgString=="!B507")  { //checksum OK?
                if (i == 18){
                    i = i;
                } else {
                    i=i+1;
                }
                myTTS.volume(i);
                myTTS.speakf("Svolume level, %D\r",i)  ;                    
                myTTS.ready();
                msgString = "";
            } else if (msgString=="!B615!B606" || msgString=="!B615" || msgString=="!B606")  {
                if (i == 0){
                    i = i;
                } else {
                    i=i-1;
                }
                myTTS.volume(i);
                myTTS.speakf("Svolume level, %D\r",i)  ;    
                myTTS.ready();
                msgString = "";
            } else if (msgString=="!B813!B804" || msgString=="!B813" || msgString=="!B804"){
                if (j == 8) {
                    j = 8;
                } else {
                    j = j + 1;
                }
                myTTS.voice(j);
                myTTS.speakf("SHello this is a sample of voice number, %D\r",j);
                myTTS.ready();
                msgString = "";
                myled=!myled ;
            } else if (msgString=="!B714!B705" || msgString=="!B714" || msgString=="!B705"){
                if (j == 0) {
                    j = 0;
                } else {
                    j = j - 1;
                }
                myTTS.voice(j);
                myTTS.speakf("SHello this is a sample of voice number, %D\r",j);
                myTTS.ready();
                msgString = "";
                myled=!myled ;
            } else if (msgString=="!B11:"){
                taze_flag = true;
                msgString = "";
            }
           
            myTTS.speakf("S");
            myTTS.speakf("%s\n",msgString);
            msgString = "";                  
            myTTS.speakf("\r");
            serial_mutex.unlock();
        }
        Thread::wait(1000);
    }
}
