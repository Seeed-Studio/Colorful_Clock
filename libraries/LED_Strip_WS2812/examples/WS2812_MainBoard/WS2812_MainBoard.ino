/*
 * LED Strip example
 * 
 * Created: 08.28.2014
 * Author: lawliet.zou@gmail.com
 */ 

#include <WS2812.h>
#include <Wire.h>

WS2812 LED(50); //LED number

void setup() 
{
    //Serial.begin(9600);
    LED.init(SIGL_PIN); //Digital Pin 5
    LED.setBrightness(50);
}

void loop() 
{  
    LED.scrolling(5, LED.RGB(255,255,0),50);
    for(int i = 0; i < 5; i++){
      LED.blinkAll(LED.RGB(0,0,255),200);
    }
    LED.passingBy(10, LED.RGB(255,0,0), LED.RGB(0,0,255), 50);
    LED.colorBrush(LED.RGB(255, 0, 0), 50); // Red
    LED.colorBrush(LED.RGB(0, 255, 0), 50); // Green
    LED.colorBrush(LED.RGB(0, 0, 255), 50); // Blue
    LED.theaterChase(LED.RGB(  0, 255,   0), 50); // green
    LED.theaterChase(LED.RGB(255,   0,   0), 50); // Red
    LED.theaterChase(LED.RGB(  0,   0, 255), 50); // Blue
    LED.rainbow(20);
    LED.rainbowCycle(10);
    LED.theaterChaseRainbow(10);
}

