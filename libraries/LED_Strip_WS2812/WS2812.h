/*
 * WS2812.h
 * LED Strip driver Library
 *
 * Copyright (c) 2014 seeed technology inc.
 * Author        :   lawliet.zou
 * Contribution  :   
 * Create Time   :   Sep 2014
 * Change Log    :   
 *  
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef WS2812_H_
#define WS2812_H_

#include <avr/interrupt.h>
#include <avr/io.h>
#ifndef F_CPU
#define  F_CPU 16000000UL
#endif
#include <util/delay.h>
#include <stdint.h>

#ifdef ARDUINO
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#include <pins_arduino.h>
#endif
#endif


#define MODE_MASTER        0x01
#define MODE_SLAVE         0x02

#define ADDR_LED_STRIP     0x21

#define CMD_INIT            0x01
#define CMD_SYNC            0x02
#define CMD_SET_RGB         0x03
#define CMD_CLEAR           0x04
#define CMD_SET_BRI         0x05
#define CMD_FILL			0x06
#define CMD_SINGLE_SCROLL   0x10
#define CMD_MULTI_SCROLL    0x11
#define CMD_PASSINGBY       0x12
#define CMD_SINGLE_BLINK    0x13
#define CMD_ALL_BLINK       0x14
#define CMD_COLOR_BRUSH     0x15
#define CMD_RAINBOW         0x16
#define CMD_RAINBOWCYCLE    0x17
#define CMD_THEATERCHASE    0x18
#define CMD_THEATERCHASE_RAINBOW    0x19

#define STATE_AVAILABLE     0x00
#define STATE_BUSY          0x01

#define CMD_HEADER_HIGH_BYTE 0x31
#define CMD_HEADER_LOW_BYTE  0x13

#define PIXEL_MODE    MODE_MASTER
//#define PIXEL_MODE  MODE_SLAVE

#define POWER_PIN	4
#define SIGL_PIN	5


class WS2812 {
public: 
    WS2812(uint16_t num_led);
    void init(uint8_t pin);
    void setRGB(uint16_t index, uint32_t px_value);
    void setBrightness(uint8_t brightness);
    void clear();
	void fill(uint32_t px_value);
    void sync();
	void ISRInit();
	void ISRService();
    uint32_t RGB(uint8_t red, uint8_t green, uint8_t blue){
		return ((uint32_t)red << 16)|((uint32_t)green << 8)|blue;
	};
    uint16_t numLeds(){
        return _leds;
    };
    void scrolling(uint32_t px_value, uint8_t time);
    void scrolling(uint8_t size, uint32_t px_value, uint8_t time);
    void passingBy(uint8_t size, uint32_t px_value1, uint32_t px_value2, uint8_t time);
    void blink(uint16_t index, uint32_t px_value, uint8_t time);
    void blinkAll(uint32_t px_value, uint8_t time);
    void colorBrush(uint32_t color, uint8_t time);
    void rainbow(uint8_t time);
    void rainbowCycle(uint8_t time);
    void theaterChase(uint32_t color, uint8_t time); 
    void theaterChaseRainbow(uint8_t time);
private:
    uint8_t *pixels;
    uint16_t _leds;
	uint32_t endTime;
#if (PIXEL_MODE == MODE_MASTER)
	boolean LEDCtrl;
	long timer;
	float userDegree;
#endif
    float degree;
    const volatile uint8_t *port;
    uint8_t pinMask;
    uint32_t Wheel(byte WheelPos);
#if(PIXEL_MODE == MODE_SLAVE)
    void _WriteI2C(uint8_t CMD, uint8_t* args, int8_t argLen);
#endif
    void ws2812_sendArray();    
};

#endif /* WS2812_H_ */