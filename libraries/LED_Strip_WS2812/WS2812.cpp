/*
 * WS2812.cpp
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

#include "WS2812.h"
#include <stdlib.h>
#include <Wire.h>

WS2812::WS2812(uint16_t num_leds) 
{
    _leds = num_leds;
    degree = 1;
#if (PIXEL_MODE == MODE_MASTER)
	timer = 0;
	LEDCtrl = true;
#endif
}

#if(PIXEL_MODE == MODE_SLAVE)
void WS2812::_WriteI2C(uint8_t CMD, uint8_t* args, int8_t argLen)
{
    Wire.beginTransmission(ADDR_LED_STRIP);
    Wire.requestFrom(ADDR_LED_STRIP, 1);
    uint8_t data = Wire.read();
	uint8_t counter = 0;
    while(data != STATE_AVAILABLE){
        delay(100);
        Wire.requestFrom(ADDR_LED_STRIP, 1);
		delay(50);
		if(Wire.available()){
			data = Wire.read();
		}else{
			data = STATE_AVAILABLE;//reset
		}
    }
	Wire.endTransmission();
    Wire.beginTransmission(ADDR_LED_STRIP);
	Wire.write(CMD_HEADER_HIGH_BYTE);
	Wire.write(CMD_HEADER_LOW_BYTE);
    Wire.write(CMD);
    if(argLen)
        Wire.write(argLen);
    if(NULL != args){
        for(int i = 0; i < argLen; i++){
            Wire.write(args[i]);
        }
    }
    Wire.endTransmission();
}
#endif

void WS2812::init(uint8_t pin) {
#if (PIXEL_MODE == MODE_MASTER)
    LEDCtrl = true;
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, LEDCtrl);
    ISRInit();
	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
    pinMask = digitalPinToBitMask(pin);
    port = portOutputRegister(digitalPinToPort(pin));
    if(pixels) free(pixels);
    pixels = (uint8_t*)malloc(_leds*3);
    clear();
#elif(PIXEL_MODE == MODE_SLAVE)
    Wire.begin();
    uint8_t cmd[] = {highByte(_leds), lowByte(_leds), pin};
    _WriteI2C(CMD_INIT, cmd, 3);
	#if 0
	Wire.beginTransmission(ADDR_LED_STRIP);
	Wire.write(CMD_HEADER_HIGH_BYTE);
	Wire.write(CMD_HEADER_LOW_BYTE);
    Wire.write(CMD_INIT);
	Wire.write(3);
	for(int i = 0; i < 3; i++){
		Wire.write(cmd[i]);
	}
    Wire.endTransmission();
	#endif
#else
    #error "please define the PIXEL_MODE. we supply two mode: MODE_SLAVE & MODE_MASTER.\
MODE_SLAVE usually means that mcu(main board) send command to slave device via IIC,\
while MODE_MASTER is the one that really works."
#endif
}

void WS2812::sync() 
{
#if (PIXEL_MODE == MODE_MASTER)
    ws2812_sendArray();   
#elif(PIXEL_MODE == MODE_SLAVE)
    _WriteI2C(CMD_SYNC, NULL, 0);
#else

#endif
}

void WS2812::setRGB(uint16_t index, uint32_t px_value) 
{
#if (PIXEL_MODE == MODE_MASTER)
    if(index < _leds) { 
        uint16_t tmp;
        tmp = index * 3;
        pixels[tmp] = degree*((px_value>>8)&0xff);//green
        pixels[tmp+1] = degree*((px_value>>16)&0xff);//red
        pixels[tmp+2] = degree*(px_value&0xff);//blue
    } 
#elif(PIXEL_MODE == MODE_SLAVE)
    if(index < _leds){
        uint8_t cmd[] = {highByte(index), lowByte(index), (px_value>>16)&0xff, (px_value>>8)&0xff, px_value&0xff};
        _WriteI2C(CMD_SET_RGB, cmd, 5);
    }
#else

#endif
}

void WS2812::clear()
{
#if (PIXEL_MODE == MODE_MASTER)
    memset(pixels, 0, _leds*3);
    sync();
#elif (PIXEL_MODE == MODE_SLAVE)
    _WriteI2C(CMD_CLEAR, NULL, 0);
#else

#endif
}

void WS2812::fill(uint32_t px_value)
{
#if (PIXEL_MODE == MODE_MASTER)
    memset(pixels, px_value, _leds*3);
    sync();
#elif (PIXEL_MODE == MODE_SLAVE)
    _WriteI2C(CMD_FILL, NULL, 0);
#else

#endif	
}

void WS2812::setBrightness(uint8_t brightness)
{
#if (PIXEL_MODE == MODE_MASTER)
    if((brightness > 255) || (brightness < 0))
        return;
    userDegree = brightness/255.0;  
	degree = userDegree;
#elif (PIXEL_MODE == MODE_SLAVE)
    uint8_t cmd[] = {brightness};
    _WriteI2C(CMD_SET_BRI, cmd, 1);
#else

#endif
}

void WS2812::scrolling(uint32_t px_value, uint8_t time)
{
#if (PIXEL_MODE == MODE_MASTER)
    for(int i = 0; i < _leds; i++){
        clear();
        setRGB(i, px_value);
        sync();
        delay(time);
    }
#elif (PIXEL_MODE == MODE_SLAVE)
    uint8_t cmd[] = {(px_value>>16)&0xff, (px_value>>8)&0xff, px_value&0xff, time};
    _WriteI2C(CMD_SINGLE_SCROLL, cmd, 4);
#else

#endif
}

void WS2812::scrolling(uint8_t size, uint32_t px_value, uint8_t time)
{
#if (PIXEL_MODE == MODE_MASTER)
    if(size > _leds){
        return;
    }
    for(int i = 1; i < _leds + size; i++){
        if(i < size){
            for(int j = 0; j < i; j++){
                setRGB(j, px_value);
            }
        }else if(i >= _leds){
            for(int k = i - size; k < _leds; k++){
                setRGB(k, px_value);
            }
        }else{
            for(int m = i-size; m < i; m++){
                setRGB(m, px_value);
            }
        }
        sync();
        delay(time);
        clear();
    }
#elif (PIXEL_MODE == MODE_SLAVE)
    uint8_t cmd[] = {size, (px_value>>16)&0xff, (px_value>>8)&0xff, px_value&0xff, time};
    _WriteI2C(CMD_MULTI_SCROLL, cmd, 5);
#else

#endif
}

void WS2812::passingBy(uint8_t size, uint32_t px_value1, uint32_t px_value2, uint8_t time)
{
#if (PIXEL_MODE == MODE_MASTER)
    if(size > _leds){
        return;
    }
    for(int i = 1; i < _leds + size; i++){
        if(i < size){
            for(int j = 0; j < i; j++){
                setRGB(j, px_value1);
                setRGB(_leds -1 - j, px_value2);
            }
        }else if(i >= _leds){
            for(int k = i - size; k < _leds; k++){
                setRGB(k, px_value1);
                setRGB(_leds -1 - k, px_value2);
            }
        }else{
            for(int m = i-size; m < i; m++){
                setRGB(m, px_value1);
                setRGB(_leds -1 - m, px_value2);
            }
        }
        sync();
        delay(time);
        clear();
    }   
#elif (PIXEL_MODE == MODE_SLAVE)
    uint8_t cmd[] = {size, (px_value1>>16)&0xff, (px_value1>>8)&0xff, px_value1&0xff, 
        (px_value2>>16)&0xff, (px_value2>>8)&0xff, px_value2&0xff, time};
    _WriteI2C(CMD_PASSINGBY, cmd, 8);
#else

#endif
}

void WS2812::blink(uint16_t index, uint32_t px_value, uint8_t time)
{
#if (PIXEL_MODE == MODE_MASTER)
    setRGB(index, px_value);
    sync();
    delay(time);
    setRGB(index, 0);
    sync();
    delay(time);
#elif (PIXEL_MODE == MODE_SLAVE)
    uint8_t cmd[] = {highByte(index), lowByte(index), (px_value>>16)&0xff, (px_value>>8)&0xff, px_value&0xff, time};
    _WriteI2C(CMD_SINGLE_BLINK, cmd, 6);    
#else

#endif
}

void WS2812::blinkAll(uint32_t px_value, uint8_t time)
{
#if (PIXEL_MODE == MODE_MASTER)
    for(int i = 0; i < _leds; i++){
        setRGB(i, px_value);
    }
    sync();
    delay(time);
    clear();
    delay(time);
#elif (PIXEL_MODE == MODE_SLAVE)
    uint8_t cmd[] = {(px_value>>16)&0xff, (px_value>>8)&0xff, px_value&0xff, time};
    _WriteI2C(CMD_ALL_BLINK, cmd, 4);   
#else

#endif
}

void WS2812::colorBrush(uint32_t px_value, uint8_t time) 
{
#if (PIXEL_MODE == MODE_MASTER)
    for(uint16_t index = 0; index < numLeds(); index++) {
        setRGB(index, px_value);
        sync();
        delay(time);
    }
#elif (PIXEL_MODE == MODE_SLAVE)
    uint8_t cmd[] = {(px_value>>16)&0xff, (px_value>>8)&0xff, px_value&0xff, time};
    _WriteI2C(CMD_COLOR_BRUSH, cmd, 4);
#else

#endif
}

void WS2812::rainbow(uint8_t time) 
{
#if (PIXEL_MODE == MODE_MASTER)
    uint16_t i, j;
    for(j = 0; j < 256; j++) {
        for(i = 0; i < numLeds(); i++) {
            setRGB(i, Wheel((i+j) & 255));
        }
        sync();
        delay(time);
    }
#elif (PIXEL_MODE == MODE_SLAVE)
    uint8_t cmd[] = {time};
    _WriteI2C(CMD_RAINBOW, cmd, 1);
#else

#endif
}

void WS2812::rainbowCycle(uint8_t time) 
{
#if (PIXEL_MODE == MODE_MASTER)
    uint16_t i, j;
    for(j = 0; j < 256*5; j++) {
        for(i = 0; i < numLeds(); i++) {
            setRGB(i, Wheel(((i * 256 / numLeds()) + j) & 255));
        }
        sync();
        delay(time);
    }
#elif (PIXEL_MODE == MODE_SLAVE)
    uint8_t cmd[] = {time};
    _WriteI2C(CMD_RAINBOWCYCLE, cmd, 1);
#else

#endif
}

void WS2812::theaterChase(uint32_t px_value, uint8_t time) 
{
#if (PIXEL_MODE == MODE_MASTER)
    for (int j = 0; j < 10; j++) {
        for (int q = 0; q < 3; q++) {
            for (int i = 0; i < numLeds(); i = i+3) {
                setRGB(i+q, px_value); 
            }
            sync();
            delay(time);
            for (int i = 0; i < numLeds(); i = i+3) {
                setRGB(i+q, 0);
            }
        }
    }
#elif (PIXEL_MODE == MODE_SLAVE)
    uint8_t cmd[] = {(px_value>>16)&0xff, (px_value>>8)&0xff, px_value&0xff, time};
    _WriteI2C(CMD_THEATERCHASE, cmd, 4);
#else

#endif
}

void WS2812::theaterChaseRainbow(uint8_t time) 
{
#if (PIXEL_MODE == MODE_MASTER)
    for (int j = 0; j < 256; j++) {
        for (int q = 0; q < 3; q++) {
            for (int i = 0; i < numLeds(); i=i+3) {
                setRGB(i+q, Wheel( (i+j) % 255));
            }
            sync();
            delay(time);
            for (int i = 0; i < numLeds(); i = i+3) {
                setRGB(i+q, 0); 
            }
        }
    }
#elif (PIXEL_MODE == MODE_SLAVE)
    uint8_t cmd[] = {time};
    _WriteI2C(CMD_THEATERCHASE_RAINBOW, cmd, 1);    
#else

#endif
}

uint32_t WS2812::Wheel(byte WheelPos) 
{
    if(WheelPos < 85) {
        return RGB(WheelPos * 3, 255 - WheelPos * 3, 0);
    } else if(WheelPos < 170) {
        WheelPos -= 85;
        return RGB(255 - WheelPos * 3, 0, WheelPos * 3);
    } else {
        WheelPos -= 170;
        return RGB(0, WheelPos * 3, 255 - WheelPos * 3);
    }
}
#if (PIXEL_MODE == MODE_MASTER)
void WS2812::ISRInit()
{
	DDRD &= ~(1 << DDD3);     // Clear the PD3 pin
    PORTD |= (1 << PORTD3);    // turn On the Pull-up
    EICRA |= (1 << ISC11);    //
    EIMSK |= (1 << INT1);     // Turns on INT1
    sei();                    // turn on interrupts 
}

void WS2812::ISRService()
{
    int counter = 0;
	if(abs(millis()-timer) > 200){
        while(digitalRead(3) == LOW){
            counter++;
            delay(1000);
        }
		timer = millis();
		if(counter < 15){ //short press
			LEDCtrl = !LEDCtrl;
			digitalWrite(4, LEDCtrl);
			if(LEDCtrl){
				degree = userDegree;
			}else{
				degree = 0;
			}
		}else{
			clear();
			sync();
			delay(20);
			asm volatile ("  jmp 0"); 
		}	
	}
}
#endif

void  WS2812::ws2812_sendArray()
{
	if(!pixels) return;
	while((micros() - endTime) < 50L);
	noInterrupts();

	volatile uint16_t i = _leds*3;
	volatile uint8_t *ptr = pixels; 
	volatile uint8_t b = *ptr++;
	volatile uint8_t hi;             
	volatile uint8_t lo;             

#if (F_CPU >= 7400000UL) && (F_CPU <= 9500000UL)
    volatile uint8_t n1, n2 = 0;
    if(port == &PORTD){
		hi = PORTD |  pinMask;
		lo = PORTD & ~pinMask;
		n1 = lo;
		if(b & 0x80) n1 = hi;
		asm volatile(
        "headD:"                   "\n\t"
        // Bit 7:
        "out  %[port] , %[hi]"    "\n\t" 
        "mov  %[n2]   , %[lo]"    "\n\t"
        "out  %[port] , %[n1]"    "\n\t"
        "rjmp .+0"                "\n\t" 
        "sbrc %[byte] , 6"        "\n\t" 
         "mov %[n2]   , %[hi]"    "\n\t" 
        "out  %[port] , %[lo]"    "\n\t" 
        "rjmp .+0"                "\n\t" 
        // Bit 6:
        "out  %[port] , %[hi]"    "\n\t" 
        "mov  %[n1]   , %[lo]"    "\n\t" 
        "out  %[port] , %[n2]"    "\n\t" 
        "rjmp .+0"                "\n\t" 
        "sbrc %[byte] , 5"        "\n\t" 
         "mov %[n1]   , %[hi]"    "\n\t" 
        "out  %[port] , %[lo]"    "\n\t" 
        "rjmp .+0"                "\n\t" 
        // Bit 5:
        "out  %[port] , %[hi]"    "\n\t" 
        "mov  %[n2]   , %[lo]"    "\n\t" 
        "out  %[port] , %[n1]"    "\n\t" 
        "rjmp .+0"                "\n\t" 
        "sbrc %[byte] , 4"        "\n\t" 
         "mov %[n2]   , %[hi]"    "\n\t" 
        "out  %[port] , %[lo]"    "\n\t" 
        "rjmp .+0"                "\n\t" 
        // Bit 4:
        "out  %[port] , %[hi]"    "\n\t" 
        "mov  %[n1]   , %[lo]"    "\n\t" 
        "out  %[port] , %[n2]"    "\n\t" 
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 3"        "\n\t" 
         "mov %[n1]   , %[hi]"    "\n\t" 
        "out  %[port] , %[lo]"    "\n\t" 
        "rjmp .+0"                "\n\t" 
        // Bit 3:
        "out  %[port] , %[hi]"    "\n\t" 
        "mov  %[n2]   , %[lo]"    "\n\t" 
        "out  %[port] , %[n1]"    "\n\t" 
        "rjmp .+0"                "\n\t" 
        "sbrc %[byte] , 2"        "\n\t" 
         "mov %[n2]   , %[hi]"    "\n\t" 
        "out  %[port] , %[lo]"    "\n\t" 
        "rjmp .+0"                "\n\t" 
        // Bit 2:
        "out  %[port] , %[hi]"    "\n\t" 
        "mov  %[n1]   , %[lo]"    "\n\t" 
        "out  %[port] , %[n2]"    "\n\t" 
        "rjmp .+0"                "\n\t" 
        "sbrc %[byte] , 1"        "\n\t" 
         "mov %[n1]   , %[hi]"    "\n\t" 
        "out  %[port] , %[lo]"    "\n\t" 
        "rjmp .+0"                "\n\t" 
        // Bit 1:
        "out  %[port] , %[hi]"    "\n\t" 
        "mov  %[n2]   , %[lo]"    "\n\t"
        "out  %[port] , %[n1]"    "\n\t" 
        "rjmp .+0"                "\n\t" 
        "sbrc %[byte] , 0"        "\n\t" 
         "mov %[n2]   , %[hi]"    "\n\t" 
        "out  %[port] , %[lo]"    "\n\t" 
        "sbiw %[count], 1"        "\n\t" 
        // Bit 0:
        "out  %[port] , %[hi]"    "\n\t" 
        "mov  %[n1]   , %[lo]"    "\n\t" 
        "out  %[port] , %[n2]"    "\n\t" 
        "ld   %[byte] , %a[ptr]+" "\n\t" 
        "sbrc %[byte] , 7"        "\n\t" 
         "mov %[n1]   , %[hi]"    "\n\t" 
        "out  %[port] , %[lo]"    "\n\t" 
        "brne headD"              "\n"
      : [byte]  "+r" (b),
        [n1]    "+r" (n1),
        [n2]    "+r" (n2),
        [count] "+w" (i)
      : [port]   "I" (_SFR_IO_ADDR(PORTD)),
        [ptr]    "e" (ptr),
        [hi]     "r" (hi),
        [lo]     "r" (lo));
    }
#elif (F_CPU >= 11100000UL) && (F_CPU <= 14300000UL)
    volatile uint8_t next;
    if(port == &PORTD) {
      hi   = PORTD |  pinMask;
      lo   = PORTD & ~pinMask;
      next = lo;
      if(b & 0x80) next = hi;
      asm volatile(
        "headD:"                   "\n\t"
        "out   %[port], %[hi]"    "\n\t" 
        "rcall bitTimeD"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeD"          "\n\t" 
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeD"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeD"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeD"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeD"          "\n\t" 
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeD"          "\n\t"
        // Bit 0:
        "out  %[port] , %[hi]"    "\n\t"
        "rjmp .+0"                "\n\t" 
        "ld   %[byte] , %a[ptr]+" "\n\t" 
        "out  %[port] , %[next]"  "\n\t" 
        "mov  %[next] , %[lo]"    "\n\t" 
        "sbrc %[byte] , 7"        "\n\t" 
         "mov %[next] , %[hi]"    "\n\t" 
        "nop"                     "\n\t" 
        "out  %[port] , %[lo]"    "\n\t" 
        "sbiw %[count], 1"        "\n\t" 
        "brne headD"              "\n\t"
         "rjmp doneD"             "\n\t"
        "bitTimeD:"               "\n\t" 
         "out  %[port], %[next]"  "\n\t" 
         "mov  %[next], %[lo]"    "\n\t" 
         "rol  %[byte]"           "\n\t"
         "sbrc %[byte], 7"        "\n\t" 
          "mov %[next], %[hi]"    "\n\t" 
         "nop"                    "\n\t" 
         "out  %[port], %[lo]"    "\n\t" 
         "ret"                    "\n\t"
         "doneD:"                 "\n"
        : [byte]  "+r" (b),
          [next]  "+r" (next),
          [count] "+w" (i)
        : [port]   "I" (_SFR_IO_ADDR(PORTD)),
          [ptr]    "e" (ptr),
          [hi]     "r" (hi),
          [lo]     "r" (lo));
    }
#elif (F_CPU >= 15400000UL) && (F_CPU <= 19000000L)
    volatile uint8_t next, bit;
    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    next = lo;
    bit  = 8;
    asm volatile(
      "head20:"                   "\n\t" 
      "st   %a[port],  %[hi]"    "\n\t" 
      "sbrc %[byte],  7"         "\n\t"
       "mov  %[next], %[hi]"     "\n\t" 
      "dec  %[bit]"              "\n\t" 
      "st   %a[port],  %[next]"  "\n\t"
      "mov  %[next] ,  %[lo]"    "\n\t"
      "breq nextbyte20"          "\n\t"
      "rol  %[byte]"             "\n\t" 
      "rjmp .+0"                 "\n\t"
      "nop"                      "\n\t"
      "st   %a[port],  %[lo]"    "\n\t"
      "nop"                      "\n\t"
      "rjmp .+0"                 "\n\t"
      "rjmp head20"              "\n\t"
     "nextbyte20:"               "\n\t" 
      "ldi  %[bit]  ,  8"        "\n\t" 
      "ld   %[byte] ,  %a[ptr]+" "\n\t" 
      "st   %a[port], %[lo]"     "\n\t" 
      "nop"                      "\n\t" 
      "sbiw %[count], 1"         "\n\t" 
       "brne head20"             "\n"
      : [port]  "+e" (port),
        [byte]  "+r" (b),
        [bit]   "+r" (bit),
        [next]  "+r" (next),
        [count] "+w" (i)
      : [ptr]    "e" (ptr),
        [hi]     "r" (hi),
        [lo]     "r" (lo));
#else
	#error "CPU SPEED NOT SUPPORTED"
#endif
	interrupts();
	endTime = micros();
}