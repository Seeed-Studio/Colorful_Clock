## LED Strip WS2812

### Introduction
WS2812 is a intelligent control LED light source that the control circuit and RGB chip are integrated in a package of 5050 components. It internal include intelligent digital port data latch and signal reshaping amplification drive circuit. For more information, you can visit our [wiki page]().

### Feature
+ Integrating control circuit and RGB chip
+ Built-in signal reshaping circuit
+ Built-in electric rest circuit and power lost reset circuit
+ 256 brightness and 16777216 color display

### Interface
***Init the WS2812***

    void init(uint8_t pin);

***Basic Function***

    void setRGB(uint16_t index, uint32_t px_value);
    void setBrightness(uint8_t brightness);
    void clear();
    void sync();
    ……    

***Funny Interface***

    void scrolling(uint32_t px_value, uint8_t time);
    void scrolling(uint8_t size, uint32_t px_value, uint8_t time);
    void passingBy(uint8_t size, uint32_t px_value1, uint32_t px_value2, uint8_t time);
    void blink(uint16_t index, uint32_t px_value, uint8_t time);
    void blinkAll(uint32_t px_value, uint8_t time);
    void colorBrush(uint32_t color, uint8_t time);
    ……

### Getting Started
We offer two modes to control the LED Strip.<br>
**1. Slave Mode** --- control the LED Strip via IIC. Usually, you can use the Xadow Main Board as the master, and the Xadow LED Strip WS2812 used as a slave. To choose this mode, you need to modify the WS2812.h as follow:<br>

    #define PIXEL_MODE    MODE_SLAVE
    
**2. Master Mode** --- Xadow Main Board is no need. You can just use Xadow LED Strip WS2812 to drive the LED. To choose this mode, you need to modify the WS2812.h as follow:<br>

    #define PIXEL_MODE    MODE_MASTER


Please take the example sketches in examples folder as reference, have fun!

----
This software is written by lawliet zou (![](http://www.seeedstudio.com/wiki/images/f/f8/Email-lawliet.zou.jpg)) for [Seeed Technology Inc.](http://www.seeed.cc) and is licensed under The GPL v3 License. Check License.txt for more information.<br>

Contributing to this software is warmly welcomed. You can do this basically by [forking](https://help.github.com/articles/fork-a-repo), committing modifications and then [pulling requests](https://help.github.com/articles/using-pull-requests) (follow the links above for operating guide). Adding change log and your contact into file header is encouraged.<br>
Thanks for your contribution.

Seeed is a hardware innovation platform for makers to grow inspirations into differentiating products. By working closely with technology providers of all scale, Seeed provides accessible technologies with quality, speed and supply chain knowledge. When prototypes are ready to iterate, Seeed helps productize 1 to 1,000 pcs using in-house engineering, supply chain management and agile manufacture forces. Seeed also team up with incubators, Chinese tech ecosystem, investors and distribution channels to portal Maker startups beyond.

[![Analytics](https://ga-beacon.appspot.com/UA-46589105-3/LED_Strip_WS2812)](https://github.com/igrigorik/ga-beacon)

