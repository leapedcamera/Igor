#ifndef IGOR_COMM
#define IGOR_COMM

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void Countdown(int nCount)
{
    // Clear the buffer
    display.setTextSize(2);
    display.clearDisplay();
    display.setCursor(0, 0);
    for( int i = 0; i < nCount; i++)
    {
        display.println( i );
        display.display();
        delay(1000);
        display.clearDisplay();
    }
    display.println( String("Start") );
    display.display();
}

#endif