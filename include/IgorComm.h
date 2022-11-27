#ifndef IGOR_COMM
#define IGOR_COMM

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
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

void printStatus(int leftOutput, int rightOutput,
 double roll, long time )
{
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(.8);
    display.println("Left, Right, Roll");
    display.setCursor(0, 8);
    display.print(leftOutput);
    display.print(", ");
    display.print(rightOutput);
    display.print(", ");
    display.print(roll);
    display.setCursor(0, 56);
    display.print(time);
    display.display();
}

template <typename T>
void printLn(T text)
{
  display.clearDisplay();
  display.setCursor(0,0); 
  display.println(text);  
  display.display();
}

template <typename T>
void print(T text)
{
  display.clearDisplay();
  display.setCursor(0,0); 
  display.print(text);  
  display.display();
}

#endif