#include "Adafruit_NeoPixel.h"
//#include "Arduino.h"
#include "SoftwareSerial.h"
//#include "DFRobotDFPlayerMini.h"
#include <DFMiniMp3.h>

#ifdef __AVR__
#include <avr/power.h>
#endif

#define PIN1 6
#define NUMPIXELS1 32
Adafruit_NeoPixel ring1 = Adafruit_NeoPixel(NUMPIXELS1, PIN1, NEO_GRB + NEO_KHZ800);

//#define PIN2 5
//#define NUMPIXELS2 16
//Adafruit_NeoPixel ring2 = Adafruit_NeoPixel(NUMPIXELS2, PIN2, NEO_GRB + NEO_KHZ800);
//
//
//#define PIN3 7
//#define NUMPIXELS2 16
//Adafruit_NeoPixel ring3 = Adafruit_NeoPixel(NUMPIXELS3, PIN3, NEO_GRB + NEO_KHZ800);


int delayval = 500; // delay for half a second

//SoftwareSerial mySoftwareSerial(10, 11); // RX, TX
//DFRobotDFPlayerMini myDFPlayer;
//void printDetail(uint8_t type, int value);

class Mp3Notify
{
  public:
    static void OnError(uint16_t errorCode)
    {
      // see DfMp3_Error for code meaning
      Serial.println();
      Serial.print("Com Error ");
      Serial.println(errorCode);
    }

    static void OnPlayFinished(uint16_t globalTrack)
    {
      Serial.println();
      Serial.print("Play finished for #");
      Serial.println(globalTrack);
    }

    static void OnCardOnline(uint16_t code)
    {
      Serial.println();
      Serial.print("Card online ");
      Serial.println(code);
    }

    static void OnCardInserted(uint16_t code)
    {
      Serial.println();
      Serial.print("Card inserted ");
      Serial.println(code);
    }

    static void OnCardRemoved(uint16_t code)
    {
      Serial.println();
      Serial.print("Card removed ");
      Serial.println(code);
    }
};

SoftwareSerial secondarySerial(10, 11); // RX, TX
DFMiniMp3<SoftwareSerial, Mp3Notify> mp3(secondarySerial);

int noisePin = A0;
int buttonPin = 2;
int buttonState = 0;
int noiseReading = 0;
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
long timer = 0;
int previousCounter = 0;
int Counter = 1;

void setup()
{
  secondarySerial.begin(9600);
  Serial.begin(115200);

  Serial.println("initializing...");

  pinMode(noisePin, INPUT);
  pinMode(buttonPin, INPUT);

  ring1.begin();
//  for (int i = 0; i < NUMPIXELS1; i++) {
//      ring1.setPixelColor(i, ring1.Color(0, 0, 0));
//    }
    ring1.show();

  mp3.begin();
  mp3.setVolume(30);

  Serial.println("starting...");

  //mySoftwareSerial.begin(9600);


  //Serial.println();
  //  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  //  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  //
  //  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
  //    Serial.println(F("Unable to begin:"));
  //    Serial.println(F("1.Please recheck the connection!"));
  //    Serial.println(F("2.Please insert the SD card!"));
  //    while(true);
  //  }
  //  Serial.println(F("DFPlayer Mini online."));

  //  myDFPlayer.setTimeOut(500); //Set serial communictaion time out 500ms
  //
  //  //----Set volume----
  //  myDFPlayer.volume(30);  //Set volume value (0~30).
  //  myDFPlayer.loop(1);
  //  myDFPlayer.pause();
}


void loop()
{

  long startMillis = millis(); // Start of sample window
  int peakToPeak = 0;   // peak-to-peak level
  int signalMax = 0;
  int signalMin = 1024;

  while (millis() - startMillis < sampleWindow)
  {
    noiseReading = analogRead(noisePin);
    if (noiseReading < 1024)  // toss out spurious readings
    {
      if (noiseReading > signalMax)
      {
        signalMax = noiseReading;  // save just the max levels
      }
      else if (noiseReading < signalMin)
      {
        signalMin = noiseReading;  // save just the min levels
      }
    }
  }
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  Serial.print ("Volume: ");
  Serial.print (peakToPeak);

  buttonState = digitalRead(buttonPin);
  Serial.print ("; Button: ");
  Serial.print (buttonState);

  Serial.print ("; Timer: ");
  Serial.println (millis() - timer);


  if ( peakToPeak >= 45 && buttonState == HIGH ) {
    //mp3.playMp3FolderTrack(1);
    mp3.playRandomTrackFromAll();
    previousCounter = Counter;
    mp3.pause();
  }

  if (previousCounter == Counter) {
    for (int i = 0; i < 10; i++) {
      mp3.start();
      delay(1000);
      mp3.pause();
      for (int j = 0; j < NUMPIXELS1; j++) {
        ring1.setPixelColor(j, ring1.Color(255, 20 * i, 0));
        //   ring2.setPixelColor(j, ring2.Color(255,20*i,0));
        //   ring3.setPixelColor(j, ring3.Color(255,20*i,0));
      }
      ring1.show();
      //   ring2.show();
      //   ring3.show();
    }
    previousCounter = 0;
  } else {
    for (int i = 0; i < NUMPIXELS1; i++) {
      ring1.setPixelColor(i, ring1.Color(0, 0, 0));
      //        ring2.setPixelColor(i, ring2.Color(0,0,0));
      //        ring3.setPixelColor(i, ring3.Color(0,0,0));
    }
    ring1.show();
    //      ring2.show();
    //      ring3.show();
  }



  delay( 100 );
}


void rainbow(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256; j++) {
    for (i = 0; i < ring1.numPixels(); i++) {
      ring1.setPixelColor(i, Wheel((i + j) & 255));
    }
    ring1.show();
    delay(wait);
  }
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return ring1.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return ring1.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return ring1.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}


