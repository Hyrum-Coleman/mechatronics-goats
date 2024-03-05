#include <Adafruit_APDS9960.h>

Adafruit_APDS9960 apds;

void setup() {
  Serial.begin(9600);

  apds.enable();

  if (!apds.begin()) {
    Serial.println("Initialization Failed :("); // might need the Adafruit_BusIO library idk tho. it's in big L Libraries
  }

  apds.enableGesture(false);
  apds.enableColor();
  // put your setup code here, to run once:

}

void loop() {
  static uint16_t r;
  static uint16_t g;
  static uint16_t b;
  static uint16_t c;

  if (!apds.colorDataReady()) {
    return;
  }

  Serial.println("Color Detected!");

  apds.getColorData(r, g, b, c);

  Serial.print("RGB: (");
  Serial.print(r);
  Serial.print(", ");
  Serial.print(g);
  Serial.print(", ");
  Serial.print(b);
  Serial.print(") Clear Channel: ");
  Serial.println(c);
}
