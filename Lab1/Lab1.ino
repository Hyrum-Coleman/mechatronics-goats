#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ArduinoSTL.h>

using namespace std;

unsigned long time = 0;
unsigned long timeOld = 0;

int main() {
  init(); // Needed to get the board moving since we're not using setup() and loop()
  Serial.begin(9600);

  vector<int> nums = {1, 3, 5, 0, 12};
  vector<int> odd_vals;
  auto pred = [](int num){
    auto ret = !(num % 2);
    cout << "In function return val: " << ret << endl;
    return ret;
  };

  remove_copy_if(nums.begin(), nums.end(), std::back_inserter(odd_vals), pred);

  for (auto val : odd_vals) {
    cout << val << " ";
  }
  cout << endl;

  time = millis();

  while (true) {
    
  }
}