#include <Arduino.h>
#include <Wire.h>
#include <iostream>
#include <sstream>
#include <cassert>
uint64_t simulated_us=0;
int applied_dac=0, raw_adc=0;
FakeSerial Serial;
FakeWire Wire, Wire1;
#include SKETCH_FILE
int main() {
  setup();
  std::string line;
  while (std::getline(std::cin,line)) {
    std::istringstream input(line);
    char operation; input >> operation;
    if (operation=='W') {
      std::string hex; input >> hex;
      for (size_t i=0;i<hex.size();i+=2) Serial.input += (char)std::stoi(hex.substr(i,2),nullptr,16);
    } else if (operation=='T') {
      int us; input >> us;
      for (int elapsed=0;elapsed<us;elapsed+=100) { simulated_us+=100; loop(); }
    } else if (operation=='C') input >> Serial.capacity;
    else if (operation=='A') std::cout << "# APPLIED," << applied_dac << '\n';
    std::cout << Serial.output;
    Serial.output.clear();
    // Serial output can end partway through a record; frame harness separately.
    std::cout << "\x1e" << std::endl;
  }
}
