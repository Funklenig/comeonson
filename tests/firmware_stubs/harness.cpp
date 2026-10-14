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
      const uint64_t end=simulated_us+us;
      while (simulated_us<end) {
#if defined(ARDUINO_UNOR4_WIFI) && defined(NO_USB)
        simulated_us+=1; // UART writes advance the clock by a byte time.
#else
        simulated_us+=100;
#endif
        loop();
      }
    } else if (operation=='B') std::cout << "# BAUD," << Serial.baud << '\n';
    else if (operation=='Z') { int baud; input >> baud; assert(baud>0); Serial.begin(baud); }
    else if (operation=='C') input >> Serial.capacity;
    else if (operation=='L') input >> Serial.write_limit;
    else if (operation=='J') input >> simulated_us;
    else if (operation=='I') {
      int bus,ch,mode; input >> bus >> ch >> mode;
      (bus==0 ? Wire : Wire1).modes.at(ch)=mode;
    } else if (operation=='D') {
      int bus,latency; input >> bus >> latency;
      (bus==0 ? Wire : Wire1).latency_us=latency;
    } else if (operation=='R') input >> Wire.flow_raw;
    else if (operation=='V') {
      std::cout << "# RECOVERY," << Wire.starts[0];
      for (int count : Wire1.starts) std::cout << ',' << count;
      std::cout << '\n';
    }
    else if (operation=='A') std::cout << "# APPLIED," << applied_dac << '\n';
    std::cout << Serial.output;
    Serial.output.clear();
    // Serial output can end partway through a record; frame harness separately.
    std::cout << "\x1e" << std::endl;
  }
}
