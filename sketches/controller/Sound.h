#ifndef Sound_h
#define Sound_h

#include <Arduino.h>
#include <SoftwareSerial.h>

#define SOUND_BAUD 38400
#define SOUND_MIN  0x80
#define SOUND_MAX  0x00

class Sound {
  public:
    Sound(byte read_pin, byte write_pin);
    void play(byte index);
    void trigger(byte index);
    void volume(byte percent);
  private:
    SoftwareSerial sp;
};

#endif
