#include "Sound.h"

Sound::Sound(byte read_pin, byte write_pin) : sp(read_pin, write_pin) {
  sp.begin(SOUND_BAUD);
};

void Sound::play(byte index) {
  sp.write('p');
  sp.write(index);
}

void Sound::trigger(byte index) {
  sp.write('t');
  sp.write(index);
}

void Sound::volume(byte percent) {
  byte new_volume = map(min(percent, 100), 0, 100, SOUND_MIN, SOUND_MAX);
  sp.write('v');
  sp.write(new_volume);
}
