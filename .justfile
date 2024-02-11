fqbn := "arduino:avr:uno"
arduino_port := `ls /dev/cu.usb*`
default_sketch := "controller"

_list:
  @just -l

@build sketch=default_sketch +flags="":
  echo Building {{sketch}}
  @arduino-cli compile --libraries {{justfile_directory()}}/libraries -b {{fqbn}} {{flags}} sketches/{{sketch}}/{{sketch}}.ino

@upload sketch=default_sketch flags="":
  just build {{sketch}} --upload -p {{arduino_port}} {{flags}}

@monitor:
  arduino-cli monitor -p {{arduino_port}} --timestamp
