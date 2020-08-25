# ATmega328_LoRa_PlantGuard

- new bootloader: OptiBoot
  pio run -t bootloader

- set fuses for 1v8 BOD:
  pio run -t fuses

- usage of EEMEM:
  generation of .pio/build/ATmega328P/firmware.eep;
  pio run -t uploadeep
  (ignore error-message about missing upload-port)

  send eep-file to at328p:
  avrdude -c avrisp2 -p m328p -B 1 -U eeprom:w:.pio/build/ATmega328P/firmware.eep

