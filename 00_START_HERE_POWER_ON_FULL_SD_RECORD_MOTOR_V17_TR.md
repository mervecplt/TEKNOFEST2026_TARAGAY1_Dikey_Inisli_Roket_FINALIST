# POWER-ON FULL SD RECORD + MOTOR V17

POWER_ON_FULL_SD_RECORD tabani korunmustur.

Yalnizca SD logger'a read-only motor telemetrisi eklendi:
- Motor gercek P110 ADC / hedef ADC
- Tur bilgisi (log icin 1023 referansli)
- Motor yonu, PWM, state/result, powered time, speed ve ilgili P110 alanlari
- 384 byte frame, 30 Hz ve DMA yolu ayni
- Power-on kayit mantigi ayni; PE9 SD kaydini gate etmez

Kontrol algoritmasi, motor surme kodu, RCS, PE9 interlock, sensorler ve scheduler degistirilmedi.
