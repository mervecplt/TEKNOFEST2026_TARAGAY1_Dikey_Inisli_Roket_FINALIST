# TARAGAY T1 - MOTOR ADC / TUR SD LOG FIX

Bu paket, kullanicinin `TARAGAY_T1_POWER_ON_FULL_SD_RECORD_MOTOR_V17(1)` klasoru uzerinden hazirlandi.

## Degisen tek firmware kaynak dosyasi
- `App/Services/SDLogger/sd_logger.c`

Kontrol algoritmasi, motor surme state-machine'i, P83/P110/P112 kontrolu, RCS, PE9, sensorler ve scheduler degistirilmedi.

## Neden degisiklik yapildi?
Aktif uretim motor yolu P83/P110/P112 iken eski `needle_valve_*` compatibility alanlari bazi durumlarda sabit kalabiliyordu. Bu nedenle motor fiziksel olarak hareket etse bile SD CSV'deki genel `needle_*` alanlari hareketi gostermeyebiliyordu.

## Yeni log davranisi
SD logger artik sadece OKUMA amaciyla:
- gercek konum ADC'sinde oncelikle `p83_filtered_adc` kullanir,
- aktif motor PWM/yon bilgisini `NeedleValveIntegrationTest_GetTelemetryStatus()` read-only telemetrisinden alir,
- `needle_adc`, `needle_target_adc`, `needle_error_adc` alanlarini gercek motor telemetrisiyle doldurur,
- `needle_position_turns_direct` hesabini gercek ADC / kapali referans / 195 ADC-tur bilgisi ile yapar,
- V17 `p110_current_adc` ve `p110_target_adc` kolonlarini fiziksel/live log gorunumu ile doldurur.

Bu bilgiler kontrol geri beslemesine verilmez. SD logger sadece kopyalar.

## Dogrulama
- Host `gcc -fsyntax-only`: PASS
- Yuklenen orijinal klasor ile kaynak karsilastirmasi: sadece `sd_logger.c` degisti.
- Debug/Release klasorleri paketten cikarildi; CubeIDE'de Clean/Build alin.
