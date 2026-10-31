# POWER-ON FULL SD RECORD

Bu paket kullanıcının gönderdiği `TARAGAY_T1_R4R3_NRF_RX_ONLY_GROUND100HZ_RF250K_XY_ONLY_FIX (2)` klasöründen hazırlanmıştır.

## Yapılan tek çalışma mantığı değişikliği
`App/Services/SDLogger/sd_logger.c` içindeki PE9 kayıt kapıları kaldırıldı.

Yeni kayıt akışı:
1. Anahtar açılır, STM32 enerji alır.
2. Mevcut SD başlangıç kodu kartı mount eder, `flight.bin` dosyasını açar ve preallocate eder.
3. TIM5 logger timerı başlar başlamaz 30 Hz kayıt başlar.
4. Konektör/PE9 açılmadan önce de kayıt devam eder.
5. PE9 açıldığında kayıt kesilmez.
6. Motor/RCS/aktif kontrol sırasında kayıt devam eder.
7. STOP komutu yalnız motor/RCS kontrolünü etkiler; SD kaydı devam eder.
8. Ana güç kesilene kadar SD logger çalışır.

## Kesinlikle değiştirilmedi
- Flight/control algoritması
- Needle valve / motor algoritması
- RCS algoritması
- PE9 güvenlik/interlock mantığı
- Sensör algoritmaları
- Scheduler oranları ve kontrol zamanlamaları
- SD frame boyu, 30 Hz logger hızı, DMA buffer boyları
- SD'nin flight sırasında fail-open kontrol güvenliği

## Güç kesme notu
Sistem fiziksel güç kesimi ile kapatıldığı için `SDLogger_Stop()` çalıştırılamaz. Mevcut tasarım bunun için dosyayı önceden ayırır ve decoder geçerli magic/sequence/CRC framelerini kurtarır. Son DMA tamponundaki henüz karta gitmemiş birkaç frame fiziksel ani güç kesiminde kaybolabilir; bu, yazılımla tamamen sıfırlanamaz.
