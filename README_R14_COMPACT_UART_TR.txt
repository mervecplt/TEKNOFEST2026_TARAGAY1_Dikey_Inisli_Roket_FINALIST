TARAGAY-T1 R14 COMPACT UART BENCH SURUMU

TABAN:
- Bir onceki calisan TARAGAY_T1_R4R3_R14P3B_R14P4_STM32_3TURN_PE9_RCS surumu.
- Needle/RCS/control dosyalari degistirilmedi.

NEDEN:
- Extended TGY73 362-field UART surumu ile saha testinde actuator davranisi onceki
  calisan surumden ayrildi. Bu surum uzun frame'i kullanmaz.

UART:
- $TGY14 compact frame
- 115200 8N1 / DMA TX / 10 Hz
- Yaklasik 35 field; heavy TGY73 yerine minimum bench gozlemi.
- Kontrol kararina UART verisi girmez.

KONTROL:
- Max needle = 3 turn
- PE9 oncesi main valve = 0
- PE9 oncesi RCS = off
- PE9 sonrasi thrust-ID ve RCS aktif

Monitor:
python monitor_uart_taragay_r14_compact.py --port COM8
