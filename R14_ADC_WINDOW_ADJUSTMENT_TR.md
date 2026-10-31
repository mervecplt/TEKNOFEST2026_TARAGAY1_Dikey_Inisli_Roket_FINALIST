# R14 ADC pencere ayari

Taban: TARAGAY_T1_BASELINE_P110_24MS_FAST_FEEDBACK

Bu revizyonda motor surme yolu, PWM limitleri, P110 24 ms breakaway aramasi,
Taragay flight logic, thrust-ID, RCS ve UART formati degistirilmedi.

Yalniz iğne vana pozisyon kabul penceresi bir miktar genisletildi:
- P110 hedef toleransi: +/-8 ADC -> +/-12 ADC
- P110 settled kabul: +/-12 ADC -> +/-16 ADC
- P110 hard overshoot guard: 12 ADC -> 20 ADC
- Autonomous target-change toleransi: 8 ADC -> 12 ADC

195 ADC/tur icin:
- 12 ADC ~= 0.062 tur
- 16 ADC ~= 0.082 tur
- 20 ADC ~= 0.103 tur

3 tur / 585 ADC mekanik limit, maksimum PWM=192 ve safety travel limitleri aynidir.
