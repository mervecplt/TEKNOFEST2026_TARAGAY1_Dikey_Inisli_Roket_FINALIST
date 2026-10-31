# TARAGAY-T1 R14p3b/R14p4 STM32 entegrasyonu

Bu ZIP, mevcut R4R3 CubeIDE projesine MATLAB/Simulink'teki son hover mantığının
ilk STM32 portunu ekler. **Uçuş onayı değildir.** Önce gazsız bench, sonra kontrollü
aktüatör testleri ve en son tether/yer testleri yapılmalıdır.

## İleride ayar değiştirmek için tek ana dosya

`App/Modules/Control/TaragayFlightLogic/taragay_control_tuning.h`

Normal kontrol ayarlarını burada değiştirin. Örnek:
- 3 m hedef: `TGY_CTRL_Z_TARGET_DELTA_M`
- 4 m hard safety: `TGY_CTRL_Z_HARD_LIMIT_M`
- probe: `TGY_CTRL_ID_PROBE_START/STEP/U_MAX`
- ASCENT/CAPTURE/HOVER gainleri
- estimator ve authority eşikleri
- fiziksel vana max turu ve Cv map normalizasyonu

## Yeni dikey state

`vertical_state`:
0 INIT, 1 SELF_CHECK, 2 PREPOSITION, 3 READY, 4 ASCENT, 5 CAPTURE,
6 HOVER, 7 DEGRADED, 8 SAFE.

Eski `mission_state` alanı SD/UART geriye uyumluluk için korunmuştur.

## Gerçek uçuş girdileri

Dikey kontrol doğrudan basınç, model kütlesi veya `Gercek_Itki_N` kullanmaz.
- z / Vz: FullStateESKF
- turns_actual: gerçek needle feedback
- release_event: PE9 flight-active latch
- health_ok: taze/geçerli ESKF+IMU gate'i

## Ana vana yolu

Controller `valve_u_cmd` (0..1) üretir. Bu komut tam 0..3 tur fiziksel Cv inverse map ile
`valve_turns_target` ve sonra `valve_actuator_cmd` (0..1) değerine çevrilir.
Maksimum mekanik komut 3 turdur (195 ADC/tur -> 585 ADC).
R8R32 fiziksel handoff artık `valve_actuator_cmd` kullanır.

PE9 öncesi controller ana vana komutu 0'dır. Fiziksel aktüatör yetkisi de mevcut
`App_IsActuatorAuthorized()` PE9 gate'i ile korunur.

## STM32'de ilk kontrol edilmesi gereken telemetri

- `vertical_state`
- `release_latched`
- `valve_u_cmd`
- `valve_actuator_cmd`
- `valve_turns_target`, `valve_turns_actual`
- `valve_u_actual`
- `b_hat_mps2`, `b_auth_mps2`, `authority_ratio`
- `a_filt_mps2`, `a_cmd_mps2`
- `thrust_id_active`, `thrust_id_complete`, `thrust_id_probe_u`
- `vertical_safe_latched`, `authority_latched`

## İlk bench sırası

1. Gaz yok, ana motor hattı pasif.
2. PE9 takılı: READY'e gelmeli, `valve_u_cmd=0` olmalı.
3. Needle zero/homing doğrulanmalı; feedback turu doğru yönde artmalı.
4. PE9 simüle edilince thrust-ID probe komutu başlamalı (gazsız testte fiziksel ana
   itki hattı mekanik olarak güvenli/izole tutulmalı).
5. 4 m hard-safety ve health fault, software-injection ile önce benchte doğrulanmalı.
6. RCS ayrı pressureless relay testinden geçirilmelidir.

## RCS PE9 kuralı

Bu revizyonda RCS, PE9 öncesinde hem yazılımsal request seviyesinde hem fiziksel röle
seviyesinde tamamen kapalıdır. Preflight emergency pulse yoktur. PE9 separation latch
oluştuğu örnekten itibaren V7.13.4 RCS kontrolü çalışabilir. PE9 öncesi RCS runtime her
adım resetlendiği için PE9 kenarına taşınabilecek eski/stale pulse bulunmaz.
