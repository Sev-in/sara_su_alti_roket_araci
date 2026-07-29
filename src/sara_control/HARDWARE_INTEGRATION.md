# SARA Donanim Entegrasyonu

Havuz görev kabiliyeti videosu için ayrı ve normal görevlerden yalıtılmış
çalıştırma akışı `VIDEO_DEMO.md` dosyasında açıklanmıştır.

## Baglanti sahipligi

- Pixhawk 6C -> Jetson USB -> MAVROS -> `/mavros/imu/data`
- SEN0368 burun -> Jetson BOARD pin 15 (aktif LOW)
- SEN0368 kuyruk -> Jetson BOARD pin 16 (aktif LOW)
- Arduino Uno -> Jetson `/dev/ttyUSB0`, 115200 baud
- SEN0257 -> Uno A0
- PCA9685 -> Uno A4/A5
- Siringa servosu sinyal -> Uno D6
- Siringa servosu guc -> harici regule 5-6 V (Uno ile ortak GND)
- Ana guc kesme karti kontrol -> Uno D5 (`LOW=enerji var`, `HIGH=kes`)
- X9C103S INC/U-D/CS -> Uno D8/D9/D10
- LM2596 enable -> Uno D7 (aktif LOW)
- Atesleme fiziksel inhibit / manuel test butonu -> Uno D2 ile GND
  arasinda (`INPUT_PULLUP`; normal gorevde basili=atesleme bloke)

Bir fiziksel kaynagi yalniz bir node acar. Uno seri portunun tek sahibi
`actuator_driver`; SEN0257 telemetrisini de bu node yayinlar.

## PCA9685 kanallari

| Kanal | Islev |
|---|---|
| CH1 | Pitch kanat servosu |
| CH2 | Yaw kanat servosu |
| CH3 | Burun kapagi servosu |
| CH4 | Cift yonlu ESC |

PCA9685 CH5 kullanilmaz. Siringa/yuzeye cikis servosunun yalnizca
sinyal kablosu Uno D6'ya baglanir. Servo Uno 5 V pininden beslenmez;
harici servo beslemesinin GND hatti Uno GND ile ortaklanir.
Yuzeye cikis komutu aktifken D6 sinyali 500 ms boyunca 1200 us, sonra
200 ms boyunca 1500 us notr olacak sekilde kesintisiz tekrarlar.
Heartbeat kaybi, `SURFACE_STOP` veya `ALL_STOP` servoyu 1500 us notr
konumuna getirir.
Bu davranis bench testinde, Gorev 1 `G1_YUZEYE_CIKIS` fazinda ve Gorev 2
`G2_GUVENLI_ATIS_BOLGESI`/`G2_TIRMANIS_35_DERECE` fazlarinda etkindir.
Gorev 2 aci-yuzey dogrulamasina gecildiginde servo durdurulur.

## Atesleme cikisi olcum modu

Firmware su anda `ATESLEME_OLCUM_TEST_MODU=true` ile derlenir ve gercek
atesleyici yerine yalniz multimetre/uygun olcum yuku kullanilir. Acilista
D7 HIGH tutularak LM2596 kapatilir; X9C103S once kalibre edilir, sonra
`X9C_3V_ADIMI=35` konumuna getirilir.

Normal gorevde 3 V cikisi yalniz safety katmaninin tum
yuzey/aci/mesafe kosullarini dogrulayip `launch_command=True` uretmesi,
heartbeat'in guncel olmasi ve D2 fiziksel inhibit butonunun basili
OLMAMASI halinde acilir. Cikis en fazla 1 saniye etkin kalir. D2
basilirsa, heartbeat kaybolursa, `ALL_STOP` veya `IGNITION=0` gelirse
D7 hemen HIGH olur ve cikis 0 V'a doner.

Eski D2 manuel olcum davranisi silinmemistir; yalniz normal gorevden
ayrilmistir. Bench launch'ta acikca etkinlestirilir:

```bash
ros2 launch sara_control imu_actuator_bench.launch.py \
  enable_motor:=false \
  enable_ignition_manual_test:=true
```

Bu modda D2 birakildiginda 0 V, basildiginda yaklasik 3 V gorulur.
Actuator driver manuel olcum boyunca ESC motor komutunu zorla sifirlar.
`ALL_STOP` sonrasinda yeniden acilmasi icin D2 once birakilip tekrar
basilmalidir. Telemetride `ignition_manual_test_active`,
`ignition_button_pressed` ve `ignition_output_active` alanlari izlenir.
Manuel test cikisi otomatik Gorev 2 atesleme onayi sayilmaz.

## ROS veri yolu

`sensor_get_data` yalniz GPIO su sensorlerini ve MAVROS IMU koprusunu
yonetir. `actuator_driver` Uno basinc telemetrisini `/sara/pressure`
olarak yayinlar ve yalniz `safety_node` ciktilarini Uno'ya yollar.

Filtre zinciri korunur:

1. Uno: 16 ornek ortalama + birinci derece LPF.
2. Navigation: fiziksel aralik/sicrama kontrolu, Hampel/median ve
   zaman-duyarli LPF.
3. Yuzey P0 kalibrasyonu: arac kuru, hareketsiz ve basinc kararliyken.
4. Su sensorleri: navigation/guidance/safety katmanlarinda debounce.
5. IMU: quaternion normu, fiziksel ivme/gyro araligi ve zaman asimi.

Standalone `PixhawkUnoServoNode` ayni anda calistirilmamalidir; IMU'dan
dogrudan servo urettigi icin autopilot ve safety katmanlarini baypas eder.

## Ana guc kesme

`guidance_node`, Gorev 1/Gorev 2 normal tamamlanmasinda veya
`GUVENLI_SONLANDIRMA` durumunda `task_complete=True` yayinlar.
Ek olarak `emergency_stop=True`, motor ve servolari durdurduktan sonra
ayni kilidi dogrudan etkinlestirir. `safety_node` bu olaylari geri alinamayan
`/sara/safety/main_power_cutoff_command=True` kilidine cevirir.
`actuator_driver`, Uno'ya bir kez `POWER_CUTOFF` yollar. Uno once ESC'yi
notre alir, kanatlari merkezler, siringayi ve ateslemeyi durdurur; sonra
D5'i HIGH yapip yazilimsal olarak kilitler. Ayni calisma oturumunda hicbir
seri komut D5'i tekrar LOW yapamaz.

D5 cikisinda acilis/reset sirasinda kararsiz seviye olusmamasi icin guc
kesme kartinin girisinde uygun bir harici pull-down kullanilmalidir.
Yazilim kilidi Uno resetlenince temizlenir; resetten sonra da kesik
kalmasi isteniyorsa kart donanimsal mandallama icermelidir.

Tum sistem tezgah testinde guc kesme varsayilan olarak kapalidir. Test
yalniz guvenli olcum yuku ile su sekilde yetkilendirilir:

```bash
ros2 launch sara_control imu_actuator_bench.launch.py \
  enable_motor:=false \
  enable_surface_servo:=true \
  enable_power_cutoff_test:=true \
  confirm_power_cutoff_test_safe:=true
```

Tum sensor ve servo kontrolleri bittikten sonra testin son adimi:

```bash
ros2 topic pub --once /sara/bench/power_cutoff_request \
  std_msgs/msg/Bool "{data: true}"
```

Bu komut geri alinamaz; bench hareket isteklerini sifirlar,
`task_complete=True` yayinlar ve normal safety/actuator_driver/Uno
zinciri D5'i HIGH yapar.

## Jetson gereksinimleri

```bash
sudo apt install python3-serial
sudo usermod -aG dialout $USER
```

Jetson oturumu yeniden acildiktan sonra:

```bash
ls -l /dev/ttyUSB0 /dev/ttyACM0
ros2 launch sara_control sara_system.launch.py mode:=hardware mission_id:=1
```

Donanimi enerjilemeden once `serial_port`, servo yonleri, ESC notr/tam
itki darbeleri ve LM2596 enable polaritesi tezgahta dogrulanmalidir.
