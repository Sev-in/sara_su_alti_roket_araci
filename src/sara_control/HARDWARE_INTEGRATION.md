# SARA Donanim Entegrasyonu

## Baglanti sahipligi

- Pixhawk 6C -> Jetson USB -> MAVROS -> `/mavros/imu/data`
- SEN0368 burun -> Jetson BOARD pin 15 (aktif LOW)
- SEN0368 kuyruk -> Jetson BOARD pin 16 (aktif LOW)
- Arduino Uno -> Jetson `/dev/ttyUSB0`, 115200 baud
- SEN0257 -> Uno A0
- PCA9685 -> Uno A4/A5

Bir fiziksel kaynagi yalniz bir node acar. Uno seri portunun tek sahibi
`actuator_driver`; SEN0257 telemetrisini de bu node yayinlar.

## PCA9685 kanallari

| Kanal | Islev |
|---|---|
| CH1 | Pitch kanat servosu |
| CH2 | Yaw kanat servosu |
| CH3 | Burun kapagi servosu |
| CH4 | Cift yonlu ESC |
| CH5 | Siringa/yuzeye cikis servosu |

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
