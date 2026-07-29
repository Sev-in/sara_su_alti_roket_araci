# SARA su alti roket araci

ROS 2 tabanli SARA kontrol yazilimi; Pixhawk 6C IMU, Arduino Uno,
PCA9685, SEN0257 basinc sensoru ve iki temassiz su sensorunu birlestirir.

## Paketler

- `sara_control`: sensor, navigasyon, gudum, PID, guvenlik ve eyleyici katmanlari
- `sara_description`: URDF, mesh ve RViz goruntuleme dosyalari
- `tools`: ROS 2 disi, izole donanim tani programlari

## Derleme

```bash
cd ~/sara_su_alti_roket_araci
colcon build --symlink-install
source install/setup.bash
```

## Simulasyon

```bash
ros2 launch sara_control sara_system.launch.py \
  mode:=test mission_id:=1
```

## Gercek donanim - guvenli baslangic

Varsayilan otomatik baslatma yerine ilk kontrolde manuel baslatma kullanin:

```bash
ros2 launch sara_control sara_system.launch.py \
  mode:=hardware mission_id:=1 auto_start:=false
```

Sistem durumu dogrulanmadan `start_command` gondermeyin. Pervane, atesleme,
ana guc kesme ve su sizdirmazligi icin fiziksel guvenlik proseduru yazilimdan
bagimsiz olarak uygulanmalidir.

## Ana dizinler

```text
src/sara_control/config/       ROS 2 parametreleri
src/sara_control/launch/       ana sistem ve bench launch dosyalari
src/sara_control/sara_control/ calisan Python dugumleri ve saf matematik
src/sara_control/firmware/     Arduino Uno firmware
src/sara_control/test/         donanim gerektirmeyen birim testleri
```

Donanim pinleri ve guc baglantilari icin
`src/sara_control/HARDWARE_INTEGRATION.md` belgesini kullanin.
