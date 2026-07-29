# SARA VIDEO_DEMO Havuz Akisi

Bu mod, `Video gösterimi için gereken şartlar.docx` belgesindeki yazılımla
gösterilebilir maddeler için hazırlanmıştır. Normal Görev 1 ve Görev 2
parametrelerini değiştirmez.

## Otonom akış

1. Pixhawk, basınç sensörü, su sensörleri ve eyleyici sürücüsü hazır olana
   kadar referanslama beklenir.
2. Operatör `start_command=True` gönderir ve kısa video-test inhibit süresi
   tamamlanır.
3. Araç ileri hareket ve kontrol yüzeyleriyle 1,10 m hedef derinliğe iner.
4. En az 65 saniye boyunca şu 68 saniyelik dizi uygulanır:
   - 12 sn düz seyir,
   - 12 sn +25 derece rota,
   - 12 sn -25 derece rota,
   - 10 sn +8 derece yunuslama,
   - 10 sn -8 derece yunuslama,
   - 12 sn yeniden düz ve dengeli seyir.
5. Burun kapağı kapalı tutularak güvenli yüzeye çıkılır.
6. Yüzey durumu 5 saniye boyunca gösterilir.
7. Kapak/3 V gösterimi kapalıysa görev tamamlanır ve D5 güç kesme kilitlenir.
8. Kapak/3 V gösterimi açıkça etkinleştirildiyse araç yüzeyde, hareketsiz ve
   olçüm firmware'i doğrulanmışken burun kapağı açılır; ardından yaklaşık
   3 V ölçüm sinyali üretilir. Onaydan sonra görev tamamlanır ve D5 kilitlenir.

## Önce simülasyon

```bash
ros2 launch sara_control video_demo.launch.py mode:=test
```

Başka bir terminalde:

```bash
ros2 topic pub --once /sara/mission_start/start_command \
  std_msgs/msg/Bool "{data: true}"
```

Durumu izlemek için:

```bash
ros2 topic echo /sara/guidance/status
ros2 topic echo /sara/safety/status
```

## Havuzda hareket ve sızdırmazlık gösterimi

Gerçek roket/ateşleyici bağlı olmadan, kapak/3 V gösterimi kapalı:

```bash
ros2 launch sara_control video_demo.launch.py \
  mode:=hardware \
  enable_ignition_demo:=false
```

Tüm durum topic'leri nominal olduktan ve havuz alanı boşaltıldıktan sonra:

```bash
ros2 topic pub --once /sara/mission_start/start_command \
  std_msgs/msg/Bool "{data: true}"
```

## Kapak ve 3 V ölçüm gösterimi

Bu çalıştırmada gerçek roket ve ateşleyici bağlı olmamalıdır. Sadece
multimetre veya akım sınırlı LED yükü kullanılmalıdır:

```bash
ros2 launch sara_control video_demo.launch.py \
  mode:=hardware \
  enable_ignition_demo:=true \
  confirm_measurement_only:=true
```

`confirm_measurement_only:=true` verilmeden 3 V gösterimi başlatılmaz.
Safety katmanı ayrıca aşağıdakilerin hepsini doğrular:

- navigasyon ve Pixhawk sağlıklı,
- hareket izni taze,
- basınç derinliği geçerli ve yüzey aralığında,
- iki su sensörünün birleşik sonucu yüzey,
- pitch açısı ±10 derece içinde,
- ileri hız en fazla 0,10 m/s,
- Uno eyleyici telemetrisi taze,
- firmware `ignition_measurement_mode=True`,
- burun kapağı açık.

## Acil durdurma gösterimi

Bu test ayrı bir çalıştırmada yapılmalıdır. Hareket başladıktan sonra:

```bash
ros2 topic pub --once /sara/safety/emergency_stop \
  std_msgs/msg/Bool "{data: true}"
```

Motor komutu sıfırlanır ve D5 kalıcı güç kesme kilidi devreye girer.
Sonraki test için Uno fiziksel olarak yeniden başlatılmalıdır.

## Videoda gösterilecek kanıtlar

- Araç en az 1 m derinlikteyken kabarcık çıkmadığını gösteren kesintisiz
  görüntü. Bu fiziksel sızdırmazlık koşulu yazılımla garanti edilemez.
- `/sara/guidance/status` içinde:
  - `maneuver_elapsed_s`,
  - `maneuver_requirement_met=True`,
  - `minimum_depth_reached=True`.
- Kapak su altındaki bütün manevra ve yüzeye çıkış boyunca kapalı.
- Yüzeyde kapak açma ve multimetre/LED üzerinde 3 V ölçümü.
- Acil durdurmada pervane/motorların durması ve D5 güç kesme durumu.
- Nihai videonun toplam süresi 2–5 dakika ve çözünürlüğü en az 720p.
