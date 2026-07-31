# SARA donanim araclari

Bu klasordeki programlar ana ROS 2 gorev zincirinin parcasi degildir.
Ayni seri porta birden fazla program baglanmamalidir.

## Uno yuzey servosu testi

Arduino Uno D6 pinine bagli siringa/yuzey servosunu ROS 2 olmadan test eder:
Testte 1200 us iceri cekme ve 1800 us disari itme yonleri sirayla uygulanir;
normal gorev yazilimi yalnizca 1800 us disari itme yonunu kullanir.

```bash
python3 tools/uno_surface_servo_test.py --port /dev/ttyUSB0
```

Test sirasinda `actuator_driver`, Arduino Serial Monitor veya baska bir
seri port programi calismamalidir.

## Uno USB kararlilik testi

Servo, ESC, atesleme veya siringa komutu gondermeden Uno/FTDI hattini izler:

```bash
python3 tools/uno_usb_stability_test.py --duration 600
```

Bu test sirasinda da `/dev/ttyUSB0` portunun tek sahibi bu program olmalidir.
