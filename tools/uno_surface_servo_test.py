#!/usr/bin/env python3
"""Arduino Uno D6'ya bagli siringa servosunu ROS 2 olmadan test eder.

Bu program ROS2 kullanmaz. sara_uno_controller.ino seri protokoluyle
uyumludur:

* Uno READY olana kadar bekler.
* Heartbeat'i 5 Hz gonderir.
* SURFACE_TEST_START ile D6 servosunun iki yonunu sirayla dener:
  1100 us -> 1500 us -> 1900 us -> 1500 us.
* Ctrl+C, sure dolumu veya hata halinde servoyu ve tum cikislari durdurur.

actuator_driver veya Arduino Serial Monitor ayni anda calistirilmamalidir.
"""

import argparse
import os
import sys
import time

try:
    import serial
except ImportError:
    print(
        'HATA: pyserial bulunamadi. Ubuntu: sudo apt install python3-serial',
        file=sys.stderr,
    )
    raise SystemExit(2)


DEFAULT_BY_ID = (
    '/dev/serial/by-id/'
    'usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0'
)
FALLBACK_PORT = '/dev/ttyUSB0'
EXPECTED_READY = 'READY,SARA_UNO_CONTROLLER_V2'
NON_FATAL_ERRORS = {
    'ERR,SERVO_COMMAND_TIMEOUT_CENTERED',
}


def default_port():
    """Jetson'da kalici by-id yolunu, yoksa ttyUSB0'i sec."""
    if os.path.exists(DEFAULT_BY_ID):
        return DEFAULT_BY_ID
    return FALLBACK_PORT


def send_command(connection, command):
    """Uno'ya satir sonlu ASCII komutu gonder."""
    connection.write((command + '\n').encode('ascii'))
    connection.flush()


def read_line(connection):
    """Varsa bir Uno satiri oku; bozuk karakterleri guvenle atla."""
    raw = connection.readline()
    if not raw:
        return ''
    return raw.decode('ascii', errors='replace').strip()


def wait_until_ready(connection, timeout_s):
    """Uno kalibrasyonunun tamamlanmasini bekle."""
    deadline = time.monotonic() + timeout_s
    telemetry_seen = False

    while time.monotonic() < deadline:
        line = read_line(connection)
        if not line:
            continue

        print(f'UNO: {line}')
        if line == EXPECTED_READY:
            return
        if line.startswith('READY,'):
            raise RuntimeError(f'Desteklenmeyen Uno firmware surumu: {line}')
        if line.startswith('voltage_v='):
            telemetry_seen = True
            break

    if telemetry_seen:
        print('UNO zaten calisiyor; telemetri goruldu ve test devam ediyor.')
        return
    raise TimeoutError(
        f'Uno {timeout_s:.1f} saniye icinde READY olmadi.'
    )


def stop_surface_servo(connection):
    """D6 servosunu ve diger cikislari guvenli duruma getir."""
    try:
        send_command(connection, 'HB')
        send_command(connection, 'SURFACE_STOP')
        send_command(connection, 'ALL_STOP')
    except Exception as exc:
        print(f'UYARI: guvenli durdurma komutu gonderilemedi: {exc}')


def run_test(port, baud, ready_timeout_s, duration_s):
    """Seri baglantiyi ac ve D6 siringa servosu testini calistir."""
    print(f'Uno aciliyor: {port} @ {baud}')
    with serial.Serial(port, baud, timeout=0.1, write_timeout=1.0) as uno:
        # Bazi Uno/FTDI baglantilari port acilinca karti yeniden baslatir.
        # Tamponu burada silmiyoruz; READY mesajini kacirmamak gerekir.
        wait_until_ready(uno, ready_timeout_s)

        send_command(uno, 'HB')
        time.sleep(0.05)
        # Onceki ROS2/bench oturumundan kalabilecek kanat zaman asimini
        # temizle ve D6 servosu disindaki kritik cikislari guvenli tut.
        send_command(uno, 'MOTOR_DISARM')
        send_command(uno, 'SERVO_CENTER')
        send_command(uno, 'IGNITION=0')
        send_command(uno, 'SURFACE_STOP')
        time.sleep(0.05)
        send_command(uno, 'SURFACE_TEST_START')
        print(
            'Arduino D6 servosunda 1100 ve 1900 us iki yon testi '
            '(aralarda 1500 us notr) baslatildi. '
            'Durdurmak icin Ctrl+C kullan.'
        )

        start_time = time.monotonic()
        next_heartbeat = start_time
        confirmed = False

        try:
            while True:
                now = time.monotonic()
                if duration_s > 0.0 and now - start_time >= duration_s:
                    print(f'{duration_s:.1f} saniyelik test tamamlandi.')
                    break

                if now >= next_heartbeat:
                    send_command(uno, 'HB')
                    next_heartbeat = now + 0.2

                line = read_line(uno)
                if line:
                    print(f'UNO: {line}')
                    if line == 'OK,SURFACE_TEST_RUNNING':
                        confirmed = True
                    if line.startswith('ERR,'):
                        if line in NON_FATAL_ERRORS:
                            print(
                                f'UYARI (test devam ediyor): {line}',
                                file=sys.stderr,
                            )
                        else:
                            raise RuntimeError(line)

                if not confirmed and now - start_time > 2.0:
                    print(
                        'UYARI: Uno SURFACE_TEST_RUNNING onayi vermedi.',
                        file=sys.stderr,
                    )
                    confirmed = True
        finally:
            stop_surface_servo(uno)
            print('D6 servosu durduruldu; Uno cikislari guvenli durumda.')


def parse_args():
    parser = argparse.ArgumentParser(
        description='SARA Arduino Uno D6 siringa servosu testi'
    )
    parser.add_argument(
        '--port',
        default=default_port(),
        help='Seri port; Jetson varsayilani by-id veya /dev/ttyUSB0',
    )
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument(
        '--ready-timeout',
        type=float,
        default=15.0,
        help='Uno READY bekleme suresi [s]',
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=0.0,
        help='Test suresi [s]; 0 ise Ctrl+C gelene kadar surekli',
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.duration < 0.0:
        raise SystemExit('--duration negatif olamaz.')
    try:
        run_test(
            args.port,
            args.baud,
            max(1.0, args.ready_timeout),
            args.duration,
        )
    except KeyboardInterrupt:
        print('\nKullanici testi durdurdu.')
    except (OSError, RuntimeError, TimeoutError, serial.SerialException) as exc:
        print(f'HATA: {exc}', file=sys.stderr)
        raise SystemExit(1)


if __name__ == '__main__':
    main()
