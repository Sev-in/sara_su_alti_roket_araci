#!/usr/bin/env python3
"""Uno/FTDI USB hattini komut vermeden uzun sure kararlilik testine alir."""

import argparse
import sys
import time

import serial


READY_LINE = 'READY,SARA_UNO_CONTROLLER_V2'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--duration', type=float, default=600.0)
    args = parser.parse_args()

    initial_calibration_seen = False
    restart_count = 0
    ready_count = 0
    telemetry_count = 0
    ready = False
    started = time.monotonic()
    next_heartbeat = started
    next_report = started + 10.0

    print(
        f'{args.port}@{args.baud} tek kez aciliyor; '
        f'{args.duration:.0f} saniye izlenecek.')
    print('Bu test servo, ESC, siringa veya atesleme komutu gondermez.')

    try:
        with serial.Serial(
            args.port, args.baud, timeout=0.1, write_timeout=1.0
        ) as connection:
            while time.monotonic() - started < args.duration:
                now = time.monotonic()
                raw = connection.readline()
                if raw:
                    line = raw.decode('utf-8', errors='replace').strip()
                    if line == 'INFO,CALIBRATION_START':
                        if ready_count == 0:
                            if not initial_calibration_seen:
                                print(
                                    f'[{now - started:7.1f}s] '
                                    'ilk CALIBRATION_START')
                            initial_calibration_seen = True
                        else:
                            restart_count += 1
                            print(
                                f'[{now - started:7.1f}s] '
                                f'YENIDEN BASLAMA #{restart_count}: '
                                'CALIBRATION_START')
                        ready = False
                    elif line == 'INFO,CALIBRATION_DONE':
                        print(f'[{now - started:7.1f}s] CALIBRATION_DONE')
                    elif line == READY_LINE:
                        ready_count += 1
                        ready = True
                        print(
                            f'[{now - started:7.1f}s] '
                            f'READY #{ready_count}')
                    elif line.startswith('ERR,'):
                        print(f'[{now - started:7.1f}s] UNO {line}')
                    elif line.startswith('voltage_v='):
                        telemetry_count += 1

                if ready and now >= next_heartbeat:
                    connection.write(b'HB\n')
                    connection.flush()
                    next_heartbeat = now + 0.2

                if now >= next_report:
                    print(
                        f'[{now - started:7.1f}s] izleniyor: '
                        f'restart={restart_count}, '
                        f'ready={ready_count}, '
                        f'telemetry={telemetry_count}')
                    next_report = now + 10.0
    except serial.SerialException as exc:
        elapsed = time.monotonic() - started
        print(
            f'[{elapsed:7.1f}s] USB/SERI HATASI: {exc}',
            file=sys.stderr)
        return 2

    print(
        'TEST TAMAMLANDI: '
        f'restart={restart_count}, ready={ready_count}, '
        f'telemetry={telemetry_count}')
    if restart_count != 0 or ready_count != 1:
        print(
            'BASARISIZ: Ilk READY sonrasinda Uno yeniden basladi.',
            file=sys.stderr)
        return 1
    print('BASARILI: Test boyunca Uno yeniden baslamadi.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
