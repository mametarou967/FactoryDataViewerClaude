import serial
import threading
import time
import datetime
import csv
import os

class LoggerService:
    def __init__(self):
        self._lock = threading.Lock()
        self._lux_red = 0
        self._lux_yellow = 0
        self._lux_green = 0
        self._current_value = 0.0
        self._running = True
        self._thread = threading.Thread(target=self._logging_loop)
        self._thread.start()

    def set_pat_light_values(self, red, yellow, green):
        with self._lock:
            self._lux_red = red
            self._lux_yellow = yellow
            self._lux_green = green

    def set_current_value(self, current):
        with self._lock:
            self._current_value = current

    def stop(self):
        self._running = False
        self._thread.join()

    def _logging_loop(self):
        print("logging start")
        while self._running:
            now = datetime.datetime.now()
            next_minute = (now + datetime.timedelta(minutes=1)).replace(second=0, microsecond=0)
            time_to_wait = (next_minute - now).total_seconds()

            time.sleep(time_to_wait)

            with self._lock:
                red = self._lux_red
                yellow = self._lux_yellow
                green = self._lux_green
                current = self._current_value
            
            print("csv write")
            print(red)
            print(yellow)
            print(green)
            print(current)
            timestamp = datetime.datetime.now().strftime("%H:%M:%S")
            date_str = datetime.datetime.now().strftime("%Y-%m-%d")

            # 保存先ディレクトリを指定
            base_dir = os.path.join("data", "sensor")
            os.makedirs(base_dir, exist_ok=True)  # ディレクトリが無ければ作成

            filename = os.path.join(base_dir, f"{date_str}.csv")
            line = [timestamp, red, yellow, green, current]

            try:
                with open(filename, "a", newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    writer.writerow(line)
            except Exception as e:
                print(f"ログ書き込みエラー: {e}")

def data_receive_action(data, logger):
    if len(data) < 4:
        return

    try:    
        mode = chr(data[3])
    except Exception:
        return

    if mode == 'C':
        try:
            for i in range(4, len(data) - 1):
                if data[i] == 0x0D and data[i + 1] == 0x0A:
                    last_index = i - 1
                    float_bytes = data[4:last_index + 1]
                    float_str = bytes(float_bytes).decode("ascii")
                    current = float(float_str)
                    print(current)
                    logger.set_current_value(current)
                    break
        except Exception as e:
            print(f"電流変換失敗: {e}")

    elif mode == 'D':
        if len(data) >= 10:
            red = (data[4] << 8) | data[5]
            yellow = (data[6] << 8) | data[7]
            green = (data[8] << 8) | data[9]
            print(red)
            print(yellow)
            print(green)
            logger.set_pat_light_values(red, yellow, green)

def main():
    print("start main")
    logger = LoggerService()
    try:
        with serial.Serial("/dev/ttyUSB0", 9600, timeout=1) as ser:
            while True:
                if ser.in_waiting:
                    data = ser.read_until(b'\r\n')
                    print(data)
                    if data:
                        data_receive_action(data, logger)
    except KeyboardInterrupt:
        print("終了します")
    finally:
        logger.stop()

if __name__ == "__main__":
    main()
