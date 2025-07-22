import asyncio
import threading
import queue
import json
import csv
import datetime
import tkinter as tk
from tkinter import scrolledtext, filedialog, messagebox
from bleak import BleakClient, BleakScanner

# BLE UUIDs (müssen mit ESP32-Firmware übereinstimmen)
SERVICE_UUID      = "12345678-1234-1234-1234-123456789012"
DATA_CHAR_UUID    = "abcdef12-3456-789a-bcde-123456789abc"
CONTROL_CHAR_UUID = "12345678-1234-1234-1234-123456789013"

class BLEWorker(threading.Thread):
    """Hintergrund-Thread für BLE-Kommunikation"""
    def __init__(self, data_queue):
        super().__init__(daemon=True)
        self.loop = asyncio.new_event_loop()
        self.client = None
        self.device_address = None
        self.data_queue = data_queue
        self.running = False

    async def find_and_connect(self):
        devices = await BleakScanner.discover()
        for d in devices:
            if d.name == "ESP32-IMU":
                self.device_address = d.address
                break
        if not self.device_address:
            self.data_queue.put({'type':'error', 'msg':'ESP32-IMU nicht gefunden'})
            return False
        self.client = BleakClient(self.device_address, loop=self.loop)
        await self.client.connect()
        await self.client.start_notify(DATA_CHAR_UUID, self.notification_handler)
        self.data_queue.put({'type':'info', 'msg':'🔗 Verbunden'})
        return True

    async def notification_handler(self, sender, data):
        try:
            payload = json.loads(data.decode('utf-8'))
            ts_pc = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            entry = {
                'pc_timestamp': ts_pc,
                'frame': payload['frame'],
                'esp_timestamp': payload['timestamp'],
                'accel': payload['accel'],
                'gyro': payload['gyro']
            }
            self.data_queue.put({'type':'data', 'entry':entry})
        except Exception as e:
            self.data_queue.put({'type':'error', 'msg':f"Parse-Error: {e}"})

    async def handle_commands(self):
        # läuft nebenbei im selben Loop
        while self.running:
            await asyncio.sleep(0.1)
        # Wenn gestoppt, sauber trennen
        if self.client and self.client.is_connected:
            await self.client.stop_notify(DATA_CHAR_UUID)
            await self.client.disconnect()
            self.data_queue.put({'type':'info', 'msg':'🔌 Getrennt'})

    def run(self):
        self.running = True
        asyncio.set_event_loop(self.loop)
        try:
            connected = self.loop.run_until_complete(self.find_and_connect())
            if connected:
                self.loop.run_until_complete(self.handle_commands())
        except Exception as e:
            self.data_queue.put({'type':'error', 'msg':str(e)})
        finally:
            self.loop.close()

    def send(self, command: str):
        """Befehl asynchron senden"""
        if self.client and self.client.is_connected:
            asyncio.run_coroutine_threadsafe(
                self.client.write_gatt_char(CONTROL_CHAR_UUID, command.encode()),
                self.loop
            )
            self.data_queue.put({'type':'info', 'msg':f"Gesendet: {command}"})

    def stop(self):
        self.running = False


class IMUGUI:
    def __init__(self, root):
        self.root = root
        root.title("ESP32-IMU GUI")

        # Queue für Nachrichten und Daten vom BLE-Thread
        self.queue = queue.Queue()
        self.ble = None

        # GUI-Elemente
        frm = tk.Frame(root)
        frm.pack(padx=10, pady=5)

        tk.Button(frm, text="Verbinden", command=self.connect).grid(row=0, column=0)
        tk.Button(frm, text="Trennen", command=self.disconnect).grid(row=0, column=1)

        tk.Label(frm, text="Hz:").grid(row=1, column=0, pady=5)
        self.rate_var = tk.StringVar(value="100")
        tk.Entry(frm, textvariable=self.rate_var, width=5).grid(row=1, column=1)
        tk.Button(frm, text="Set Rate", command=self.set_rate).grid(row=1, column=2)

        tk.Button(frm, text="Display ON", command=lambda: self.send_cmd("DISPLAY:ON")).grid(row=2, column=0)
        tk.Button(frm, text="Display OFF", command=lambda: self.send_cmd("DISPLAY:OFF")).grid(row=2, column=1)

        tk.Button(frm, text="Daten speichern", command=self.save_csv).grid(row=2, column=2)

        self.log = scrolledtext.ScrolledText(root, width=80, height=20, state='disabled')
        self.log.pack(padx=10, pady=5)

        self.data_log = []  # zum CSV export
        self.root.after(100, self.process_queue)

    def connect(self):
        if self.ble and self.ble.is_alive():
            messagebox.showinfo("Info", "Schon verbunden")
            return
        self.ble = BLEWorker(self.queue)
        self.ble.start()

    def disconnect(self):
        if self.ble:
            self.ble.stop()

    def set_rate(self):
        hz = self.rate_var.get()
        if not hz.isdigit() or not (1 <= int(hz) <= 500):
            messagebox.showerror("Fehler", "Hz muss 1–500 sein")
            return
        self.send_cmd(f"RATE:{hz}")

    def send_cmd(self, cmd):
        if self.ble:
            self.ble.send(cmd)
        else:
            messagebox.showwarning("Warnung", "Nicht verbunden")

    def process_queue(self):
        try:
            while True:
                msg = self.queue.get_nowait()
                if msg['type'] == 'data':
                    e = msg['entry']
                    line = (f"{e['pc_timestamp']} | F{e['frame']} "
                            f"A[{e['accel'][0]:.2f}, {e['accel'][1]:.2f}, {e['accel'][2]:.2f}] "
                            f"G[{e['gyro'][0]:.2f}, {e['gyro'][1]:.2f}, {e['gyro'][2]:.2f}]\n")
                    self.append_log(line)
                    self.data_log.append(e)
                elif msg['type'] == 'info':
                    self.append_log(f"{msg['msg']}\n")
                elif msg['type'] == 'error':
                    self.append_log(f"ERROR: {msg['msg']}\n")
        except queue.Empty:
            pass
        finally:
            self.root.after(100, self.process_queue)

    def append_log(self, text):
        self.log.configure(state='normal')
        self.log.insert(tk.END, text)
        self.log.yview(tk.END)
        self.log.configure(state='disabled')

    def save_csv(self):
        if not self.data_log:
            messagebox.showinfo("Info", "Keine Daten zum Speichern")
            return
        fname = filedialog.asksaveasfilename(defaultextension='.csv',
                                             filetypes=[('CSV Dateien','*.csv')])
        if not fname:
            return
        with open(fname, 'w', newline='') as f:
            fields = ['pc_timestamp','frame','esp_timestamp',
                      'accel','gyro']
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for e in self.data_log:
                writer.writerow(e)
        messagebox.showinfo("Erfolg", f"CSV gespeichert: {fname}")

if __name__ == '__main__':
    root = tk.Tk()
    app = IMUGUI(root)
    root.mainloop()
