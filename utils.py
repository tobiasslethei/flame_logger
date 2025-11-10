import os
import serial
import time
import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

class SerialClient:
    def __init__(self, port="COM3", baud=115200, timeout=1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        self.open()

    def open(self):
        if self.ser and self.ser.is_open:
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(1.0)
        except serial.SerialException as exc:
            raise RuntimeError(f"Failed to open {self.port}: {exc}") from exc

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send(self, command: str) -> str:
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial port is not open")
        self.ser.reset_input_buffer()
        self.ser.write((command + "\r\n").encode())
        return self.ser.readline().decode(errors="ignore").strip()

    def set_pos(self, pulse_us: int) -> str:
        return self.send(f"SET {pulse_us}")

    def get_data(self):
        resp = self.send("GET")
        if not resp.startswith("DATA"):
            return resp
        parts = resp.split(",")
        if len(parts) < 5:
            return resp
        result = {
            "timestamp": int(parts[1]),
            "position": int(parts[2]),
        }
        if parts[3] == "FAULT":
            result["fault"] = True
            if len(parts) >= 9:
                result["tc_temp"] = float(parts[4])
                result["internal_temp"] = float(parts[5])
                result["scv"] = int(parts[6])
                result["scg"] = int(parts[7])
                result["oc"] = int(parts[8])
        else:
            result["fault"] = False
            result["tc_temp"] = float(parts[3])
            result["internal_temp"] = float(parts[4])
        return result

    def motor_on(self):
        return self.send("MOTOR ON")

    def motor_off(self):
        return self.send("MOTOR OFF")

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def start_csv_flame_scan(self, filename: str, start_pos: int = 1500, measurement_delay_ms: int = 100,
                    step_size: int = 1, min_pulse: int = 1300, max_pulse: int = 1700):

        measurement_delay_s = measurement_delay_ms / 1000

        self.motor_on()
        self.set_pos(start_pos)
        time.sleep(1)
        pos = start_pos
        direction = 1 if start_pos <= min_pulse else -1

        header = "timestamp,position,tc_temp,internal_temp\n"
        with open(filename, "w") as f:
            f.write(header)
            print(f"Logging to {filename} (Ctrl+C to stop)")
            try:
                while True:
                    self.set_pos(pos)
                    time.sleep(measurement_delay_s)
                    data = self.get_data()
                    if isinstance(data, dict) and not data.get("fault", False):
                        f.write(
                            f"{data['timestamp']},{data['position']},{data['tc_temp']},{data['internal_temp']}\n"
                        )
                        f.flush()
                        print(
                            f"pos={data['position']} tc={data['tc_temp']:.2f}C internal={data['internal_temp']:.2f}C"
                        )
                    else:
                        print(f"warning: unexpected response -> {data}")
                    pos += direction * step_size
                    if pos >= max_pulse:
                        pos = max_pulse
                        direction = -1
                    elif pos <= min_pulse:
                        pos = min_pulse
                        direction = 1
            except KeyboardInterrupt:
                print("Logging stopped by user")

class FlameScan:
    def __init__(self, data_path, radius_mm_servo=16,
                 ø_ref_min=np.deg2rad(-90), ø_ref_max=np.deg2rad(90),
                 p_ref_min=2500, p_ref_max=500,
                 burn_dt_s=10*60, burn_dy_mm=40):
        
        self.data_path = data_path
        self.radius = radius_mm_servo
        self.ø_ref_min = ø_ref_min
        self.ø_ref_max = ø_ref_max
        self.p_ref_min = p_ref_min
        self.p_ref_max = p_ref_max
        self.delta_burn_mm_s = burn_dy_mm / burn_dt_s

        self._load()
        self._sample()

    def _load(self):
        with open(self.data_path, newline="") as csvfile:
            reader = csv.DictReader(csvfile)
            self.data = list(reader)

    def _ø_rad(self, p):
        return self.ø_ref_min + (self.ø_ref_max - self.ø_ref_min) * \
               (p - self.p_ref_min) / (self.p_ref_max - self.p_ref_min)

    def _x_servo(self, ø_rad):
        return self.radius * np.sin(ø_rad)
    
    def _y_servo(self, ø_rad):
        return self.radius * np.cos(ø_rad)

    def _displacement_mm(self, s):
        return self.delta_burn_mm_s * s

    def _sample(self):
        t0 = int(self.data[0]["timestamp"])
        time_s, temp_C, internal_C, x_mm, y_mm, positions = [], [], [], [], [], []

        for row in self.data:
            t = (int(row["timestamp"]) - t0) / 1000
            p = int(row["position"])
            ø = self._ø_rad(p)
            x = self._x_servo(ø)
            y = self._y_servo(ø) + self._displacement_mm(t) - self.radius
            time_s.append(t)
            x_mm.append(x)
            y_mm.append(y)
            temp_C.append(float(row["tc_temp"]))
            internal_C.append(float(row["internal_temp"]))
            positions.append(p)

        self.time_s = np.array(time_s)
        self.temp_C = np.array(temp_C)
        self.internal_temp_C = np.array(internal_C)
        self.x_mm = np.array(x_mm)
        self.y_mm = np.array(y_mm)
        self.positions = np.array(positions)

    def _configure_axes(self, ax):
        """Apply consistent axis limits and ticks across plots."""
        x_limit = float(np.ceil(max(abs(self.x_mm.min()), abs(self.x_mm.max()))))
        ax.set_xlim(-x_limit, x_limit)
        ax.set_xticks([-x_limit, 0.0, x_limit])

        y_min = float(np.floor(self.y_mm.min()))
        y_max = float(np.ceil(self.y_mm.max()))
        ax.set_ylim(y_min, y_max)
        ax.set_yticks(np.linspace(y_min, y_max, 5))

    def plot_points(self, mirror=False, cmap="turbo", save_path=None, dpi=300, show=True):
        """Plot raw points, optionally mirrored around y-axis."""
        if mirror:
            x_all = np.concatenate([self.x_mm, -self.x_mm])
            y_all = np.concatenate([self.y_mm,  self.y_mm])
            t_all = np.concatenate([self.temp_C, self.temp_C])
            title = "Points (Mirrored)"
        else:
            x_all, y_all, t_all = self.x_mm, self.y_mm, self.temp_C
            title = "Points"

        fig, ax = plt.subplots(figsize=(8, 6))
        sc = ax.scatter(x_all, y_all, c=t_all, cmap=cmap, s=6)
        cbar = fig.colorbar(sc, ax=ax)
        cbar.set_label("Temperature (degC)")
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_aspect("equal")
        ax.set_title(title)

        self._configure_axes(ax)

        if save_path:
            directory = os.path.dirname(save_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            fig.savefig(save_path, dpi=dpi, bbox_inches="tight")

        if show:
            plt.show()
        else:
            plt.close(fig)

    def plot_heatmap(self, mirror=True, cmap="turbo", save_path=None, dpi=300, show=True):
        """Interpolated 2D heatmap, optionally using mirrored data."""
        if mirror:
            x_all = np.concatenate([self.x_mm, -self.x_mm])
            y_all = np.concatenate([self.y_mm,  self.y_mm])
            t_all = np.concatenate([self.temp_C, self.temp_C])
            title = "Interpolated (Mirrored)"
        else:
            x_all, y_all, t_all = self.x_mm, self.y_mm, self.temp_C
            title = "Interpolated"

        xi = np.linspace(min(x_all), max(x_all), 300)
        yi = np.linspace(min(y_all), max(y_all), 300)
        Xi, Yi = np.meshgrid(xi, yi)
        Ti = griddata((x_all, y_all), t_all, (Xi, Yi), method="linear")

        fig, ax = plt.subplots(figsize=(8, 6))
        im = ax.imshow(
            Ti,
            extent=(xi.min(), xi.max(), yi.min(), yi.max()),
            origin="lower",
            cmap=cmap,
            aspect="equal",
        )
        cbar = fig.colorbar(im, ax=ax)
        cbar.set_label("Temperature (degC)")
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_title(title)

        self._configure_axes(ax)

        if save_path:
            directory = os.path.dirname(save_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            fig.savefig(save_path, dpi=dpi, bbox_inches="tight")

        if show:
            plt.show()
        else:
            plt.close(fig)

    def plot_centerline(self, save_path=None, dpi=300, show=True):
        """Temperature along centerline (pulse = 1500 us)."""
        mask = self.positions == 1500
        if not np.any(mask):
            return

        fig, ax = plt.subplots(figsize=(8, 5))
        ax.plot(self.y_mm[mask], self.temp_C[mask], "o-")
        ax.set_xlabel("Height (mm)")
        ax.set_ylabel("Center Temperature (degC)")
        ax.set_title("Centerline Temperature vs Height")
        ax.grid(True)

        if save_path:
            directory = os.path.dirname(save_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            fig.savefig(save_path, dpi=dpi, bbox_inches="tight")

        if show:
            plt.show()
        else:
            plt.close(fig)

    def max_temperature(self):
        return np.max(self.temp_C)
