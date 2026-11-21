"""
visor_fft.py — Visualización espectral en vivo de la Pico 2W (REQ-AUD-09).

La FFT se calcula EN LA PICO (src/fft.c); este script sólo dibuja lo que llega
por USB-serial. Opcionalmente (tecla 'r') pide un volcado crudo y compara la FFT
de la Pico contra numpy.fft como verificación cruzada.

Uso:
    pip install pyserial numpy matplotlib
    python visor_fft.py            (detecta el puerto de la Pico)
    python visor_fft.py COM5

Teclas en la ventana: r = volcado crudo + verificación | c = calibrar silencio |
                      t = estadísticas de tiempo real | s = guardar PNG/CSV
"""
import sys
import threading
import time

import numpy as np
import matplotlib.pyplot as plt
import serial
import serial.tools.list_ports

# Quita los atajos de matplotlib que usan las mismas teclas que el visor
# (s = diálogo de guardar, r/h = restablecer vista, c = vista anterior).
VISOR_KEYS = {"r", "c", "t", "s", "h"}
for _km in [k for k in plt.rcParams if k.startswith("keymap.")]:
    plt.rcParams[_km] = [k for k in plt.rcParams[_km] if k not in VISOR_KEYS]

TONES = {"CAMPO_A": 1000.0, "CAMPO_B": 1500.0}
FULL_SCALE = 2048.0          # amplitud pico de un ADC de 12 bits
DBFS_OFFSET = 20 * np.log10(FULL_SCALE)


def find_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x2E8A:      # Raspberry Pi
            return p.device
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if not ports:
        sys.exit("No se encontró ningún puerto serie. ¿Está conectada la Pico?")
    return ports[0]


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.fft_db = None
        self.fs = 16000
        self.fft_us = 0
        self.ampl = [0.0, 0.0]
        self.rms = 0.0
        self.cmd = "NINGUNO"
        self.raw = None
        self.log = []


def reader(ser, st):
    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
        except serial.SerialException:
            print("Puerto cerrado")
            return
        if not line:
            continue
        parts = line.split(",")
        tag = parts[0]
        with st.lock:
            if tag == "F" and len(parts) > 4:
                st.fs = int(parts[1])
                st.fft_us = int(parts[3])
                st.fft_db = np.array(parts[4:], dtype=float) / 10.0
            elif tag == "G" and len(parts) >= 6:
                st.ampl = [float(parts[2]), float(parts[3])]
                st.rms = float(parts[4])
            elif tag == "R" and len(parts) > 3:
                st.raw = np.array(parts[3:], dtype=float)
                print(f"Volcado crudo recibido: {len(st.raw)} muestras")
            elif tag == "DET":
                st.cmd = parts[1]
                print(line)
            else:
                print(line)
            st.log.append(line)


def numpy_check(raw, fs, pico_db):
    """FFT de referencia con numpy sobre el volcado crudo (sólo verificación)."""
    n = len(raw)
    x = raw - raw.mean()
    w = np.hanning(n)
    X = np.fft.rfft(x * w)[: n // 2]
    ref_db = 20 * np.log10(np.abs(X) * 2 / (n * w.mean()) + 1e-6)
    f = np.arange(n // 2) * fs / n
    fig, ax = plt.subplots(2, 1, figsize=(10, 6))
    ax[0].plot(np.arange(n) / fs * 1000, raw)
    ax[0].set_xlabel("t [ms]"); ax[0].set_ylabel("cuentas ADC"); ax[0].set_title("Señal cruda (Ts constante)")
    ax[1].plot(f, ref_db - DBFS_OFFSET, label="numpy.fft (referencia)")
    if pico_db is not None and len(pico_db) == n // 2:
        ax[1].plot(f, pico_db - DBFS_OFFSET, "--", label="FFT en la Pico (bloque distinto)")
    ax[1].set_xlabel("f [Hz]"); ax[1].set_ylabel("dBFS"); ax[1].legend(); ax[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.show()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    print(f"Abriendo {port}")
    ser = serial.Serial(port, 115200, timeout=1)
    time.sleep(0.5)
    st = State()
    threading.Thread(target=reader, args=(ser, st), daemon=True).start()
    ser.write(b"g")   # stream Goertzel
    ser.write(b"f")   # stream FFT

    plt.ion()
    fig = plt.figure(figsize=(12, 7))
    gs = fig.add_gridspec(2, 3)
    ax_fft = fig.add_subplot(gs[:, :2])
    ax_g = fig.add_subplot(gs[0, 2])
    ax_txt = fig.add_subplot(gs[1, 2]); ax_txt.axis("off")

    line_fft, = ax_fft.plot([], [], lw=1)
    peak_hold = None
    line_hold, = ax_fft.plot([], [], lw=0.8, alpha=0.5, label="máximo retenido")
    for name, f in TONES.items():
        ax_fft.axvline(f, color="r", ls="--", lw=0.8)
        ax_fft.axvspan(f - 75, f + 75, color="r", alpha=0.08)
        ax_fft.text(f, 2, name, color="r", ha="center", fontsize=8)
    ax_fft.set_xlim(0, 4000); ax_fft.set_ylim(-110, 5)
    ax_fft.set_xlabel("Frecuencia [Hz]"); ax_fft.set_ylabel("Magnitud [dBFS]")
    ax_fft.set_title("FFT de diagnóstico (N=1024, Hann) — calculada en la Pico 2W")
    ax_fft.grid(alpha=0.3); ax_fft.legend(loc="upper right")

    bars = ax_g.bar(list(TONES.keys()), [0, 0], color=["tab:blue", "tab:orange"])
    ax_g.set_title("Goertzel: amplitud por tono"); ax_g.set_ylabel("cuentas ADC")
    txt = ax_txt.text(0, 0.9, "", va="top", family="monospace", fontsize=11)

    def on_key(ev):
        nonlocal peak_hold
        if ev.key in ("r", "c", "t"):
            ser.write(ev.key.encode())
        elif ev.key == "h":
            peak_hold = None
        elif ev.key == "s":
            stamp = time.strftime("%Y%m%d_%H%M%S")
            fig.savefig(f"espectro_{stamp}.png", dpi=150)
            with st.lock:
                if st.fft_db is not None:
                    f = np.arange(len(st.fft_db)) * st.fs / (2 * len(st.fft_db))
                    np.savetxt(f"espectro_{stamp}.csv", np.c_[f, st.fft_db - DBFS_OFFSET],
                               delimiter=",", header="f_hz,dbfs", comments="")
            print(f"Guardado espectro_{stamp}.png/.csv")
    fig.canvas.mpl_connect("key_press_event", on_key)

    while plt.fignum_exists(fig.number):
        with st.lock:
            db, fs, raw = st.fft_db, st.fs, st.raw
            ampl, rms, cmd, fft_us = list(st.ampl), st.rms, st.cmd, st.fft_us
            st.raw = None
        if db is not None:
            f = np.arange(len(db)) * fs / (2 * len(db))
            dbfs = db - DBFS_OFFSET
            peak_hold = dbfs if peak_hold is None or len(peak_hold) != len(dbfs) else np.maximum(peak_hold, dbfs)
            line_fft.set_data(f, dbfs)
            line_hold.set_data(f, peak_hold)
            ax_fft.set_xlim(0, fs / 2)
        for b, a in zip(bars, ampl):
            b.set_height(a)
        ax_g.set_ylim(0, max(10, max(ampl) * 1.2))
        txt.set_text(f"Comando: {cmd}\n\nA(1000 Hz) = {ampl[0]:7.2f}\nA(1500 Hz) = {ampl[1]:7.2f}\n"
                     f"RMS bloque = {rms:7.2f}\nFFT en Pico = {fft_us} us\n\n"
                     "[r] crudo+numpy  [c] calibrar\n[t] tiempos  [s] guardar  [h] reset hold")
        if raw is not None:
            numpy_check(raw, fs, db)
        fig.canvas.draw_idle()
        plt.pause(0.05)

    ser.close()


if __name__ == "__main__":
    main()
