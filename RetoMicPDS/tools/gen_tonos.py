"""
gen_tonos.py — Genera WAVs limpios de los tonos de comando para la prueba
(reprodúcelos con el celular/bocina frente al MAX4466). Sólo usa la biblioteca estándar.

    python gen_tonos.py      -> crea tonos/campo_A_1000Hz.wav, campo_B_1500Hz.wav,
                                ambiguo_1000+1500Hz.wav y secuencia_A_B.wav
"""
import math
import os
import struct
import wave

FS = 44100
TONOS = {"campo_A_1000Hz": [1000.0], "campo_B_1500Hz": [1500.0], "ambiguo_1000+1500Hz": [1000.0, 1500.0]}
DUR_S = 3.0
AMPL = 0.6
FADE_S = 0.01   # rampa para evitar clics


def senal(freqs, dur):
    n = int(FS * dur)
    out = []
    for i in range(n):
        env = min(1.0, i / (FADE_S * FS), (n - i) / (FADE_S * FS))
        s = sum(math.sin(2 * math.pi * f * i / FS) for f in freqs) / len(freqs)
        out.append(AMPL * env * s)
    return out


def guardar(nombre, muestras):
    os.makedirs("tonos", exist_ok=True)
    ruta = os.path.join("tonos", nombre + ".wav")
    with wave.open(ruta, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(FS)
        w.writeframes(b"".join(struct.pack("<h", int(32767 * s)) for s in muestras))
    print("creado", ruta)


if __name__ == "__main__":
    for nombre, fr in TONOS.items():
        guardar(nombre, senal(fr, DUR_S))
    silencio = [0.0] * int(FS * 1.0)
    guardar("secuencia_A_B", senal([1000.0], 2) + silencio + senal([1500.0], 2) + silencio)
