# Reto TE2004B — Módulo PDS · Fase 1 (revisión del viernes)

Adquisición determinista del micrófono, mapeo de tonos + Goertzel y FFT de diagnóstico
en una **Raspberry Pi Pico 2W** (C, Pico SDK 2.3.1), montada en la PCBTractor sobre el Zumo 2040.

> En esta fase **no** hay ruido inyectado (REQ-AUD-03) ni filtros FIR/IIR (REQ-AUD-04/05), como pidió el profesor.

## Estructura

```
RetoMicPDS/
├── pds_pico/                 Firmware C para la Pico 2W
│   ├── CMakeLists.txt        (compatible con la extensión Raspberry Pi Pico de VS Code)
│   ├── src/config.h          Fs, N, tonos, umbrales, pines
│   ├── src/adc_capture.c     ADC + DMA ping-pong (REQ-AUD-02)
│   ├── src/goertzel.c        Goertzel (REQ-AUD-07)
│   ├── src/tone_detect.c     Mapeo tono → campo (REQ-AUD-01)
│   ├── src/fft.c             FFT radix-2 propia + Hann (REQ-AUD-09)
│   ├── src/main.c            Lazo principal y protocolo por USB-serial
│   └── test/test_dsp.c       Pruebas de los algoritmos en la PC (sin hardware)
└── tools/
    ├── visor_fft.py          Visor del espectro y de Goertzel en vivo (PC)
    └── gen_tonos.py          Genera WAVs de prueba (1000 Hz, 1500 Hz, ambiguo, secuencia)
```

## Hardware (según el esquemático de la PCBTractor)

| Señal | Pin de la Pico 2W |
|---|---|
| MAX4466 OUT | GPIO26 / ADC0 (pin 31) |
| MAX4466 GND | AGND (pin 33) |
| MAX4466 VCC | 3V3 (pin 36) |
| MCP2515 (Pico) MISO / CS / SCK / MOSI / INT | GP16 / GP17 / GP18 / GP19 / GP20 (SPI0, para REQ-AUD-10, todavía sin usar) |

## Decisiones de diseño

### REQ-AUD-01 · Tonos de comando
| Campo | Tono | Bin Goertzel (k = f·N/Fs) |
|---|---|---|
| Campo A (dirección 1 del Zumo) | **1000 Hz** | k = 20 |
| Campo B (dirección 2 del Zumo) | **1500 Hz** | k = 30 |

- |F_A − F_B| = 500 Hz ≥ 150 Hz, y ambos son > 100 Hz.
- Están lejos de la zona de ruido grave de los motores (< 500 Hz) y dentro de lo que reproduce bien la bocina de un celular.
- Sus armónicos 2.º y 3.º (2000, 3000, 4500 Hz) quedan por debajo de Fs/2, así que no se doblan (no hay aliasing).
- Un tono no es armónico del otro.
- La separación de 500 Hz deja hueco para las bandas de transición de los filtros FIR/IIR de la siguiente fase.

### REQ-AUD-02 · Muestreo determinista
- **Fs = 16 000 Hz**, Ts = 62.5 µs. Nyquist = 8 kHz, más de 5 veces el tono más alto, y cubre los armónicos que importan.
- **Por qué es determinista:** el ADC corre en modo *free-running* con su divisor de reloj: `Fs = 48 MHz / (1 + 2999)`, que da 16 000.000 Hz exactos. Dos canales DMA encadenados en *ping-pong* pasan las muestras a dos buffers de N = 320. La CPU nunca toca el instante de muestreo: el jitter de Ts lo fija el cristal, no el software. Como la Pico no tiene sistema operativo, no hay latencia de SO que controlar.
- **Antialiasing:** el MAX4466 tiene GBW = 600 kHz. Con ganancia de 25× a 125× su ancho de banda queda entre unos 24 kHz y 4.8 kHz, así que con el potenciómetro de ganancia alto funciona como un pasa-bajas analógico natural. Se puede mencionar en la revisión como limitación conocida.
- **Evidencia en vivo:** el comando `t` imprime el periodo medido entre bloques (debe ser 20 000 µs ± unos cuantos µs de latencia de interrupción) y `overruns = 0`.

### REQ-AUD-07 · Goertzel
Para cada tono: `v[n] = 2cos(ω₀)·v[n−1] − v[n−2] + x[n]` y `|X(k)|² = v₁² + v₂² − 2cos(ω₀)·v₁·v₂`
(Tan & Jiang, sección del algoritmo de Goertzel).

- Amplitud estimada: `A = 2|X(k)|/N`.
- Con N = 320 la resolución es de 50 Hz, y los dos tonos caen **exactamente** en un bin, así que no hay fuga espectral.
- Costo: **1 MAC por muestra por tono**, contra N·log₂N de una FFT.
- La media del bloque se resta antes, porque el MAX4466 está polarizado a VCC/2.

**Decisión básica (se refina en REQ-AUD-08).** Un tono gana si cumple las tres condiciones:
1. Su amplitud es mayor o igual al umbral (que se calibra con `c`).
2. Es al menos 4 veces mayor que el otro tono (unos 12 dB). Si no, el comando se rechaza como ambiguo.
3. Se mantiene durante 3 bloques seguidos (60 ms).

### REQ-AUD-09 · FFT de diagnóstico
- FFT radix-2 iterativa escrita desde cero en `fft.c`, sin CMSIS ni librerías.
- N = 1024 (64 ms, resolución de 15.6 Hz), con ventana de Hann y corrección de ganancia de la ventana para leer la amplitud en dB.
- Se calcula **en la Pico**; la PC sólo la dibuja.
- `visor_fft.py` también puede pedir un volcado crudo (`r`) y comparar contra `numpy.fft` como verificación cruzada. Esto es sólo para validar en la PC; ningún filtro usa librerías.

## Compilar y cargar

Con la extensión **Raspberry Pi Pico** de VS Code: abrir la carpeta `pds_pico` y dar clic en *Compile* y luego en *Run*.

Si prefieres la terminal (PowerShell):

```powershell
$env:PATH = "$HOME\.pico-sdk\cmake\v4.3.4\bin;$HOME\.pico-sdk\ninja\v1.13.2;$HOME\.pico-sdk\toolchain\15_2_Rel1\bin;$env:PATH"
```
```powershell
cmake -G Ninja -S pds_pico -B pds_pico/build; ninja -C pds_pico/build
```

Después, conecta la Pico con BOOTSEL presionado y copia `pds_pico/build/pds_pico.uf2` a la unidad `RP2350`.

## Pruebas en la PC (sin hardware)

```powershell
gcc -O2 -Ipds_pico/src pds_pico/test/test_dsp.c pds_pico/src/goertzel.c pds_pico/src/fft.c pds_pico/src/tone_detect.c -lm -o test_dsp.exe; .\test_dsp.exe
```

Las pruebas verifican:
- la amplitud que da Goertzel;
- Goertzel contra la DFT directa;
- el pico de la FFT y su nivel en dB;
- que el DC se elimina;
- el mapeo 1000 Hz → A y 1500 Hz → B;
- el rechazo de los casos ambiguo, silencio y 1250 Hz.

## Demo del viernes (guion sugerido)

1. `pip install pyserial numpy matplotlib` y luego `python tools/visor_fft.py`.
2. En silencio, presiona **c** para calibrar el umbral y después **t** para mostrar el periodo de 20 000 µs, `overruns = 0` y la carga de CPU.
3. Reproduce `tools/tonos/campo_A_1000Hz.wav`: el pico aparece en 1000 Hz, la barra de Goertzel A sube y la consola imprime `DET,CAMPO_A`.
4. Haz lo mismo con `campo_B_1500Hz.wav`, que debe dar `DET,CAMPO_B`.
5. Con `ambiguo_1000+1500Hz.wav` la salida es `DET,NINGUNO` (rechazado).
6. Presiona **r** para el volcado crudo: aparecen la señal en el tiempo y la comparación contra la FFT de numpy.
7. Presiona **s** para guardar el PNG y el CSV del espectro para el reporte.

Si prefieres no usar el visor, sirve cualquier monitor serie (el de VS Code o PuTTY) con los comandos `h g f r t c`.

## Siguiente fase (todavía no implementado)
- REQ-AUD-03: ruido AWGN y NoiseX-92.
- REQ-AUD-04/05: FIR multibanda y banco IIR de bicuadráticas. Se insertan entre la eliminación del DC y Goertzel, en `process_block()` de `main.c`, con búferes circulares.
- REQ-AUD-08: estimación de SNR.
- REQ-AUD-10: envío por CAN (MCP2515 en SPI0) hacia el Zumo 2040.
