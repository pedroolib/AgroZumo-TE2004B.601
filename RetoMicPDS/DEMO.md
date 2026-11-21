# Guion de la demo (revisión del viernes)

La demo corre en la Pico 2W de la PCBTractor. El Zumo no participa en esta fase.

## Antes de ir a la oficina

Haz estos pasos en tu laptop, en la carpeta del proyecto.

```bash
cd /home/pedrolib/Semestre/Embebidos/AgroZumo-TE2004B.601/RetoMicPDS
```

1. Corre las pruebas en la PC. Todas deben decir `OK`.

   ```bash
   gcc -O2 -Ipds_pico/src pds_pico/test/test_dsp.c pds_pico/src/goertzel.c pds_pico/src/fft.c pds_pico/src/tone_detect.c -lm -o test_dsp && ./test_dsp
   ```

2. Compila el firmware.

   ```bash
   export PATH=$HOME/.pico-sdk/cmake/v4.3.4/bin:$HOME/.pico-sdk/ninja/v1.13.2:$HOME/.pico-sdk/toolchain/15_2_Rel1/bin:$PATH && cmake -G Ninja -S pds_pico -B pds_pico/build && ninja -C pds_pico/build
   ```

3. Presiona BOOTSEL en la Pico 2W y conéctala por USB. Aparece la unidad `RP2350`.
4. Copia el firmware. Si la unidad dice `RPI-RP2`, es el Zumo: no copies nada ahí.

   ```bash
   cp pds_pico/build/pds_pico.uf2 /run/media/$USER/RP2350/
   ```

5. Haz la demo completa una vez en casa. Anota el umbral que da la calibración y el volumen que funcionó.
6. Lleva la laptop, la Pico con el micrófono, el cable USB y el celular con los WAV de `tools/tonos/`.

## En la oficina

1. Conecta la Pico a la laptop. No presiones BOOTSEL.
2. Cierra cualquier monitor serie abierto, por ejemplo el de VS Code.
3. Abre el visor.

   ```bash
   uv run --no-project --with pyserial --with numpy --with matplotlib python tools/visor_fft.py
   ```

4. Haz clic en la ventana del visor. Las teclas solo funcionan con la ventana activa.

### Paso 1. Muestreo determinista (REQ-AUD-02)

1. Pide silencio en la oficina.
2. Presiona `c` y espera un segundo. La consola imprime `CAL,ruido_max=...,umbral=...`.
3. Presiona `t`. La consola imprime una línea `S,...`.
4. Señala estos valores al profesor:
   - `periodo_us` cerca de 20000. Es el tiempo de un bloque de 320 muestras a 16 kHz.
   - `overruns=0`. La CPU procesó todos los bloques a tiempo.
   - `carga_cpu`. Es el porcentaje del bloque que usa Goertzel.

Qué decir: el ADC se dispara por hardware a 48 MHz / 3000 = 16 kHz exactos. El DMA llena dos buffers alternados. La CPU no decide el instante de muestreo, así que no hay jitter por software ni latencia de sistema operativo.

### Paso 2. Tono del campo A (REQ-AUD-01 y REQ-AUD-07)

1. Pon el celular cerca del micrófono.
2. Reproduce `campo_A_1000Hz.wav`.
3. Señala el pico en 1000 Hz en la FFT y la barra azul de Goertzel.
4. Señala la línea `DET,CAMPO_A` en la consola.

Qué decir: Goertzel calcula la energía solo en 1000 Hz y 1500 Hz. Cuesta una multiplicación-acumulación por muestra por tono. Con N = 320, la resolución es 50 Hz y los dos tonos caen exacto en un bin.

### Paso 3. Tono del campo B

1. Reproduce `campo_B_1500Hz.wav`.
2. Señala el pico en 1500 Hz, la barra naranja y la línea `DET,CAMPO_B`.

Qué decir: los tonos están separados 500 Hz. El requisito pide al menos 150 Hz.

### Paso 4. Rechazo de un comando ambiguo

1. Reproduce `ambiguo_1000+1500Hz.wav`.
2. Señala que la consola dice `DET,NINGUNO`.

Qué decir: un tono gana solo si pasa el umbral, si es 4 veces mayor que el otro tono y si dura 60 ms. Con los dos tonos iguales, el sistema no elige ninguno.

### Paso 5. Secuencia completa

1. Reproduce `secuencia_A_B.wav`.
2. Señala el orden en la consola: `CAMPO_A`, `NINGUNO`, `CAMPO_B`, `NINGUNO`.

### Paso 6. FFT de diagnóstico (REQ-AUD-09)

1. Presiona `r`. Se abre una ventana con la señal cruda y la FFT.
2. Señala que la FFT de la Pico y la de numpy coinciden.
3. Presiona `s` en la ventana principal. Se guardan un PNG y un CSV del espectro para el reporte.

Qué decir: la FFT radix-2 de 1024 puntos con ventana de Hann corre en la Pico. La PC solo la dibuja. Numpy solo sirve para comparar.

## Si algo falla

- Si el visor dice que no encuentra el puerto, desconecta y conecta la Pico otra vez.
- Si el puerto está ocupado, cierra el monitor serie de VS Code.
- Si no aparece `DET`, sube el volumen o la ganancia del MAX4466. Después calibra otra vez con `c`, en silencio.
- Si detecta tonos sin sonido, calibra otra vez con `c` en silencio.
- Si `overruns` no es 0, dilo como limitación conocida. El envío de la FFT por USB comparte la CPU con Goertzel.
- Si la Pico no responde, cierra el visor, desconecta la Pico, conéctala y abre el visor otra vez.

## Preguntas probables del profesor

- ¿Por qué la Pico y no una Raspberry Pi? El ADC y el DMA marcan el muestreo por hardware. No hay sistema operativo que agregue latencia.
- ¿Por qué Fs = 16 kHz? Nyquist es 8 kHz, más de 5 veces el tono más alto. Los armónicos 2 y 3 también quedan debajo de 8 kHz.
- ¿Cuál es la ecuación de Goertzel? `v[n] = 2cos(w0)·v[n−1] − v[n−2] + x[n]` y `|X(k)|² = v1² + v2² − 2cos(w0)·v1·v2` (Tan y Jiang).
- ¿Qué sigue? Ruido AWGN y NoiseX-92, filtros FIR e IIR entre la resta del DC y Goertzel, estimación de SNR y envío por CAN al Zumo.
