# UltraTac firmware

STM32 firmware to pulse an ultrasonic transducer and capture the echo envelope over USB. `host.py` saves and plots the samples.

The current target is a provisional STM32F446RC with a 25 MHz crystal. The code builds and passes software tests, but hasn't been tested on the board yet. Raw waveform capture isn't supported yet.

## Build

Run these from `code/`, with CMake, Ninja, and `arm-none-eabi-gcc` on PATH:

```sh
cmake -S . -B .build/arm -G Ninja
cmake --build .build/arm
```

Dependencies download on the first build. The firmware files are in `.build/arm/`.

## Capture

Once the board has been checked and flashed:

```sh
python -m pip install pyserial matplotlib
python host.py ports
python host.py capture --port COM7 --output shot.efw --plot
```

Replace `COM7` with your board's port. Add `--baseline` to collect samples without firing a pulse.

To export a saved capture:

```sh
python host.py read shot.efw --csv shot.csv
```

## Tests

These run without a board and need desktop GCC and Python:

```sh
cmake -S . -B .build/host -G Ninja -DHOST_TEST=ON -DCMAKE_C_COMPILER=gcc
cmake --build .build/host
ctest --test-dir .build/host --output-on-failure
python host.py selftest
```

Before firing the first burst, check the board's power rails and input protection, then verify pulse timing with a scope.
