# Pico SDR

Using RP2040 / Raspberry Pi Pico as a software-defined radio receiver.

1. Clone using `git clone --recursive` as this package is using a custom USB
   stdio library for better throughput and to avoid deadlocks.

2. Build and flash the firmware as usual:

   ```bash
   export PICO_SDK_PATH=/path/to/pico-sdk
   cmake -B build src
   cmake --build build
   picotool load -f build/pico_sdr.uf2
   ```

3. Start the USB serial to TCP bridge, setting the frequency to 88.2 MHz:

   ```bash
   python util/bridge.py -f 88200000
   ```

4. Open `grc/PicoSDR-WBFM.grc` in GNU Radio Companion.

5. Press `F6`.
