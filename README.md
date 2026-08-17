# ESP32 Reachy Mini Motion Controller


## What is this

This is a motion controller for the [Reachy Mini](https://www.pollen-robotics.com/reachy-mini/) robot: IMU readings from a [Waveshare ESP32-S3-Touch-AMOLED-1.8](https://www.waveshare.com/esp32-s3-touch-amoled-1.8.htm) board are translated directly onto Reachy's head, making this a sort of puppeteering device.

<img src="assets/hero.gif" alt="Reachy Mini motion controller demo">

Hold the board like a remote (screen toward you, USB and side buttons down). Press **BOOT** to engage — the head follows your tilt and turn. Press BOOT again to clutch.

## How to run it

**1. Start the Reachy Mini Control App** and install **esp32_motion_controller** from the Reachy Mini App Store: [V4C38/esp32_motion_controller](https://huggingface.co/spaces/V4C38/esp32_motion_controller).

**2. Set WiFi credentials** — The board needs your home WiFi name and password for the Websocket connection.

Open `firmware/sdkconfig.local` (copy first) in a text editor and put your WiFi name and password on these two lines:

```
CONFIG_RMC_WIFI_SSID="your-wifi-name"
CONFIG_RMC_WIFI_PASSWORD="your-wifi-password"
```

**3. Flash the firmware** — Connect the board, then install the program onto it (requires [ESP-IDF 5.5](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/get-started/index.html) installed once).

```bash
. ~/esp/esp-idf/export.sh
cd firmware
idf.py set-target esp32s3    # first time only
idf.py -p /dev/cu.usbmodem101 flash
```

If flash fails, check the USB port name with `ls /dev/cu.usbmodem*`.

The board finds the app automatically on the same WiFi. Double-tap the display for settings (gain, connect). Untethered use needs a 3.7V MX1.25 LiPo.

## Design decisions

The ESP32 owns *how the device moved* (Mahony IMU fusion at 250 Hz, samples sent at 20 Hz). Python owns *what the robot should do* (workspace limits, clutch, body follow). They talk over a fire-and-forget UDP JSON contract ([`PROTOCOL.md`](PROTOCOL.md)) — firmware and app must be upgraded together.

Engage snapshots the current pose as IMU zero. Head `x`/`y`/`z` stay put; only roll/pitch/yaw follow the board. Yaw is 1:1, using the board's right-edge heading so a nod does not yaw the head.

## License

[MIT](LICENSE)
