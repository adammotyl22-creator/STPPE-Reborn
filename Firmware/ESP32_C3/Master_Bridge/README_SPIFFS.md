Upload UI to SPIFFS and flash sketch

This sketch now expects the web UI to be stored in SPIFFS (LittleFS) under the `data/` folder.

Files:
- data/index_esp32.html  (already copied)
- add any other assets (JS, CSS, images) into `data/` preserving relative paths used by the HTML

Arduino IDE (recommended):
1. Install the "ESP32" board package and the "ESP32 Sketch Data Upload" plugin (Tools -> ESP32 Sketch Data Upload).
2. Select the correct board (ESP32C3 Dev Module) and the correct COM port.
3. Build the sketch (verify) and then use "ESP32 Sketch Data Upload" to upload the contents of `data/` to SPIFFS.
4. Then upload the sketch as normal (Sketch -> Upload). Restart the board.

PlatformIO (VSCode):
1. Ensure `platformio.ini` selects the correct board (esp32c3) and framework (arduino).
2. From the project root (the folder containing `platformio.ini`), run:

```bash
pio run -t uploadfs    # upload files in data/ to the filesystem
pio run -t upload      # build and flash the firmware
```

ESPTool / esptool.py (advanced):
- You can build a spiffs image and write it with `esptool.py` but using the Arduino/PlatformIO helpers is easier.

After upload:
- Connect your phone to `STPE_Robot` AP, open a browser and go to `http://192.168.4.1/`.
- The page should load and auto-connect to `ws://192.168.4.1:81`.

If you want, I can also add a `/status` HTTP endpoint in the sketch; say "yes" and I'll patch it in.