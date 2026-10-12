# STPPE Reborn

STPPE Reborn contains the robot firmware, browser control application, mechanical assets, and project documentation.

## Project Areas

- `Firmware/` - Teensy 4.1 firmware.
- `WebControl/` - Zero-build browser control application and its 3D visualization assets.
- `Models/` - Standalone 3MF models.
- `Pliki graficzne/` - Technical documents, poster/site files, and product images.

## WebControl

From the workspace root, start the local server with:

```powershell
python WebControl/launch.py
```

Run its automated tests with:

```powershell
Push-Location WebControl
npm.cmd test
Pop-Location
```

The launcher prints the local URL and chooses an available port automatically.

To open WebControl on a phone on the same trusted Wi-Fi network, start it in LAN mode:

```powershell
python WebControl/launch.py --lan
```

Open the printed **Phone URL** on the phone. If Windows Firewall asks, allow Python on private networks only. Without `--lan`, the server remains accessible only on this PC.




