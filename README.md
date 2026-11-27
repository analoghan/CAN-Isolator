# CAN Isolator

A two-bus CAN isolator/gateway for the TechOverflow CANipulator (ESP32-C6).
It sits between a MoTeC M1 ECU (Bus B) and the rest of the car (Bus A). It forwards only the frames you choose, can rate-limit,
rewrite and generate frames, and is configured from a browser over USB.

## Layout

| Folder | What it is |
|---|---|
| `CanIsolatorC6/` | Current firmware for the TechOverflow CANipulator (ESP32-C6, two CAN controllers). PlatformIO project. |
| `ConfigTool/` | Browser configuration tool (single `index.html`, Chrome or Edge via Web Serial). Has built-in help — press `?`. |
| `Firmware/` | Pre-built firmware images, loadable from the Config Tool's *More… → Update firmware*. |
| `CanipulatorTest/` | Bring-up test firmware for the CANipulator board (self test, pin test, cross test, listen-only monitor). |

## Building the firmware

```
cd CanIsolatorC6
pio run            # build; the image is copied to ../Firmware/
pio run -t upload  # flash over USB
```

After the first USB flash, later updates can be done from the Config Tool page.

## Using the Config Tool

Open `ConfigTool/index.html` in Chrome or Edge (on macOS, double-click
`ConfigTool/Open Config Tool.command`), plug in the board, and click **Connect**.
See `ConfigTool/README.md` and the in-page **? Help** tab.

## Hardware

Only the CANipulator (ESP32-C6) is supported.

The earlier Teensy 4.0 version (`CanIsolator.ino`, and the `CanProbe` wiring
check) is **deprecated** and no longer in this tree. It is preserved at the git
tag [`teensy-final`](../../tree/teensy-final) if you ever need it:

```
git checkout teensy-final -- CanIsolator CanProbe
```

## License

GPL-3.0 — see `LICENSE`.
