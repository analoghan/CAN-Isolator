# Roadmap

Ideas under consideration. Nothing here is scheduled yet.

## To evaluate

### Bluetooth LE connection (monitor without USB)

Use the config tool from a laptop (or Android phone) over Bluetooth instead of the USB cable.

- **How:** the board would accept the same text commands over a Bluetooth LE service as over USB, and the page would gain a second transport, so Connect offers "USB" or "Bluetooth". Every tab would keep working.
- **Browsers:** Chrome and Edge on macOS and Windows 10/11 (and Android) support Web Bluetooth. On Linux, Chrome may need a setting enabled. iPhone browsers don't support it.
- **Good fit:** status, bus load, event history, ID sniffer, reading and sending settings (8–15 KB), and the monitor or graphs filtered to a few IDs.
- **Poor fit (keep on USB):** recording a whole busy bus, replay, and firmware updates. Bluetooth carries roughly a tenth of what USB does.
- **Security:** read-only by default. Changing settings over Bluetooth would require pairing with a PIN set over USB, and there would be a setting to turn Bluetooth off entirely.
- **To check first:**
  - **Antenna.** The board's ESP32-C6-WROOM-**1U** has no built-in antenna, only a connector for an external 2.4 GHz one. Without an antenna the range is a few centimetres. In a metal enclosure the antenna must be outside it.
  - **Forwarding speed.** The C6 has one processor core shared between the Bluetooth stack and CAN forwarding. Measure latency with Bluetooth connected and busy. The bar is the same as for USB: typically 0.1–0.2 ms, and no frame over 2 ms.
  - **Size.** The Bluetooth stack adds about 100 KB of firmware and 30–50 KB of RAM. Both fit within current headroom.
- **Effort:** medium to large.

## Other ideas

- **On-board "black box" capture.** Keep the last few seconds of traffic in RAM and save them to flash on bus-off, error passive, an error burst, or a wheel-button press. Download the capture later into Record & replay or Graph.
- **Settings profiles.** Two to four complete settings sets on the board (street / track / bench), switched from the page, a wheel button or a CAN signal.
- **GitHub Pages hosting.** Open the config tool from a link on any computer, with firmware images offered straight from the repo.
