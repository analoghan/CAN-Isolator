# CAN Isolator config tool

`index.html` sets up the CANipulator isolator (firmware `CanIsolatorC6` 2.0 or newer) over USB-C.
There's nothing to install.

1. Double-click **`Open Config Tool.command`**. It opens `index.html` in Chrome (or Edge).
   Safari and Firefox can't talk to USB serial devices, and Safari is the Mac's default for `.html` files.
   The first time, macOS may ask you to confirm opening the script: right-click it, choose Open, then Open again.
2. Close any serial monitor (PlatformIO, Arduino IDE): only one program can use the port.
3. Click **Connect** and pick the "USB JTAG/serial debug unit".

**Help is built into the page:** the **? Help** tab, or press <kbd>?</kbd> on any tab for that tab's section. It covers getting started, how routing decisions are made, every tab, how-to recipes, bit numbering, firmware updates, recovery, troubleshooting, LED colours, safety notes and a glossary, and it works offline.

## Buttons

| Button | What it does |
|---|---|
| Send to board | Runs the edits now, in RAM only. A power cycle goes back to the saved settings, so this is the safe way to try a change. |
| Save to board | Writes the settings to flash. Saving briefly pauses CAN reception, so do it with the engine off. |
| Discard edits | Reloads what the board is running. |
| More… → Export / Import | Back up your settings to a `.json` file, or load them from one. Import also works offline. |

- Bit rate and connector-swap changes take effect after Save plus a restart. Everything else applies immediately.
- **Factory reset without a computer:** hold BOOT for 3 s while the isolator is running. The LED flashes white,
  the saved settings are erased, and the board restarts on the defaults compiled into the firmware.
- Don't hold BOOT while powering up: that enters flashing mode instead.

## Tabs

- **Status**: live counts per bus, bus state, error counters, CPU load, and the generated frames.
- **Routes**, **Buttons**, **Bus settings**: the settings.
- **Transforms**: named lists of steps that rewrite the frames a route forwards. A route picks one in its Transform column.
  - Up to 8 transforms, 6 steps each.
  - Step types: change the ID or length; set a field to a value (also how you clear or set bits); copy a field from this frame or another frame, with scaling; forward only if a field matches (=, ≠, <, ≤, >, ≥, has all bits, has any bit); rolling counter.
  - "Try it" runs a test frame through the transform with the same math as the firmware, and shows which step drops it and why.
- **Generated frames**: frames the isolator builds and sends on a timer.
  - Each has an ID, bus, period, length, constant bytes, and up to 4 fields.
  - A field is copied from another frame (out = raw × factor + offset, rounded and clamped) or is a rolling counter.
  - Bit numbering follows DBC (Motorola/Intel). "Pick bits in the grid" sets a field by clicking.
  - "Work it out from units…" computes the factor and offset.
  - The preview uses the same math as the firmware, so its bytes match what the board sends.
  - A frame stops while any source is stale. While it's on, a copy with the same ID arriving on the other bus is dropped.
  - Up to 8 frames. The Aston cluster frames are the factory defaults.
- **ID sniffer**: every ID on each bus, its rate and latest data, and what the route table does with it.
  `+ route` adds a row for that ID; `monitor` opens it in the frame monitor.
- **Frame monitor**: captures individual frames, RX and/or TX on either bus, with up to 8 ID or range filters.
  - Changed bytes are highlighted, and Δt is shown per ID. Capture can be saved as CSV.
  - Click a frame to open the **bit analyzer**:
    - Brighter cells are bits that change more often.
    - Click the first and last bit of a value, choose Motorola/Intel, scale and offset, and see the value decoded live and graphed.
    - It points out changing bytes and rolling counters.
    - Signals can be saved; they're kept in the browser and in exported settings files. A DBC `SG_` line can be pasted in, and each signal is shown as one.
- **Send test frames**: send a frame once, or repeat up to 4 frames on a timer. Repeats stop by themselves 5 s after
  the page closes or the cable is pulled.

- **DBC files**: load `.dbc` files, such as the MoTeC M1 ones or your wheel's.
  - Each file is kept in this browser and set to describe Bus A, Bus B or either.
  - It adds message names to the sniffer and monitor, and decoded signals with units and value names to the bit analyzer (click one to chart it).
  - It lets you pick a generated frame's source by signal name.
  - The Browse box searches IDs, messages and signals.

**Bus A hardware filter** (Bus settings, on by default):
- The CAN controller rejects noisy-side frames the isolator doesn't need, worked out from the route table, so they cost no CPU time.
- On the bench, a flood of 3,000 unneeded frames per second used 0% CPU with the filter versus 18% without.
- It's only used while unrouted A → B IDs are dropped and nothing needed from Bus A is 29-bit (the hardware rejects every 29-bit frame in this mode).
- It opens by itself while the sniffer or monitor captures Bus A. The Status tab shows its state.

**Hex / Dec** (top right) switches how IDs and data bytes are shown and typed. Typing `0x…` always means hex.

## Updating the firmware

**More… → Update firmware…**, then choose `Firmware/CanIsolatorC6-<version>.bin`. Every PlatformIO build puts a copy there.
- The page checks the file is isolator firmware and shows the old and new versions.
- It sends the file in about 10 s while the isolator keeps running, then restarts the board and reconnects by itself.
- **Automatic rollback:** the new firmware must run properly for 10 s before it confirms itself. If it crashes or stalls first, the board goes back to the previous firmware by itself.
- Settings are kept. Do updates with the engine off.

Boards running firmware older than 2.4 need one USB flash with PlatformIO (`pio run -t upload` in `CanIsolatorC6`). That installs the partition table with the update slot and the 64 KB settings area.

The protocol is documented at `handleCommand()` in `CanIsolatorC6/CanIsolatorC6.ino`.
