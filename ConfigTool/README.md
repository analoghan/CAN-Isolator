# CAN Isolator config tool

`index.html` sets up the CANipulator isolator (firmware `CanIsolatorC6` 2.0 or newer) over USB-C.
There's nothing to install.

1. Open the page in **Chrome or Edge** (any Chromium browser with Web Serial) on a Mac, Windows or Linux computer.
   Safari, Firefox and phones/tablets can't talk to USB serial devices.
   - **Mac:** double-click **`Open Config Tool.command`**. The first time, macOS may ask you to confirm: right-click it, choose Open, then Open again.
   - **Windows 10/11:** double-click **`Open Config Tool.bat`** (Chrome if installed, otherwise Edge, which is built in). No driver install is needed: the board shows up as "USB Serial Device (COM…)".
   - **Linux:** open `index.html` in Chrome; your user needs serial-port access (usually the `dialout` group).
2. Close any serial monitor (PlatformIO, Arduino IDE): only one program can use the port.
3. Click **Connect** and pick the "USB JTAG/serial debug unit" (on Windows it may be listed as "USB Serial Device (COM…)").

**Help is built into the page:** the **? Help** tab, or press <kbd>?</kbd> on any tab for that tab's section. It covers getting started, how routing decisions are made, every tab, how-to recipes, bit numbering, firmware updates, recovery, troubleshooting, LED colours, safety notes and a glossary, and it works offline.

## Buttons

| Button | What it does |
|---|---|
| Send to board | Runs the edits now, in RAM only. A power cycle goes back to the saved settings, so this is the safe way to try a change. |
| Save to board | Writes the settings to flash. First lists exactly what changes compared with the settings already saved on the board (firmware 2.6+), so you can cancel. Saving briefly pauses CAN reception, so do it with the engine off. |
| Review changes | Lists what your unsent edits change compared with what the board is running. It can also be shown before every Send. |
| Discard edits | Reloads what the board is running. |
| More… → Export / Import | Back up your settings to a `.json` file, or load them from one. Import also works offline. |

- Bit rate and connector-swap changes take effect after Save plus a restart. Everything else applies immediately.
- **Factory reset without a computer:** hold BOOT for 3 s while the isolator is running. The LED flashes white,
  the saved settings are erased, and the board restarts on the defaults compiled into the firmware.
- Don't hold BOOT while powering up: that enters flashing mode instead.

## Tabs

- **Status**: live counts per bus, bus state, error counters, CPU load, and the generated frames.
  With firmware 2.6+ also **bus load** (now, peak of the busiest 100 ms, and a 15-minute chart) and an **event history**: bus state changes and bursts of errors, overflows and stale frames, kept on the isolator since it started, so you can read what happened on a drive afterwards.
- **Routes**, **Bus settings**: the settings.
- **Buttons** (old): only shown while settings still use the old built-in "Merge wheel buttons". Transforms → **Convert to a transform** replaces it with an equivalent transform (firmware 2.7+).
- **Transforms**: named lists of steps that rewrite the frames a route forwards. A route picks one in its Transform column.
  - Up to 8 transforms, 6 steps each.
  - Step types: change the ID or length; set a field to a value (also how you clear or set bits); copy a field from this frame or another frame, with scaling; forward only if a field matches (=, ≠, <, ≤, >, ≥, has all bits, has any bit); rolling counter.
  - Copy-step options (firmware 2.7+): write 0 when the source frame is missing or stale ("released"), copy only the bits in a mask, and catch short presses (a press shorter than one forwarded frame still shows once).
  - **+ Merge bytes from another frame** starts a transform that copies whole bytes from another frame, for example steering wheel buttons into the ABS frame. This replaces the old Buttons tab.
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
  - **What changed?** (firmware 2.6+) finds the bits behind a button or switch: record the background, then the action in a few tries. Bits that change by themselves are ignored, and the isolator notes every bit that flips, so nothing is missed on a busy bus.
- **Frame monitor**: captures individual frames, RX and/or TX on either bus, with up to 8 ID or range filters.
  - Changed bytes are highlighted, and Δt is shown per ID. Capture can be saved as CSV.
  - Click a frame to open the **bit analyzer**:
    - Brighter cells are bits that change more often.
    - Click the first and last bit of a value, choose Motorola/Intel, scale and offset, and see the value decoded live and graphed.
    - It points out changing bytes and rolling counters.
    - Signals can be saved; they're kept in the browser and in exported settings files. A DBC `SG_` line can be pasted in, and each signal is shown as one.
- **Graph**: plots signals (from DBC files or saved analyzer signals) over time, one chart per signal with a shared, synced crosshair. Live from the frame monitor, or from a recording or log file.
- **Record & replay**:
  - Record frames with their timing (everything on chosen buses, or the monitor's filters) and save as a candump log or CSV.
  - Replay (firmware 2.6+) a recording or a log file (candump `-l`, CSV from this tool or SavvyCAN, Vector `.asc`).
    Each channel can be *fed in as received* on Bus A or B (it goes through routes and transforms, for bench testing with only the ECU or dash connected) or *sent* on a bus.
  - Speed ¼× to 4×, loop, time range and ID filter. The isolator times the frames itself (about 1 ms); the page streams them. Replay stops by itself 5 s after the page closes.
- **Send test frames**: send a frame once, or repeat up to 4 frames on a timer. Repeats stop by themselves 5 s after
  the page closes or the cable is pulled.

- **DBC files**: load `.dbc` files, such as your ECU maker's or your steering wheel's.
  - Each file is kept in this browser and set to describe Bus A, Bus B or either.
  - It adds message names to the sniffer and monitor, and decoded signals with units and value names to the bit analyzer (click one to chart it).
  - It lets you pick a generated frame's source by signal name.
  - The Browse box searches IDs, messages and signals.

**Bus A hardware filter** (Bus settings, on by default):
- The CAN controller rejects chassis-side frames the isolator doesn't need, worked out from the route table, so they cost no CPU time.
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
