# Radioddity GD-73A CHIRP Driver

An experimental CHIRP driver for the **Radioddity GD-73 / GD-73A** DMR handheld using the radio's native **C7000 USB** programming interface.

This driver provides direct codeplug **download and upload** in CHIRP without using a serial/COM-port programming cable. The radio is opened directly as USB **VID 1206 / PID 0227**.

## What is supported

- Native USB download and upload
- Analog and DMR channels
- Channel names, RX/TX frequencies, duplex/offset, power and bandwidth
- Analog CTCSS/DCS tones and cross modes
- DMR Color Code, Time Slot, TX Contact and RX Group
- Digital Contacts
- RX Groups
- Zones
- Scan Lists
- DTMF Codes
- DTMF PTT Templates
- Quick Text
- A selection of radio-wide settings
- Preservation of unknown/unimplemented codeplug bytes

The large GD-73 collections are exposed as normal CHIRP subdevices rather than being buried in the Settings dialog.

## Tested

The driver has been tested on a Radioddity GD-73A with CHIRP Next on Windows.

Read/write validation included:

- Downloading a codeplug, writing the same image back, and downloading again with a **byte-for-byte identical** result.
- Changing a channel name from `North USA` to `North USB`, writing it to the radio, confirming the radio displayed the new name, and reading it back. The raw codeplug differed by exactly the expected single byte.

This is still an experimental/community driver. **Save a known-good backup image before making changes.** Some less-common CPS settings remain only partially mapped.

## Windows USB driver requirement

The GD-73A is **not programmed as a COM-port radio**. It uses native USB.

Install the official Radioddity GD-73 programming software/CPS first so Windows has the radio's **libusb-win32 (`libusb0.dll`)** support installed. The CHIRP driver uses that same USB stack.

Do **not** replace the radio's Windows driver with WinUSB if you still want the official CPS to work.

## Installing the driver in CHIRP

1. Download [`gd73.py`](gd73.py) from this repository.
2. Start CHIRP with Developer Mode available.
3. Choose **Developer → Load Module**.
4. Select `gd73.py`.
5. CHIRP should register **Radioddity → GD-73A**.

Because stock CHIRP's clone dialog expects a serial port even though this radio does not use one, select **Fake NOP** for the Port. The driver ignores that serial-port object and opens the GD-73A directly over native USB.

## Downloading from the radio

1. Power on the GD-73A and connect it to the computer by USB.
2. In CHIRP choose **Radio → Download From Radio**.
3. Select:
   - **Port:** `Fake NOP`
   - **Vendor:** `Radioddity`
   - **Model:** `GD-73A`
4. Start the download.
5. Save the resulting `.img` file as a backup before editing.

## Uploading to the radio

1. Open or download a GD-73A image in CHIRP.
2. Make your changes.
3. Choose **Radio → Upload To Radio**.
4. Again use **Fake NOP** as the Port.
5. Allow the upload to complete before disconnecting or powering off the radio.

## Why `Fake NOP`?

CHIRP's standard clone dialog normally opens a serial port before calling a radio driver's clone routines. The GD-73A does not use that path. `Fake NOP` simply satisfies CHIRP's UI requirement; `gd73.py` then talks directly to the C7000 USB interface.

## USB protocol notes

The driver uses the GD-73A/C7000 native bulk endpoints:

- VID: `0x1206`
- PID: `0x0227`
- Interface: `0`
- Bulk OUT: `0x02`
- Bulk IN: `0x81`
- Codeplug size: `0x22014` bytes
- Block size: `0x35` (53) bytes

The read/write protocol and timing were validated against the OEM CPS USB traffic. On Windows, a high-resolution short turnaround wait is used rather than `time.sleep(0.001)`, because the latter can be rounded up substantially and make cloning unnecessarily slow.

## Status

The core channel editor and native USB read/write path are working and have been validated on real hardware. Some advanced or uncommon CPS areas are still being mapped conservatively. Unknown bytes are intentionally preserved whenever possible rather than regenerated.

This project is not affiliated with Radioddity or the CHIRP project.
