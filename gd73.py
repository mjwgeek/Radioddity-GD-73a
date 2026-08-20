# Experimental CHIRP driver for Radioddity GD-73 / GD-73A
#
# Stage 1: full-layout image driver scaffold.
#
# Goals:
#   * Recognize/open the vendor 0x22014-byte .rdt image.
#   * Decode/edit channel basics while preserving all unknown bytes.
#   * Present a CPS-like Settings tree for the full codeplug layout.
#   * Only expose VERIFIED writable settings.
#   * No live USB clone transport yet.
#
# Verified from the GD-73 codeplug map:
#   0x00000..0x00060  Basic information
#   0x00061..0x0010A  General/settings block
#   0x0010B..0x00D4B  Zones
#   0x00D4C..0x1254D  Channels
#   0x125FF..0x1C200  Contacts
#   0x1C201..0x2130F  RX groups
#   0x21310..0x21910  Scan lists
#   0x21911..0x2191E  DMR settings
#   0x2191F..0x2196E  Encryption keys
#   0x2196F..0x21E7F  Quick text/messages
#   0x21E80..0x21E93  DTMF systems
#   0x21E94..0x21F23  DTMF numbers
#   0x21F24..0x21FC3  DTMF PTT settings
#
# Unknown/unimplemented fields are preserved verbatim.

import logging
import struct
import time
import os
import sys

try:
    import usb.core
    import usb.util
except ImportError:
    usb = None

try:
    import libusb_package
except ImportError:
    libusb_package = None

from chirp import chirp_common, directory, errors, memmap
from chirp.settings import (
    RadioSetting,
    RadioSettingGroup,
    RadioSettingValueBoolean,
    RadioSettingValueInteger,
    RadioSettingValueList,
    RadioSettingValueString,
    RadioSettings,
)

LOG = logging.getLogger(__name__)
DRIVER_REVISION = "public-read-write-v1"

IMAGE_SIZE = 0x22014

USB_VID = 0x1206
USB_PID = 0x0227
USB_INTERFACE = 0
USB_EP_OUT = 0x02
USB_EP_IN = 0x81
USB_TIMEOUT_MS = 1000

BLOCK_SIZE = 0x35
BLOCK_COUNT = IMAGE_SIZE // BLOCK_SIZE

INFO_BASE = 0x00000
SETTINGS_BASE = 0x00061
ZONE_BANK = 0x0010B
CHANNEL_BANK = 0x00D4C
CONTACT_BANK = 0x125FF
RXGROUP_BANK = 0x1C201
SCANLIST_BANK = 0x21310
DMRSETTINGS_BASE = 0x21911
ENCRYPT_BANK = 0x2191F
MESSAGE_BANK = 0x2196F
DTMF_SYSTEM_BANK = 0x21E80
DTMF_NUMBER_BANK = 0x21E94
DTMF_PTT_BANK = 0x21F24

# Stage-4 aliases / additional mapped blocks.
# Keep the original *_BANK names above for compatibility with earlier stages.
MESSAGE_BASE = MESSAGE_BANK
DTMF_SYSTEM_BASE = DTMF_SYSTEM_BANK
DTMF_NUMBER_BASE = DTMF_NUMBER_BANK
DTMF_PTT_BASE = DTMF_PTT_BANK

DTMF_SYSTEM_SIZE = 0x05
DTMF_NUMBER_SIZE = 0x09
DTMF_PTT_SIZE = 0x05

# Digital Emergency System occupies the 0xB1-byte gap immediately after
# the channel bank and before the contact bank in this GD-73 codeplug.
EMERGENCY_BASE = 0x1254E
EMERGENCY_RECORDS = EMERGENCY_BASE + 0x01
EMERGENCY_SIZE = 0xB0

CHANNEL_SIZE = 0x46
CONTACT_SIZE = 0x25
ZONE_SIZE = 0x31
RXGROUP_SIZE = 0x53
SCANLIST_SIZE = 0x5F

ZONE_RECORDS = ZONE_BANK + 0x01
RXGROUP_RECORDS = RXGROUP_BANK + 0x01
SCANLIST_RECORDS = SCANLIST_BANK + 0x11

CHANNEL_RECORDS = CHANNEL_BANK + 0x0002
CONTACT_RECORDS = CONTACT_BANK + 0x0802

O_NAME = 0x00
O_BANDWIDTH = 0x20
O_SCANLIST = 0x21
O_TYPE = 0x22
O_TALKAROUND = 0x23
O_RXONLY = 0x24
O_SCANAUTOSTART = 0x26
O_RXFREQ = 0x27
O_TXFREQ = 0x2B
O_POWER = 0x30
O_ADMIT = 0x31
O_RX_TONE_MODE = 0x34
O_RX_CTCSS = 0x35
O_RX_DCS = 0x36
O_TX_TONE_MODE = 0x37
O_TX_CTCSS = 0x38
O_TX_DCS = 0x39
O_TIMESLOT = 0x3C
O_COLORCODE = 0x3D
O_GROUPLIST = 0x3E
O_CONTACT = 0x40

POWER_LEVELS = [
    chirp_common.PowerLevel("Low", watts=1.0),
    chirp_common.PowerLevel("High", watts=2.0),
]

# Exact GD-73 tables used by QDMR.
CTCSS_CODES = [
    62.5, 67.0, 69.3, 71.9, 74.4, 77.0, 79.7, 82.5,
    85.4, 88.5, 91.5, 94.8, 97.4, 100.0, 103.5, 107.2,
    110.9, 114.8, 118.8, 123.0, 127.3, 131.8, 136.5, 141.3,
    146.2, 151.4, 156.7, 159.8, 162.2, 165.5, 167.9, 171.3,
    173.8, 177.3, 179.9, 183.5, 186.2, 189.9, 192.8, 196.6,
    199.5, 203.5, 206.5, 210.7, 218.1, 225.7, 229.1, 233.6,
    241.8, 250.3, 254.1,
]

DCS_CODES = [
    23, 25, 26, 31, 32, 36, 43, 47, 51, 53, 54, 65, 71, 72, 73, 74,
    114, 115, 116, 122, 125, 131, 132, 134, 143, 145, 152, 155, 156,
    162, 165, 172, 174, 205, 212, 223, 225, 226, 243, 244, 245, 246,
    251, 252, 255, 261, 263, 265, 266, 271, 274, 306, 311, 315, 325,
    331, 332, 343, 346, 351, 356, 364, 365, 371, 411, 412, 413, 423,
    431, 432, 445, 446, 452, 454, 455, 462, 464, 465, 466, 503, 506,
    516, 523, 526, 532, 546, 565, 606, 612, 624, 627, 631, 632, 645,
    654, 662, 703, 712, 723, 731, 732, 734, 743, 754,
]

CONTACT_TYPES = ["Group Call", "Private Call", "All Call"]
CHANNEL_DISPLAY_MODES = ["Name", "Frequency"]
BOOT_DISPLAY_MODES = ["Off", "Text", "Image", "Image And Text"]

BUTTON_FUNCTIONS = [
    "None", "Radio Enable", "Radio Check", "Radio Disable",
    "Power Level", "Monitor", "Emergency On", "Emergency Off",
    "Zone Switch", "Scan On/Off", "VOX On/Off",
    "One Touch 1", "One Touch 2", "One Touch 3", "One Touch 4",
    "One Touch 5", "Talkaround", "Lone Worker", "TBST", "Call Swell",
]

PTT_ID_TYPES = ["None", "Pre Only"]
PTT_ID_MODES = ["Forbid", "Each"]

# Settings block offsets confirmed from QDMR's GD-73 map.
S_NAME = 0x00
S_DMRID = 0x20
S_LANGUAGE = 0x24
S_VOX = 0x26
S_SQUELCH = 0x27
S_TOT = 0x28
S_TX_INTERRUPT = 0x29
S_POWER_SAVE = 0x2A
S_POWER_SAVE_TIMEOUT = 0x2B
S_READ_LOCK = 0x2C
S_WRITE_LOCK = 0x2D
S_CHANNEL_DISPLAY = 0x2F
S_READ_PIN = 0x30
S_WRITE_PIN = 0x36
S_DMR_MIC_GAIN = 0x3D
S_FM_MIC_GAIN = 0x3F
S_LONE_RESPONSE = 0x40
S_LONE_REMINDER = 0x42
S_BOOT_MODE = 0x43
S_BOOT_TEXT1 = 0x44
S_BOOT_TEXT2 = 0x64

# DMR settings block offsets.
D_CALL_HANG = 0x00
D_ACTIVE_WAIT = 0x01
D_ACTIVE_RETRIES = 0x02
D_TX_PREAMBLES = 0x03
D_DEC_DISABLE = 0x04
D_DEC_CHECK = 0x05
D_DEC_ENABLE = 0x06


def _u16(data, off):
    raw = data[off:off + 2]
    if isinstance(raw, str):
        raw = raw.encode("latin1")
    return struct.unpack("<H", bytes(raw))[0]


def _u32(data, off):
    raw = data[off:off + 4]
    if isinstance(raw, str):
        raw = raw.encode("latin1")
    return struct.unpack("<I", bytes(raw))[0]


def _byte(data, off):
    """Return one byte as int from bytes/bytearray/CHIRP MemoryMapBytes."""
    raw = data[off]
    if isinstance(raw, int):
        return raw
    if isinstance(raw, str):
        return ord(raw[0])
    raw = bytes(raw)
    if not raw:
        raise IndexError("empty byte at offset 0x%X" % off)
    return raw[0]


def _p16(value):
    return struct.pack("<H", int(value))


def _p32(value):
    return struct.pack("<I", int(value))


def _set_bytes(mmap_obj, offset, payload):
    """Write raw bytes using CHIRP MemoryMapBytes.set()."""
    mmap_obj.set(int(offset), bytes(payload))


def _ascii_z(raw):
    return bytes(raw).split(b"\x00", 1)[0].decode("ascii", "replace")


def _decode_utf16(raw):
    raw = bytes(raw)
    end = len(raw)
    for i in range(0, len(raw) - 1, 2):
        if raw[i] == 0 and raw[i + 1] == 0:
            end = i
            break
    return raw[:end].decode("utf-16le", "replace")


def _encode_utf16(text, chars):
    raw = (text or "")[:chars].encode("utf-16le", "replace")
    return raw[:chars * 2].ljust(chars * 2, b"\x00")


def _ro_string(name, label, value, maxlen=96):
    value = str(value)
    v = RadioSettingValueString(
        0, max(maxlen, len(value)), value,
        autopad=False,
        charset=chirp_common.CHARSET_ASCII)
    v.set_mutable(False)
    return RadioSetting(name, label, v)


def _volatile_integer_setting(name, label, minimum, maximum, current):
    setting = RadioSetting(
        name, label,
        RadioSettingValueInteger(minimum, maximum, current))
    setting.set_volatile(True)
    return setting


def _section_status(name, text):
    return _ro_string(name, "Status", text, 128)



def _decode_gd73_tone(mode, ctcss_index, dcs_index):
    """Return CHIRP split-tone tuple: (mode, value, polarity)."""
    mode = int(mode)
    if mode == 0:
        return ("", None, None)
    if mode == 1:
        if 0 <= ctcss_index < len(CTCSS_CODES):
            return ("Tone", CTCSS_CODES[ctcss_index], None)
        return ("", None, None)
    if mode in (2, 3):
        if 0 <= dcs_index < len(DCS_CODES):
            return ("DTCS", DCS_CODES[dcs_index],
                    "R" if mode == 3 else "N")
    return ("", None, None)


def _encode_gd73_tone(tone):
    """Encode CHIRP split-tone tuple into GD-73 mode/index fields."""
    mode, value, polarity = tone
    if not mode:
        return (0, 0, 0)

    if mode == "Tone":
        try:
            idx = CTCSS_CODES.index(float(value))
        except (ValueError, TypeError):
            raise errors.RadioError(
                "Unsupported GD-73 CTCSS tone: %s" % value)
        return (1, idx, 0)

    if mode == "DTCS":
        try:
            idx = DCS_CODES.index(int(value))
        except (ValueError, TypeError):
            raise errors.RadioError(
                "Unsupported GD-73 DCS code: %s" % value)
        return (3 if polarity == "R" else 2, 0, idx)

    raise errors.RadioError("Unsupported GD-73 tone mode: %s" % mode)


def _read_ascii_field(data, off, length):
    raw = bytes(data[off:off + length])
    return raw.split(b"\\x00", 1)[0].decode("ascii", "replace")


def _write_ascii_field(data, off, length, value):
    raw = str(value).encode("ascii", "ignore")[:length]
    _set_bytes(data, off, raw.ljust(length, b"\\x00"))



def _c7000_packet(command, subcommand, flags=0x0F, payload=b""):
    """Build the C7000 packet format used by GD-73/QDMR."""
    payload = bytes(payload)
    out = bytearray(9 + len(payload))
    out[0] = 0x68
    out[1] = flags & 0xFF
    out[2] = command & 0xFF
    out[3] = subcommand & 0xFF
    out[6:8] = struct.pack("<H", len(payload))
    out[8:8 + len(payload)] = payload
    out[8 + len(payload)] = 0x10

    crc = 0xFFFF
    for i in range(len(out) // 2):
        v = out[2 * i] | (out[2 * i + 1] << 8)
        if crc < v:
            crc += 0xFFFF
        crc -= v
    if len(out) & 1:
        v = out[-1]
        if crc < v:
            crc += 0xFFFF
        crc -= v
    out[4:6] = struct.pack("<H", crc & 0xFFFF)
    return bytes(out)


def _gd73_precise_wait(seconds):
    """High-resolution short wait for C7000 USB turn-around.

    On packaged Windows CHIRP, time.sleep(0.001) can round up toward the
    Windows scheduler quantum. The OEM CPS capture shows a median
    request-to-IN-submit delay of about 872 microseconds.
    """
    deadline = time.perf_counter() + float(seconds)
    while time.perf_counter() < deadline:
        pass


def _parse_c7000(data):
    data = bytes(data)
    if len(data) < 9 or data[0] != 0x68:
        raise ValueError("Not a C7000 packet")
    plen = struct.unpack_from("<H", data, 6)[0]
    total = 9 + plen
    if len(data) < total:
        raise ValueError("Truncated C7000 packet")
    data = data[:total]
    if data[8 + plen] != 0x10:
        raise ValueError("Bad C7000 terminator")
    return data[8:8 + plen]



# ---------------------------------------------------------------------------
# Stock-CHIRP Windows transport.
#
# IMPORTANT: everything ctypes/libusb0-specific is created lazily inside the
# function below. This keeps CHIRP's Developer -> Load Module import path
# completely free of DLL/ctypes structure initialization.
# ---------------------------------------------------------------------------


def _open_libusb0_gd73():
    """Open GD-73A through libusb-win32 using only Python stdlib ctypes."""
    if sys.platform != "win32":
        return None

    import ctypes

    path_max = 511

    class USBPack(ctypes.Structure):
        _pack_ = 1

    class USBDeviceDescriptor(USBPack):
        _fields_ = [
            ("bLength", ctypes.c_uint8),
            ("bDescriptorType", ctypes.c_uint8),
            ("bcdUSB", ctypes.c_uint16),
            ("bDeviceClass", ctypes.c_uint8),
            ("bDeviceSubClass", ctypes.c_uint8),
            ("bDeviceProtocol", ctypes.c_uint8),
            ("bMaxPacketSize0", ctypes.c_uint8),
            ("idVendor", ctypes.c_uint16),
            ("idProduct", ctypes.c_uint16),
            ("bcdDevice", ctypes.c_uint16),
            ("iManufacturer", ctypes.c_uint8),
            ("iProduct", ctypes.c_uint8),
            ("iSerialNumber", ctypes.c_uint8),
            ("bNumConfigurations", ctypes.c_uint8),
        ]

    class USBDevice(USBPack):
        pass

    class USBBus(USBPack):
        pass

    USBDevice._fields_ = [
        ("next", ctypes.POINTER(USBDevice)),
        ("prev", ctypes.POINTER(USBDevice)),
        ("filename", ctypes.c_int8 * (path_max + 1)),
        ("bus", ctypes.POINTER(USBBus)),
        ("descriptor", USBDeviceDescriptor),
        ("config", ctypes.c_void_p),
        ("dev", ctypes.c_void_p),
        ("devnum", ctypes.c_uint8),
        ("num_children", ctypes.c_ubyte),
        ("children", ctypes.POINTER(ctypes.POINTER(USBDevice))),
    ]

    USBBus._fields_ = [
        ("next", ctypes.POINTER(USBBus)),
        ("prev", ctypes.POINTER(USBBus)),
        ("dirname", ctypes.c_char * (path_max + 1)),
        ("devices", ctypes.POINTER(USBDevice)),
        ("location", ctypes.c_uint32),
        ("root_dev", ctypes.POINTER(USBDevice)),
    ]

    def usb_error(lib, prefix):
        try:
            raw = lib.usb_strerror()
            detail = (raw.decode("utf-8", "replace")
                      if raw else "unknown libusb0 error")
        except Exception:
            detail = "unknown libusb0 error"
        return "%s: %s" % (prefix, detail)

    candidates = ["libusb0.dll"]

    windir = os.environ.get("WINDIR", r"C:\Windows")
    candidates.extend([
        os.path.join(windir, "System32", "libusb0.dll"),
        os.path.join(windir, "SysWOW64", "libusb0.dll"),
    ])

    lib = None
    loaded_from = None
    seen = set()
    last_error = None
    for candidate in candidates:
        if not candidate:
            continue
        key = candidate.lower()
        if key in seen:
            continue
        seen.add(key)
        try:
            lib = ctypes.WinDLL(candidate)
            loaded_from = candidate
            break
        except Exception as e:
            last_error = e

    if lib is None:
        raise errors.RadioError(
            "Could not load libusb0.dll required for GD-73A USB access: %s" %
            last_error)

    # Declare only the small libusb-0.1 API subset we actually use.
    lib.usb_init.argtypes = []
    lib.usb_init.restype = None
    lib.usb_find_busses.argtypes = []
    lib.usb_find_busses.restype = ctypes.c_int
    lib.usb_find_devices.argtypes = []
    lib.usb_find_devices.restype = ctypes.c_int
    lib.usb_get_busses.argtypes = []
    lib.usb_get_busses.restype = ctypes.POINTER(USBBus)
    lib.usb_open.argtypes = [ctypes.POINTER(USBDevice)]
    lib.usb_open.restype = ctypes.c_void_p
    lib.usb_close.argtypes = [ctypes.c_void_p]
    lib.usb_close.restype = ctypes.c_int
    lib.usb_claim_interface.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.usb_claim_interface.restype = ctypes.c_int
    lib.usb_release_interface.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.usb_release_interface.restype = ctypes.c_int
    lib.usb_bulk_write.argtypes = [
        ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p,
        ctypes.c_int, ctypes.c_int]
    lib.usb_bulk_write.restype = ctypes.c_int
    lib.usb_bulk_read.argtypes = [
        ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p,
        ctypes.c_int, ctypes.c_int]
    lib.usb_bulk_read.restype = ctypes.c_int
    lib.usb_strerror.argtypes = []
    lib.usb_strerror.restype = ctypes.c_char_p

    LOG.info("GD73 stock-CHIRP backend loaded: %s", loaded_from)

    lib.usb_init()
    rc = lib.usb_find_busses()
    if rc < 0:
        raise errors.RadioError(usb_error(lib, "usb_find_busses failed"))
    rc = lib.usb_find_devices()
    if rc < 0:
        raise errors.RadioError(usb_error(lib, "usb_find_devices failed"))

    class LibUSB0Device:
        _is_libusb0 = True

        def __init__(self, devptr, handle):
            self._devptr = devptr
            self._handle = handle
            self.bus = "libusb0"
            self.address = int(devptr.contents.devnum)
            self._closed = False

        def write(self, endpoint, payload, timeout=USB_TIMEOUT_MS):
            payload = bytes(payload)
            buf = ctypes.create_string_buffer(payload, len(payload))
            rc = lib.usb_bulk_write(
                self._handle, int(endpoint),
                ctypes.cast(buf, ctypes.c_void_p),
                len(payload), int(timeout))
            if rc < 0:
                raise errors.RadioError(
                    usb_error(lib, "GD-73A libusb0 bulk write failed"))
            return rc

        def read(self, endpoint, size, timeout=USB_TIMEOUT_MS):
            buf = ctypes.create_string_buffer(int(size))
            rc = lib.usb_bulk_read(
                self._handle, int(endpoint),
                ctypes.cast(buf, ctypes.c_void_p),
                int(size), int(timeout))
            if rc < 0:
                try:
                    raw = lib.usb_strerror()
                    detail = (raw.decode("utf-8", "replace").lower()
                              if raw else "")
                except Exception:
                    detail = ""
                if "timeout" in detail or "timed out" in detail:
                    return b""
                raise errors.RadioError(
                    usb_error(lib, "GD-73A libusb0 bulk read failed"))
            return bytes(buf.raw[:rc])

        def close(self):
            if self._closed:
                return
            self._closed = True
            try:
                lib.usb_release_interface(self._handle, USB_INTERFACE)
            except Exception:
                pass
            try:
                lib.usb_close(self._handle)
            except Exception:
                pass

    bus = lib.usb_get_busses()
    while bool(bus):
        dev = bus.contents.devices
        while bool(dev):
            desc = dev.contents.descriptor
            if desc.idVendor == USB_VID and desc.idProduct == USB_PID:
                handle = lib.usb_open(dev)
                if not handle:
                    raise errors.RadioError(
                        usb_error(lib, "Could not open GD-73A USB device"))

                wrapped = LibUSB0Device(dev, handle)
                rc = lib.usb_claim_interface(handle, USB_INTERFACE)
                if rc < 0:
                    wrapped.close()
                    raise errors.RadioError(
                        usb_error(lib, "Could not claim USB interface 0"))

                LOG.info(
                    "GD73 libusb0 selected address=%s VID=%04X PID=%04X",
                    wrapped.address, desc.idVendor, desc.idProduct)
                return wrapped

            dev = dev.contents.next
        bus = bus.contents.next

    raise errors.RadioError(
        "GD-73A USB device 1206:0227 was not found through libusb-win32. "
        "Confirm the radio is powered on and its Windows driver is "
        "libusb-win32.")


@directory.register
class GD73ARadio(chirp_common.CloneModeRadio):
    """Radioddity GD-73 / GD-73A experimental full-layout image driver."""

    VENDOR = "Radioddity"
    MODEL = "GD-73A"
    VARIANT = "Native USB read v4 / mmap parser fix"
    BAUD_RATE = 57600

    @classmethod
    def get_prompts(cls):
        prompts = chirp_common.RadioPrompts()
        prompts.experimental = (
            "Experimental GD-73A native-USB read/write support. "
            "The CHIRP Port selection is only a dummy because this driver "
            "opens USB VID 1206 PID 0227 directly. Select "
            "Fake NOP as the port. The serial port is ignored because this radio uses native USB.")
        prompts.pre_download = (
            "Connect and power on the GD-73A. The selected serial port is "
            "ignored by the driver; use Fake NOP as the dummy port for both download and upload.")
        return prompts

    @classmethod
    def match_model(cls, filedata, filename):
        if len(filedata) != IMAGE_SIZE:
            return False
        model = filedata[0x21:0x31].split(b"\x00", 1)[0]
        if model not in (b"GD-73", b"GD-73A"):
            return False
        count = struct.unpack_from("<H", filedata, CHANNEL_BANK)[0]
        return 0 <= count <= 1024

    def process_mmap(self):
        pass

    def _usb_backend(self):
        """Return the optional PyUSB backend used only as a fallback."""
        if usb is None:
            return None

        if libusb_package is not None:
            backend = libusb_package.get_libusb1_backend()
            if backend is not None:
                LOG.info("GD73 fallback USB backend: libusb-package/libusb-1.0")
                return backend

        LOG.info("GD73 fallback USB backend: PyUSB automatic backend")
        return None

    def _open_usb(self):
        # Stock Windows CHIRP path: no third-party Python packages required.
        if sys.platform == "win32":
            try:
                dev = _open_libusb0_gd73()
                if dev is not None:
                    LOG.info(
                        "GD73 USB interface 0 claimed through libusb-win32 "
                        "without SET_CONFIGURATION")
                    LOG.info(
                        "GD73 C7000 packet flags default = 0x0F "
                        "(from successful CPS capture)")
                    return dev
            except errors.RadioError:
                # If PyUSB exists (our development/source environment), allow
                # it as a fallback. In packaged stock CHIRP, this branch will
                # re-raise because usb is normally unavailable.
                if usb is None:
                    raise
                LOG.exception(
                    "GD73 direct libusb0 backend failed; trying PyUSB fallback")

        if usb is None:
            raise errors.RadioError(
                "GD-73A USB support requires libusb-win32 (libusb0.dll). "
                "The radio must use the libusb-win32 device driver, as it does "
                "with the OEM CPS.")

        backend = self._usb_backend()
        kwargs = {
            "find_all": True,
            "idVendor": USB_VID,
            "idProduct": USB_PID,
        }
        if backend is not None:
            kwargs["backend"] = backend

        devices = list(usb.core.find(**kwargs) or [])
        if not devices:
            raise errors.RadioError(
                "GD-73A USB device 1206:0227 was not found. "
                "Connect and power on the radio.")

        dev = devices[0]
        LOG.info(
            "GD73 PyUSB selected bus=%s address=%s",
            getattr(dev, "bus", "?"), getattr(dev, "address", "?"))

        try:
            try:
                if dev.is_kernel_driver_active(USB_INTERFACE):
                    dev.detach_kernel_driver(USB_INTERFACE)
            except (NotImplementedError, usb.core.USBError):
                pass

            usb.util.claim_interface(dev, USB_INTERFACE)
            LOG.info(
                "GD73 USB interface 0 claimed through PyUSB "
                "without SET_CONFIGURATION")
            LOG.info(
                "GD73 C7000 packet flags default = 0x0F "
                "(from successful CPS capture)")
        except Exception as e:
            raise errors.RadioError(
                "Could not claim GD-73A USB interface 0: %s" % e)

        return dev

    def _usb_send_recv(self, dev, packet):
        LOG.debug("GD73 USB TX: %s", packet.hex(" "))

        try:
            wrote = dev.write(
                USB_EP_OUT, packet, timeout=USB_TIMEOUT_MS)
        except Exception as e:
            raise errors.RadioError("GD-73A USB write failed: %s" % e)

        if wrote != len(packet):
            raise errors.RadioError(
                "GD-73A short USB write (%d/%d)" % (wrote, len(packet)))

        # OEM CPS capture: median request -> IN submission ~= 872 us.
        # Avoid time.sleep(0.001) here because packaged Windows Python can
        # oversleep by an order of magnitude.
        _gd73_precise_wait(0.00085)

        # QDMR's known receive endpoint is 0x81. Ignore zero-filled frames
        # briefly because the vendor stack has been observed returning them.
        for attempt in range(11):
            try:
                rx = bytes(dev.read(
                    USB_EP_IN, 64, timeout=USB_TIMEOUT_MS))
            except Exception as e:
                if (usb is not None and
                        isinstance(e, usb.core.USBTimeoutError)):
                    LOG.debug(
                        "GD73 USB RX timeout attempt %d", attempt + 1)
                    continue
                raise errors.RadioError("GD-73A USB read failed: %s" % e)

            LOG.debug("GD73 USB RX: %s", rx.hex(" "))
            if rx and rx[0] == 0x68:
                try:
                    return _parse_c7000(rx)
                except ValueError as e:
                    raise errors.RadioError(
                        "Invalid GD-73A C7000 response: %s" % e)

            if rx and any(rx):
                LOG.warning(
                    "GD73 non-C7000 RX attempt %d: %s",
                    attempt + 1, rx.hex(" "))

        raise errors.RadioError(
            "GD-73A did not return a valid C7000 response on USB 0x81 "
            "after 11 QDMR-style reads (only empty/zero responses were received).")

    def _read_usb_image(self, dev):
        # Enter programming mode.
        payload = self._usb_send_recv(
            dev, _c7000_packet(0x01, 0x04))
        LOG.info("GD73 programming-mode payload: %s", payload.hex(" "))

        image = bytearray()
        last_sequence = 0xFFFF

        status = chirp_common.Status()
        status.max = BLOCK_COUNT
        status.cur = 0
        status.msg = "Reading GD-73A"
        if self.status_fn:
            self.status_fn(status)

        for block in range(BLOCK_COUNT):
            if block == 0:
                request = _c7000_packet(0x01, 0x02)
            else:
                request = _c7000_packet(
                    0x04, 0x01, 0x0F,
                    struct.pack("<H", last_sequence))

            payload = self._usb_send_recv(dev, request)
            if len(payload) < 2 + BLOCK_SIZE:
                raise errors.RadioError(
                    "GD-73A short block response at %d: %d bytes" %
                    (block, len(payload)))

            seq = struct.unpack_from("<H", payload, 0)[0]
            if seq != block:
                raise errors.RadioError(
                    "GD-73A sequence error: expected %d got %d" %
                    (block, seq))

            image.extend(payload[2:2 + BLOCK_SIZE])
            last_sequence = seq

            status.cur = block + 1
            if self.status_fn and (
                    block == 0 or (block + 1) % 10 == 0 or
                    block + 1 == BLOCK_COUNT):
                self.status_fn(status)

        if len(image) != IMAGE_SIZE:
            raise errors.RadioError(
                "GD-73A image size mismatch: got %d expected %d" %
                (len(image), IMAGE_SIZE))

        return bytes(image)

    def sync_in(self):
        """Download directly from the native C7000 USB device."""
        dev = None
        try:
            dev = self._open_usb()
            started = time.perf_counter()
            image = self._read_usb_image(dev)
            elapsed = time.perf_counter() - started
            LOG.info(
                "GD73 USB image read: %d bytes in %.3f seconds (%.1f KiB/s)",
                len(image), elapsed,
                (len(image) / 1024.0 / elapsed) if elapsed else 0.0)
            self._mmap = memmap.MemoryMapBytes(image)
            self.process_mmap()
            LOG.info("GD73 live USB download complete: %d bytes", len(image))
        finally:
            if dev is not None:
                if getattr(dev, "_is_libusb0", False):
                    dev.close()
                elif usb is not None:
                    try:
                        usb.util.release_interface(dev, USB_INTERFACE)
                    except Exception:
                        pass
                    try:
                        usb.util.dispose_resources(dev)
                    except Exception:
                        pass

    def _write_usb_image(self, dev, image):
        """Write a complete C7000 codeplug image using the OEM CPS sequence."""
        image = bytes(image)
        if len(image) != IMAGE_SIZE:
            raise errors.RadioError(
                "Refusing GD-73A upload: image is %d bytes, expected exactly %d" %
                (len(image), IMAGE_SIZE))

        # OEM capture enters programming mode before the first write block.
        payload = self._usb_send_recv(
            dev, _c7000_packet(0x01, 0x04))
        LOG.info("GD73 write programming-mode payload: %s", payload.hex(" "))

        status = chirp_common.Status()
        status.max = BLOCK_COUNT
        status.cur = 0
        status.msg = "Writing GD-73A"
        if self.status_fn:
            self.status_fn(status)

        for block in range(BLOCK_COUNT):
            start = block * BLOCK_SIZE
            chunk = image[start:start + BLOCK_SIZE]
            if len(chunk) != BLOCK_SIZE:
                raise errors.RadioError(
                    "Refusing GD-73A upload: short source block %d (%d bytes)" %
                    (block, len(chunk)))

            # OEM CPS capture:
            #   request command/sub = 0x01/0x00
            #   payload length = 0x37 = 55 bytes
            #   payload = uint16_le(sequence) + 53-byte codeplug block
            request = _c7000_packet(
                0x01, 0x00, 0x0F,
                struct.pack("<H", block) + chunk)

            # IMPORTANT: _usb_send_recv() may retry IN reads after a zero
            # frame, but it never resends the write request. This avoids
            # ambiguous duplicate writes.
            ack = self._usb_send_recv(dev, request)

            # OEM ACK command/sub is 0x00/0x01 and its payload is the
            # two-byte sequence number. _usb_send_recv() returns payload.
            if len(ack) != 2:
                raise errors.RadioError(
                    "GD-73A invalid write ACK at block %d: expected 2 bytes, "
                    "got %d" % (block, len(ack)))

            ack_seq = struct.unpack("<H", ack)[0]
            if ack_seq != block:
                raise errors.RadioError(
                    "GD-73A write sequence ACK mismatch: sent %d, got %d" %
                    (block, ack_seq))

            status.cur = block + 1
            if self.status_fn and (
                    block == 0 or (block + 1) % 10 == 0 or
                    block + 1 == BLOCK_COUNT):
                self.status_fn(status)

        LOG.info("GD73 USB upload complete: %d blocks / %d bytes",
                 BLOCK_COUNT, len(image))

    def sync_out(self):
        """Upload the current image to the native C7000 USB device."""
        if self._mmap is None:
            raise errors.RadioError(
                "Refusing GD-73A upload: no codeplug image is loaded.")

        image = self._mmap.get_packed()
        if not isinstance(image, bytes):
            image = bytes(image)

        if len(image) != IMAGE_SIZE:
            raise errors.RadioError(
                "Refusing GD-73A upload: image is %d bytes, expected exactly %d" %
                (len(image), IMAGE_SIZE))

        dev = None
        try:
            dev = self._open_usb()
            started = time.perf_counter()
            self._write_usb_image(dev, image)
            elapsed = time.perf_counter() - started
            LOG.info(
                "GD73 USB image write: %d bytes in %.3f seconds (%.1f KiB/s)",
                len(image), elapsed,
                (len(image) / 1024.0 / elapsed) if elapsed else 0.0)
        finally:
            if dev is not None:
                if getattr(dev, "_is_libusb0", False):
                    dev.close()
                elif usb is not None:
                    try:
                        usb.util.release_interface(dev, USB_INTERFACE)
                    except Exception:
                        pass
                    try:
                        usb.util.dispose_resources(dev)
                    except Exception:
                        pass

    def _m(self):
        return self._mmap.get_byte_compatible()

    def _channel_count(self):
        return min(_u16(self._m(), CHANNEL_BANK), 1024)

    def _contact_count(self):
        return min(_u16(self._m(), CONTACT_BANK), 1024)

    def _channel_off(self, number):
        if number < 1 or number > self._channel_count():
            raise errors.InvalidMemoryLocation(
                "Memory %s outside GD-73 channel count" % number)
        return CHANNEL_RECORDS + (number - 1) * CHANNEL_SIZE

    def _contact(self, index0):
        if index0 < 0 or index0 >= self._contact_count():
            return None
        d = self._m()
        off = CONTACT_RECORDS + index0 * CONTACT_SIZE
        return {
            "index": index0,
            "name": _decode_utf16(d[off:off + 0x20]),
            "type": _byte(d, off + 0x20),
            "id": _u32(d, off + 0x21),
        }

    def _contact_choices(self):
        out = ["None"]
        for i in range(self._contact_count()):
            c = self._contact(i)
            out.append("%d: %s (%d)" % (i + 1, c["name"], c["id"]))
        return out

    def _channel_name(self, index0):
        if index0 < 0 or index0 >= self._channel_count():
            return ""
        d = self._m()
        off = CHANNEL_RECORDS + index0 * CHANNEL_SIZE
        return _decode_utf16(d[off:off + 0x20])

    def _channel_choices(self):
        out = ["None"]
        for i in range(self._channel_count()):
            out.append("%d: %s" % (i + 1, self._channel_name(i)))
        return out

    def _zone_count(self):
        return min(_byte(self._m(), ZONE_BANK), 64)

    def _zone(self, index0):
        if index0 < 0 or index0 >= self._zone_count():
            return None
        d = self._m()
        off = ZONE_RECORDS + index0 * ZONE_SIZE
        count = min(_byte(d, off + 0x10), 16)
        members = []
        for i in range(16):
            raw = _u16(d, off + 0x11 + i * 2)
            members.append(raw - 1 if raw else None)
        return {
            "index": index0,
            "name": _decode_utf16(d[off:off + 0x10]),
            "count": count,
            "members": members,
        }

    def _zone_choices(self):
        out = ["None"]
        for i in range(self._zone_count()):
            z = self._zone(i)
            out.append("%d: %s" % (i + 1, z["name"]))
        return out

    def _rxgroup_count(self):
        return min(_byte(self._m(), RXGROUP_BANK), 250)

    def _rxgroup(self, index0):
        if index0 < 0 or index0 >= self._rxgroup_count():
            return None
        d = self._m()
        off = RXGROUP_RECORDS + index0 * RXGROUP_SIZE
        count = min(_byte(d, off + 0x10), 33)
        members = []
        for i in range(33):
            raw = _u16(d, off + 0x11 + i * 2)
            members.append(raw - 1 if raw else None)
        return {
            "index": index0,
            "name": _decode_utf16(d[off:off + 0x10]),
            "count": count,
            "members": members,
        }

    def _rxgroup_choices(self):
        # The channel field is encoded specially:
        #   0 = use TX contact, 1 = all match, 2+ = RX group index + 2.
        out = ["TX Contact", "All"]
        for i in range(self._rxgroup_count()):
            g = self._rxgroup(i)
            out.append("%d: %s" % (i + 1, g["name"]))
        return out

    def _scanlist_count(self):
        return min(_byte(self._m(), SCANLIST_BANK), 16)

    def _scanlist(self, index0):
        if index0 < 0 or index0 >= self._scanlist_count():
            return None
        d = self._m()
        off = SCANLIST_RECORDS + index0 * SCANLIST_SIZE
        count = min(_byte(d, off + 0x10), 32)
        members = []
        for i in range(32):
            raw = _u16(d, off + 0x11 + i * 2)
            members.append(raw - 1 if raw else None)
        return {
            "index": index0,
            "name": _decode_utf16(d[off:off + 0x10]),
            "count": count,
            "members": members,
            "pri1_mode": _byte(d, off + 0x51),
            "pri2_mode": _byte(d, off + 0x52),
            "pri1_zone": _byte(d, off + 0x53),
            "pri2_zone": _byte(d, off + 0x54),
            "pri1_channel": _byte(d, off + 0x55),
            "pri2_channel": _byte(d, off + 0x57),
            "revert_mode": _byte(d, off + 0x59),
            "revert_zone": _byte(d, off + 0x5A),
            "revert_channel": _byte(d, off + 0x5B),
            "rx_hold_raw": _byte(d, off + 0x5D),
            "tx_hold_raw": _byte(d, off + 0x5E),
        }

    def _scanlist_choices(self):
        # Channel scan-list field is 0=None, then 1-based scan-list index.
        out = ["None"]
        for i in range(self._scanlist_count()):
            s = self._scanlist(i)
            out.append("%d: %s" % (i + 1, s["name"]))
        return out

    def get_features(self):
        rf = chirp_common.RadioFeatures()
        rf.has_settings = True
        rf.has_bank = False
        rf.has_name = True
        rf.has_offset = True
        rf.has_mode = True
        rf.has_tuning_step = False
        rf.has_ctone = True
        rf.has_cross = True
        rf.has_rx_dtcs = True
        rf.has_dtcs_polarity = True
        rf.valid_modes = ["FM", "NFM", "DMR"]
        rf.valid_tmodes = ["", "Tone", "TSQL", "DTCS", "Cross"]
        rf.valid_cross_modes = [
            "Tone->Tone",
            "DTCS->",
            "->DTCS",
            "Tone->DTCS",
            "DTCS->Tone",
            "->Tone",
            "DTCS->DTCS",
        ]
        rf.valid_duplexes = ["", "-", "+", "split", "off"]
        rf.valid_skips = [""]
        rf.valid_power_levels = POWER_LEVELS
        rf.valid_name_length = 16
        rf.valid_characters = chirp_common.CHARSET_ASCII
        rf.valid_bands = [(400000000, 470000000)]
        rf.memory_bounds = (1, max(1, self._channel_count()))
        rf.has_sub_devices = (self.VARIANT == "Native USB read v4 / mmap parser fix")
        return rf

    def get_raw_memory(self, number):
        d = self._m()
        off = self._channel_off(number)
        return repr(bytes(d[off:off + CHANNEL_SIZE]))

    def get_memory(self, number):
        d = self._m()
        off = self._channel_off(number)
        raw = bytes(d[off:off + CHANNEL_SIZE])

        mem = chirp_common.Memory()
        mem.number = number
        mem.name = _decode_utf16(raw[O_NAME:O_NAME + 0x20])
        mem.freq = _u32(raw, O_RXFREQ)
        tx = _u32(raw, O_TXFREQ)

        if raw[O_RXONLY]:
            mem.duplex = "off"
            mem.offset = 0
        elif tx == mem.freq:
            mem.duplex = ""
            mem.offset = 0
        else:
            delta = tx - mem.freq
            if abs(delta) <= 10000000:
                mem.duplex = "+" if delta > 0 else "-"
                mem.offset = abs(delta)
            else:
                mem.duplex = "split"
                mem.offset = tx

        if raw[O_TYPE] == 1:
            mem.mode = "DMR"
        else:
            # Exact GD-73 encoding: 0=Narrow (12.5K), 1=Wide (25K).
            mem.mode = "FM" if raw[O_BANDWIDTH] == 1 else "NFM"

            tx_tone = _decode_gd73_tone(
                raw[O_TX_TONE_MODE],
                raw[O_TX_CTCSS],
                raw[O_TX_DCS])
            rx_tone = _decode_gd73_tone(
                raw[O_RX_TONE_MODE],
                raw[O_RX_CTCSS],
                raw[O_RX_DCS])
            chirp_common.split_tone_decode(mem, tx_tone, rx_tone)

        if raw[O_POWER] in (0, 1):
            mem.power = POWER_LEVELS[raw[O_POWER]]

        extra = RadioSettingGroup("extra", "GD-73A")
        extra.append(RadioSetting(
            "rx_only", "RX Only",
            RadioSettingValueBoolean(bool(raw[O_RXONLY]))))
        extra.append(RadioSetting(
            "talkaround", "Talkaround",
            RadioSettingValueBoolean(bool(raw[O_TALKAROUND]))))
        extra.append(RadioSetting(
            "scan_autostart", "Scan Auto Start",
            RadioSettingValueBoolean(bool(raw[O_SCANAUTOSTART]))))

        if raw[O_TYPE] == 1:
            extra.append(RadioSetting(
                "timeslot", "DMR Time Slot",
                RadioSettingValueList(
                    ["TS1", "TS2"],
                    current_index=1 if raw[O_TIMESLOT] else 0)))
            extra.append(RadioSetting(
                "colorcode", "DMR Color Code",
                RadioSettingValueInteger(
                    0, 15, min(15, int(raw[O_COLORCODE])))))

            contact_raw = _u16(raw, O_CONTACT)
            choices = self._contact_choices()
            if contact_raw >= len(choices):
                contact_raw = 0
            extra.append(RadioSetting(
                "contact", "DMR TX Contact",
                RadioSettingValueList(
                    choices, current_index=contact_raw)))

            group_raw = _u16(raw, O_GROUPLIST)
            group_choices = self._rxgroup_choices()
            if group_raw >= len(group_choices):
                group_raw = 1
            extra.append(RadioSetting(
                "group_list", "RX Group List",
                RadioSettingValueList(
                    group_choices, current_index=group_raw)))

        scan_raw = int(raw[O_SCANLIST])
        scan_choices = self._scanlist_choices()
        if scan_raw >= len(scan_choices):
            scan_raw = 0
        extra.append(RadioSetting(
            "scan_list", "Scan List",
            RadioSettingValueList(
                scan_choices, current_index=scan_raw)))

        mem.extra = extra
        return mem

    def set_memory(self, mem):
        d = self._m()
        off = self._channel_off(mem.number)
        raw = bytearray(bytes(d[off:off + CHANNEL_SIZE]))

        raw[O_NAME:O_NAME + 0x20] = _encode_utf16(mem.name, 16)
        raw[O_RXFREQ:O_RXFREQ + 4] = _p32(mem.freq)

        if mem.duplex == "off":
            tx = mem.freq
            raw[O_RXONLY] = 1
        elif mem.duplex == "+":
            tx = mem.freq + mem.offset
            raw[O_RXONLY] = 0
        elif mem.duplex == "-":
            tx = mem.freq - mem.offset
            raw[O_RXONLY] = 0
        elif mem.duplex == "split":
            tx = mem.offset
            raw[O_RXONLY] = 0
        else:
            tx = mem.freq
            raw[O_RXONLY] = 0
        raw[O_TXFREQ:O_TXFREQ + 4] = _p32(tx)

        if mem.mode == "DMR":
            raw[O_TYPE] = 1
        else:
            raw[O_TYPE] = 0
            # Exact GD-73 encoding: 0=Narrow, 1=Wide.
            raw[O_BANDWIDTH] = 1 if mem.mode == "FM" else 0

            tx_tone, rx_tone = chirp_common.split_tone_encode(mem)
            tx_mode, tx_ctcss, tx_dcs = _encode_gd73_tone(tx_tone)
            rx_mode, rx_ctcss, rx_dcs = _encode_gd73_tone(rx_tone)

            raw[O_TX_TONE_MODE] = tx_mode
            raw[O_TX_CTCSS] = tx_ctcss
            raw[O_TX_DCS] = tx_dcs
            raw[O_RX_TONE_MODE] = rx_mode
            raw[O_RX_CTCSS] = rx_ctcss
            raw[O_RX_DCS] = rx_dcs

        if mem.power is not None:
            try:
                raw[O_POWER] = POWER_LEVELS.index(mem.power)
            except ValueError:
                pass

        for setting in getattr(mem, "extra", []):
            name = setting.get_name()
            if name == "rx_only":
                raw[O_RXONLY] = 1 if bool(setting.value) else 0
            elif name == "talkaround":
                raw[O_TALKAROUND] = 1 if bool(setting.value) else 0
            elif name == "scan_autostart":
                raw[O_SCANAUTOSTART] = 1 if bool(setting.value) else 0
            elif name == "timeslot":
                raw[O_TIMESLOT] = int(setting.value)
            elif name == "colorcode":
                raw[O_COLORCODE] = int(setting.value)
            elif name == "contact":
                raw[O_CONTACT:O_CONTACT + 2] = _p16(int(setting.value))
            elif name == "group_list":
                raw[O_GROUPLIST:O_GROUPLIST + 2] = _p16(int(setting.value))
            elif name == "scan_list":
                raw[O_SCANLIST] = int(setting.value)

        _set_bytes(d, off, bytes(raw))

    def _message_count(self):
        return min(_byte(self._m(), MESSAGE_BASE), 16)

    def _message(self, index0):
        if index0 < 0 or index0 >= 16:
            return ""
        d = self._m()
        off = MESSAGE_BASE + 1 + index0 * 0x51
        length = min(_byte(d, off), 40)
        raw = bytes(d[off + 1:off + 1 + length * 2])
        try:
            return raw.decode("utf-16le", "replace").rstrip("\x00")
        except Exception:
            return ""

    def _dtmf_system(self, index0):
        d = self._m()
        off = DTMF_SYSTEM_BASE + index0 * DTMF_SYSTEM_SIZE
        return {
            "sidetone": bool(_byte(d, off + 0)),
            "pre_ms": _byte(d, off + 1) * 10,
            # Verified: raw 0 = 30 ms in CPS.
            "tone_ms": 30 + _byte(d, off + 2) * 10,
            "interval_ms": 30 + _byte(d, off + 3) * 10,
            # Verified: raw 10 = 2.2 s in CPS.
            "restore_ms": 200 + _byte(d, off + 4) * 200,
        }

    def _dtmf_number(self, index0):
        """Decode one GD-73 DTMF code.

        CPS-validated format:
          byte 0: digit count, 0..16
          bytes 1..8: packed BCD, high nibble first

        Example:
          "1"  -> 01 10 00 00 00 00 00 00 00
          "10" -> 02 10 00 00 00 00 00 00 00
          "16" -> 02 16 00 00 00 00 00 00 00

        Numeric digits 0-9 are fully validated. Non-numeric DTMF symbols
        remain intentionally unsupported until captured from CPS.
        """
        d = self._m()
        off = DTMF_NUMBER_BASE + index0 * DTMF_NUMBER_SIZE
        count = min(_byte(d, off), 16)
        if not count:
            return ""

        digits = []
        for b in bytes(d[off + 1:off + 9]):
            digits.append((b >> 4) & 0x0F)
            digits.append(b & 0x0F)

        decode_map = {
            0x0: "0", 0x1: "1", 0x2: "2", 0x3: "3", 0x4: "4",
            0x5: "5", 0x6: "6", 0x7: "7", 0x8: "8", 0x9: "9",
            0xA: "A", 0xB: "B", 0xC: "C", 0xD: "D",
            0xE: "*",
            0xF: "#",
        }
        return "".join(decode_map.get(nibble, "?")
                       for nibble in digits[:count])


    def _encode_dtmf_number(self, value):
        """Encode a validated numeric GD-73 DTMF code into its 9-byte record."""
        value = str(value).strip().upper()
        if len(value) > 16:
            raise errors.RadioError(
                "GD-73 DTMF codes are limited to 16 characters")

        encode_map = {
            "0": 0x0, "1": 0x1, "2": 0x2, "3": 0x3, "4": 0x4,
            "5": 0x5, "6": 0x6, "7": 0x7, "8": 0x8, "9": 0x9,
            "A": 0xA, "B": 0xB, "C": 0xC, "D": 0xD,
            "*": 0xE, "#": 0xF,
        }

        invalid = [ch for ch in value if ch not in encode_map]
        if invalid:
            raise errors.RadioError(
                "Unsupported GD-73 DTMF character(s): %s. "
                "Allowed: 0-9, A-D, *, #." %
                ", ".join(sorted(set(invalid))))

        packed = bytearray(8)
        for i, ch in enumerate(value):
            nibble = encode_map[ch]
            byte_index = i // 2
            if i % 2 == 0:
                packed[byte_index] |= nibble << 4
            else:
                packed[byte_index] |= nibble
        return bytes([len(value)]) + bytes(packed)

    def _ptt_template(self, index0):
        d = self._m()
        off = DTMF_PTT_BASE + index0 * DTMF_PTT_SIZE
        return {
            "system": _byte(d, off + 0),
            "type": _byte(d, off + 1),
            "mode": _byte(d, off + 2),
            "connect": _byte(d, off + 3),
            "disconnect": _byte(d, off + 4),
        }

    def _emergency_count(self):
        return min(_byte(self._m(), EMERGENCY_BASE), 1)

    def _emergency(self, index0=0):
        if index0 >= self._emergency_count():
            return None
        d = self._m()
        off = EMERGENCY_RECORDS + index0 * EMERGENCY_SIZE
        return {
            "name": _decode_utf16(d[off:off + 0x10]),
            # These offsets are validated against the supplied CPS screen:
            # Sys 1 / Silent / Emergency / None / 3.
            "type": _byte(d, off + 0x10),
            "mode": _byte(d, off + 0x11),
            "revert": _byte(d, off + 0x12),
            "retries": _byte(d, off + 0x14),
        }

    def get_settings(self):
        d = self._m()

        base = RadioSettingGroup("base_info", "Base Information")
        general = RadioSettingGroup("general", "General Settings")
        dmr = RadioSettingGroup("dmr_services", "DMR Services")
        contacts = RadioSettingGroup("digital_contacts", "Digital Contact")

        alert = RadioSettingGroup("alert_tone", "Alert Tone")
        buttons = RadioSettingGroup("buttons", "Buttons")
        quick = RadioSettingGroup("quick_text", "Quick Text")
        encrypt = RadioSettingGroup("encrypt", "Encrypt")
        emergency = RadioSettingGroup(
            "digital_emergency", "Digital Emergency System")
        rxgroups = RadioSettingGroup(
            "digital_rx_groups", "Digital RX Group List")
        zones = RadioSettingGroup("zones", "Zone")
        scans = RadioSettingGroup("scan_lists", "Scan List")
        channels = RadioSettingGroup(
            "channel_information", "Channel Information")
        dtmf = RadioSettingGroup("dtmf_signaling", "DTMF Signaling")

        # --------------------------------------------------------------
        # Base Information
        # --------------------------------------------------------------
        base.append(_ro_string(
            "serial", "Serial",
            _ascii_z(d[INFO_BASE + 0x11:INFO_BASE + 0x21])))
        base.append(_ro_string(
            "model", "Model",
            _ascii_z(d[INFO_BASE + 0x21:INFO_BASE + 0x31])))
        base.append(_ro_string(
            "device_id", "Device ID",
            _ascii_z(d[INFO_BASE + 0x31:INFO_BASE + 0x41])))
        base.append(_ro_string(
            "model_number", "Model Number",
            _ascii_z(d[INFO_BASE + 0x41:INFO_BASE + 0x51])))
        base.append(_ro_string(
            "software_version", "Software Version",
            _ascii_z(d[INFO_BASE + 0x51:INFO_BASE + 0x61])))

        # --------------------------------------------------------------
        # General Settings - mapped from QDMR and checked against CPS.
        # --------------------------------------------------------------
        s = SETTINGS_BASE

        general.append(RadioSetting(
            "radio_name", "Radio Name",
            RadioSettingValueString(
                0, 16,
                _decode_utf16(d[s + S_NAME:s + S_NAME + 0x20]),
                autopad=False)))

        general.append(RadioSetting(
            "dmr_id", "Radio ID",
            RadioSettingValueInteger(
                0, 0xFFFFFF, _u32(d, s + S_DMRID))))

        general.append(RadioSetting(
            "vox_level", "VOX Level",
            RadioSettingValueInteger(
                0, 10, _byte(d, s + S_VOX) + 1)))

        general.append(RadioSetting(
            "squelch_level", "Squelch Level",
            RadioSettingValueInteger(
                0, 10, _byte(d, s + S_SQUELCH))))

        tot_raw = _byte(d, s + S_TOT)
        tot_seconds = 0 if tot_raw == 0 else (tot_raw * 10 + 20)
        general.append(RadioSetting(
            "tot_seconds", "TX Time-out Time (s)",
            RadioSettingValueInteger(0, 600, tot_seconds)))

        general.append(RadioSetting(
            "channel_display", "Display",
            RadioSettingValueList(
                CHANNEL_DISPLAY_MODES,
                current_index=min(
                    _byte(d, s + S_CHANNEL_DISPLAY),
                    len(CHANNEL_DISPLAY_MODES) - 1))))

        general.append(RadioSetting(
            "tx_interrupt", "TX Interrupt",
            RadioSettingValueBoolean(
                bool(_byte(d, s + S_TX_INTERRUPT)))))

        general.append(RadioSetting(
            "power_save", "Power Save",
            RadioSettingValueBoolean(
                bool(_byte(d, s + S_POWER_SAVE)))))

        general.append(RadioSetting(
            "power_save_timeout", "Save Start Time-out (s)",
            RadioSettingValueInteger(
                0, 255, _byte(d, s + S_POWER_SAVE_TIMEOUT))))

        general.append(RadioSetting(
            "lone_worker_response", "Lone Worker Response Time (min)",
            RadioSettingValueInteger(
                0, 65535, _u16(d, s + S_LONE_RESPONSE))))

        # The CPS image has this as a one-byte seconds field at 0x42;
        # 0x43 is independently the boot-display mode.
        general.append(RadioSetting(
            "lone_worker_reminder", "Lone Worker Reminder Time (s)",
            RadioSettingValueInteger(
                0, 255, _byte(d, s + S_LONE_REMINDER))))

        general.append(RadioSetting(
            "dmr_mic_gain", "Digital Mic Gain",
            RadioSettingValueInteger(
                1, 10, _byte(d, s + S_DMR_MIC_GAIN))))

        general.append(RadioSetting(
            "fm_mic_gain", "Analog Mic Gain",
            RadioSettingValueInteger(
                1, 10, _byte(d, s + S_FM_MIC_GAIN))))

        general.append(RadioSetting(
            "write_lock", "Write Lock",
            RadioSettingValueBoolean(
                bool(_byte(d, s + S_WRITE_LOCK)))))
        general.append(RadioSetting(
            "write_pin", "Write Password",
            RadioSettingValueString(
                0, 6, _read_ascii_field(d, s + S_WRITE_PIN, 6),
                autopad=False)))

        general.append(RadioSetting(
            "read_lock", "Read Lock",
            RadioSettingValueBoolean(
                bool(_byte(d, s + S_READ_LOCK)))))
        general.append(RadioSetting(
            "read_pin", "Read Password",
            RadioSettingValueString(
                0, 6, _read_ascii_field(d, s + S_READ_PIN, 6),
                autopad=False)))

        general.append(RadioSetting(
            "boot_mode", "Power-on Display",
            RadioSettingValueList(
                BOOT_DISPLAY_MODES,
                current_index=min(
                    _byte(d, s + S_BOOT_MODE),
                    len(BOOT_DISPLAY_MODES) - 1))))

        general.append(RadioSetting(
            "boot_text1", "Power-on Text 1",
            RadioSettingValueString(
                0, 16,
                _decode_utf16(d[s + S_BOOT_TEXT1:s + S_BOOT_TEXT1 + 0x20]),
                autopad=False)))
        general.append(RadioSetting(
            "boot_text2", "Power-on Text 2",
            RadioSettingValueString(
                0, 16,
                _decode_utf16(d[s + S_BOOT_TEXT2:s + S_BOOT_TEXT2 + 0x20]),
                autopad=False)))

        # --------------------------------------------------------------
        # DMR Services
        # --------------------------------------------------------------
        ds = DMRSETTINGS_BASE
        dmr.append(RadioSetting(
            "dmr_call_hang", "Call Hang Time (s)",
            RadioSettingValueInteger(
                1, 90, _byte(d, ds + D_CALL_HANG) + 1)))

        active_wait = 120 + _byte(d, ds + D_ACTIVE_WAIT) * 5
        dmr.append(RadioSetting(
            "dmr_active_wait", "Active Wait Time (ms)",
            RadioSettingValueInteger(120, 600, active_wait)))

        dmr.append(RadioSetting(
            "dmr_active_retries", "Active Retries",
            RadioSettingValueInteger(
                0, 255, _byte(d, ds + D_ACTIVE_RETRIES))))

        dmr.append(RadioSetting(
            "dmr_tx_preambles", "TX Preambles",
            RadioSettingValueInteger(
                0, 255, _byte(d, ds + D_TX_PREAMBLES))))

        dmr.append(RadioSetting(
            "dmr_decode_disable", "Radio Disable Decode",
            RadioSettingValueBoolean(
                bool(_byte(d, ds + D_DEC_DISABLE)))))
        dmr.append(RadioSetting(
            "dmr_decode_check", "Radio Check Decode",
            RadioSettingValueBoolean(
                bool(_byte(d, ds + D_DEC_CHECK)))))
        dmr.append(RadioSetting(
            "dmr_decode_enable", "Radio Enable Decode",
            RadioSettingValueBoolean(
                bool(_byte(d, ds + D_DEC_ENABLE)))))

        # --------------------------------------------------------------
        # Remaining mapped sections - preserve now, deeper editors next.
        # --------------------------------------------------------------
        alert.append(_section_status(
            "alert_status",
            "Mapped in settings block; detailed tone controls pending."))

        # --------------------------------------------------------------
        # Buttons
        # QDMR offsets: long press 0x88, P1 short/long 0x8B/0x8C,
        # P2 short/long 0x8D/0x8E.
        # User CPS validates raw 04/09/05/08 as
        # Power Level / Scan On-Off / Monitor / Zone Switch.
        # --------------------------------------------------------------
        long_raw = _byte(d, SETTINGS_BASE + 0x88)
        long_opts = ["%.1f" % (x * 0.5) for x in range(1, 21)]
        long_idx = max(0, min(len(long_opts) - 1, long_raw - 1))
        buttons.append(RadioSetting(
            "button_long_press", "Long Press Duration (s)",
            RadioSettingValueList(long_opts, current_index=long_idx)))

        for key, label, off in (
                ("p1_short", "Front Button 1 Short Press", 0x8B),
                ("p1_long", "Front Button 1 Long Press", 0x8C),
                ("p2_short", "Front Button 2 Short Press", 0x8D),
                ("p2_long", "Front Button 2 Long Press", 0x8E)):
            rawv = _byte(d, SETTINGS_BASE + off)
            if rawv >= len(BUTTON_FUNCTIONS):
                rawv = 0
            buttons.append(RadioSetting(
                "button_" + key, label,
                RadioSettingValueList(
                    BUTTON_FUNCTIONS, current_index=rawv)))

        # One Touch Access. The current image has five all-zero records,
        # matching CPS: Digital / None / Call / None.
        one_touch = RadioSettingGroup("one_touch", "One Touch Access")
        for i in range(5):
            off = SETTINGS_BASE + 0x90 + i * 5
            action = _byte(d, off + 3)
            og = RadioSettingGroup(
                "onetouch_%d" % i, "One Touch %d" % (i + 1))
            og.append(_ro_string(
                "onetouch_%d_mode" % i, "Call Mode", "Digital"))
            og.append(_ro_string(
                "onetouch_%d_action" % i, "Call Type",
                "Message" if action == 1 else "Call"))
            contact_raw = _u16(d, off + 1)
            contact_text = "None"
            if contact_raw and contact_raw <= self._contact_count():
                c = self._contact(contact_raw - 1)
                contact_text = "%d: %s" % (contact_raw, c["name"])
            og.append(_ro_string(
                "onetouch_%d_contact" % i, "Call List", contact_text))
            one_touch.append(og)
        buttons.append(one_touch)

        encrypt.append(_section_status(
            "encrypt_status",
            "16-key encryption bank mapped at 0x2191F."))

        # --------------------------------------------------------------
        # Digital Emergency System
        # --------------------------------------------------------------
        emergency.append(_ro_string(
            "emergency_count", "System Count",
            str(self._emergency_count())))
        em = self._emergency()
        if em:
            eg = RadioSettingGroup("emergency_0", "1: " + (em["name"] or "(blank)"))
            eg.append(RadioSetting(
                "emergency_name", "System Name",
                RadioSettingValueString(
                    0, 8, em["name"], autopad=False)))
            eg.append(RadioSetting(
                "emergency_type", "Emergency Type",
                RadioSettingValueList(
                    ["Regular", "Silent"],
                    current_index=min(em["type"], 1))))
            eg.append(RadioSetting(
                "emergency_mode", "Emergency Mode",
                RadioSettingValueList(
                    ["Emergency", "Emergency + Call"],
                    current_index=min(em["mode"], 1))))
            chchoices = self._channel_choices()
            rev = em["revert"]
            if rev >= len(chchoices):
                rev = 0
            eg.append(RadioSetting(
                "emergency_revert", "Revert Channel",
                RadioSettingValueList(
                    chchoices, current_index=rev)))
            eg.append(RadioSetting(
                "emergency_retries", "Impolite Retries",
                RadioSettingValueInteger(
                    0, 255, em["retries"])))
            emergency.append(eg)
        channels.append(_ro_string(
            "channel_count", "Channel Count", str(self._channel_count())))
        channels.append(_section_status(
            "channels_status",
            "Main CHIRP memory editor is active for channel records."))

        dtmf_system = RadioSettingGroup("dtmf_system", "DTMF System")
        dtmf_code = RadioSettingGroup("dtmf_code", "DTMF Code")
        ptt_template = RadioSettingGroup("ptt_template", "PTT Template")

        # Four 5-byte DTMF timing systems.
        for i in range(4):
            sysv = self._dtmf_system(i)
            dg = RadioSettingGroup(
                "dtmf_system_%d" % i, "DTMF %d" % (i + 1))
            dg.append(RadioSetting(
                "dtmf_%d_sidetone" % i, "Side Tone",
                RadioSettingValueBoolean(sysv["sidetone"])))
            dg.append(RadioSetting(
                "dtmf_%d_pre" % i, "Pre Time (ms)",
                RadioSettingValueInteger(0, 1000, sysv["pre_ms"])))
            dg.append(RadioSetting(
                "dtmf_%d_tone" % i, "Tone Duration (ms)",
                RadioSettingValueInteger(30, 1900, sysv["tone_ms"])))
            dg.append(RadioSetting(
                "dtmf_%d_interval" % i, "Tone Interval (ms)",
                RadioSettingValueInteger(30, 1900, sysv["interval_ms"])))
            dg.append(RadioSetting(
                "dtmf_%d_restore" % i, "Restoration Time (ms)",
                RadioSettingValueInteger(200, 33000, sysv["restore_ms"])))
            dtmf_system.append(dg)

        dtmf.append(dtmf_system)

        return RadioSettings(
            base,
            general,
            alert,
            buttons,
            encrypt,
            emergency,
            dmr,
            dtmf,
        )

    def get_sub_devices(self):
        """Expose GD-73 codeplug collections using CHIRP's normal subdevice API.

        This is the same mechanism used by radios such as the Icom IC-W32A:
        every subdevice shares the exact same MemoryMapBytes object.
        """
        return [
            GD73ChannelsRadio(self._mmap),
            GD73ContactsRadio(self._mmap),
            GD73RXGroupsRadio(self._mmap),
            GD73ZonesRadio(self._mmap),
            GD73ScanListsRadio(self._mmap),
            GD73DTMFCodesRadio(self._mmap),
            GD73PTTTemplatesRadio(self._mmap),
            GD73QuickTextRadio(self._mmap),
        ]

    def set_settings(self, settings):
        """Apply only mapped fields; all unrelated bytes remain untouched."""
        d = self._m()
        s = SETTINGS_BASE
        ds = DMRSETTINGS_BASE

        for setting in settings.walk():
            if not setting.changed():
                continue

            name = setting.get_name()
            value = setting.value

            # Active bank counts
            if name == "contact_count":
                old_count = self._contact_count()
                new_count = int(value)
                if new_count > old_count:
                    for i in range(old_count, new_count):
                        _set_bytes(
                            d, CONTACT_RECORDS + i * CONTACT_SIZE,
                            b"\x00" * CONTACT_SIZE)
                _set_bytes(d, CONTACT_BANK, _p16(new_count))

            elif name == "rxgroup_count":
                old_count = self._rxgroup_count()
                new_count = int(value)
                if new_count > old_count:
                    for i in range(old_count, new_count):
                        _set_bytes(
                            d, RXGROUP_RECORDS + i * RXGROUP_SIZE,
                            b"\x00" * RXGROUP_SIZE)
                _set_bytes(d, RXGROUP_BANK, bytes([new_count]))

            elif name == "zone_count":
                old_count = self._zone_count()
                new_count = int(value)
                if new_count > old_count:
                    for i in range(old_count, new_count):
                        _set_bytes(
                            d, ZONE_RECORDS + i * ZONE_SIZE,
                            b"\x00" * ZONE_SIZE)
                _set_bytes(d, ZONE_BANK, bytes([new_count]))

            elif name == "scanlist_count":
                old_count = self._scanlist_count()
                new_count = int(value)
                if new_count > old_count:
                    for i in range(old_count, new_count):
                        _set_bytes(
                            d, SCANLIST_RECORDS + i * SCANLIST_SIZE,
                            b"\x00" * SCANLIST_SIZE)
                _set_bytes(d, SCANLIST_BANK, bytes([new_count]))

            elif name == "quick_count":
                old_count = self._message_count()
                new_count = int(value)
                if new_count > old_count:
                    for i in range(old_count, new_count):
                        _set_bytes(
                            d, MESSAGE_BASE + 1 + i * 0x51,
                            b"\x00" * 0x51)
                _set_bytes(d, MESSAGE_BASE, bytes([new_count]))

            # General settings
            elif name == "radio_name":
                _set_bytes(
                    d, s + S_NAME,
                    _encode_utf16(str(value).rstrip(), 16))
            elif name == "dmr_id":
                _set_bytes(d, s + S_DMRID, _p32(int(value)))
            elif name == "vox_level":
                _set_bytes(d, s + S_VOX, bytes([max(0, int(value) - 1)]))
            elif name == "squelch_level":
                _set_bytes(d, s + S_SQUELCH, bytes([int(value)]))
            elif name == "tot_seconds":
                seconds = int(value)
                raw = 0 if seconds <= 0 else max(0, (seconds - 20) // 10)
                _set_bytes(d, s + S_TOT, bytes([min(255, raw)]))
            elif name == "channel_display":
                _set_bytes(d, s + S_CHANNEL_DISPLAY, bytes([int(value)]))
            elif name == "tx_interrupt":
                _set_bytes(d, s + S_TX_INTERRUPT, bytes([1 if bool(value) else 0]))
            elif name == "power_save":
                _set_bytes(d, s + S_POWER_SAVE, bytes([1 if bool(value) else 0]))
            elif name == "power_save_timeout":
                _set_bytes(d, s + S_POWER_SAVE_TIMEOUT, bytes([int(value)]))
            elif name == "lone_worker_response":
                _set_bytes(d, s + S_LONE_RESPONSE, _p16(int(value)))
            elif name == "lone_worker_reminder":
                _set_bytes(d, s + S_LONE_REMINDER, bytes([int(value)]))
            elif name == "dmr_mic_gain":
                _set_bytes(d, s + S_DMR_MIC_GAIN, bytes([int(value)]))
            elif name == "fm_mic_gain":
                _set_bytes(d, s + S_FM_MIC_GAIN, bytes([int(value)]))
            elif name == "write_lock":
                _set_bytes(d, s + S_WRITE_LOCK, bytes([1 if bool(value) else 0]))
            elif name == "write_pin":
                _write_ascii_field(d, s + S_WRITE_PIN, 6, value)
            elif name == "read_lock":
                _set_bytes(d, s + S_READ_LOCK, bytes([1 if bool(value) else 0]))
            elif name == "read_pin":
                _write_ascii_field(d, s + S_READ_PIN, 6, value)
            elif name == "boot_mode":
                _set_bytes(d, s + S_BOOT_MODE, bytes([int(value)]))
            elif name == "boot_text1":
                _set_bytes(
                    d, s + S_BOOT_TEXT1,
                    _encode_utf16(str(value).rstrip(), 16))
            elif name == "boot_text2":
                _set_bytes(
                    d, s + S_BOOT_TEXT2,
                    _encode_utf16(str(value).rstrip(), 16))

            # DMR service settings
            elif name == "dmr_call_hang":
                _set_bytes(d, ds + D_CALL_HANG, bytes([max(0, int(value) - 1)]))
            elif name == "dmr_active_wait":
                raw = max(0, min(96, (int(value) - 120) // 5))
                _set_bytes(d, ds + D_ACTIVE_WAIT, bytes([raw]))
            elif name == "dmr_active_retries":
                _set_bytes(d, ds + D_ACTIVE_RETRIES, bytes([int(value)]))
            elif name == "dmr_tx_preambles":
                _set_bytes(d, ds + D_TX_PREAMBLES, bytes([int(value)]))
            elif name == "dmr_decode_disable":
                _set_bytes(d, ds + D_DEC_DISABLE, bytes([1 if bool(value) else 0]))
            elif name == "dmr_decode_check":
                _set_bytes(d, ds + D_DEC_CHECK, bytes([1 if bool(value) else 0]))
            elif name == "dmr_decode_enable":
                _set_bytes(d, ds + D_DEC_ENABLE, bytes([1 if bool(value) else 0]))

            # Contact table
            elif name.startswith("contact_"):
                parts = name.split("_")
                if len(parts) >= 3 and parts[1].isdigit():
                    idx = int(parts[1])
                    if idx < self._contact_count():
                        off = CONTACT_RECORDS + idx * CONTACT_SIZE
                        field = "_".join(parts[2:])
                        if field == "name":
                            _set_bytes(
                                d, off,
                                _encode_utf16(str(value).rstrip(), 16))
                        elif field == "type":
                            _set_bytes(d, off + 0x20, bytes([int(value)]))
                        elif field == "id":
                            _set_bytes(d, off + 0x21, _p32(int(value)))

            # RX Group Lists
            elif name.startswith("rxgroup_"):
                parts = name.split("_")
                if len(parts) >= 3 and parts[1].isdigit():
                    idx = int(parts[1])
                    if idx < self._rxgroup_count():
                        off = RXGROUP_RECORDS + idx * RXGROUP_SIZE
                        field = "_".join(parts[2:])
                        if field == "name":
                            _set_bytes(
                                d, off,
                                _encode_utf16(str(value).rstrip(), 8))
                        elif field == "count":
                            _set_bytes(d, off + 0x10, bytes([int(value)]))
                        elif field.startswith("member_"):
                            member = int(field.split("_")[1])
                            if member < 33:
                                _set_bytes(
                                    d, off + 0x11 + member * 2,
                                    _p16(int(value)))

            # Zones
            elif name.startswith("zone_"):
                parts = name.split("_")
                if len(parts) >= 3 and parts[1].isdigit():
                    idx = int(parts[1])
                    if idx < self._zone_count():
                        off = ZONE_RECORDS + idx * ZONE_SIZE
                        field = "_".join(parts[2:])
                        if field == "name":
                            _set_bytes(
                                d, off,
                                _encode_utf16(str(value).rstrip(), 8))
                        elif field == "count":
                            _set_bytes(d, off + 0x10, bytes([int(value)]))
                        elif field.startswith("member_"):
                            member = int(field.split("_")[1])
                            if member < 16:
                                _set_bytes(
                                    d, off + 0x11 + member * 2,
                                    _p16(int(value)))

            # Scan Lists
            elif name.startswith("scanlist_"):
                parts = name.split("_")
                if len(parts) >= 3 and parts[1].isdigit():
                    idx = int(parts[1])
                    if idx < self._scanlist_count():
                        off = SCANLIST_RECORDS + idx * SCANLIST_SIZE
                        field = "_".join(parts[2:])
                        if field == "name":
                            _set_bytes(
                                d, off,
                                _encode_utf16(str(value).rstrip(), 8))
                        elif field == "count":
                            _set_bytes(d, off + 0x10, bytes([int(value)]))
                        elif field == "pri1_mode":
                            _set_bytes(d, off + 0x51, bytes([int(value)]))
                        elif field == "pri2_mode":
                            _set_bytes(d, off + 0x52, bytes([int(value)]))
                        elif field == "pri1_zone":
                            _set_bytes(d, off + 0x53, bytes([int(value)]))
                        elif field == "pri2_zone":
                            _set_bytes(d, off + 0x54, bytes([int(value)]))
                        elif field == "pri1_channel":
                            _set_bytes(d, off + 0x55, bytes([int(value)]))
                        elif field == "pri2_channel":
                            _set_bytes(d, off + 0x57, bytes([int(value)]))
                        elif field == "revert_mode":
                            _set_bytes(d, off + 0x59, bytes([int(value)]))
                        elif field == "revert_zone":
                            _set_bytes(d, off + 0x5A, bytes([int(value)]))
                        elif field == "revert_channel":
                            _set_bytes(d, off + 0x5B, bytes([int(value)]))
                        elif field == "rx_hold":
                            raw = max(0, min(20, int(value) // 500))
                            _set_bytes(d, off + 0x5D, bytes([raw]))
                        elif field == "tx_hold":
                            raw = max(0, min(20, int(value) // 500))
                            _set_bytes(d, off + 0x5E, bytes([raw]))
                        elif field.startswith("member_"):
                            member = int(field.split("_")[1])
                            if member < 32:
                                _set_bytes(
                                    d, off + 0x11 + member * 2,
                                    _p16(int(value)))
            # Buttons
            elif name == "button_long_press":
                # List index 0 is 0.5 s, raw value 1.
                _set_bytes(
                    d, SETTINGS_BASE + 0x88,
                    bytes([int(value) + 1]))
            elif name.startswith("button_"):
                button_offsets = {
                    "button_p1_short": 0x8B,
                    "button_p1_long": 0x8C,
                    "button_p2_short": 0x8D,
                    "button_p2_long": 0x8E,
                }
                if name in button_offsets:
                    off = SETTINGS_BASE + button_offsets[name]
                    _set_bytes(d, off, bytes([int(value)]))

            # Quick Text
            elif name.startswith("quick_") and name[6:].isdigit():
                idx = int(name[6:])
                if 0 <= idx < 16:
                    off = MESSAGE_BASE + 1 + idx * 0x51
                    msg = str(value).rstrip()[:40]
                    enc = msg.encode("utf-16le")
                    _set_bytes(d, off, bytes([len(msg)]))
                    _set_bytes(d, off + 1, enc.ljust(0x50, b"\x00"))

            # Emergency system - fields validated by CPS screenshot.
            elif name.startswith("emergency_") and self._emergency_count():
                off = EMERGENCY_RECORDS
                if name == "emergency_name":
                    _set_bytes(
                        d, off,
                        _encode_utf16(str(value).rstrip(), 8))
                elif name == "emergency_type":
                    _set_bytes(d, off + 0x10, bytes([int(value)]))
                elif name == "emergency_mode":
                    _set_bytes(d, off + 0x11, bytes([int(value)]))
                elif name == "emergency_revert":
                    _set_bytes(d, off + 0x12, bytes([int(value)]))
                elif name == "emergency_retries":
                    _set_bytes(d, off + 0x14, bytes([int(value)]))

            # DTMF Codes: exact 9-byte CPS-validated numeric BCD records.
            elif name.startswith("dtmf_code_") and name[10:].isdigit():
                idx = int(name[10:])
                if 0 <= idx < 16:
                    off = DTMF_NUMBER_BASE + idx * DTMF_NUMBER_SIZE
                    _set_bytes(
                        d, off, self._encode_dtmf_number(value))

            # DTMF systems
            elif name.startswith("dtmf_") and "_sidetone" in name:
                idx = int(name.split("_")[1])
                off = DTMF_SYSTEM_BASE + idx * DTMF_SYSTEM_SIZE
                _set_bytes(d, off, bytes([1 if bool(value) else 0]))
            elif name.startswith("dtmf_") and "_pre" in name:
                idx = int(name.split("_")[1])
                off = DTMF_SYSTEM_BASE + idx * DTMF_SYSTEM_SIZE
                _set_bytes(
                    d, off + 1,
                    bytes([max(0, min(100, int(value)//10))]))
            elif name.startswith("dtmf_") and "_tone" in name:
                idx = int(name.split("_")[1])
                off = DTMF_SYSTEM_BASE + idx * DTMF_SYSTEM_SIZE
                raw = max(0, min(187, (int(value)-30)//10))
                _set_bytes(d, off + 2, bytes([raw]))
            elif name.startswith("dtmf_") and "_interval" in name:
                idx = int(name.split("_")[1])
                off = DTMF_SYSTEM_BASE + idx * DTMF_SYSTEM_SIZE
                raw = max(0, min(187, (int(value)-30)//10))
                _set_bytes(d, off + 3, bytes([raw]))
            elif name.startswith("dtmf_") and "_restore" in name:
                idx = int(name.split("_")[1])
                off = DTMF_SYSTEM_BASE + idx * DTMF_SYSTEM_SIZE
                raw = max(0, min(164, (int(value)-200)//200))
                _set_bytes(d, off + 4, bytes([raw]))

            # PTT templates
            elif name.startswith("ptt_"):
                parts = name.split("_")
                if len(parts) >= 3 and parts[1].isdigit():
                    idx = int(parts[1])
                    if idx < 32:
                        off = DTMF_PTT_BASE + idx * DTMF_PTT_SIZE
                        field = "_".join(parts[2:])
                        field_offsets = {
                            "system": 0, "type": 1, "mode": 2,
                            "connect": 3, "disconnect": 4,
                        }
                        if field in field_offsets:
                            o = off + field_offsets[field]
                            _set_bytes(d, o, bytes([int(value)]))

            else:
                LOG.debug("Ignoring non-writable GD-73 setting %s", name)



# ============================================================================
# Standard CHIRP subdevices
#
# CHIRP's regular editor creates one Memories editor per subdevice returned by
# get_sub_devices(). This requires no wx/UI modification and therefore remains
# a normal drop-in CHIRP driver.
# ============================================================================

_COLLECTION_IMMUTABLE = [
    "freq", "duplex", "offset", "tmode", "rtone", "ctone",
    "dtcs", "rx_dtcs", "dtcs_polarity", "cross_mode",
    "mode", "tuning_step", "skip", "power",
]


def _extra_index(mem, name, default=0):
    try:
        return int(mem.extra[name].value)
    except (AttributeError, KeyError, TypeError, ValueError):
        return default


def _extra_string(mem, name, default=""):
    try:
        return str(mem.extra[name].value).strip()
    except (AttributeError, KeyError):
        return default


class GD73CollectionRadio(GD73ARadio):
    """Base for non-RF GD-73 tables shown in CHIRP's normal Memories editor."""

    MAX_RECORDS = 1
    NAME_LENGTH = 16
    VARIANT = "Collection"

    def get_features(self):
        rf = chirp_common.RadioFeatures()
        rf.memory_bounds = (1, self.MAX_RECORDS)
        rf.has_sub_devices = False
        rf.has_settings = False
        rf.has_bank = False
        rf.has_bank_names = False
        rf.has_name = True
        rf.has_comment = False
        rf.has_mode = False
        rf.has_offset = False
        rf.has_tuning_step = False
        rf.has_ctone = False
        rf.has_dtcs = False
        rf.has_rx_dtcs = False
        rf.has_dtcs_polarity = False
        rf.has_cross = False
        # Keep a valid internal RF profile so CHIRP's generic Memory object
        # remains happy, but selectively empty only the feature lists whose
        # columns we want hidden in these non-RF collection editors.
        rf.valid_modes = ["FM"]              # hidden by has_mode=False
        rf.valid_tmodes = []                 # hide Tone Mode
        rf.valid_tones = []                  # hide Tone / Tone Squelch
        rf.valid_dtcs_codes = []             # DTCS already hidden by flags
        rf.valid_cross_modes = []
        rf.valid_duplexes = []               # hide Duplex
        rf.valid_tuning_steps = [5.0]        # hidden by has_tuning_step=False
        rf.valid_bands = [(400000000, 470000000)]  # valid UHF placeholder band
        rf.valid_skips = []                  # hide Skip
        rf.valid_power_levels = []           # hide Power
        rf.valid_name_length = self.NAME_LENGTH
        rf.valid_characters = chirp_common.CHARSET_ASCII
        rf.can_delete = True
        return rf

    def _base_memory(self, number, name="", empty=False):
        mem = chirp_common.Memory()
        mem.number = number
        mem.empty = empty
        # Internal-only placeholder; frequency is immutable and the
        # collection feature profile hides the RF controls.
        mem.freq = 400000000
        mem.name = name or ""
        mem.duplex = ""
        mem.mode = "FM"
        mem.tmode = ""
        mem.skip = ""
        mem.extra = RadioSettingGroup("extra", "Extra")
        mem.immutable = list(_COLLECTION_IMMUTABLE)
        return mem

    def _zero_record(self, offset, size):
        _set_bytes(self._m(), offset, b"\x00" * size)


class GD73ChannelsRadio(GD73ARadio):
    """The actual RF channel table."""

    VARIANT = "Channels"

    def get_features(self):
        rf = super().get_features()
        rf.has_sub_devices = False
        rf.has_settings = False
        return rf


class GD73ContactsRadio(GD73CollectionRadio):
    """Digital Contact table as a normal CHIRP subdevice."""

    VARIANT = "Digital Contacts"
    MAX_RECORDS = 1024
    NAME_LENGTH = 16

    def get_memory(self, number):
        if number < 1 or number > self.MAX_RECORDS:
            raise errors.InvalidMemoryLocation(number)

        count = self._contact_count()
        if number > count:
            return self._base_memory(number, empty=True)

        d = self._m()
        off = CONTACT_RECORDS + (number - 1) * CONTACT_SIZE
        raw = bytes(d[off:off + CONTACT_SIZE])
        if raw == b"\x00" * CONTACT_SIZE:
            return self._base_memory(number, empty=True)

        c = self._contact(number - 1)
        mem = self._base_memory(number, c["name"], empty=False)

        ctype = c["type"]
        if ctype >= len(CONTACT_TYPES):
            ctype = 0
        mem.extra.append(RadioSetting(
            "contact_type", "Call Type",
            RadioSettingValueList(CONTACT_TYPES, current_index=ctype)))
        mem.extra.append(RadioSetting(
            "dmr_id", "DMR ID",
            RadioSettingValueInteger(0, 0xFFFFFF, c["id"])))
        return mem

    def set_memory(self, mem):
        number = mem.number
        if number < 1 or number > self.MAX_RECORDS:
            raise errors.InvalidMemoryLocation(number)

        if mem.empty:
            return self.erase_memory(number)

        d = self._m()
        old_count = self._contact_count()
        if number > old_count:
            for i in range(old_count, number):
                self._zero_record(
                    CONTACT_RECORDS + i * CONTACT_SIZE, CONTACT_SIZE)
            _set_bytes(d, CONTACT_BANK, _p16(number))

        off = CONTACT_RECORDS + (number - 1) * CONTACT_SIZE
        raw = bytearray(CONTACT_SIZE)
        raw[0x00:0x20] = _encode_utf16(mem.name, 16)
        raw[0x20] = _extra_index(mem, "contact_type", 0)
        raw[0x21:0x25] = _p32(_extra_index(mem, "dmr_id", 0))
        _set_bytes(d, off, raw)

    def erase_memory(self, number):
        if number < 1 or number > self.MAX_RECORDS:
            raise errors.InvalidMemoryLocation(number)
        d = self._m()
        self._zero_record(
            CONTACT_RECORDS + (number - 1) * CONTACT_SIZE, CONTACT_SIZE)

        count = self._contact_count()
        while count > 0:
            off = CONTACT_RECORDS + (count - 1) * CONTACT_SIZE
            if bytes(d[off:off + CONTACT_SIZE]) != b"\x00" * CONTACT_SIZE:
                break
            count -= 1
        _set_bytes(d, CONTACT_BANK, _p16(count))


class GD73ZonesRadio(GD73CollectionRadio):
    """Zone table with channel membership as editable extra columns."""

    VARIANT = "Zones"
    MAX_RECORDS = 64
    NAME_LENGTH = 8

    def get_memory(self, number):
        if number < 1 or number > self.MAX_RECORDS:
            raise errors.InvalidMemoryLocation(number)
        if number > self._zone_count():
            return self._base_memory(number, empty=True)

        d = self._m()
        off = ZONE_RECORDS + (number - 1) * ZONE_SIZE
        raw = bytes(d[off:off + ZONE_SIZE])
        if raw == b"\x00" * ZONE_SIZE:
            return self._base_memory(number, empty=True)

        z = self._zone(number - 1)
        mem = self._base_memory(number, z["name"], empty=False)
        choices = self._channel_choices()
        for i, member in enumerate(z["members"]):
            current = 0 if member is None else member + 1
            if current >= len(choices):
                current = 0
            mem.extra.append(RadioSetting(
                "member_%02d" % (i + 1), "Channel %d" % (i + 1),
                RadioSettingValueList(choices, current_index=current)))
        return mem

    def set_memory(self, mem):
        number = mem.number
        if mem.empty:
            return self.erase_memory(number)

        d = self._m()
        old_count = self._zone_count()
        if number > old_count:
            for i in range(old_count, number):
                self._zero_record(ZONE_RECORDS + i * ZONE_SIZE, ZONE_SIZE)
            _set_bytes(d, ZONE_BANK, bytes([number]))

        off = ZONE_RECORDS + (number - 1) * ZONE_SIZE
        raw = bytearray(ZONE_SIZE)
        raw[0x00:0x10] = _encode_utf16(mem.name, 8)

        members = []
        for i in range(16):
            members.append(_extra_index(mem, "member_%02d" % (i + 1), 0))

        active = 0
        for i, value in enumerate(members):
            if value:
                active = i + 1
            raw[0x11 + i * 2:0x13 + i * 2] = _p16(value)
        raw[0x10] = active
        _set_bytes(d, off, raw)

    def erase_memory(self, number):
        d = self._m()
        self._zero_record(ZONE_RECORDS + (number - 1) * ZONE_SIZE, ZONE_SIZE)
        count = self._zone_count()
        while count > 0:
            off = ZONE_RECORDS + (count - 1) * ZONE_SIZE
            if bytes(d[off:off + ZONE_SIZE]) != b"\x00" * ZONE_SIZE:
                break
            count -= 1
        _set_bytes(d, ZONE_BANK, bytes([count]))


class GD73RXGroupsRadio(GD73CollectionRadio):
    """Digital RX Group Lists as a normal CHIRP subdevice."""

    VARIANT = "RX Groups"
    MAX_RECORDS = 250
    NAME_LENGTH = 8

    def get_memory(self, number):
        if number < 1 or number > self.MAX_RECORDS:
            raise errors.InvalidMemoryLocation(number)
        if number > self._rxgroup_count():
            return self._base_memory(number, empty=True)

        d = self._m()
        off = RXGROUP_RECORDS + (number - 1) * RXGROUP_SIZE
        raw = bytes(d[off:off + RXGROUP_SIZE])
        if raw == b"\x00" * RXGROUP_SIZE:
            return self._base_memory(number, empty=True)

        g = self._rxgroup(number - 1)
        mem = self._base_memory(number, g["name"], empty=False)
        choices = self._contact_choices()
        for i, member in enumerate(g["members"]):
            current = 0 if member is None else member + 1
            if current >= len(choices):
                current = 0
            mem.extra.append(RadioSetting(
                "member_%02d" % (i + 1), "Contact %d" % (i + 1),
                RadioSettingValueList(choices, current_index=current)))
        return mem

    def set_memory(self, mem):
        number = mem.number
        if mem.empty:
            return self.erase_memory(number)

        d = self._m()
        old_count = self._rxgroup_count()
        if number > old_count:
            for i in range(old_count, number):
                self._zero_record(
                    RXGROUP_RECORDS + i * RXGROUP_SIZE, RXGROUP_SIZE)
            _set_bytes(d, RXGROUP_BANK, bytes([number]))

        off = RXGROUP_RECORDS + (number - 1) * RXGROUP_SIZE
        raw = bytearray(RXGROUP_SIZE)
        raw[0x00:0x10] = _encode_utf16(mem.name, 8)

        members = []
        for i in range(33):
            members.append(_extra_index(mem, "member_%02d" % (i + 1), 0))
        active = 0
        for i, value in enumerate(members):
            if value:
                active = i + 1
            raw[0x11 + i * 2:0x13 + i * 2] = _p16(value)
        raw[0x10] = active
        _set_bytes(d, off, raw)

    def erase_memory(self, number):
        d = self._m()
        self._zero_record(
            RXGROUP_RECORDS + (number - 1) * RXGROUP_SIZE, RXGROUP_SIZE)
        count = self._rxgroup_count()
        while count > 0:
            off = RXGROUP_RECORDS + (count - 1) * RXGROUP_SIZE
            if bytes(d[off:off + RXGROUP_SIZE]) != b"\x00" * RXGROUP_SIZE:
                break
            count -= 1
        _set_bytes(d, RXGROUP_BANK, bytes([count]))


class GD73ScanListsRadio(GD73CollectionRadio):
    """Scan Lists with members and priority/revert fields."""

    VARIANT = "Scan Lists"
    MAX_RECORDS = 16
    NAME_LENGTH = 8

    def get_memory(self, number):
        if number < 1 or number > self.MAX_RECORDS:
            raise errors.InvalidMemoryLocation(number)
        if number > self._scanlist_count():
            return self._base_memory(number, empty=True)

        d = self._m()
        off = SCANLIST_RECORDS + (number - 1) * SCANLIST_SIZE
        raw = bytes(d[off:off + SCANLIST_SIZE])
        if raw == b"\x00" * SCANLIST_SIZE:
            return self._base_memory(number, empty=True)

        sl = self._scanlist(number - 1)
        mem = self._base_memory(number, sl["name"], empty=False)
        modes = ["None", "Fixed", "Selected"]
        zone_choices = self._zone_choices()
        channel_choices = self._channel_choices()

        for key, label, raw_mode in (
                ("pri1_mode", "Priority 1 Mode", sl["pri1_mode"]),
                ("pri2_mode", "Priority 2 Mode", sl["pri2_mode"]),
                ("revert_mode", "Revert Mode", sl["revert_mode"])):
            mem.extra.append(RadioSetting(
                key, label,
                RadioSettingValueList(
                    modes, current_index=min(raw_mode, len(modes) - 1))))

        for key, label, value in (
                ("pri1_zone", "Priority 1 Zone", sl["pri1_zone"]),
                ("pri2_zone", "Priority 2 Zone", sl["pri2_zone"]),
                ("revert_zone", "Revert Zone", sl["revert_zone"])):
            if value >= len(zone_choices):
                value = 0
            mem.extra.append(RadioSetting(
                key, label,
                RadioSettingValueList(zone_choices, current_index=value)))

        for key, label, value in (
                ("pri1_channel", "Priority 1 Channel", sl["pri1_channel"]),
                ("pri2_channel", "Priority 2 Channel", sl["pri2_channel"]),
                ("revert_channel", "Revert Channel", sl["revert_channel"])):
            if value >= len(channel_choices):
                value = 0
            mem.extra.append(RadioSetting(
                key, label,
                RadioSettingValueList(channel_choices, current_index=value)))

        mem.extra.append(RadioSetting(
            "rx_hold", "RX Hold (ms)",
            RadioSettingValueInteger(0, 10000, sl["rx_hold_raw"] * 500)))
        mem.extra.append(RadioSetting(
            "tx_hold", "TX Hold (ms)",
            RadioSettingValueInteger(0, 10000, sl["tx_hold_raw"] * 500)))

        for i, member in enumerate(sl["members"]):
            current = 0 if member is None else member + 1
            if current >= len(channel_choices):
                current = 0
            mem.extra.append(RadioSetting(
                "member_%02d" % (i + 1), "Member %d" % (i + 1),
                RadioSettingValueList(
                    channel_choices, current_index=current)))
        return mem

    def set_memory(self, mem):
        number = mem.number
        if mem.empty:
            return self.erase_memory(number)

        d = self._m()
        old_count = self._scanlist_count()
        if number > old_count:
            for i in range(old_count, number):
                self._zero_record(
                    SCANLIST_RECORDS + i * SCANLIST_SIZE, SCANLIST_SIZE)
            _set_bytes(d, SCANLIST_BANK, bytes([number]))

        off = SCANLIST_RECORDS + (number - 1) * SCANLIST_SIZE
        raw = bytearray(SCANLIST_SIZE)
        raw[0x00:0x10] = _encode_utf16(mem.name, 8)

        members = []
        for i in range(32):
            members.append(_extra_index(mem, "member_%02d" % (i + 1), 0))
        active = 0
        for i, value in enumerate(members):
            if value:
                active = i + 1
            raw[0x11 + i * 2:0x13 + i * 2] = _p16(value)
        raw[0x10] = active

        raw[0x51] = _extra_index(mem, "pri1_mode", 0)
        raw[0x52] = _extra_index(mem, "pri2_mode", 0)
        raw[0x53] = _extra_index(mem, "pri1_zone", 0)
        raw[0x54] = _extra_index(mem, "pri2_zone", 0)
        raw[0x55] = _extra_index(mem, "pri1_channel", 0)
        raw[0x57] = _extra_index(mem, "pri2_channel", 0)
        raw[0x59] = _extra_index(mem, "revert_mode", 0)
        raw[0x5A] = _extra_index(mem, "revert_zone", 0)
        raw[0x5B] = _extra_index(mem, "revert_channel", 0)
        raw[0x5D] = max(0, min(20, _extra_index(mem, "rx_hold", 0) // 500))
        raw[0x5E] = max(0, min(20, _extra_index(mem, "tx_hold", 0) // 500))
        _set_bytes(d, off, raw)

    def erase_memory(self, number):
        d = self._m()
        self._zero_record(
            SCANLIST_RECORDS + (number - 1) * SCANLIST_SIZE, SCANLIST_SIZE)
        count = self._scanlist_count()
        while count > 0:
            off = SCANLIST_RECORDS + (count - 1) * SCANLIST_SIZE
            if bytes(d[off:off + SCANLIST_SIZE]) != b"\x00" * SCANLIST_SIZE:
                break
            count -= 1
        _set_bytes(d, SCANLIST_BANK, bytes([count]))


class GD73DTMFCodesRadio(GD73CollectionRadio):
    """The fixed sixteen-entry DTMF Code table."""

    VARIANT = "DTMF Codes"
    MAX_RECORDS = 16
    NAME_LENGTH = 16

    def get_memory(self, number):
        if number < 1 or number > 16:
            raise errors.InvalidMemoryLocation(number)
        code = self._dtmf_number(number - 1)
        if not code:
            return self._base_memory(number, empty=True)
        return self._base_memory(number, code, empty=False)

    def set_memory(self, mem):
        if mem.empty:
            return self.erase_memory(mem.number)
        off = DTMF_NUMBER_BASE + (mem.number - 1) * DTMF_NUMBER_SIZE
        _set_bytes(self._m(), off, self._encode_dtmf_number(mem.name))

    def erase_memory(self, number):
        off = DTMF_NUMBER_BASE + (number - 1) * DTMF_NUMBER_SIZE
        _set_bytes(self._m(), off, b"\x00" * DTMF_NUMBER_SIZE)


class GD73PTTTemplatesRadio(GD73CollectionRadio):
    """The fixed 32-entry DTMF PTT Template table."""

    VARIANT = "PTT Templates"
    MAX_RECORDS = 32
    NAME_LENGTH = 8

    def get_features(self):
        rf = super().get_features()
        rf.can_delete = False
        return rf

    def get_memory(self, number):
        if number < 1 or number > 32:
            raise errors.InvalidMemoryLocation(number)
        p = self._ptt_template(number - 1)
        mem = self._base_memory(
            number, "PTT %02d" % number, empty=False)
        mem.immutable.append("name")

        systems = ["DTMF1", "DTMF2", "DTMF3", "DTMF4"]
        ids = ["None"] + [
            "DTMF Code %d" % (i + 1) for i in range(16)]

        mem.extra.append(RadioSetting(
            "system", "DTMF System",
            RadioSettingValueList(
                systems, current_index=min(p["system"], 3))))
        mem.extra.append(RadioSetting(
            "type", "PTT ID Type",
            RadioSettingValueList(
                PTT_ID_TYPES,
                current_index=min(p["type"], len(PTT_ID_TYPES) - 1))))
        mem.extra.append(RadioSetting(
            "mode", "PTT ID Mode",
            RadioSettingValueList(
                PTT_ID_MODES,
                current_index=min(p["mode"], len(PTT_ID_MODES) - 1))))
        mem.extra.append(RadioSetting(
            "connect", "Connect ID",
            RadioSettingValueList(
                ids, current_index=min(p["connect"], 16))))
        mem.extra.append(RadioSetting(
            "disconnect", "Disconnect ID",
            RadioSettingValueList(
                ids, current_index=min(p["disconnect"], 16))))
        return mem

    def set_memory(self, mem):
        p = self._ptt_template(mem.number - 1)
        off = DTMF_PTT_BASE + (mem.number - 1) * DTMF_PTT_SIZE
        raw = bytearray(DTMF_PTT_SIZE)
        raw[0] = _extra_index(mem, "system", p["system"])
        raw[1] = _extra_index(mem, "type", p["type"])
        raw[2] = _extra_index(mem, "mode", p["mode"])
        raw[3] = _extra_index(mem, "connect", p["connect"])
        raw[4] = _extra_index(mem, "disconnect", p["disconnect"])
        _set_bytes(self._m(), off, raw)


class GD73QuickTextRadio(GD73CollectionRadio):
    """Quick Text message table."""

    VARIANT = "Quick Text"
    MAX_RECORDS = 16
    NAME_LENGTH = 40

    def get_memory(self, number):
        if number < 1 or number > 16:
            raise errors.InvalidMemoryLocation(number)
        if number > self._message_count():
            return self._base_memory(number, empty=True)
        value = self._message(number - 1)
        if not value:
            return self._base_memory(number, empty=True)
        return self._base_memory(number, value, empty=False)

    def set_memory(self, mem):
        number = mem.number
        if mem.empty:
            return self.erase_memory(number)

        d = self._m()
        old_count = self._message_count()
        if number > old_count:
            for i in range(old_count, number):
                _set_bytes(
                    d, MESSAGE_BASE + 1 + i * 0x51,
                    b"\x00" * 0x51)
            _set_bytes(d, MESSAGE_BASE, bytes([number]))

        off = MESSAGE_BASE + 1 + (number - 1) * 0x51
        msg = mem.name[:40]
        enc = msg.encode("utf-16le")
        _set_bytes(d, off, bytes([len(msg)]))
        _set_bytes(d, off + 1, enc.ljust(0x50, b"\x00"))

    def erase_memory(self, number):
        d = self._m()
        _set_bytes(
            d, MESSAGE_BASE + 1 + (number - 1) * 0x51,
            b"\x00" * 0x51)
        count = self._message_count()
        while count > 0:
            off = MESSAGE_BASE + 1 + (count - 1) * 0x51
            if bytes(d[off:off + 0x51]) != b"\x00" * 0x51:
                break
            count -= 1
        _set_bytes(d, MESSAGE_BASE, bytes([count]))
