# Experimental CHIRP driver for Radioddity GD-73 / GD-73A
#
# Provides native-USB codeplug download/upload plus CHIRP editing support.
# The radio uses its C7000 USB interface directly (VID 1206 / PID 0227);
# CHIRP's serial-port selection is only a UI placeholder. Use Fake NOP.
#
# Unknown/unimplemented codeplug fields are preserved verbatim.
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
DRIVER_REVISION = "stage8c-public-github"

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
    23, 25, 26, 31, 32, 36, 43, 47, 51, 53, 54, 65, 71, 72,
    73, 74, 114, 115, 116, 122, 125, 131, 132, 134, 143, 145,
    152, 155, 156, 162, 165, 172, 174, 205, 212, 223, 225, 226,
    243, 244, 245, 246, 251, 252, 255, 261, 263, 265, 266, 271,
    274, 306, 311, 315, 325, 331, 332, 343, 346, 351, 356, 364,
    365, 371, 411, 412, 413, 423, 431, 432, 445, 446, 452, 454,
    455, 462, 464, 465, 466, 503, 506, 516, 523, 526, 532, 546,
    565, 606, 612, 624, 627, 631, 632, 654, 662, 664, 703, 712,
    723, 731, 732, 734, 743, 754,
]

ADMIT_CHOICES = ["Always", "Channel Free", "Color Code"]
CALL_TYPES = ["Group", "Private", "All Call"]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _byte(data, off):
    value = data[off]
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        return ord(value)
    if isinstance(value, (bytes, bytearray)):
        return value[0]
    return int(value)


def _u16(data, off):
    return struct.unpack_from("<H", bytes(data[off:off + 2]))[0]


def _u32(data, off):
    return struct.unpack_from("<I", bytes(data[off:off + 4]))[0]


def _set_bytes(mmap_obj, offset, payload):
    mmap_obj.set(int(offset), bytes(payload))


def _set_u16(mmap_obj, offset, value):
    _set_bytes(mmap_obj, offset, struct.pack("<H", int(value) & 0xFFFF))


def _set_u32(mmap_obj, offset, value):
    _set_bytes(mmap_obj, offset, struct.pack("<I", int(value) & 0xFFFFFFFF))


def _decode_utf16(data, off, size):
    raw = bytes(data[off:off + size])
    try:
        text = raw.decode("utf-16-le", errors="ignore")
    except Exception:
        return ""
    return text.split("\x00", 1)[0].strip()


def _encode_utf16(text, size):
    raw = str(text).encode("utf-16-le")[:size]
    return raw + (b"\x00" * (size - len(raw)))


def _set_utf16(mmap_obj, offset, text, size):
    _set_bytes(mmap_obj, offset, _encode_utf16(text, size))


def _clamp_index(value, choices, default=0):
    try:
        value = int(value)
    except Exception:
        return default
    if 0 <= value < len(choices):
        return value
    return default


def _decode_ctcss(raw):
    if raw < len(CTCSS_CODES):
        return CTCSS_CODES[raw]
    return None


def _decode_dcs(raw):
    if raw < len(DCS_CODES):
        return DCS_CODES[raw]
    return None


def _encode_ctcss(value):
    if value is None:
        return 0
    value = float(value)
    best = min(range(len(CTCSS_CODES)), key=lambda i: abs(CTCSS_CODES[i] - value))
    return best


def _encode_dcs(value):
    if value is None:
        return 0
    try:
        return DCS_CODES.index(int(value))
    except ValueError:
        return 0


def _gd73_precise_wait(seconds):
    """High-resolution short wait for C7000 USB turn-around.

    On packaged Windows CHIRP, time.sleep(0.001) can round up toward the
    Windows scheduler quantum. The OEM CPS capture shows a median
    request-to-IN-submit delay of about 872 microseconds.
    """
    deadline = time.perf_counter() + float(seconds)
    while time.perf_counter() < deadline:
        pass


def _c7000_checksum(packet):
    raw = bytearray(packet)
    if len(raw) >= 6:
        raw[4] = 0
        raw[5] = 0
    crc = 0xFFFF
    i = 0
    while i + 1 < len(raw):
        value = raw[i] | (raw[i + 1] << 8)
        if crc < value:
            crc += 0xFFFF
        crc -= value
        crc &= 0xFFFFFFFF
        i += 2
    if i < len(raw):
        value = raw[i]
        if crc < value:
            crc += 0xFFFF
        crc -= value
        crc &= 0xFFFFFFFF
    return crc & 0xFFFF


def _c7000_packet(command, subcommand, flags=0x0F, payload=b""):
    payload = bytes(payload)
    packet = bytearray(9 + len(payload))
    packet[0] = 0x68
    packet[1] = flags & 0xFF
    packet[2] = command & 0xFF
    packet[3] = subcommand & 0xFF
    struct.pack_into("<H", packet, 6, len(payload))
    packet[8:8 + len(payload)] = payload
    packet[8 + len(payload)] = 0x10
    struct.pack_into("<H", packet, 4, _c7000_checksum(packet))
    return bytes(packet)


def _parse_c7000(data):
    raw = bytes(data)
    if len(raw) < 9 or raw[0] != 0x68:
        raise ValueError("not a C7000 frame")
    payload_len = struct.unpack_from("<H", raw, 6)[0]
    frame_len = 9 + payload_len
    if len(raw) < frame_len:
        raise ValueError("short C7000 frame")
    frame = raw[:frame_len]
    if frame[-1] != 0x10:
        raise ValueError("bad C7000 terminator")
    return frame[1], frame[2], frame[3], frame[8:-1]


# ---------------------------------------------------------------------------
# Direct libusb-win32 backend for packaged CHIRP on Windows
# ---------------------------------------------------------------------------


def _open_libusb0_direct():
    if os.name != "nt":
        return None

    try:
        import ctypes
    except Exception as e:
        LOG.debug("ctypes unavailable for libusb0 backend: %s", e)
        return None

    candidates = [
        "libusb0.dll",
        os.path.join(os.environ.get("WINDIR", r"C:\Windows"), "System32", "libusb0.dll"),
        os.path.join(os.environ.get("WINDIR", r"C:\Windows"), "SysWOW64", "libusb0.dll"),
    ]

    dll = None
    load_error = None
    for candidate in candidates:
        try:
            dll = ctypes.CDLL(candidate)
            LOG.info("Loaded libusb-win32 backend: %s", candidate)
            break
        except Exception as e:
            load_error = e

    if dll is None:
        LOG.debug("Could not load libusb0.dll: %s", load_error)
        return None

    class usb_device_descriptor(ctypes.Structure):
        _pack_ = 1
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

    class usb_bus(ctypes.Structure):
        pass

    class usb_device(ctypes.Structure):
        pass

    usb_device_p = ctypes.POINTER(usb_device)
    usb_bus_p = ctypes.POINTER(usb_bus)

    usb_device._pack_ = 1
    usb_device._fields_ = [
        ("next", usb_device_p),
        ("prev", usb_device_p),
        ("filename", ctypes.c_char * 512),
        ("bus", usb_bus_p),
        ("descriptor", usb_device_descriptor),
        ("config", ctypes.c_void_p),
        ("dev", ctypes.c_void_p),
        ("devnum", ctypes.c_uint8),
        ("num_children", ctypes.c_ubyte),
        ("children", ctypes.POINTER(usb_device_p)),
    ]

    usb_bus._pack_ = 1
    usb_bus._fields_ = [
        ("next", usb_bus_p),
        ("prev", usb_bus_p),
        ("dirname", ctypes.c_char * 512),
        ("devices", usb_device_p),
        ("location", ctypes.c_uint32),
        ("root_dev", usb_device_p),
    ]

    dll.usb_init.argtypes = []
    dll.usb_init.restype = None
    dll.usb_find_busses.argtypes = []
    dll.usb_find_busses.restype = ctypes.c_int
    dll.usb_find_devices.argtypes = []
    dll.usb_find_devices.restype = ctypes.c_int
    dll.usb_get_busses.argtypes = []
    dll.usb_get_busses.restype = usb_bus_p
    dll.usb_open.argtypes = [usb_device_p]
    dll.usb_open.restype = ctypes.c_void_p
    dll.usb_close.argtypes = [ctypes.c_void_p]
    dll.usb_close.restype = ctypes.c_int
    dll.usb_claim_interface.argtypes = [ctypes.c_void_p, ctypes.c_int]
    dll.usb_claim_interface.restype = ctypes.c_int
    dll.usb_release_interface.argtypes = [ctypes.c_void_p, ctypes.c_int]
    dll.usb_release_interface.restype = ctypes.c_int
    dll.usb_bulk_write.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
    dll.usb_bulk_write.restype = ctypes.c_int
    dll.usb_bulk_read.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
    dll.usb_bulk_read.restype = ctypes.c_int
    try:
        dll.usb_strerror.argtypes = []
        dll.usb_strerror.restype = ctypes.c_char_p
    except Exception:
        pass

    def usb_error():
        try:
            err = dll.usb_strerror()
            if err:
                return err.decode(errors="replace")
        except Exception:
            pass
        return "libusb-win32 error"

    dll.usb_init()
    dll.usb_find_busses()
    dll.usb_find_devices()

    bus = dll.usb_get_busses()
    found = None
    while bool(bus):
        dev = bus.contents.devices
        while bool(dev):
            desc = dev.contents.descriptor
            if desc.idVendor == USB_VID and desc.idProduct == USB_PID:
                found = dev
                break
            dev = dev.contents.next
        if found is not None:
            break
        bus = bus.contents.next

    if found is None:
        return None

    handle = dll.usb_open(found)
    if not handle:
        raise errors.RadioError("Could not open GD-73A USB device: %s" % usb_error())

    ret = dll.usb_claim_interface(handle, USB_INTERFACE)
    if ret < 0:
        dll.usb_close(handle)
        raise errors.RadioError("Could not claim GD-73A USB interface 0: %s" % usb_error())

    class _Libusb0Device:
        _is_libusb0 = True

        def __init__(self, dll_obj, handle_obj, error_fn):
            self._dll = dll_obj
            self._handle = handle_obj
            self._error_fn = error_fn
            self._closed = False

        def write(self, endpoint, data, timeout=1000):
            payload = bytes(data)
            buf = ctypes.create_string_buffer(payload, len(payload))
            ret = self._dll.usb_bulk_write(
                self._handle, int(endpoint), buf, len(payload), int(timeout))
            if ret < 0:
                raise IOError(self._error_fn())
            return ret

        def read(self, endpoint, size, timeout=1000):
            buf = ctypes.create_string_buffer(int(size))
            ret = self._dll.usb_bulk_read(
                self._handle, int(endpoint), buf, int(size), int(timeout))
            if ret < 0:
                msg = self._error_fn()
                if "timeout" in msg.lower():
                    return b""
                raise IOError(msg)
            return bytes(buf.raw[:ret])

        def close(self):
            if self._closed:
                return
            self._closed = True
            try:
                self._dll.usb_release_interface(self._handle, USB_INTERFACE)
            finally:
                self._dll.usb_close(self._handle)

    return _Libusb0Device(dll, handle, usb_error)


# ---------------------------------------------------------------------------
# Radio
# ---------------------------------------------------------------------------


@directory.register
class GD73ARadio(chirp_common.CloneModeRadio):
    VENDOR = "Radioddity"
    MODEL = "GD-73A"
    VARIANT = "Native USB read v4 / mmap parser fix"
    BAUD_RATE = 9600

    def get_features(self):
        rf = chirp_common.RadioFeatures()
        rf.has_settings = True
        rf.has_sub_devices = (self.VARIANT == "Native USB read v4 / mmap parser fix")
        rf.has_bank = False
        rf.has_name = True
        rf.has_offset = True
        rf.has_mode = True
        rf.has_dtcs = True
        rf.has_rx_dtcs = True
        rf.has_ctone = True
        rf.has_cross = True
        rf.has_tuning_step = False
        rf.can_odd_split = True
        rf.memory_bounds = (1, max(1, self._channel_count()))
        rf.valid_name_length = 16
        rf.valid_modes = ["FM", "NFM", "DMR"]
        rf.valid_duplexes = ["", "+", "-", "split", "off"]
        rf.valid_tmodes = ["", "Tone", "TSQL", "DTCS", "Cross"]
        rf.valid_cross_modes = [
            "Tone->Tone", "Tone->DTCS", "DTCS->Tone", "->Tone",
            "->DTCS", "DTCS->", "DTCS->DTCS",
        ]
        rf.valid_power_levels = POWER_LEVELS
        rf.valid_bands = [(136000000, 174000000), (400000000, 480000000)]
        rf.valid_skips = [""]
        rf.valid_characters = chirp_common.CHARSET_ASCII
        return rf

    @classmethod
    def get_prompts(cls):
        prompts = chirp_common.RadioPrompts()
        prompts.experimental = (
            "GD-73A native-USB read/write support. "
            "The CHIRP Port selection is only a dummy because this driver "
            "opens USB VID 1206 PID 0227 directly. Select "
            "Fake NOP as the port. The serial port is ignored because this radio uses native USB.")
        prompts.pre_download = (
            "Connect and power on the GD-73A. The selected serial port is "
            "ignored by the driver; use Fake NOP as the dummy port for both download and upload.")
        return prompts

    @classmethod
    def match_model(cls, filedata, filename):
        return len(filedata) >= IMAGE_SIZE

    def _channel_count(self):
        if not getattr(self, "_mmap", None):
            return 1
        try:
            return max(1, min(1024, _u16(self._mmap, CHANNEL_BANK)))
        except Exception:
            return 1

    def _contact_count(self):
        if not getattr(self, "_mmap", None):
            return 0
        try:
            return min(1024, _u16(self._mmap, CONTACT_BANK))
        except Exception:
            return 0

    def _zone_count(self):
        if not getattr(self, "_mmap", None):
            return 0
        try:
            return min(64, _byte(self._mmap, ZONE_BANK))
        except Exception:
            return 0

    def _rxgroup_count(self):
        if not getattr(self, "_mmap", None):
            return 0
        try:
            return min(250, _byte(self._mmap, RXGROUP_BANK))
        except Exception:
            return 0

    def _scanlist_count(self):
        if not getattr(self, "_mmap", None):
            return 0
        try:
            return min(16, _byte(self._mmap, SCANLIST_BANK))
        except Exception:
            return 0

    def _contact_name(self, index1):
        if not index1:
            return "None"
        index = index1 - 1
        if index < 0 or index >= self._contact_count():
            return "Contact %d" % index1
        off = CONTACT_RECORDS + index * CONTACT_SIZE
        name = _decode_utf16(self._mmap, off, 0x20)
        dmrid = _u32(self._mmap, off + 0x21)
        if name:
            return "%d: %s (%d)" % (index1, name, dmrid)
        return "%d: %d" % (index1, dmrid)

    def _rxgroup_name(self, index1):
        if not index1:
            return "None"
        index = index1 - 1
        if index < 0 or index >= self._rxgroup_count():
            return "RX Group %d" % index1
        off = RXGROUP_RECORDS + index * RXGROUP_SIZE
        name = _decode_utf16(self._mmap, off, 0x10)
        return "%d: %s" % (index1, name or "RX Group")

    def _scanlist_name(self, index1):
        if not index1:
            return "None"
        index = index1 - 1
        if index < 0 or index >= self._scanlist_count():
            return "Scan List %d" % index1
        off = SCANLIST_RECORDS + index * SCANLIST_SIZE
        name = _decode_utf16(self._mmap, off, 0x10)
        return "%d: %s" % (index1, name or "Scan List")

    def get_raw_memory(self, number):
        off = CHANNEL_RECORDS + (number - 1) * CHANNEL_SIZE
        return repr(bytes(self._mmap[off:off + CHANNEL_SIZE]))

    def get_memory(self, number):
        mem = chirp_common.Memory()
        mem.number = number

        count = self._channel_count()
        if number < 1 or number > count:
            mem.empty = True
            return mem

        off = CHANNEL_RECORDS + (number - 1) * CHANNEL_SIZE
        d = self._mmap

        mem.name = _decode_utf16(d, off + O_NAME, 0x20)
        mem.freq = _u32(d, off + O_RXFREQ)
        txfreq = _u32(d, off + O_TXFREQ)

        if _byte(d, off + O_RXONLY):
            mem.duplex = "off"
            mem.offset = 0
        elif txfreq == mem.freq:
            mem.duplex = ""
            mem.offset = 0
        else:
            delta = txfreq - mem.freq
            if abs(delta) <= 10000000:
                mem.duplex = "+" if delta > 0 else "-"
                mem.offset = abs(delta)
            else:
                mem.duplex = "split"
                mem.offset = txfreq

        ch_type = _byte(d, off + O_TYPE)
        if ch_type == 1:
            mem.mode = "DMR"
        else:
            mem.mode = "FM" if _byte(d, off + O_BANDWIDTH) else "NFM"

        power = _byte(d, off + O_POWER)
        mem.power = POWER_LEVELS[1 if power else 0]

        if ch_type == 0:
            self._decode_analog_tones(mem, off)

        extra = RadioSettingGroup("extra", "GD-73 Channel")

        if ch_type == 1:
            raw_ts = _byte(d, off + O_TIMESLOT)
            ts = 2 if raw_ts in (1, 3) else 1
            extra.append(RadioSetting(
                "timeslot", "Time Slot",
                RadioSettingValueList(["1", "2"], current_index=ts - 1)))

            cc = min(15, _byte(d, off + O_COLORCODE))
            extra.append(RadioSetting(
                "colorcode", "Color Code",
                RadioSettingValueInteger(0, 15, cc)))

            contact_index = _u16(d, off + O_CONTACT)
            contact_choices = ["None"] + [
                self._contact_name(i) for i in range(1, self._contact_count() + 1)
            ]
            contact_index = min(contact_index, len(contact_choices) - 1)
            extra.append(RadioSetting(
                "contact", "Contact",
                RadioSettingValueList(contact_choices, current_index=contact_index)))

            group_index = _byte(d, off + O_GROUPLIST)
            group_choices = ["None"] + [
                self._rxgroup_name(i) for i in range(1, self._rxgroup_count() + 1)
            ]
            group_index = min(group_index, len(group_choices) - 1)
            extra.append(RadioSetting(
                "rxgroup", "RX Group",
                RadioSettingValueList(group_choices, current_index=group_index)))

            admit = _clamp_index(_byte(d, off + O_ADMIT), ADMIT_CHOICES)
            extra.append(RadioSetting(
                "admit", "Admit Criteria",
                RadioSettingValueList(ADMIT_CHOICES, current_index=admit)))

        scan_index = _byte(d, off + O_SCANLIST)
        scan_choices = ["None"] + [
            self._scanlist_name(i) for i in range(1, self._scanlist_count() + 1)
        ]
        scan_index = min(scan_index, len(scan_choices) - 1)
        extra.append(RadioSetting(
            "scanlist", "Scan List",
            RadioSettingValueList(scan_choices, current_index=scan_index)))

        extra.append(RadioSetting(
            "talkaround", "Talkaround",
            RadioSettingValueBoolean(bool(_byte(d, off + O_TALKAROUND)))))
        extra.append(RadioSetting(
            "scanautostart", "Scan Auto Start",
            RadioSettingValueBoolean(bool(_byte(d, off + O_SCANAUTOSTART)))))

        mem.extra = extra
        return mem

    def _decode_analog_tones(self, mem, off):
        d = self._mmap
        rxmode = _byte(d, off + O_RX_TONE_MODE)
        txmode = _byte(d, off + O_TX_TONE_MODE)

        tx_tone = None
        rx_tone = None
        tx_dcs = None
        rx_dcs = None

        if txmode == 1:
            tx_tone = _decode_ctcss(_byte(d, off + O_TX_CTCSS))
        elif txmode in (2, 3):
            tx_dcs = _decode_dcs(_byte(d, off + O_TX_DCS))

        if rxmode == 1:
            rx_tone = _decode_ctcss(_byte(d, off + O_RX_CTCSS))
        elif rxmode in (2, 3):
            rx_dcs = _decode_dcs(_byte(d, off + O_RX_DCS))

        if txmode == 0 and rxmode == 0:
            mem.tmode = ""
        elif txmode == 1 and rxmode == 0:
            mem.tmode = "Tone"
            if tx_tone is not None:
                mem.rtone = tx_tone
        elif txmode == 1 and rxmode == 1 and tx_tone == rx_tone:
            mem.tmode = "TSQL"
            if tx_tone is not None:
                mem.rtone = tx_tone
                mem.ctone = tx_tone
        elif txmode in (2, 3) and rxmode in (2, 3) and tx_dcs == rx_dcs:
            mem.tmode = "DTCS"
            if tx_dcs is not None:
                mem.dtcs = tx_dcs
                mem.rx_dtcs = tx_dcs
            mem.dtcs_polarity = (
                ("R" if txmode == 3 else "N") +
                ("R" if rxmode == 3 else "N"))
        else:
            mem.tmode = "Cross"
            tx_name = ""
            rx_name = ""
            if txmode == 1:
                tx_name = "Tone"
                if tx_tone is not None:
                    mem.rtone = tx_tone
            elif txmode in (2, 3):
                tx_name = "DTCS"
                if tx_dcs is not None:
                    mem.dtcs = tx_dcs
            if rxmode == 1:
                rx_name = "Tone"
                if rx_tone is not None:
                    mem.ctone = rx_tone
            elif rxmode in (2, 3):
                rx_name = "DTCS"
                if rx_dcs is not None:
                    mem.rx_dtcs = rx_dcs
            mem.cross_mode = "%s->%s" % (tx_name, rx_name)
            mem.dtcs_polarity = (
                ("R" if txmode == 3 else "N") +
                ("R" if rxmode == 3 else "N"))

    def set_memory(self, mem):
        off = CHANNEL_RECORDS + (mem.number - 1) * CHANNEL_SIZE
        d = self._mmap

        _set_utf16(d, off + O_NAME, mem.name, 0x20)
        _set_u32(d, off + O_RXFREQ, mem.freq)

        if mem.duplex == "off":
            _set_bytes(d, off + O_RXONLY, b"\x01")
            txfreq = mem.freq
        elif mem.duplex == "+":
            _set_bytes(d, off + O_RXONLY, b"\x00")
            txfreq = mem.freq + mem.offset
        elif mem.duplex == "-":
            _set_bytes(d, off + O_RXONLY, b"\x00")
            txfreq = mem.freq - mem.offset
        elif mem.duplex == "split":
            _set_bytes(d, off + O_RXONLY, b"\x00")
            txfreq = mem.offset
        else:
            _set_bytes(d, off + O_RXONLY, b"\x00")
            txfreq = mem.freq
        _set_u32(d, off + O_TXFREQ, txfreq)

        if mem.mode == "DMR":
            _set_bytes(d, off + O_TYPE, b"\x01")
        else:
            _set_bytes(d, off + O_TYPE, b"\x00")
            _set_bytes(d, off + O_BANDWIDTH, b"\x01" if mem.mode == "FM" else b"\x00")
            self._encode_analog_tones(mem, off)

        try:
            high = str(mem.power) == str(POWER_LEVELS[1])
        except Exception:
            high = True
        _set_bytes(d, off + O_POWER, b"\x01" if high else b"\x00")

        if getattr(mem, "extra", None):
            for setting in mem.extra:
                name = setting.get_name()
                value = setting.value
                if name == "timeslot":
                    idx = int(value)
                    _set_bytes(d, off + O_TIMESLOT, bytes([idx]))
                elif name == "colorcode":
                    _set_bytes(d, off + O_COLORCODE, bytes([int(value) & 0x0F]))
                elif name == "contact":
                    _set_u16(d, off + O_CONTACT, int(value))
                elif name == "rxgroup":
                    _set_bytes(d, off + O_GROUPLIST, bytes([int(value) & 0xFF]))
                elif name == "admit":
                    _set_bytes(d, off + O_ADMIT, bytes([int(value) & 0xFF]))
                elif name == "scanlist":
                    _set_bytes(d, off + O_SCANLIST, bytes([int(value) & 0xFF]))
                elif name == "talkaround":
                    _set_bytes(d, off + O_TALKAROUND, b"\x01" if bool(value) else b"\x00")
                elif name == "scanautostart":
                    _set_bytes(d, off + O_SCANAUTOSTART, b"\x01" if bool(value) else b"\x00")

    def _encode_analog_tones(self, mem, off):
        d = self._mmap
        txmode = 0
        rxmode = 0

        if mem.tmode == "Tone":
            txmode = 1
            _set_bytes(d, off + O_TX_CTCSS, bytes([_encode_ctcss(mem.rtone)]))
        elif mem.tmode == "TSQL":
            txmode = rxmode = 1
            tone = getattr(mem, "ctone", mem.rtone)
            _set_bytes(d, off + O_TX_CTCSS, bytes([_encode_ctcss(mem.rtone)]))
            _set_bytes(d, off + O_RX_CTCSS, bytes([_encode_ctcss(tone)]))
        elif mem.tmode == "DTCS":
            txmode = 3 if getattr(mem, "dtcs_polarity", "NN")[0] == "R" else 2
            rxmode = 3 if getattr(mem, "dtcs_polarity", "NN")[1] == "R" else 2
            _set_bytes(d, off + O_TX_DCS, bytes([_encode_dcs(mem.dtcs)]))
            _set_bytes(d, off + O_RX_DCS, bytes([_encode_dcs(mem.rx_dtcs)]))
        elif mem.tmode == "Cross":
            tx_name, rx_name = mem.cross_mode.split("->")
            pol = getattr(mem, "dtcs_polarity", "NN")
            if tx_name == "Tone":
                txmode = 1
                _set_bytes(d, off + O_TX_CTCSS, bytes([_encode_ctcss(mem.rtone)]))
            elif tx_name == "DTCS":
                txmode = 3 if pol[0] == "R" else 2
                _set_bytes(d, off + O_TX_DCS, bytes([_encode_dcs(mem.dtcs)]))
            if rx_name == "Tone":
                rxmode = 1
                _set_bytes(d, off + O_RX_CTCSS, bytes([_encode_ctcss(mem.ctone)]))
            elif rx_name == "DTCS":
                rxmode = 3 if pol[1] == "R" else 2
                _set_bytes(d, off + O_RX_DCS, bytes([_encode_dcs(mem.rx_dtcs)]))

        _set_bytes(d, off + O_TX_TONE_MODE, bytes([txmode]))
        _set_bytes(d, off + O_RX_TONE_MODE, bytes([rxmode]))

    def process_mmap(self):
        # Parsing is performed lazily by getters to preserve unknown fields.
        return

    # ------------------------------------------------------------------
    # Native USB transport
    # ------------------------------------------------------------------

    def _open_usb(self):
        direct = _open_libusb0_direct()
        if direct is not None:
            return direct

        if usb is None:
            raise errors.RadioError(
                "GD-73A USB device could not be opened. On Windows install/use "
                "the OEM libusb-win32 driver (libusb0.dll).")

        backend = None
        if libusb_package is not None:
            try:
                backend = libusb_package.get_libusb1_backend()
            except Exception:
                backend = None

        dev = usb.core.find(idVendor=USB_VID, idProduct=USB_PID, backend=backend)
        if dev is None:
            raise errors.RadioError(
                "GD-73A native USB device 1206:0227 not found. Connect and power on the radio.")

        try:
            dev.detach_kernel_driver(USB_INTERFACE)
        except Exception:
            pass

        try:
            usb.util.claim_interface(dev, USB_INTERFACE)
        except Exception as e:
            raise errors.RadioError("Could not claim GD-73A USB interface: %s" % e)

        return dev

    def _usb_send_recv(self, dev, packet):
        packet = bytes(packet)
        LOG.debug("GD73 USB TX: %s", packet.hex(" "))
        try:
            wrote = dev.write(USB_EP_OUT, packet, timeout=USB_TIMEOUT_MS)
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
                    return _parse_c7000(rx)[3]
                except ValueError as e:
                    raise errors.RadioError(
                        "Invalid GD-73A C7000 response: %s" % e)

            if rx and any(rx):
                LOG.warning(
                    "GD73 non-C7000 RX attempt %d: %s",
                    attempt + 1, rx.hex(" "))

        raise errors.RadioError(
            "GD-73A did not return a C7000 response after 11 reads")

    def _read_usb_image(self, dev):
        status = chirp_common.Status()
        status.max = BLOCK_COUNT
        status.cur = 0
        status.msg = "Reading GD-73A"
        if self.status_fn:
            self.status_fn(status)

        # Enter programming mode.
        payload = self._usb_send_recv(dev, _c7000_packet(0x01, 0x04))
        LOG.info("GD73 programming-mode payload: %s", payload.hex(" "))

        # Initial block request.
        payload = self._usb_send_recv(dev, _c7000_packet(0x01, 0x02))
        if len(payload) < 2 + BLOCK_SIZE:
            raise errors.RadioError(
                "GD-73A initial block response too short (%d)" % len(payload))

        image = bytearray()
        seq = struct.unpack_from("<H", payload, 0)[0]
        image.extend(payload[2:2 + BLOCK_SIZE])
        last_sequence = seq

        status.cur = 1
        if self.status_fn:
            self.status_fn(status)

        for block in range(1, BLOCK_COUNT):
            request = _c7000_packet(
                0x04, 0x01, 0x0F, struct.pack("<H", last_sequence))
            payload = self._usb_send_recv(dev, request)
            if len(payload) < 2 + BLOCK_SIZE:
                raise errors.RadioError(
                    "GD-73A block %d response too short (%d)" %
                    (block, len(payload)))

            seq = struct.unpack_from("<H", payload, 0)[0]
            expected = (last_sequence + 1) & 0xFFFF
            if seq != expected:
                raise errors.RadioError(
                    "GD-73A block sequence mismatch at block %d: expected %d, got %d" %
                    (block, expected, seq))

            image.extend(payload[2:2 + BLOCK_SIZE])
            last_sequence = seq

            status.cur = block + 1
            if self.status_fn and (
                    block == 0 or (block + 1) % 10 == 0 or
                    block + 1 == BLOCK_COUNT):
                self.status_fn(status)

        if len(image) != IMAGE_SIZE:
            raise errors.RadioError(
                "GD-73A image size mismatch: got %d, expected %d" %
                (len(image), IMAGE_SIZE))
        return bytes(image)

    def sync_in(self):
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

        payload = self._usb_send_recv(dev, _c7000_packet(0x01, 0x04))
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

            request = _c7000_packet(
                0x01, 0x00, 0x0F,
                struct.pack("<H", block) + chunk)

            ack = self._usb_send_recv(dev, request)
            if len(ack) != 2:
                raise errors.RadioError(
                    "GD-73A invalid write ACK at block %d: expected 2 bytes, got %d" %
                    (block, len(ack)))

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

    # ------------------------------------------------------------------
    # Settings / subdevices are defined below in the full driver.
    # ------------------------------------------------------------------

    def get_sub_devices(self):
        if self.VARIANT != "Native USB read v4 / mmap parser fix":
            return []
        children = []
        for cls in (
                GD73ChannelsRadio,
                GD73ContactsRadio,
                GD73RXGroupsRadio,
                GD73ZonesRadio,
                GD73ScanListsRadio,
                GD73DTMFCodesRadio,
                GD73PTTTemplatesRadio,
                GD73QuickTextRadio):
            child = cls(self.pipe)
            child._mmap = self._mmap
            child.status_fn = self.status_fn
            children.append(child)
        return children

    def get_settings(self):
        root = RadioSettings()
        general = RadioSettingGroup("general", "General")
        root.append(general)

        if not getattr(self, "_mmap", None):
            return root

        d = self._mmap

        # Keep this area deliberately conservative. These settings are
        # radio-wide values; large collections live in subdevices.
        general.append(RadioSetting(
            "radio_id", "Radio ID",
            RadioSettingValueInteger(0, 16777215, _u32(d, SETTINGS_BASE))))

        general.append(RadioSetting(
            "radio_name", "Radio Name",
            RadioSettingValueString(0, 16, _decode_utf16(d, SETTINGS_BASE + 0x04, 0x20))))

        return root

    def set_settings(self, settings):
        for element in settings:
            if not isinstance(element, RadioSetting):
                self.set_settings(element)
                continue
            name = element.get_name()
            value = element.value
            if name == "radio_id":
                _set_u32(self._mmap, SETTINGS_BASE, int(value))
            elif name == "radio_name":
                _set_utf16(self._mmap, SETTINGS_BASE + 0x04, str(value), 0x20)


# ---------------------------------------------------------------------------
# Collection subdevices
# ---------------------------------------------------------------------------


class _GD73CollectionRadio(GD73ARadio):
    COLLECTION_MAX = 1
    COLLECTION_LABEL = "Collection"

    def get_features(self):
        rf = chirp_common.RadioFeatures()
        rf.has_settings = False
        rf.has_sub_devices = False
        rf.has_bank = False
        rf.has_name = True
        rf.has_offset = False
        rf.has_mode = False
        rf.has_dtcs = False
        rf.has_rx_dtcs = False
        rf.has_ctone = False
        rf.has_cross = False
        rf.has_tuning_step = False
        rf.can_odd_split = False
        rf.memory_bounds = (1, self.COLLECTION_MAX)
        rf.valid_name_length = 16
        rf.valid_modes = ["FM"]
        rf.valid_duplexes = [""]
        rf.valid_tmodes = [""]
        rf.valid_power_levels = []
        rf.valid_bands = [(400000000, 400000000)]
        rf.valid_skips = [""]
        rf.valid_characters = chirp_common.CHARSET_ASCII
        return rf

    def _base_memory(self, number, name=""):
        mem = chirp_common.Memory()
        mem.number = number
        mem.name = name
        mem.freq = 400000000
        mem.duplex = ""
        mem.offset = 0
        mem.mode = "FM"
        mem.tmode = ""
        return mem


class GD73ChannelsRadio(GD73ARadio):
    VARIANT = "Channels"

    def get_features(self):
        rf = super().get_features()
        rf.has_sub_devices = False
        rf.has_settings = False
        return rf


class GD73ContactsRadio(_GD73CollectionRadio):
    VARIANT = "Digital Contacts"
    COLLECTION_MAX = 1024

    def get_memory(self, number):
        mem = self._base_memory(number)
        count = self._contact_count()
        if number > count:
            mem.empty = False
            mem.name = ""
            call_type = 0
            dmrid = 0
        else:
            off = CONTACT_RECORDS + (number - 1) * CONTACT_SIZE
            mem.name = _decode_utf16(self._mmap, off, 0x20)
            call_type = min(2, _byte(self._mmap, off + 0x20))
            dmrid = _u32(self._mmap, off + 0x21)
        extra = RadioSettingGroup("extra", "Digital Contact")
        extra.append(RadioSetting(
            "call_type", "Call Type",
            RadioSettingValueList(CALL_TYPES, current_index=call_type)))
        extra.append(RadioSetting(
            "dmr_id", "DMR ID",
            RadioSettingValueInteger(0, 16777215, dmrid)))
        mem.extra = extra
        return mem

    def set_memory(self, mem):
        number = mem.number
        off = CONTACT_RECORDS + (number - 1) * CONTACT_SIZE
        _set_utf16(self._mmap, off, mem.name, 0x20)
        for setting in getattr(mem, "extra", []):
            if setting.get_name() == "call_type":
                _set_bytes(self._mmap, off + 0x20, bytes([int(setting.value)]))
            elif setting.get_name() == "dmr_id":
                _set_u32(self._mmap, off + 0x21, int(setting.value))
        if mem.name and number > self._contact_count():
            _set_u16(self._mmap, CONTACT_BANK, number)


class GD73RXGroupsRadio(_GD73CollectionRadio):
    VARIANT = "RX Groups"
    COLLECTION_MAX = 250

    def get_memory(self, number):
        mem = self._base_memory(number)
        if number <= self._rxgroup_count():
            off = RXGROUP_RECORDS + (number - 1) * RXGROUP_SIZE
            mem.name = _decode_utf16(self._mmap, off, 0x10)
            count = min(33, _byte(self._mmap, off + 0x10))
            members = [_u16(self._mmap, off + 0x11 + i * 2) for i in range(count)]
        else:
            mem.name = ""
            members = []
        extra = RadioSettingGroup("extra", "RX Group")
        extra.append(RadioSetting(
            "members", "Contact Numbers (comma separated)",
            RadioSettingValueString(0, 160, ",".join(str(x) for x in members))))
        mem.extra = extra
        return mem

    def set_memory(self, mem):
        number = mem.number
        off = RXGROUP_RECORDS + (number - 1) * RXGROUP_SIZE
        _set_utf16(self._mmap, off, mem.name, 0x10)
        members = []
        for setting in getattr(mem, "extra", []):
            if setting.get_name() == "members":
                for item in str(setting.value).split(","):
                    item = item.strip()
                    if item:
                        try:
                            members.append(int(item))
                        except ValueError:
                            pass
        members = members[:33]
        _set_bytes(self._mmap, off + 0x10, bytes([len(members)]))
        _set_bytes(self._mmap, off + 0x11, b"\x00" * 66)
        for i, member in enumerate(members):
            _set_u16(self._mmap, off + 0x11 + i * 2, member)
        if mem.name and number > self._rxgroup_count():
            _set_bytes(self._mmap, RXGROUP_BANK, bytes([number]))


class GD73ZonesRadio(_GD73CollectionRadio):
    VARIANT = "Zones"
    COLLECTION_MAX = 64

    def get_memory(self, number):
        mem = self._base_memory(number)
        if number <= self._zone_count():
            off = ZONE_RECORDS + (number - 1) * ZONE_SIZE
            mem.name = _decode_utf16(self._mmap, off, 0x10)
            count = min(16, _byte(self._mmap, off + 0x10))
            members = [_u16(self._mmap, off + 0x11 + i * 2) for i in range(count)]
        else:
            mem.name = ""
            members = []
        extra = RadioSettingGroup("extra", "Zone")
        extra.append(RadioSetting(
            "members", "Channel Numbers (comma separated)",
            RadioSettingValueString(0, 100, ",".join(str(x) for x in members))))
        mem.extra = extra
        return mem

    def set_memory(self, mem):
        number = mem.number
        off = ZONE_RECORDS + (number - 1) * ZONE_SIZE
        _set_utf16(self._mmap, off, mem.name, 0x10)
        members = []
        for setting in getattr(mem, "extra", []):
            if setting.get_name() == "members":
                for item in str(setting.value).split(","):
                    item = item.strip()
                    if item:
                        try:
                            members.append(int(item))
                        except ValueError:
                            pass
        members = members[:16]
        _set_bytes(self._mmap, off + 0x10, bytes([len(members)]))
        _set_bytes(self._mmap, off + 0x11, b"\x00" * 32)
        for i, member in enumerate(members):
            _set_u16(self._mmap, off + 0x11 + i * 2, member)
        if mem.name and number > self._zone_count():
            _set_bytes(self._mmap, ZONE_BANK, bytes([number]))


class GD73ScanListsRadio(_GD73CollectionRadio):
    VARIANT = "Scan Lists"
    COLLECTION_MAX = 16

    def get_memory(self, number):
        mem = self._base_memory(number)
        if number <= self._scanlist_count():
            off = SCANLIST_RECORDS + (number - 1) * SCANLIST_SIZE
            mem.name = _decode_utf16(self._mmap, off, 0x10)
            count = min(32, _byte(self._mmap, off + 0x10))
            members = [_u16(self._mmap, off + 0x11 + i * 2) for i in range(count)]
        else:
            mem.name = ""
            members = []
        extra = RadioSettingGroup("extra", "Scan List")
        extra.append(RadioSetting(
            "members", "Channel Numbers (comma separated)",
            RadioSettingValueString(0, 180, ",".join(str(x) for x in members))))
        mem.extra = extra
        return mem

    def set_memory(self, mem):
        number = mem.number
        off = SCANLIST_RECORDS + (number - 1) * SCANLIST_SIZE
        _set_utf16(self._mmap, off, mem.name, 0x10)
        members = []
        for setting in getattr(mem, "extra", []):
            if setting.get_name() == "members":
                for item in str(setting.value).split(","):
                    item = item.strip()
                    if item:
                        try:
                            members.append(int(item))
                        except ValueError:
                            pass
        members = members[:32]
        _set_bytes(self._mmap, off + 0x10, bytes([len(members)]))
        _set_bytes(self._mmap, off + 0x11, b"\x00" * 64)
        for i, member in enumerate(members):
            _set_u16(self._mmap, off + 0x11 + i * 2, member)
        if mem.name and number > self._scanlist_count():
            _set_bytes(self._mmap, SCANLIST_BANK, bytes([number]))


_DTMF_CHARS = "0123456789ABCD*#"


def _decode_dtmf_record(d, off):
    length = min(16, _byte(d, off))
    result = []
    for i in range((length + 1) // 2):
        value = _byte(d, off + 1 + i)
        result.append(_DTMF_CHARS[(value >> 4) & 0x0F])
        if len(result) < length:
            result.append(_DTMF_CHARS[value & 0x0F])
    return "".join(result[:length])


def _encode_dtmf_record(text):
    text = "".join(ch for ch in str(text).upper() if ch in _DTMF_CHARS)[:16]
    out = bytearray(9)
    out[0] = len(text)
    for i in range(0, len(text), 2):
        hi = _DTMF_CHARS.index(text[i])
        lo = _DTMF_CHARS.index(text[i + 1]) if i + 1 < len(text) else 0
        out[1 + i // 2] = (hi << 4) | lo
    return bytes(out)


class GD73DTMFCodesRadio(_GD73CollectionRadio):
    VARIANT = "DTMF Codes"
    COLLECTION_MAX = 16

    def get_memory(self, number):
        off = DTMF_NUMBER_BASE + (number - 1) * DTMF_NUMBER_SIZE
        code = _decode_dtmf_record(self._mmap, off)
        mem = self._base_memory(number, code)
        return mem

    def set_memory(self, mem):
        off = DTMF_NUMBER_BASE + (mem.number - 1) * DTMF_NUMBER_SIZE
        _set_bytes(self._mmap, off, _encode_dtmf_record(mem.name))


class GD73PTTTemplatesRadio(_GD73CollectionRadio):
    VARIANT = "PTT Templates"
    COLLECTION_MAX = 32

    def get_memory(self, number):
        off = DTMF_PTT_BASE + (number - 1) * DTMF_PTT_SIZE
        raw = bytes(self._mmap[off:off + DTMF_PTT_SIZE])
        mem = self._base_memory(number, "PTT %d" % number)
        extra = RadioSettingGroup("extra", "PTT Template")
        labels = ["System", "Type", "Mode", "Connect", "Disconnect"]
        for i, label in enumerate(labels):
            extra.append(RadioSetting(
                "byte%d" % i, label,
                RadioSettingValueInteger(0, 255, raw[i])))
        mem.extra = extra
        return mem

    def set_memory(self, mem):
        off = DTMF_PTT_BASE + (mem.number - 1) * DTMF_PTT_SIZE
        raw = bytearray(self._mmap[off:off + DTMF_PTT_SIZE])
        for setting in getattr(mem, "extra", []):
            if setting.get_name().startswith("byte"):
                idx = int(setting.get_name()[4:])
                raw[idx] = int(setting.value) & 0xFF
        _set_bytes(self._mmap, off, bytes(raw))


class GD73QuickTextRadio(_GD73CollectionRadio):
    VARIANT = "Quick Text"
    COLLECTION_MAX = 16

    def _message_count(self):
        return min(16, _byte(self._mmap, MESSAGE_BASE))

    def get_memory(self, number):
        off = MESSAGE_BASE + 1 + (number - 1) * 0x51
        if number <= self._message_count():
            length = min(40, _byte(self._mmap, off))
            raw = bytes(self._mmap[off + 1:off + 1 + 80])
            text = raw.decode("utf-16-le", errors="ignore")[:length]
        else:
            text = ""
        mem = self._base_memory(number, text)
        return mem

    def set_memory(self, mem):
        number = mem.number
        off = MESSAGE_BASE + 1 + (number - 1) * 0x51
        text = str(mem.name)[:40]
        _set_bytes(self._mmap, off, bytes([len(text)]))
        _set_utf16(self._mmap, off + 1, text, 80)
        if text and number > self._message_count():
            _set_bytes(self._mmap, MESSAGE_BASE, bytes([number]))
