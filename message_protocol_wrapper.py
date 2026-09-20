import ctypes
from ctypes import (
    c_bool,
    c_uint8,
    c_uint32,
    c_int,
    POINTER,
    byref,
)

LIBRARY_PATH = "./bin/message_protocol.so"


# ---------------------------------------------------------------------------
# C types
# ---------------------------------------------------------------------------


class MessageProtocolStatus:
    OK = 0
    NO_PACKET = 1
    NOT_ENOUGH_SPACE = 2
    ERR_EXTRACTION_BUFFER_TOO_SMALL = 3
    RING_FULL = 4


# ---------------------------------------------------------------------------
# Load library
# ---------------------------------------------------------------------------

_lib = ctypes.CDLL(LIBRARY_PATH)


# ---------------------------------------------------------------------------
# Function declarations
# ---------------------------------------------------------------------------

# message_protocol_status_t encapsulate_message(
#     uint8_t* dest,
#     uint32_t dest_size,
#     void* payload,
#     uint32_t payload_size
# );

_lib.encapsulate_message.argtypes = [
    POINTER(c_uint8),
    c_uint32,
    ctypes.c_void_p,
    c_uint32,
]

_lib.encapsulate_message.restype = c_int


# message_protocol_status_t extract_message(
#     void* payload_dest,
#     uint32_t payload_size
# );

_lib.extract_message.argtypes = [
    ctypes.c_void_p,
    c_uint32,
]

_lib.extract_message.restype = c_int


# message_protocol_status_t add_received_bytes(
#     uint8_t* buff,
#     uint32_t size
# );

_lib.add_received_bytes.argtypes = [
    POINTER(c_uint8),
    c_uint32,
]

_lib.add_received_bytes.restype = c_int


# ---------------------------------------------------------------------------
# Python API
# ---------------------------------------------------------------------------

# Extra space reserved for protocol framing/CRC.
# Increase this if the protocol grows in the future.
ENCAPSULATION_OVERHEAD = 16


def encapsulate_message(payload: bytes) -> bytes:
    """
    Encode a payload into a protocol frame.

    The C protocol requires the destination buffer to be larger than
    the payload buffer to accommodate framing and CRC.

    Returns:
        bytes: Complete protocol frame ready to send over USB CDC.

    Raises:
        RuntimeError: If the C protocol encoder reports an error.
    """

    payload_size = len(payload)

    # Allocate enough room for the payload plus protocol overhead.
    dest_size = payload_size + ENCAPSULATION_OVERHEAD

    # Destination buffer owned by Python.
    dest = (c_uint8 * dest_size)()

    # Payload buffer owned by Python.
    payload_buffer = (c_uint8 * payload_size).from_buffer_copy(payload)

    status = _lib.encapsulate_message(
        dest,
        dest_size,
        payload_buffer,
        payload_size,
    )

    if status != MessageProtocolStatus.OK:
        raise RuntimeError(f"encapsulate_message() failed with status {status}")

    # The C function has populated dest with the complete frame.
    return bytes(dest)


def add_received_bytes(data: bytes) -> None:
    """
    Feed bytes received from USB CDC into the protocol parser.
    """

    if not data:
        return

    buffer = (c_uint8 * len(data)).from_buffer_copy(data)

    status = _lib.add_received_bytes(
        buffer,
        len(data),
    )

    if status == MessageProtocolStatus.OK:
        return

    if status == MessageProtocolStatus.RING_FULL:
        raise RuntimeError("Protocol receive ring is full")

    raise RuntimeError(f"add_received_bytes() failed with status {status}")


def extract_message(struct) -> bytes | None:
    """
    Extract one complete payload from the protocol receive buffer.

    Returns:
        bytes: Payload if a packet is available.
        None: If there is currently no complete packet.
    """

    # Maximum expected payload size.
    #
    # Ideally this should come from the protocol itself.
    buffer_size = 4096

    payload = (c_uint8 * buffer_size)()

    status = _lib.extract_message(
        ctypes.byref(struct),
        ctypes.sizeof(struct),
    )

    if status == MessageProtocolStatus.OK:
        return True
    elif status == MessageProtocolStatus.NO_PACKET:
        return False
    else:
        raise RuntimeError(f"extract_message() failed with status {status}")
