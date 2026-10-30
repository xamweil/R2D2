import array
from typing import Any, Dict, Tuple

import orjson

# Field names per ROS message class: get_fields_and_field_types() copies a dict
# on every call
_MSG_FIELDS: Dict[type, Tuple[str, ...]] = {}


def _default(obj: Any) -> Any:
    """Called by orjson for every value it can't serialize natively."""
    fields = _MSG_FIELDS.get(type(obj))
    if fields is None and hasattr(obj, "get_fields_and_field_types"):
        fields = tuple(obj.get_fields_and_field_types().keys())
        _MSG_FIELDS[type(obj)] = fields

    if fields is not None:
        return {field: getattr(obj, field) for field in fields}

    # Avoid huge payloads: uint8[] fields (e.g. JPEG data) arrive as array('B')
    if isinstance(obj, (bytes, bytearray)) or (
        isinstance(obj, array.array) and obj.typecode == "B"
    ):
        return {"_bytes_len": len(obj)}

    # Sequence fields arrive as array.array
    if hasattr(obj, "tolist") and callable(obj.tolist):
        return obj.tolist()

    return str(obj)


def dumps(obj: Any) -> bytes:
    return orjson.dumps(obj, default=_default, option=orjson.OPT_SERIALIZE_NUMPY)


def dumps_text(obj: Any) -> str:
    return dumps(obj).decode("utf-8")
