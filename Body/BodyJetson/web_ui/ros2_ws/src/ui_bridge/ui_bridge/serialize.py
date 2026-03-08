from typing import Any, Dict

def to_jsonable(obj: Any) -> Any:
    if obj is None or isinstance(obj, (bool, int, float, str)):
        return obj

    if isinstance(obj, (bytes, bytearray)):
        # Avoid huge payloads
        return {"_bytes_len": len(obj)}

    # Handle numpy arrays / array-like ROS fixed arrays
    if hasattr(obj, "tolist") and callable(obj.tolist):
        return to_jsonable(obj.tolist())

    if isinstance(obj, (list, tuple)):
        return [to_jsonable(x) for x in obj]

    if isinstance(obj, dict):
        return {str(k): to_jsonable(v) for k, v in obj.items()}

    if hasattr(obj, "get_fields_and_field_types"):
        out: Dict[str, Any] = {}
        for field in obj.get_fields_and_field_types().keys():
            try:
                out[field] = to_jsonable(getattr(obj, field))
            except Exception:
                out[field] = None
        return out

    return str(obj)