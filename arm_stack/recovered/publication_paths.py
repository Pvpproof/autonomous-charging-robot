"""Explicit local paths for historical sources; never discovers robot data."""
import os
from pathlib import Path

def required_path(name, *, file=False):
    value=os.environ.get(name, '')
    if not value:
        raise RuntimeError(f'{name} must be set explicitly; no field data is bundled')
    path=Path(value)
    if not path.is_absolute():
        raise ValueError(f'{name} must be an absolute path')
    if not (path.is_file() if file else path.is_dir()):
        raise ValueError(f'{name} does not name an existing '+('file' if file else 'directory'))
    return path

def vision_path(relative=''):
    child=Path(relative)
    if child.is_absolute() or '..' in child.parts:
        raise ValueError('vision path must remain under the explicit root')
    return required_path('CHARGING_SOCKET_VISION_ROOT') / child

def hand_eye_path():
    return required_path('CHARGING_HAND_EYE_FILE', file=True)
