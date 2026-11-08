"""Reconstructed publication helper (2026-11-08), not recovered field code.

Only atomic(path, data) is evidenced by insert_execute.py. No motion API exists.
"""
import json
import os
from pathlib import Path
import tempfile

RECONSTRUCTED = True

def atomic(path, data):
    """Replace one JSON file; serialization failure preserves the previous file."""
    path = Path(path)
    payload = json.dumps(data, ensure_ascii=False, allow_nan=False, indent=2) + '\n'
    # Require the caller's already selected directory; do not discover/create field roots.
    fd, temporary = tempfile.mkstemp(prefix='.' + path.name + '.', suffix='.tmp', dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)

def __getattr__(name):
    raise AttributeError(f'insertion_runtime reconstruction supports atomic only; {name!r} is not recovered')

if __name__ == '__main__':
    raise SystemExit('Reconstructed file helper only; no insertion execution entry point')
