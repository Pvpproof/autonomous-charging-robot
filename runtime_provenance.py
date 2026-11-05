"""Read-only process provenance. Used to invalidate a prepared motion lease."""
from pathlib import Path
import hashlib
import os


def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def process_identity(kind):
    found=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            args=[x.decode(errors='replace') for x in (p/'cmdline').read_bytes().split(b'\0') if x]
            if not args:continue
            name=Path(args[0]).name
            wanted=(kind=='driver' and name=='ychd_robot_node') or (
                kind=='localizer' and name.startswith('component_container') and any('nav2_container' in x for x in args))
            if not wanted:continue
            stat=(p/'stat').read_text();tail=stat[stat.rfind(')')+2:].split()
            found.append(dict(pid=int(p.name),start_ticks=tail[19],executable=os.readlink(p/'exe')))
        except (OSError,PermissionError):continue
    return found[0] if len(found)==1 else None


def panel_identity(epoch,arbiter_status):
    return dict(boot_id=boot_id(),panel_epoch=epoch,
                arbiter_instance=arbiter_status.get('instance_id'),
                driver_identity=arbiter_status.get('driver_identity'),
                localizer_identity=arbiter_status.get('localizer_identity'))
