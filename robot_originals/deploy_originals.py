#!/usr/bin/env python3
"""Copy verified originals into this robot's layout. Default is read-only preview.
Does not install dependencies, build workspaces, start services or execute motion.
"""
import argparse,datetime,hashlib,json,shutil
from pathlib import Path

def manifest_files(bundle):
    rows=json.loads((bundle/'source-manifest.json').read_text())['files']
    for row in rows:
        name=Path(row['path'])
        if name.is_absolute() or '..' in name.parts:raise ValueError('Invalid manifest path')
        p=bundle/name
        if hashlib.sha256(p.read_bytes()).hexdigest()!=row['sha256']:raise ValueError('Source hash mismatch: '+str(name))
    return rows

def plan(bundle,target):
    rows=manifest_files(bundle);result=[]
    for row in rows:
        dest=target/row['path']
        if dest.is_symlink() or any(p.is_symlink() for p in dest.parents if p!=target.parent):raise ValueError('Symlink target needs manual review: '+str(dest))
        state='same' if dest.is_file() and hashlib.sha256(dest.read_bytes()).hexdigest()==row['sha256'] else 'replace' if dest.exists() else 'add'
        result.append((row,state))
    return result

def restore(bundle,target,apply=False):
    entries=plan(bundle,target)
    if not apply:return {s:sum(state==s for _,state in entries) for s in ('same','replace','add')}
    backup=target/'.charging-source-backups'/datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    for row,state in entries:
        if state=='same':continue
        src=bundle/row['path'];dest=target/row['path']
        if state=='replace':
            if not dest.is_file():raise ValueError('Refuse replacing non-file: '+str(dest))
            save=backup/row['path'];save.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(dest,save)
        dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dest)
        if dest.suffix=='.sh':dest.chmod(dest.stat().st_mode|0o100)
        if hashlib.sha256(dest.read_bytes()).hexdigest()!=row['sha256']:raise ValueError('Copied hash mismatch')
    return {'restored':len(entries),'backup':str(backup),'services_started':False}

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--target-home',type=Path,default=Path('/home/nvidia'));parser.add_argument('--apply',action='store_true');args=parser.parse_args()
    if not args.target_home.is_absolute():parser.error('target home must be absolute')
    print(json.dumps(restore(Path(__file__).resolve().parent,args.target_home,args.apply),indent=2))
if __name__=='__main__':main()
