#!/usr/bin/env python3

import os
import time
import tempfile
import shutil
import subprocess
from pathlib import Path
from datetime import datetime
import hashlib

# ===== Config (ASCII only) =====
SERVER_IP   = "172.20.11.20"
SHARE_NAME  = "\u5168\u793e\u516c\u958b\u60c5\u5831"  # "全社公開情報"
DOMAIN      = "KUMA-DOM"
USER_ID     = "13045"
PASSWORD    = "<<<PUT_DOMAIN_PASSWORD_HERE>>>" # IMPORTANT!!! PUT DOMAIN PASSWORD HERE!!!!
REMOTE_PATH = (
    "00_\u90e8\u9580\u5225\u60c5\u5831/"
    "90_\u672c\u793e\u5de5\u5834/"
    "\u30d1\u30c8\u30e9\u30a4\u30c8/"
    "\u4f5c\u696d\u8a18\u9332/"
    "202509"
)  # "00_部門別情報/90_本社工場/パトライト/作業記録/202509"

# Destination under the project directory
DEST_DIR    = Path(__file__).resolve().parent / "data" / "hinmoku"
INTERVAL_SEC = 60
SMBCLIENT    = "smbclient"
# ===============================

def log(msg: str) -> None:
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{now}] {msg}", flush=True)

def run(cmd, **kwargs) -> subprocess.CompletedProcess:
    kwargs.setdefault("check", True)
    kwargs.setdefault("text", True)
    return subprocess.run(cmd, **kwargs)

def make_credentials_file(user: str, password: str, domain: str) -> str:
    fd, path = tempfile.mkstemp(prefix="smbcred_", text=True)
    os.close(fd)
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"username={user}\npassword={password}\ndomain={domain}\n")
    os.chmod(path, 0o600)
    return path

def file_sha1(p: Path, chunk=1024*1024) -> str:
    h = hashlib.sha1()
    with open(p, 'rb') as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()

def copy_newer_only(src_root: Path, dst_root: Path) -> None:
    for src_dir, _, files in os.walk(src_root):
        sdir = Path(src_dir)
        rel  = sdir.relative_to(src_root)
        ddir = dst_root / rel
        ddir.mkdir(parents=True, exist_ok=True)

        for fn in files:
            sfile = sdir / fn
            dfile = ddir / fn

            if not dfile.exists():
                shutil.copy2(sfile, dfile)
                log(f"[NEW ] {dfile}")
                continue

            if sfile.stat().st_size != dfile.stat().st_size:
                shutil.copy2(sfile, dfile)
                log(f"[UPDATE(size)] {dfile}")
                continue

            if file_sha1(sfile) != file_sha1(dfile):
                shutil.copy2(sfile, dfile)
                log(f"[UPDATE(hash)] {dfile}")
            else:
                log(f"[SKIP] {dfile}")


def fetch_to_staging(staging: Path) -> None:
    staging.mkdir(parents=True, exist_ok=True)
    cred_path = make_credentials_file(USER_ID, PASSWORD, DOMAIN)
    try:
        smb_cmd = f'lcd "{staging}"; cd "{REMOTE_PATH}"; recurse ON; prompt OFF; mget *'
        cmd = [SMBCLIENT, f"//{SERVER_IP}/{SHARE_NAME}", "-A", cred_path, "-c", smb_cmd]
        run(cmd)
    finally:
        # Best-effort secure delete
        try:
            with open(cred_path, "w", encoding="utf-8") as f:
                f.write("\0" * 256)
        except Exception:
            pass
        try:
            os.remove(cred_path)
        except FileNotFoundError:
            pass

def one_cycle() -> None:
    DEST_DIR.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="hinmoku_stage_") as tmpdir:
        staging = Path(tmpdir)
        log("Downloading to staging...")
        fetch_to_staging(staging)
        log("Applying diffs (copy newer only)...")
        copy_newer_only(staging, DEST_DIR)
    log("Sync done.")

def main() -> None:
    # Sanity: smbclient existence
    try:
        run([SMBCLIENT, "-V"], capture_output=True)
    except Exception:
        print("smbclient not found. Install it with: sudo apt install smbclient", flush=True)
        raise SystemExit(1)

    log(f"Start watching //{SERVER_IP}/{SHARE_NAME}/{REMOTE_PATH} -> {DEST_DIR}")
    while True:
        try:
            one_cycle()
        except subprocess.CalledProcessError as e:
            log(f"ERROR: command failed (rc={e.returncode})")
            log(f"cmd: {e.cmd}")
            if e.stdout:
                log(f"stdout: {e.stdout}")
            if e.stderr:
                log(f"stderr: {e.stderr}")
        except Exception as e:
            log(f"ERROR: {e}")
        time.sleep(INTERVAL_SEC)

if __name__ == "__main__":
    main()
