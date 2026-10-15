#!/usr/bin/env python3
"""
Builds Linux installation packages from a verified baseline ELF for StructuraX:
1. dist/linux/structurax-1.0.0-linux-x86_64.tar.gz (tarball with install.sh / uninstall.sh)
2. dist/linux/structurax_1.0.0_amd64.deb (Debian / Ubuntu / Linux Mint / Pop!_OS .deb installer)
3. dist/linux/StructuraX-1.0.0-linux-x86_64.sh (self-extracting installer; target runtime acceptance is separate)
4. Also packages Windows x86_64 (.zip) if target/x86_64-pc-windows-gnu/release/structurax.exe is present.
"""
import base64
import hashlib
import io
import os
import shutil
import stat
import tarfile
import time
import zipfile
from pathlib import Path

from make_linux_portable import check_glibc_requirement

VERSION = "1.0.0"
ROOT = Path(__file__).resolve().parent.parent


INSTALL_SH = """#!/usr/bin/env bash
# Universal Linux Installer for StructuraX v1.0.0
# Requires the packaged binary's GLIBC baseline and desktop runtime dependencies.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ "$(id -u)" -eq 0 ]; then
    PREFIX="${PREFIX:-/usr/local}"
else
    PREFIX="${PREFIX:-$HOME/.local}"
fi

while [ $# -gt 0 ]; do
    case "$1" in
        --prefix)
            PREFIX="$2"
            shift 2
            ;;
        --system)
            PREFIX="/usr/local"
            shift
            ;;
        --user)
            PREFIX="$HOME/.local"
            shift
            ;;
        *)
            echo "Usage: $0 [--user | --system | --prefix <dir>]"
            exit 1
            ;;
    esac
done

echo "==> Installing StructuraX v1.0.0 to ${PREFIX}..."
mkdir -p "${PREFIX}/bin"
mkdir -p "${PREFIX}/share/structurax/sections"
mkdir -p "${PREFIX}/share/applications"
mkdir -p "${PREFIX}/share/licenses/structurax"
install -m 0644 "${SCRIPT_DIR}/LICENSE" "${PREFIX}/share/licenses/structurax/LICENSE"
mkdir -p "${PREFIX}/share/icons/hicolor/scalable/apps"

install -m 0755 "${SCRIPT_DIR}/bin/structurax" "${PREFIX}/bin/structurax"
install -m 0644 "${SCRIPT_DIR}/share/applications/structurax.desktop" "${PREFIX}/share/applications/structurax.desktop"
install -m 0644 "${SCRIPT_DIR}/share/icons/hicolor/scalable/apps/structurax.svg" "${PREFIX}/share/icons/hicolor/scalable/apps/structurax.svg"

if compgen -G "${SCRIPT_DIR}/share/structurax/sections/*.pro" > /dev/null; then
    install -m 0644 "${SCRIPT_DIR}/share/structurax/sections/"*.pro "${PREFIX}/share/structurax/sections/"
fi

# Update desktop database if available
if command -v update-desktop-database >/dev/null 2>&1; then
    update-desktop-database "${PREFIX}/share/applications" 2>/dev/null || true
fi

echo "==> StructuraX v1.0.0 installed successfully!"
echo "    Binary:   ${PREFIX}/bin/structurax"
echo "    Sections: ${PREFIX}/share/structurax/sections/"
echo "    Run GUI:  ${PREFIX}/bin/structurax"
echo "    Verify:   ${PREFIX}/bin/structurax verify"
"""

UNINSTALL_SH = """#!/usr/bin/env bash
set -euo pipefail
if [ "$(id -u)" -eq 0 ]; then
    PREFIX="${PREFIX:-/usr/local}"
else
    PREFIX="${PREFIX:-$HOME/.local}"
fi
rm -f "${PREFIX}/bin/structurax"
rm -rf "${PREFIX}/share/structurax"
rm -f "${PREFIX}/share/applications/structurax.desktop"
rm -f "${PREFIX}/share/icons/hicolor/scalable/apps/structurax.svg"
echo "==> Uninstalled StructuraX from ${PREFIX}."
"""


def add_bytes_to_tar(
    tf: tarfile.TarFile, arcname: str, data: bytes, mode: int = 0o644
) -> None:
    ti = tarfile.TarInfo(name=arcname)
    ti.size = len(data)
    ti.mtime = int(time.time())
    ti.mode = mode
    ti.uid = 0
    ti.gid = 0
    ti.uname = "root"
    ti.gname = "root"
    tf.addfile(ti, io.BytesIO(data))


def make_ar_entry(name: str, data: bytes, mode: int = 0o100644) -> bytes:
    mode_str = f"{mode:o}"
    header = (
        f"{name:<16}"
        f"{int(time.time()):<12}"
        f"{0:<6}"
        f"{0:<6}"
        f"{mode_str:<8}"
        f"{len(data):<10}"
        "`\n"
    ).encode("ascii")
    assert len(header) == 60
    pad = b"\n" if len(data) % 2 == 1 else b""
    return header + data + pad


def build_deb_package(
    out_deb: Path,
    bin_bytes: bytes,
    desktop_bytes: bytes,
    icon_bytes: bytes,
    pro_files: list[Path],
) -> None:
    # 1. Build data.tar.gz
    data_buf = io.BytesIO()
    installed_bytes = len(bin_bytes) + len(desktop_bytes) + len(icon_bytes)
    with tarfile.open(fileobj=data_buf, mode="w:gz") as dtar:
        add_bytes_to_tar(dtar, "./usr/bin/structurax", bin_bytes, mode=0o755)
        add_bytes_to_tar(dtar, "./usr/share/doc/structurax/copyright", (ROOT / "LICENSE").read_bytes())
        add_bytes_to_tar(
            dtar,
            "./usr/share/applications/structurax.desktop",
            desktop_bytes,
            mode=0o644,
        )
        add_bytes_to_tar(
            dtar,
            "./usr/share/icons/hicolor/scalable/apps/structurax.svg",
            icon_bytes,
            mode=0o644,
        )
        for pro in pro_files:
            pbytes = pro.read_bytes()
            installed_bytes += len(pbytes)
            add_bytes_to_tar(
                dtar,
                f"./usr/share/structurax/sections/{pro.name}",
                pbytes,
                mode=0o644,
            )
    data_tar_gz = data_buf.getvalue()

    # 2. Build control.tar.gz
    installed_kb = max(1, installed_bytes // 1024)
    control_text = f"""Package: structurax
Version: {VERSION}
Section: science
Priority: optional
Architecture: amd64
Installed-Size: {installed_kb}
Depends: libc6 (>= 2.35), libgcc-s1
Recommends: libx11-6, libgl1
Maintainer: StructuraX contributors <5492505+firesand@users.noreply.github.com>
Homepage: https://github.com/firesand/StructuraX
Description: 3D Finite Element Structural Analysis & Code-Based Member Design Suite
 StructuraX is a native 3D structural mechanics and finite element engineering
 application supporting spatial frames, catenary cables, membrane/plate shells,
 8-node 3D solids, nonlinear links/dampers, modal/buckling/spectrum/time-history
 dynamics, AISC 360-10 & Eurocode 3 steel design, and ACI 318-14 & Eurocode 2
 concrete member design.
"""
    ctrl_buf = io.BytesIO()
    with tarfile.open(fileobj=ctrl_buf, mode="w:gz") as ctar:
        add_bytes_to_tar(ctar, "./control", control_text.encode("utf-8"), mode=0o644)
    control_tar_gz = ctrl_buf.getvalue()

    # 3. Assemble standard Debian `ar` archive
    ar_bytes = bytearray(b"!<arch>\n")
    ar_bytes.extend(make_ar_entry("debian-binary", b"2.0\n"))
    ar_bytes.extend(make_ar_entry("control.tar.gz", control_tar_gz))
    ar_bytes.extend(make_ar_entry("data.tar.gz", data_tar_gz))
    out_deb.write_bytes(ar_bytes)


def build_self_extracting_sh(out_sh: Path, tar_gz_path: Path) -> None:
    b64_payload = base64.b64encode(tar_gz_path.read_bytes()).decode("ascii")
    script = f"""#!/usr/bin/env bash
# StructuraX v{VERSION} Universal Cross-Distro Linux Self-Extracting Installer
# Compatible with Ubuntu, Debian, Fedora, Arch Linux, RHEL/Rocky/Alma, openSUSE, Linux Mint, and Gentoo.
set -euo pipefail

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${{TMP_DIR}}"' EXIT

echo "==> Extracting StructuraX v{VERSION} installer payload..."
base64 -d << 'EOF_STRUCTURAX_PAYLOAD' | tar -xzf - -C "${{TMP_DIR}}"
{b64_payload}
EOF_STRUCTURAX_PAYLOAD

bash "${{TMP_DIR}}/structurax-{VERSION}-linux-x86_64/install.sh" "$@"
"""
    out_sh.write_text(script, encoding="utf-8")
    out_sh.chmod(out_sh.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def main() -> None:
    linux_bin = ROOT / "target" / "release" / "structurax"
    if not linux_bin.is_file():
        raise SystemExit(
            "Error: release binary missing; run cargo build --locked --release --bin structurax"
        )

    dist_linux = ROOT / "dist" / "linux"
    dist_linux.mkdir(parents=True, exist_ok=True)

    # Inspect the original binary before producing any package. Never rewrite ABI tags.
    check_glibc_requirement(linux_bin)

    # Stage the unchanged binary built against the supported baseline.
    staged_bin = dist_linux / "structurax"
    shutil.copy2(linux_bin, staged_bin)
    staged_bin.chmod(0o755)

    bin_bytes = staged_bin.read_bytes()
    desktop_bytes = (ROOT / "packaging" / "linux" / "structurax.desktop").read_bytes()
    icon_bytes = (ROOT / "packaging" / "linux" / "structurax.svg").read_bytes()

    # Vendor section libraries are local reference inputs, not release payloads.
    pro_files: list[Path] = []

    # 1. Build portable .tar.gz archive
    tar_name = f"structurax-{VERSION}-linux-x86_64"
    tar_path = dist_linux / f"{tar_name}.tar.gz"
    with tarfile.open(tar_path, "w:gz") as tf:
        add_bytes_to_tar(tf, f"{tar_name}/bin/structurax", bin_bytes, mode=0o755)
        add_bytes_to_tar(tf, f"{tar_name}/LICENSE", (ROOT / "LICENSE").read_bytes())
        add_bytes_to_tar(
            tf,
            f"{tar_name}/share/applications/structurax.desktop",
            desktop_bytes,
            mode=0o644,
        )
        add_bytes_to_tar(
            tf,
            f"{tar_name}/share/icons/hicolor/scalable/apps/structurax.svg",
            icon_bytes,
            mode=0o644,
        )
        add_bytes_to_tar(
            tf,
            f"{tar_name}/install.sh",
            INSTALL_SH.encode("utf-8"),
            mode=0o755,
        )
        add_bytes_to_tar(
            tf,
            f"{tar_name}/uninstall.sh",
            UNINSTALL_SH.encode("utf-8"),
            mode=0o755,
        )
        for pro in pro_files:
            add_bytes_to_tar(
                tf,
                f"{tar_name}/share/structurax/sections/{pro.name}",
                pro.read_bytes(),
                mode=0o644,
            )
    print(f"[dist] Created Linux tarball: {tar_path}")

    # 2. Build Debian/Ubuntu/Mint .deb package
    deb_path = dist_linux / f"structurax_{VERSION}_amd64.deb"
    build_deb_package(deb_path, bin_bytes, desktop_bytes, icon_bytes, pro_files)
    print(f"[dist] Created Debian/Ubuntu/Mint package:   {deb_path}")

    # 3. Build universal self-extracting .sh installer
    sh_path = dist_linux / f"StructuraX-{VERSION}-linux-x86_64.sh"
    build_self_extracting_sh(sh_path, tar_path)
    print(f"[dist] Created Linux .sh installer: {sh_path}")

    # 4. Copy Arch PKGBUILD and Fedora/RHEL/openSUSE RPM spec into dist/linux/
    shutil.copy2(ROOT / "packaging" / "linux" / "PKGBUILD", dist_linux / "PKGBUILD")
    shutil.copy2(
        ROOT / "packaging" / "linux" / "structurax.spec",
        dist_linux / "structurax.spec",
    )

    # 5. Check if Windows cross-compiled binary exists and package it
    for win_target in ["x86_64-pc-windows-gnu", "x86_64-pc-windows-msvc"]:
        win_exe = ROOT / "target" / win_target / "release" / "structurax.exe"
        if win_exe.is_file():
            dist_win = ROOT / "dist" / "windows"
            dist_win.mkdir(parents=True, exist_ok=True)
            shutil.copy2(win_exe, dist_win / "structurax.exe")
            zip_path = dist_win / f"StructuraX-{VERSION}-windows-x86_64.zip"
            with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
                zf.write(win_exe, "StructuraX/structurax.exe")
                for pro in pro_files:
                    zf.write(pro, f"StructuraX/sections/{pro.name}")
            print(f"[dist] Created Windows x64 archive:          {zip_path}")

    # 6. Write SHA256SUMS manifest
    checksums = []
    for p in sorted((ROOT / "dist").rglob("*")):
        if p.is_file() and p.name != "SHA256SUMS.txt":
            h = hashlib.sha256(p.read_bytes()).hexdigest()
            rel = p.relative_to(ROOT / "dist")
            checksums.append(f"{h}  {rel}")
    (ROOT / "dist" / "SHA256SUMS.txt").write_text(
        "\n".join(checksums) + "\n", encoding="utf-8"
    )
    print(f"[dist] Wrote checksum manifest:              {ROOT / 'dist' / 'SHA256SUMS.txt'}")


if __name__ == "__main__":
    main()
