#!/usr/bin/env bash
# Build & Package StructuraX for macOS Apple Silicon (aarch64-apple-darwin)
# Produces:
#   - dist/macos/StructuraX.app (macOS Application Bundle)
#   - dist/macos/StructuraX-1.0.0-macos-arm64.tar.gz (Portable Archive)
#   - dist/macos/StructuraX-1.0.0-macos-arm64.dmg (macOS Disk Image Installer)

set -euo pipefail

VERSION="1.0.0"
TARGET="aarch64-apple-darwin"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

echo "==> Building StructuraX v${VERSION} for macOS Apple Silicon (${TARGET})..."
cargo build --locked --release --target "${TARGET}" --bin structurax

APP_BUNDLE="dist/macos/StructuraX.app"
rm -rf "${APP_BUNDLE}"
mkdir -p "${APP_BUNDLE}/Contents/MacOS"
mkdir -p "${APP_BUNDLE}/Contents/Resources/sections"

cp "target/${TARGET}/release/structurax" "${APP_BUNDLE}/Contents/MacOS/structurax"
chmod 755 "${APP_BUNDLE}/Contents/MacOS/structurax"
cp "packaging/macos/Info.plist" "${APP_BUNDLE}/Contents/Info.plist"
cp "packaging/linux/structurax.svg" "${APP_BUNDLE}/Contents/Resources/structurax.svg"

# Vendor section libraries are not distributed.
cp LICENSE "${APP_BUNDLE}/Contents/Resources/LICENSE"

# Ad-hoc signing only; this is not Developer ID signing or notarization.
if command -v codesign >/dev/null 2>&1; then
    codesign --force --deep --sign - "${APP_BUNDLE}"
fi

tar -czf "dist/macos/StructuraX-${VERSION}-macos-arm64.tar.gz" -C dist/macos StructuraX.app
echo "==> Created macOS Apple Silicon App Bundle archive: dist/macos/StructuraX-${VERSION}-macos-arm64.tar.gz"

if command -v hdiutil >/dev/null 2>&1; then
    DMG_STAGE="dist/macos/dmg_stage"
    rm -rf "${DMG_STAGE}"
    mkdir -p "${DMG_STAGE}"
    cp -R "${APP_BUNDLE}" "${DMG_STAGE}/"
    ln -s /Applications "${DMG_STAGE}/Applications"
    hdiutil create -volname "StructuraX ${VERSION}" \
        -srcfolder "${DMG_STAGE}" \
        -ov -format UDZO \
        "dist/macos/StructuraX-${VERSION}-macos-arm64.dmg"
    rm -rf "${DMG_STAGE}"
    echo "==> Created macOS Apple Silicon DMG Installer: dist/macos/StructuraX-${VERSION}-macos-arm64.dmg"
fi

