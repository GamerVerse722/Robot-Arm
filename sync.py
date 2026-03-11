#!/usr/bin/env python3
import shutil
import subprocess
from pathlib import Path

# === CONFIG ===
GITHUB_URL = "https://github.com/GamerVerse722/GamersForge.git"
LIB_NAME = "gamers-forge"
LIB_LOCAL_DIR = Path(".gamers-forge-lib")  # hidden folder to store repo clone


def run_cmd(cmd):
    """Run shell command and handle errors gracefully."""
    try:
        subprocess.run(cmd, shell=True, check=True)
    except subprocess.CalledProcessError:
        print(f"❌ Command failed: {cmd}")
        exit(1)


def clone_or_update():
    """Clone or pull the GamersForge library."""
    if not LIB_LOCAL_DIR.exists():
        print("📦 Cloning GamersForge library...")
        run_cmd(f"git clone --depth=1 {GITHUB_URL} {LIB_LOCAL_DIR}")
    else:
        print("🔄 Updating existing GamersForge library...")
        run_cmd(f"git -C {LIB_LOCAL_DIR} pull origin main")


def copytree(src, dest):
    """Copy and replace folder."""
    if dest.exists():
        shutil.rmtree(dest)
    shutil.copytree(src, dest)


def sync_library():
    """Copy include/ and src/ from the GamersForge library."""
    include_src = LIB_LOCAL_DIR / "include" / LIB_NAME
    src_src = LIB_LOCAL_DIR / "src" / LIB_NAME

    include_dest = Path("include") / LIB_NAME
    src_dest = Path("src") / LIB_NAME

    print("\n🚀 Syncing library files...")

    copytree(include_src, include_dest)
    copytree(src_src, src_dest)

    print("✅ Library synced successfully!")


def main():
    print("=== GamersForge Library Sync ===\n")

    # Ensure we're in a PROS project (must have project.pros)
    if not Path("project.pros").exists():
        print("❌ Error: This script must be run inside a PROS project folder.")
        exit(1)

    clone_or_update()
    sync_library()

    print("\n🎉 Done! Your GamersForge library is now up to date.")


if __name__ == "__main__":
    main()
