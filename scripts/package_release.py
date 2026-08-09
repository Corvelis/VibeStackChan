Import("env")

import hashlib
from pathlib import Path
import subprocess


def package_release(source, target, env):
    project_dir = Path(env.subst("$PROJECT_DIR"))
    build_dir = Path(env.subst("$BUILD_DIR"))
    version = env.GetProjectOption("custom_firmware_version")
    output_dir = project_dir / "dist"
    output_dir.mkdir(parents=True, exist_ok=True)

    platform = env.PioPlatform()
    esptool = Path(platform.get_package_dir("tool-esptoolpy")) / "esptool.py"
    framework = Path(
        platform.get_package_dir("framework-arduinoespressif32")
    )
    python = env.subst("$PYTHONEXE")

    output_name = f"VibeStackChan-v{version}-CoreS3.factory.bin"
    output_path = output_dir / output_name
    command = [
        python,
        str(esptool),
        "--chip",
        "esp32s3",
        "merge_bin",
        "--output",
        str(output_path),
        "0x0",
        str(build_dir / "bootloader.bin"),
        "0x8000",
        str(build_dir / "partitions.bin"),
        "0xe000",
        str(framework / "tools" / "partitions" / "boot_app0.bin"),
        "0x10000",
        str(build_dir / "firmware.bin"),
    ]
    subprocess.run(command, check=True)

    digest = hashlib.sha256(output_path.read_bytes()).hexdigest()
    checksum_path = output_dir / f"{output_name}.sha256"
    checksum_path.write_text(f"{digest}  {output_name}\n", encoding="utf-8")
    print(f"[release] created {output_path}")
    print(f"[release] created {checksum_path}")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", package_release)
