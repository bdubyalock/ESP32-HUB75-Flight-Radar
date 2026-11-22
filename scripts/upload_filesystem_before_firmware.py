import subprocess
import sys
import os

Import("env")


def upload_filesystem(source, target, env):
    command = [
        sys.executable,
        "-m",
        "platformio",
        "run",
        "--project-dir",
        env.subst("$PROJECT_DIR"),
        "--environment",
        env.subst("$PIOENV"),
        "--target",
        "uploadfs",
    ]
    upload_port = env.subst("$UPLOAD_PORT")
    if upload_port:
        command.extend(["--upload-port", upload_port])
    subprocess.run(command, check=True)

# Read the custom option from platformio.ini
version = env.GetProjectOption("custom_firmware_version")

# Inject custom option into C++ build flags as a string literal
env.Append(CPPDEFINES=[
    ("FIRMWARE_VERSION", f'\\"{version}\\"')
])

# Generate a JS file in the LittleFS data/ folder
data_dir = env.get("PROJECT_DATA_DIR")
os.makedirs(data_dir, exist_ok=True)
version_js_path = os.path.join(data_dir, "version.js")

new_content = f'const FIRMWARE_VERSION = "{version}";\n'

# Only overwrite the file if the version actually changed (saves flash wear/build time)
current_content = ""
if os.path.exists(version_js_path):
    with open(version_js_path, "r", encoding="utf-8") as f:
        current_content = f.read()

if current_content != new_content:
    with open(version_js_path, "w", encoding="utf-8") as f:
        f.write(new_content)
    print(f"Updated {version_js_path} to version {version}")

# Ensure the filesystem is uploaded before the main firmware
env.AddPreAction("upload", upload_filesystem)
