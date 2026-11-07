import subprocess
import sys

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


env.AddPreAction("upload", upload_filesystem)
