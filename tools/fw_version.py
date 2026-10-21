# PlatformIO pre-build script: puts the git commit into the firmware as FW_VERSION, e.g. "a1b2c3d+ robot-test"
# ("+" = Main/ had uncommitted changes when it was built). Printed at start-up and shown on the LCD for a moment,
# so every robot log says which firmware ran.
Import("env")
import subprocess

def git(*args):
    try:
        return subprocess.check_output(["git"] + list(args), cwd=env.subst("$PROJECT_DIR"),
                                       stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return ""

commit = git("rev-parse", "--short", "HEAD") or "unknown"
branch = git("rev-parse", "--abbrev-ref", "HEAD")
dirty = "+" if git("status", "--porcelain", "--", "Main") else ""
env.Append(CPPDEFINES=[("FW_VERSION", env.StringifyMacro("%s%s %s" % (commit, dirty, branch)))])
