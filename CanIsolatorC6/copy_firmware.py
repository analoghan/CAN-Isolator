# PlatformIO post-build step: copy the app image to ../Firmware/CanIsolatorC6-<version>.bin,
# the file the config tool's "Update firmware" takes.
import os, re, shutil
Import("env")

def copy_bin(source, target, env):
    proj = env.subst("$PROJECT_DIR")
    src = open(os.path.join(proj, "CanIsolatorC6.ino"), encoding="utf-8").read()
    m = re.search(r'#define\s+FW_VERSION_STR\s+"([^"]+)"', src)
    ver = m.group(1) if m else "unknown"
    out_dir = os.path.normpath(os.path.join(proj, "..", "Firmware"))
    os.makedirs(out_dir, exist_ok=True)
    dst = os.path.join(out_dir, "CanIsolatorC6-%s.bin" % ver)
    shutil.copyfile(str(target[0]), dst)
    print("Firmware for the config tool: %s" % dst)

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", copy_bin)
