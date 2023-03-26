# SPDX-License-Identifier: MIT
import subprocess
import datetime
import os

Import("env")

def get_firmware_specifier_build_flag():
    try:
        ret = subprocess.run(["git", "describe", "--always", "--dirty"], stdout=subprocess.PIPE, text=True)
    except FileNotFoundError:
        # git not found
        return ("")
    g =  ret.stdout.strip()
    if g[0] == 'v':
        s = g[1:]
    else:
        s = g

    p = s.split('-')

    n = p[0].split('.')
    if len(n) != 3:
         raise RuntimeError("versioning scheme error")
    v = ".".join([str(int(x)) for x in n][0:3])

    t = ""
    if p[-1] == "dirty":
        t = "0." + v
    else:
        t = v
        if len(p) == 3:
            t += "." + p[1]
        else:
            t += "." + "0"

    x = datetime.datetime.now()
    d = x.strftime("%Y-%m-%d")

    n2kv = t + "\ \(" + d + "\)"

    # TODO:  env.StringifyMacro()
    build_flag = "-D N2K_SOFTWARE_VERSION=\\\"" + n2kv + "\\\" -D GIT_DESCRIBE=\\\"" + g + "\\\""
    print ("Firmware Revision: " + t)
    print ("git Describe: " + g)
    return (build_flag)

def before_upload(source, target, env):
    # killall minicom
    if (os.environ.get("USER")=="soenke"):
        try:
            print("killall minicom")
            subprocess.run(["killall", "-u", os.environ.get("USER"), "minicom"], stdout=subprocess.PIPE, text=True)
        except FileNotFoundError:
            # killall not found
            return

env.Append(
    BUILD_FLAGS=[get_firmware_specifier_build_flag()]
)

env.AddPreAction("upload", before_upload)
