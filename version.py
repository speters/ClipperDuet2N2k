# SPDX-License-Identifier: MIT

# see https://community.platformio.org/t/how-to-build-got-revision-into-binary-for-version-output/15380/6

import pkg_resources

Import("env")

required_pkgs = {'dulwich'}
installed_pkgs = {pkg.key for pkg in pkg_resources.working_set}
missing_pkgs = required_pkgs - installed_pkgs

if missing_pkgs:
    env.Execute('"$PYTHONEXE" -m pip install dulwich')

import subprocess
import datetime
import os

from contextlib import suppress
from dulwich import porcelain
from dulwich.errors import NotGitRepository

# returns a version description like "git describe --always --dirty" would
def get_git_describe_always_dirty(repo=None):
    repo = repo or os.getcwd()
    with suppress(NotGitRepository):
        revid = None
        with porcelain.open_repo_closing(repo) as r:
            revid = r.head().decode("ascii")
        version = porcelain.describe(repo, abbrev=9)
        # version = revid[0:9] if version[0] == 'g' else version
        status = porcelain.status(repo=repo, untracked_files="no")
        dirtymark = "-dirty" if status.staged["add"] or status.staged["delete"] or status.staged["modify"] or status.unstaged else ""
        # return f"{version}~git:{revid}{dirtymark}"
        return f"{version}{dirtymark}"
    return "unknown"

# construct a pseudo-version string from git describe, which follows format requirements of NMEA2000
def make_n2k_version(git_dirty_describe):
    g = git_dirty_describe
    if g[0] == 'v':
        s = g[1:]
    elif g == "unknown":
        s = "0.0.0"
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
    return t

def get_firmware_specifier_build_flag():
    g =  get_git_describe_always_dirty('.')

    x = datetime.datetime.now()
    d = x.strftime("%Y-%m-%d")

    t = make_n2k_version(g)

    n2kv = t + "\ \(" + d + "\)"

    # TODO:  env.StringifyMacro()
    build_flag = "-D N2K_SOFTWARE_VERSION=\\\"" + n2kv + "\\\" -D GIT_DESCRIBE=\\\"" + g + "\\\""
    print ("Firmware Revision for N2k: " + t)
    print ("git Describe: " + g)
    return (build_flag)

# This is a convenience-function for _me_
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
