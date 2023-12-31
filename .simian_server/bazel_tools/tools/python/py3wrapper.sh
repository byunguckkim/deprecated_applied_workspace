#!/bin/sh

# Don't set -e because we don't have robust trapping and printing of errors.
set -u

# We use /bin/sh rather than /bin/bash for portability. See discussion here:
# https://groups.google.com/forum/?nomobile=true#!topic/bazel-dev/4Ql_7eDcLC0
# We do lose the ability to set -o pipefail.

STRICT="1"

if [ "$STRICT" = "1" ]; then
  FAILURE_HEADER="\
Error occurred while attempting to use the default Python toolchain \
(@rules_python//python:autodetecting_toolchain)."
else
  FAILURE_HEADER="\
Error occurred while attempting to use the non-strict autodetecting Python \
toolchain (@rules_python//python:autodetecting_toolchain_nonstrict)."
fi

die() {
  echo "$FAILURE_HEADER" 1>&2
  echo "$1" 1>&2
  exit 1
}

# We use `which` to locate the Python interpreter command on PATH. `command -v`
# is another option, but it doesn't check whether the file it finds has the
# executable bit.
#
# A tricky situation happens when this wrapper is invoked as part of running a
# tool, e.g. passing a py_binary target to `ctx.actions.run()`. Bazel will unset
# the PATH variable. Then the shell will see there's no PATH and initialize its
# own, sometimes without exporting it. This causes `which` to fail and this
# script to think there's no Python interpreter installed. To avoid this we
# explicitly pass PATH to each `which` invocation. We can't just export PATH
# because that would modify the environment seen by the final user Python
# program.
#
# See also:
#
#     https://github.com/bazelbuild/continuous-integration/issues/578
#     https://github.com/bazelbuild/bazel/issues/8414
#     https://github.com/bazelbuild/bazel/issues/8415

# Try the "python3" command name first, then fall back on "python".
PYTHON_BIN="$(PATH="$PATH" which python3 2> /dev/null)"
USED_FALLBACK="0"
if [ -z "${PYTHON_BIN:-}" ]; then
  PYTHON_BIN="$(PATH="$PATH" which python 2>/dev/null)"
  USED_FALLBACK="1"
fi
if [ -z "${PYTHON_BIN:-}" ]; then
  die "Neither 'python3' nor 'python' were found on the target \
platform's PATH, which is:

$PATH

Please ensure an interpreter is available on this platform (and marked \
executable), or else register an appropriate Python toolchain as per the \
documentation for py_runtime_pair \
(https://github.com/bazelbuild/rules_python/blob/master/docs/python.md#py_runtime_pair)."
fi

if [ "$STRICT" = "1" ]; then
  # Verify that we grabbed an interpreter with the right version.
  VERSION_STR="$("$PYTHON_BIN" -V 2>&1)" \
      || die "Could not get interpreter version via '$PYTHON_BIN -V'"
  if ! echo "$VERSION_STR" | grep -q " 3\." ; then
      die "According to '$PYTHON_BIN -V', version is '$VERSION_STR', but we \
need version 3. PATH is:

$PATH

Please ensure an interpreter with version 3 is available on this \
platform as 'python3' or 'python', or else register an appropriate \
Python toolchain as per the documentation for py_runtime_pair \
(https://github.com/bazelbuild/rules_python/blob/master/docs/python.md#py_runtime_pair).

Note that prior to Bazel 0.27, there was no check to ensure that the \
interpreter's version matched the version declared by the target (#4815). If \
your build worked prior to Bazel 0.27, and you're sure your targets do not \
require Python 3, you can opt out of this version check by using the \
non-strict autodetecting toolchain instead of the standard autodetecting \
toolchain. This can be done by passing the flag \
\`--extra_toolchains=@rules_python//python:autodetecting_toolchain_nonstrict\` \
on the command line or adding it to your bazelrc."
  fi
fi

exec "$PYTHON_BIN" "-S" "$@"
