"""Early enabling of solibs.

solibs (or .so files) are typically included as data dependencies of a
py_binary, causing them to show up in //_solib_k8/<module_path>/...
This isn't part of the normal PYTHONPATH, so python can't always find them,
but it also isn't part of LD_LIBRARY_PATH, which means if one depends on
another, then it can't find it either.

By adding each folder to both PYTHONPATH and LD_LIBRARY_PATH, we can enable the
detection and loading of them by later modules. However, we cannot modify
LD_LIBRARY_PATH after starting, so we have to relaunch the binary with it set in
the environment to enable dl to load it correctly.
"""
try:
    from tools.py import resources as _resources
except ImportError:
    resources = None
else:
    resources = _resources

try:
    from subpar.runtime import support as subpar_support

    HAS_SUBPAR = True
except ImportError:
    subpar_support = None
    HAS_SUBPAR = False

import imp  # noqa: F401
import os
import sys
import zipfile

# This can change when the CROSSTOOL changes!
SOLIB_CPU = "_solib_k8"


def is_on_windows():
    return os.name == "nt"


def _in_par():
    return HAS_SUBPAR and hasattr(sys.modules["__main__"], "__loader__")


def setup(import_roots, zip_safe, remove):
    global resources

    # Must 'import site' because it does a few things:
    #  * make sys.path absolute and unique
    #  * make pkg_resources available
    # If this is done after subpar.support.setup(), it makes the import_roots from
    # inside the par file absolute, which conflicts with the par file's notion of
    # its own path (which is relative). We cannot change that (by making
    # __main__.__file__ absolute) because the loader's notion is also absolute and
    # at this point we're messing with too many things.

    original_modules = set(sys.modules.keys())
    import site  # noqa: F401

    # site will import a bunch of modules in an odd way, causing certain system
    # libraries to be made available prior to the par's versions even though the
    # par's paths are in sys.path first. For instance, disabling this will cause
    # the system's google.protobuf to override the par's google.protobuf module,
    # even though if you investigate sys.path, the par's module shows up first.
    # This is because the google module is a package module (it modifies
    # __path__), and if you import the modules in the wrong order (by, as site
    # does, importing site-packages and dist-packages before looking at
    # sys.path), then __path__ will 'remember' that import order. By deleting the
    # modules, Python will clean up the google module's __path__ variable and
    # when we go to import google.protobuf again, it will be setup correctly from
    # then on.
    for key in list(sys.modules.keys()):
        if key not in original_modules:
            del sys.modules[key]

    # We need to reset sys.path to the archive path, since it's not always set
    # 'correctly'. Sometimes its relative or absolute, and subpar doesn't
    # normalize paths.
    if _in_par():
        sys.path[0] = subpar_support._find_archive()
        subpar_support.setup(import_roots=import_roots, zip_safe=zip_safe)
    else:
        """Add extra entries to PYTHONPATH so that modules can be imported."""
        # We try to match to order of Bazel's stub
        main_mod = sys.modules["__main__"]
        root_path = os.path.dirname(main_mod.__file__)  # type: ignore[type-var] # ignore baseline; see #61427
        full_roots = [
            os.path.join(os.getcwd(), root_path, import_root) for import_root in import_roots  # type: ignore[arg-type] # ignore baseline; see #61427
        ]
        sys.path[1:1] = full_roots

    try:
        for mod in [m for m in sys.modules if m.startswith("tools")]:
            del sys.modules[mod]
        from tools.py import resources
    except ImportError:
        resources = None

    if resources and not is_on_windows():
        _enable_solibs()

    if _in_par():
        _extract_cexts(import_roots)
        if resources and remove:
            resources.register_cleanup()


def _relaunch_exec(new_args=None, new_env=None):
    # NOTE: We import cython.compiled in this function and in this way because:
    # 1. The import can only be resolved after the PYTHONPATH is set, which is
    # done in enable_solibs() right before calling this function.
    # 2. Cannot use <"import cython" ... "if cython.compiled:"> because when cython is
    # enabled it tries to replace it with a cimport. cimport is only allowed at the module level
    # but due to above reason in 1., we know this import cannot be moved out of this function.
    try:
        from cython import compiled as _compiled

        is_compiled = _compiled
    except:  # noqa: E722
        is_compiled = False

    if is_compiled:
        # If cythonization is enabled, the cythonized binary is captured in sys.argv[0]
        # and the remaining arguments are captured in sys.argv[1:].
        exe = sys.argv[0]
        args = []
    else:
        # If sys.executable is blank, ensure that PATH is not empty. If it is,
        # Python fails to find itself on the PATH and assumes it's embedded in
        # something else.
        exe = sys.executable
        # -S: Don't import the site module. We will import it when we need to.
        # If it's imported too early, we may load libraries from the system before
        # we load from our archive, causing version mismatches.
        # Removing it as of 12/15/2017 causes google.protobuf to get loaded early and
        # error out with:
        #   TypeError: __init__() got an unexpected keyword argument 'file'
        # -B: Don't write out .pyc files. If our files are written in a 'negative'
        # timezone (PST is -8), but then run in UTC (or any timezone later than when
        # they were written), there is a chance that Python will prefer the written
        # out .pyc files from a previous run even if the .py file has changed, until
        # the timestamp is out-of-date hour(s) later.
        args = [exe, "-SB"]

    if new_args:
        args += new_args
    else:
        args += sys.argv

    if new_env:
        os.execve(exe, args, new_env)
    else:
        os.execv(exe, args)


def _all_solibs():
    solib_resource = resources.Resource.FromPath("//%s" % SOLIB_CPU)  # type: ignore[union-attr] # ignore baseline; see #61427
    try:
        if not solib_resource.Exists():
            return
    except ImportError:
        return

    for solib in solib_resource.List():
        if not solib.IsDir():
            continue
        dirpath = solib.GetFilename()
        yield dirpath


def _extract_cexts(import_roots):
    """Extract C-extension modules.

    Strategy:
      For every import root, check if it has any C-extensions.
      For those that do, extract it to disk and replace its position in sys.path
          with the on-disk place. Use tools.resources to minimize extra code.

    Strategies attempted:
        1) Leave everything up to zipimport:
            zipimport doesn't extract C-extensions on its own. It explicitly only
            checks for .py{,c,o} files.
        2) Extract only the .so files and add them to sys.path:
            This splits an import root to two places: in-zip and on-disk. Python
            doesn't handle this well and attempts to import the .so files from
            in-zip or the .py files from on-disk (depending on order in sys.path)
        3) Extract only the .so files and push them into the import system:
            This should work (imp.load_module return successfully, and the module
            is in sys.modules already), but it doesn't. Specifically,
            'from . import multiarray' doesn't work in numpy/core/__init__.py
        4) Extract only the import roots with .so files in them:
            Current strategy!
    """
    import pkg_resources

    # Sometimes import_roots are sub-paths of each other. If we sort import_roots
    # by length (longest to shortest), then ignore already-seen files, we won't
    # visit a file twice.
    # We can go 'out-of-order' because we're not changing the order, only
    # replacing.
    seen_files = set()
    for import_root in sorted(import_roots, reverse=True):
        archive_path = subpar_support._find_archive()
        zf = zipfile.ZipFile(archive_path)
        any_so = False
        # TODO(fahhem): if any(...):
        for info in zf.infolist():
            fn = info.filename
            if (
                fn in seen_files
                or not fn.startswith(import_root)
                or not fn.endswith(".so")
                or ("/%s/" % SOLIB_CPU) in fn
            ):
                continue
            seen_files.add(fn)
            any_so = True
        if any_so:
            assert resources and pkg_resources, (
                "This py_binary depends on a C-extension, so you must depend on"
                "//tools/py:resources to run it as a .par."
            )
            provider = pkg_resources.get_provider("__main__")
            for info in zf.infolist():
                fn = info.filename
                if not fn.startswith(import_root):
                    continue
                provider.get_resource_filename(pkg_resources, fn)  # type: ignore[attr-defined] # ignore baseline; see #61427
            # Get the last path that was created. We assume all files were created
            # with a common top-level and their path preserved on disk. If that's no
            # longer true, non-top-level modules will fail to import.
            path = provider.get_resource_filename(pkg_resources, fn)  # type: ignore[attr-defined] # ignore baseline; see #61427
            index = sys.path.index(os.path.join(archive_path, import_root))
            sys.path[index] = os.path.join(path.replace(fn, ""), import_root)


def _enable_solibs():
    orig_ld_lib_path = os.environ.get("LD_LIBRARY_PATH", "")
    ld_lib_path = orig_ld_lib_path.split(":")
    orig_pythonpath = os.environ.get("PYTHONPATH", "")
    pythonpath = orig_pythonpath.split(":")

    # First, look for solibs that are potentially used by others or by binaries.
    # These need to be part of LD_LIBRARY_PATH too.
    for dirpath in _all_solibs():
        if dirpath not in pythonpath:
            pythonpath.insert(0, dirpath)
        if dirpath not in ld_lib_path:
            ld_lib_path.insert(0, dirpath)

    new_ld_lib_path = ":".join(ld_lib_path)
    new_pythonpath = ":".join(pythonpath)

    if new_ld_lib_path != os.environ.get("LD_LIBRARY_PATH"):
        new_env = dict(os.environ)
        new_env.update(
            {
                "LD_LIBRARY_PATH": new_ld_lib_path,
                "PYTHONPATH": new_pythonpath,
            }
        )
        _relaunch_exec(new_env=new_env)
    elif new_pythonpath != os.environ.get("PYTHONPATH"):
        # Simpler path in case someone already changed LD_LIBRARY_PATH but not
        # PYTHONPATH. We don't have to re-execute since sys.path is loaded
        # dynamically.
        sys.path.extend(pythonpath)


if resources is not None and not is_on_windows():
    # This binary has //tools/py:resources included, so we should check for any
    # solibs and enable them ASAP. This doesn't run in a PAR (resources is always
    # None), only when not in a PAR.
    _enable_solibs()
