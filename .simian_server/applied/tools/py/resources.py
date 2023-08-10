"""Resource-handling for Applied.

from tools import resources

resource -> File path
  Always separated by / with : between the path and the filename
  Example:
    //modules/applied/optic:optic

resources.GetResource(resource) -> Resource
resources.GetResourceFile(resource) -> file object
resources.GetResourceContents(resource) -> content: bytes
resources.GetResourceFilename(resource) -> filename: unicode
resources.HasResource(resource) -> bool
resources.ListResources(resource) -> [filename: unicode]
resources.RepoPathToResource(repo path) -> Resource

Resource:
    path: unicode
    Exists() -> bool
    GetFile() -> file object
    GetContents() -> content: bytes
    GetFilename() -> filename: unicode
    List() -> [Resource()]


Example usage:

resources.GetResourceFilename('//modules/applied/optic/optic')
"""
import atexit
import hashlib
import os
import typing
import zipimport

# TODO (cindy): this import takes 0.5s, determine if we need to import entire library
import pkg_resources


def GetResource(resource):
    return Resource.FromPath(resource)


def GetResourceFile(resource):
    return Resource.FromPath(resource).GetFile()


def GetResourceContents(resource):
    return Resource.FromPath(resource).GetContents()


def GetResourceFilename(resource):
    return Resource.FromPath(resource).GetFilename()


def HasResource(resource) -> bool:
    resource_exists = Resource.FromPath(resource).Exists()
    assert isinstance(resource_exists, bool)
    return resource_exists


def ListResources(resource):
    return [r.path for r in Resource.FromPath(resource).List()]


def ExtractPackage(resource):
    package = Resource.FromPath(resource)
    return package.ExtractSubpath()


def RepoPathToResource(repo_path):
    """Convert m/a/d to Resource(//m/a:d)."""
    resource_path = "//" + ":".join(repo_path.rsplit("/", 1))
    return Resource.FromPath(resource_path)


def FilenameToRepoPath(filename):
    """Convert '...runfiles/applied/foo' to 'foo'."""
    workspace = "applied"
    return filename.split(workspace + "/")[-1]


class Resource:
    @classmethod
    def FromPath(cls, resource):
        # A target named @//... is the same as //..., since @ is short for
        # @$workspace, which is @applied here.
        if resource.startswith("@//"):
            resource = resource[1:]

        if resource.startswith("@"):
            workspace, resource = resource[1:].split("//", 1)
        else:
            # TODO(fahhem): Pull this from bazel/skylark somehow.
            # From ctx.workspace_name?
            workspace = "applied"
            while resource.startswith("/"):
                resource = resource[1:]
        obj = resource.replace(":", "/")
        return cls(resource, workspace, obj)

    def __init__(self, path, workspace, obj):
        self._path = path
        self._obj = obj
        self._provider = pkg_resources.get_provider(workspace)

    @property
    def path(self):
        return self._path

    def GetFile(self):
        return self._provider.get_resource_stream(pkg_resources, self._obj)

    def GetContents(self):
        return self._provider.get_resource_string(pkg_resources, self._obj)

    def Exists(self) -> bool:
        has_resource = self._provider.has_resource(self._obj)
        assert isinstance(has_resource, bool)
        return has_resource

    def GetFilename(self):
        return self._provider.get_resource_filename(pkg_resources, self._obj)

    def List(self):
        return [
            self.FromPath(self.path.replace(":", "/") + ":" + path)
            for path in self._provider.resource_listdir(self._obj)
        ]

    def IsDir(self):
        return self._provider.resource_isdir(self._obj)

    def ExtractSubpath(self):
        if not self.IsDir():
            return self.GetFilename()

        for res in self.List():
            res.ExtractSubpath()
        return self.GetFilename()


# Replace the builtin ZipProvider with our slightly improved one.
class ZipProvider(pkg_resources.ZipProvider):
    """ZipProvider that handles extracting resources from a non-egg zip file."""

    loader: zipimport.zipimporter

    def _setup_prefix(self):
        # We essentially pretend like we're a legit egg here.
        name = os.path.realpath(self.loader.archive)
        self.egg_name = hashlib.md5(name.encode("utf8")).hexdigest()
        self.egg_root = self.loader.archive
        self.egg_info = self.egg_root


pkg_resources.register_loader_type(zipimport.zipimporter, ZipProvider)  # type: ignore


# Now replace the default cleanup_resources (it's blank!) with this kinda
# useful one:
def cleanup_resources(
    force: bool = False,  # noqa: ARG001
) -> typing.List[str]:
    paths_unable_to_remove = []
    # Cleanup the default extraction path first.
    # pkg_resources appears to declare this at runtime.
    extract_path = pkg_resources.extraction_path or pkg_resources.get_default_cache()  # type: ignore[attr-defined]
    try:
        os.rmdir(extract_path)
    except OSError:
        paths_unable_to_remove.append(extract_path)
    # pkg_resources appears to declare this at runtime.
    for path in list(pkg_resources.cached_files.keys()):  # type: ignore[attr-defined]
        if not os.path.exists(path):
            continue
        try:
            os.remove(path)
        except OSError:
            paths_unable_to_remove.append(path)
        d = os.path.dirname(path)
        if not os.path.exists(d):
            continue
        # If the folder is now empty, we can delete it.
        if all(p for p in os.listdir(d) if os.path.isdir(p)):
            try:
                os.rmdir(os.path.dirname(path))
            except OSError:
                paths_unable_to_remove.append(path)
    return paths_unable_to_remove


def register_cleanup():
    pkg_resources.cleanup_resources = cleanup_resources
    # And make sure it's called when cleaning up the process.
    atexit.register(pkg_resources.cleanup_resources)
