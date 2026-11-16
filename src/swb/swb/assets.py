"""``ros2 run swb assets fetch|list|check|prune``: a project's assets, by name, from a read-only cache.

docs/design/projects-and-pilots.md, "Assets". A dataset is a Hugging Face dataset (namespace
``jerry73204`` unless ``repo:`` names another). ``fetch``:

1. resolves each dataset to a commit (``revision:``, else the newest tag, else ``main``); a
   dataset already in ``swb.lock`` with the same ``repo``, ``take`` and ``revision`` keeps its
   locked commit (runs read the lock, never the tag; ``--update`` resolves again);
2. downloads only the ``take:`` subpaths into ``<cache>/datasets/<repo>/<commit>/``; each file
   lands complete (downloaded beside, verified, renamed in) and read-only, and is never written
   again; the directories are read-only between fetches;
3. verifies each file against the dataset's ``MANIFEST.sha256`` (or ``SHA256SUMS``, both
   in sha256sum's format) when it has one;
4. writes ``swb.lock``;
5. links ``<project>/assets/``: every map directory (one holding ``lanelet2_map.osm``) as
   ``assets/maps/<map>``, the CARLA USD set as ``assets/carla-usd``, any other dataset as
   ``assets/<dataset>``; a ``path:`` asset as ``assets/maps/<as>`` (a map) or ``assets/<as>``.

``prune`` removes from the cache every commit no project uses: fetch records each project in
``<cache>/projects.json``, and a commit is kept while a recorded project's ``swb.lock`` names
it (a project directory that is gone, or has no lock, no longer counts).

The Hub is reached through huggingface_hub when it is installed and the host setting
``hub_url`` is the public Hub; otherwise over plain HTTPS (urllib), against ``hub_url``.
"""
import argparse
import fnmatch
import hashlib
import json
import os
import shutil
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path, PurePosixPath

import yaml

from swb import config
from swb.findings import INFRA, REFUSED, Finding, add_project_option, exit_code, ros_args

DEFAULT_NAMESPACE = "jerry73204"
DEFAULT_HUB = "https://huggingface.co"
CARLA_USD = "carla-0.9.16-isaac-usd"
MANIFEST = "MANIFEST.sha256"
MANIFESTS = (MANIFEST, "SHA256SUMS")  # the first a dataset has is its manifest
MAP_MARKER = "lanelet2_map.osm"
# files at a dataset's root that are about the dataset, not part of what it holds
ROOT_DOCS = {"README.md", "NOTICE.md", "NOTICE", "LICENSE", "LICENSE.md", ".gitattributes",
             *MANIFESTS}
LOCK_VERSION = 1
FETCH = "ros2 run swb assets fetch"

# the public datasets: name, size, what
KNOWN = [
    (CARLA_USD, "25.5 GB",
     "CARLA 0.9.16's towns, vehicles, props and walkers as Isaac Sim USD (CC BY 4.0)"),
    ("carla-town-autoware-maps", "0.73 GB",
     "Autoware maps of CARLA's towns (lanelet2, point cloud), Town05 ring, Town01 Isaac world"),
    ("haruna-tsukuba-courses", "0.49 GB",
     "the Mount Haruna and Tsukuba courses: maps and Isaac worlds (ODbL, GSI)"),
    ("isaac-road-physics-stages", "2.6 MB",
     "Isaac road surfaces, a proving ground and procedural stages (Apache-2.0)"),
    ("newslab-golfcart-models", "6 MB",
     "the NEWSLab golf cart's models (Apache-2.0)"),
]

DATASET_KEYS = ("dataset", "repo", "take", "revision")
PATH_KEYS = ("path", "as")


class AssetError(RuntimeError):
    """An asset that cannot be fetched or linked; the message names the file and the fix."""


class HubUnreachable(AssetError):
    """The Hub did not answer (an infrastructure error, exit 2)."""


@dataclass
class DatasetSpec:
    name: str
    repo: str
    take: list = None
    revision: str = None

    def lock_key(self):
        return {"repo": self.repo, "take": self.take, "revision": self.revision}


@dataclass
class PathSpec:
    name: str
    path: str
    resolved: Path = None


@dataclass
class RevisionInfo:
    commit: str
    files: list = field(default_factory=list)
    last_modified: str = None
    ref: str = None


# --- the project file's assets: ------------------------------------------------------------

def parse_assets(project: Path, settings: dict):
    """``(datasets, paths, findings)`` from the project file's ``assets:``."""
    project = Path(project)
    datasets, paths, findings = [], [], []
    entries = settings.get("assets")
    if entries is None:
        return datasets, paths, findings
    if not isinstance(entries, list):
        return datasets, paths, [Finding(config.PROJECT_FILE, "assets: is not a list",
                                         "a list of {dataset: ...} or {path: ...} entries",
                                         refused=True)]
    seen = set()
    for i, e in enumerate(entries):
        where = f"assets[{i}]"
        if not isinstance(e, dict) or ("dataset" in e) == ("path" in e):
            findings.append(Finding(config.PROJECT_FILE,
                                    f"{where}: needs exactly one of dataset: or path:",
                                    "- dataset: <name> or - path: <dir>", refused=True))
            continue
        keys = DATASET_KEYS if "dataset" in e else PATH_KEYS
        unknown = [k for k in e if k not in keys]
        if unknown:
            findings.append(Finding(config.PROJECT_FILE,
                                    f"{where}: unknown key {', '.join(map(str, unknown))}",
                                    f"use only {', '.join(keys)}", refused=True))
            continue
        if "dataset" in e:
            name = e["dataset"]
            if not isinstance(name, str) or not name or "/" in name:
                findings.append(Finding(config.PROJECT_FILE,
                                        f"{where}: dataset: {name!r} is not a dataset name",
                                        "a name such as carla-town-autoware-maps; repo: for"
                                        " another namespace", refused=True))
                continue
            take = e.get("take")
            if isinstance(take, str):
                take = [take]
            if take is not None and (not isinstance(take, list) or
                                     not all(isinstance(t, str) and t for t in take)):
                findings.append(Finding(config.PROJECT_FILE,
                                        f"{where}: take: is not a list of paths",
                                        "take: [Town01, Town05]", refused=True))
                continue
            repo = e.get("repo") or f"{DEFAULT_NAMESPACE}/{name}"
            if not isinstance(repo, str) or repo.count("/") != 1:
                findings.append(Finding(config.PROJECT_FILE,
                                        f"{where}: repo: {repo!r} is not <namespace>/<name>",
                                        refused=True))
                continue
            rev = e.get("revision")
            spec = DatasetSpec(name, repo, take or None, str(rev) if rev is not None else None)
            datasets.append(spec)
        else:
            p = e["path"]
            if not isinstance(p, str) or not p:
                findings.append(Finding(config.PROJECT_FILE, f"{where}: path: is empty",
                                        refused=True))
                continue
            resolved = (project / Path(p).expanduser()).resolve()
            name = e.get("as") or resolved.name
            spec = PathSpec(str(name), p, resolved)
            paths.append(spec)
        if spec.name in seen:
            findings.append(Finding(config.PROJECT_FILE,
                                    f"{where}: {spec.name} is named twice",
                                    "one entry per dataset; as: renames a path", refused=True))
        seen.add(spec.name)
    return datasets, paths, findings


# --- the lock: -----------------------------------------------------------------------------

def read_lock(project: Path) -> dict:
    path = Path(project) / config.LOCK_FILE
    if not path.is_file():
        return {}
    lock = config.read_yaml(path)
    if lock.get("version") != LOCK_VERSION:
        raise AssetError(f"{config.LOCK_FILE}: version {lock.get('version')!r}, this swb reads"
                         f" {LOCK_VERSION} (fix: delete it and run {FETCH})")
    lock.setdefault("datasets", {})
    lock.setdefault("paths", {})
    return lock


def lock_text(lock: dict) -> str:
    return ("# written by `ros2 run swb assets fetch`; commit it\n"
            + yaml.safe_dump(lock, sort_keys=False, default_flow_style=False))


def _entry_matches(entry: dict, spec: DatasetSpec) -> bool:
    return (entry.get("repo") == spec.repo and entry.get("take") == spec.take
            and entry.get("revision") == spec.revision)


# --- the Hub: ------------------------------------------------------------------------------

class HttpHub:
    """The Hub's HTTP API, with urllib only."""

    def __init__(self, base=DEFAULT_HUB, retries=4, retry_wait=2.0):
        self.base = base.rstrip("/")
        self.retries, self.retry_wait = retries, retry_wait

    def _open(self, url):
        req = urllib.request.Request(url, headers={"User-Agent": "swb-assets"})
        for attempt in range(self.retries):
            try:
                return urllib.request.urlopen(req, timeout=60)
            except urllib.error.HTTPError as e:
                if e.code in (429, 500, 502, 503, 504) and attempt + 1 < self.retries:
                    time.sleep(self.retry_wait * 2 ** attempt)
                    continue
                if e.code in (401, 403, 404):
                    raise AssetError(f"{url}: HTTP {e.code} (fix: check the dataset's name"
                                     " and revision; `ros2 run swb assets list`)") from None
                raise HubUnreachable(f"{url}: HTTP {e.code}") from None
            except (urllib.error.URLError, OSError) as e:
                if attempt + 1 < self.retries:
                    time.sleep(self.retry_wait * 2 ** attempt)
                    continue
                raise HubUnreachable(f"{url}: {getattr(e, 'reason', e)}") from None

    def _json(self, path):
        with self._open(self.base + path) as r:
            return json.load(r)

    def tags(self, repo):
        refs = self._json(f"/api/datasets/{repo}/refs")
        return [t["name"] for t in refs.get("tags", [])]

    def revision(self, repo, rev):
        d = self._json(f"/api/datasets/{repo}/revision/{urllib.parse.quote(rev, safe='')}")
        return RevisionInfo(d["sha"], [s["rfilename"] for s in d.get("siblings", [])],
                            d.get("lastModified"), rev)

    def download(self, repo, commit, rel, tmpd: Path) -> Path:
        url = f"{self.base}/datasets/{repo}/resolve/{commit}/{urllib.parse.quote(rel)}"
        out = Path(tmpd) / "file"
        with self._open(url) as r, open(out, "wb") as f:
            shutil.copyfileobj(r, f, 1 << 20)
        return out


class HfHub:
    """The Hub through huggingface_hub (its auth, its rate-limit handling)."""

    def __init__(self):
        from huggingface_hub import HfApi
        self.api = HfApi()

    def _call(self, what, fn, *a, **kw):
        try:
            return fn(*a, **kw)
        except Exception as e:  # huggingface_hub's errors vary by version
            raise AssetError(f"{what}: {e} (fix: check the dataset's name and revision)") from None

    def tags(self, repo):
        refs = self._call(repo, self.api.list_repo_refs, repo, repo_type="dataset")
        return [t.name for t in refs.tags]

    def revision(self, repo, rev):
        i = self._call(f"{repo}@{rev}", self.api.dataset_info, repo, revision=rev)
        lm = i.last_modified.isoformat() if getattr(i, "last_modified", None) else None
        return RevisionInfo(i.sha, [s.rfilename for s in (i.siblings or [])], lm, rev)

    def download(self, repo, commit, rel, tmpd: Path) -> Path:
        from huggingface_hub import hf_hub_download
        return Path(self._call(f"{repo}/{rel}", hf_hub_download, repo_id=repo, filename=rel,
                               repo_type="dataset", revision=commit, local_dir=str(tmpd)))


def make_hub(settings: dict):
    url = (settings.get("hub_url") or DEFAULT_HUB).rstrip("/")
    if url == DEFAULT_HUB:
        try:
            import huggingface_hub  # noqa: F401
            return HfHub()
        except ImportError:
            pass
    return HttpHub(url)


def resolve(hub, spec: DatasetSpec) -> RevisionInfo:
    if spec.revision:
        return hub.revision(spec.repo, spec.revision)
    tags = hub.tags(spec.repo)
    if not tags:
        return hub.revision(spec.repo, "main")
    infos = [hub.revision(spec.repo, t) for t in tags]
    return max(infos, key=lambda i: (i.last_modified or "", i.ref))


# --- the cache: ----------------------------------------------------------------------------

def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def commit_dir(cache, repo, commit) -> Path:
    return Path(cache) / "datasets" / repo / commit


def select(files, take):
    """``(taken, unmatched patterns)``: files under or matching each ``take`` glob."""
    if not take:
        return sorted(files), []
    chosen, unmatched = set(), []
    for p in take:
        p = p.strip("/")
        hit = [f for f in files if fnmatch.fnmatchcase(f, p) or fnmatch.fnmatchcase(f, p + "/*")]
        if not hit:
            unmatched.append(p)
        chosen.update(hit)
    return sorted(chosen), unmatched


def parse_manifest(path: Path) -> dict:
    out = {}
    for line in Path(path).read_text().splitlines():
        parts = line.strip().split(None, 1)
        if len(parts) == 2:
            name = parts[1].lstrip("*")
            out[name[2:] if name.startswith("./") else name] = parts[0].lower()
    return out


def _set_tree_mode(root: Path, writable: bool):
    if not root.is_dir():
        return
    dirs = [root] + [Path(d) / n for d, ns, _ in os.walk(root) for n in ns]
    for d in dirs:
        os.chmod(d, 0o755 if writable else 0o555)


class _Store:
    """Adds complete files to one commit's directory; reseals it on the way out."""

    def __init__(self, hub, cache, repo, commit):
        self.hub, self.repo, self.commit = hub, repo, commit
        self.dir = commit_dir(cache, repo, commit)
        self.tmp_root = Path(cache) / "tmp"

    def __enter__(self):
        self.dir.mkdir(parents=True, exist_ok=True)
        self.tmp_root.mkdir(parents=True, exist_ok=True)
        _set_tree_mode(self.dir, True)
        return self

    def __exit__(self, *exc):
        _set_tree_mode(self.dir, False)

    def add(self, rel, want=None) -> str:
        """Download ``rel``; check it against ``want`` (hex); returns its sha256 (hex)."""
        tmpd = Path(tempfile.mkdtemp(dir=self.tmp_root))
        try:
            part = self.hub.download(self.repo, self.commit, rel, tmpd)
            digest = sha256(part)
            if want and digest != want:
                raise AssetError(f"{self.repo}@{self.commit[:12]}: {rel}: sha256 {digest[:16]}..."
                                 " but the manifest says {want[:16]}... (fix: fetch again; if it"
                                 " repeats, the dataset is damaged)")
            os.chmod(part, 0o444)
            dest = self.dir / rel
            dest.parent.mkdir(parents=True, exist_ok=True)
            if not dest.exists():  # another fetch may have completed it meanwhile
                os.replace(part, dest)
            return digest
        finally:
            shutil.rmtree(tmpd, ignore_errors=True)


def fetch_dataset(hub, cache, spec: DatasetSpec, entry: dict, update=False):
    """``(lock entry, what changed or None)`` for one dataset."""
    old_commit = entry.get("commit") if entry else None
    if entry and not update and _entry_matches(entry, spec):
        files = dict(entry.get("files") or {})
        cdir = commit_dir(cache, spec.repo, entry["commit"])
        missing = [f for f in files if not (cdir / f).is_file()]
        if not missing:
            return entry, None
        with _Store(hub, cache, spec.repo, entry["commit"]) as store:
            for f in missing:
                want = files[f].split(":", 1)[-1]
                store.add(f, want)
        return entry, f"{len(missing)} missing file(s) fetched again"

    info = resolve(hub, spec)
    taken, unmatched = select(info.files, spec.take)
    if unmatched:
        raise AssetError(f"{config.PROJECT_FILE}: {spec.name}: take {', '.join(unmatched)}"
                         f" matches nothing in {spec.repo} at {info.commit[:12]}"
                         " (fix: a path of the dataset, relative to its root)")
    if not taken:
        raise AssetError(f"{config.PROJECT_FILE}: {spec.name}: {spec.repo} holds no files")
    cdir = commit_dir(cache, spec.repo, info.commit)
    known = {}
    if entry and entry.get("commit") == info.commit:
        known = {f: h.split(":", 1)[-1] for f, h in (entry.get("files") or {}).items()}
    new = 0
    files = {}
    with _Store(hub, cache, spec.repo, info.commit) as store:
        manifest = {}
        name = next((m for m in MANIFESTS if m in info.files), None)
        if name:
            if not (cdir / name).is_file():
                store.add(name)
            manifest = parse_manifest(cdir / name)
        for f in taken:
            # a dataset's card and notices are edited on the Hub after its manifest is made
            want = None if f in ROOT_DOCS else manifest.get(f)
            if (cdir / f).is_file():
                digest = known.get(f) or sha256(cdir / f)
                if want and digest != want:
                    raise AssetError(f"{cdir / f}: differs from the manifest (fix: delete"
                                     f" {cdir} after chmod -R u+w, then {FETCH})")
            else:
                digest = store.add(f, want)
                new += 1
            files[f] = "sha256:" + digest
    out = {"repo": spec.repo, "commit": info.commit}
    if spec.revision:
        out["revision"] = spec.revision
    if spec.take:
        out["take"] = list(spec.take)
    out["files"] = files
    if old_commit and old_commit != info.commit:
        what = f"{old_commit[:12]} -> {info.commit[:12]}, {new} new file(s)"
    elif entry and _entry_matches(entry, spec) and not new:
        what = None if entry == out else "lock refreshed"
    else:
        what = f"{info.commit[:12]}: {len(files)} file(s), {new} downloaded"
    return out, what


# --- links: --------------------------------------------------------------------------------

def _under(f: str, d: str) -> bool:
    return d == "." or f == d or f.startswith(d + "/")


# --- datasets a host keeps itself (roadmap 077, G8) ----------------------------------------
#
# A dataset this host has a copy of (a private one on the lab's NAS, say) is named in the host
# settings, ``datasets: {<name>: <directory>}``: `assets fetch` links it from there, with the
# project's own ``take:``, instead of fetching it, so the project file names it the same way
# on every host. Its file list is the host's, kept in the project's state (git-ignored), and
# swb.lock keeps only what came from the Hub.
LOCAL_FILE = "assets.local.json"


def local_sources(host: dict) -> dict:
    """``{dataset name: directory}`` from the host settings' ``datasets:``."""
    d = host.get("datasets") if isinstance(host.get("datasets"), dict) else {}
    return {str(k): Path(os.path.expanduser(str(v))) for k, v in d.items() if v}


def local_entry(spec, root: Path) -> dict:
    """The dataset's files under ``root`` that its ``take:`` selects; AssetError when
    ``root`` is not a directory or a ``take:`` pattern matches nothing."""
    if not root.is_dir():
        raise AssetError(f"{spec.name}: the host's copy {root} is not a directory (fix: mount "
                         "it, or drop datasets: {" + spec.name + ": ...} from the host settings)")
    files = [p.relative_to(root).as_posix() for p in root.rglob("*")
             if p.is_file() or (p.is_symlink() and not p.is_dir())]
    taken, unmatched = select(files, spec.take)
    if unmatched:
        raise AssetError(f"{spec.name}: take: {', '.join(unmatched)} matches nothing in the "
                         f"host's copy {root}")
    return {"dir": str(root), "files": taken}


def read_local(project: Path) -> dict:
    p = Path(project) / "state" / LOCAL_FILE
    try:
        return json.loads(p.read_text()) if p.is_file() else {}
    except (OSError, ValueError):
        return {}


def plan_links(project: Path, lock: dict, cache, local=None) -> dict:
    """``{path under assets/: target}`` for the lock's datasets and paths, and the datasets
    the host keeps itself (``local``: ``{name: {dir, files}}``)."""
    want = {}

    def put(key, target, who):
        if key in want and want[key] != target:
            raise AssetError(f"{config.PROJECT_FILE}: {who} and another asset both want"
                             f" assets/{key} (fix: take one of them, or rename a path with as:)")
        want[key] = target

    local = local or {}
    sources = [(n, Path(e["dir"]), list(e["files"])) for n, e in local.items()]
    for name, e in (lock.get("datasets") or {}).items():
        if name not in local:
            sources.append((name, commit_dir(cache, e["repo"], e["commit"]),
                            list((e.get("files") or {}).keys())))
    for name, cdir, files in sources:
        map_dirs = sorted({str(PurePosixPath(f).parent) for f in files
                           if PurePosixPath(f).name == MAP_MARKER})
        for d in map_dirs:
            put(f"maps/{name if d == '.' else PurePosixPath(d).name}",
                cdir if d == "." else cdir / d, name)
        rest = [f for f in files if not any(_under(f, d) for d in map_dirs)
                and f not in ROOT_DOCS]
        if rest:
            put("carla-usd" if name == CARLA_USD else name, cdir, name)
    for name, p in (lock.get("paths") or {}).items():
        target = (Path(project) / Path(p["path"]).expanduser()).resolve()
        put(f"maps/{name}" if (target / MAP_MARKER).is_file() else name, target, name)
    return want


def _managed_links(assets: Path):
    for d in (assets, assets / "maps"):
        if d.is_dir():
            for p in d.iterdir():
                if p.is_symlink():
                    yield p


def apply_links(project: Path, want: dict) -> list:
    assets = Path(project) / "assets"
    (assets / "maps").mkdir(parents=True, exist_ok=True)
    msgs = []
    for p in list(_managed_links(assets)):
        key = p.relative_to(assets).as_posix()
        if key not in want:
            p.unlink()
            msgs.append(f"unlinked assets/{key}")
    for key, target in sorted(want.items()):
        link = assets / key
        if link.is_symlink():
            if Path(os.readlink(link)) == target:
                continue
            link.unlink()
        elif link.exists():
            raise AssetError(f"assets/{key}: a file or directory swb did not make (fix: move it"
                             " away; assets/ holds only links)")
        link.symlink_to(target, target_is_directory=True)
        msgs.append(f"linked assets/{key} -> {target}")
    return msgs


# --- the commands: -------------------------------------------------------------------------

def fetch(project: Path, update=False, hub=None, out=print) -> dict:
    """Fetch the project's assets, write its lock and links; returns the lock."""
    project = Path(project)
    settings = config.host_settings(project)
    datasets, paths, findings = parse_assets(project, config.project_settings(project))
    if findings:
        raise AssetError("; ".join(f"{f.file}: {f.what}" for f in findings))
    cache = Path(settings["cache"])
    old = read_lock(project)
    lock = {"version": LOCK_VERSION, "datasets": {}, "paths": {}}
    mine, local = local_sources(settings), {}
    for spec in datasets:
        if spec.name in mine:
            local[spec.name] = local_entry(spec, mine[spec.name])
            if spec.name in old.get("datasets", {}):
                lock["datasets"][spec.name] = old["datasets"][spec.name]   # the Hub's, kept
            out(f"{spec.name}: the host's copy {mine[spec.name]}, "
                f"{len(local[spec.name]['files'])} file(s)")
            continue
        if hub is None:
            hub = make_hub(settings)
        entry, what = fetch_dataset(hub, cache, spec, old.get("datasets", {}).get(spec.name),
                                    update)
        lock["datasets"][spec.name] = entry
        out(f"{spec.name}: {what or 'unchanged'}")
    for spec in paths:
        if not spec.resolved.is_dir():
            raise AssetError(f"{config.PROJECT_FILE}: path {spec.path}: no such directory"
                             f" ({spec.resolved})")
        lock["paths"][spec.name] = {"path": spec.path}
    for name in old.get("datasets", {}):
        if name not in lock["datasets"]:
            out(f"{name}: dropped (no longer in {config.PROJECT_FILE})")
    for msg in apply_links(project, plan_links(project, lock, cache, local)):
        out(msg)
    (project / "state").mkdir(exist_ok=True)
    lp = project / "state" / LOCAL_FILE
    if local:
        lp.write_text(json.dumps(local, indent=1) + "\n")
    elif lp.exists():
        lp.unlink()
    path = project / config.LOCK_FILE
    text = lock_text(lock)
    if not path.is_file() or path.read_text() != text:
        path.write_text(text)
        out(f"{config.LOCK_FILE} written")
    if lock["datasets"]:
        register(cache, project)
    return lock


PROJECTS_FILE = "projects.json"


def _projects(cache) -> list:
    try:
        data = json.loads((Path(cache) / PROJECTS_FILE).read_text())
        return [str(x) for x in data] if isinstance(data, list) else []
    except (OSError, ValueError):
        return []


def register(cache, project: Path):
    """Record ``project`` as a user of the cache (for ``prune``)."""
    cache = Path(cache)
    known = _projects(cache)
    me = str(Path(project).resolve())
    if me not in known:
        cache.mkdir(parents=True, exist_ok=True)
        tmp = cache / (PROJECTS_FILE + ".tmp")
        tmp.write_text(json.dumps(sorted(known + [me]), indent=1) + "\n")
        tmp.replace(cache / PROJECTS_FILE)


def _tree_size(root: Path) -> int:
    return sum(f.stat().st_size for f in Path(root).rglob("*") if f.is_file()
               and not f.is_symlink())


def prune(cache, dry_run=False, out=print) -> list:
    """Remove the cache's commits no recorded project's lock names; the removed dirs."""
    cache = Path(cache)
    used, alive = set(), []
    for proj in _projects(cache):
        try:
            lock = read_lock(Path(proj)) if Path(proj).is_dir() else {}
        except (AssetError, config.ConfigError):
            lock = {}
        if not lock.get("datasets"):
            continue
        alive.append(proj)
        for e in lock["datasets"].values():
            if e.get("repo") and e.get("commit"):
                used.add((e["repo"], e["commit"]))
    removed, freed = [], 0
    root = cache / "datasets"
    for cdir in sorted(root.glob("*/*/*")) if root.is_dir() else []:
        if not cdir.is_dir():
            continue
        repo = f"{cdir.parent.parent.name}/{cdir.parent.name}"
        if (repo, cdir.name) in used:
            continue
        size = _tree_size(cdir)
        freed += size
        removed.append(cdir)
        out(f"{'would remove' if dry_run else 'removed'} {repo}@{cdir.name[:12]} "
            f"({size / 1024 ** 2:.1f} MB)")
        if not dry_run:
            _set_tree_mode(cdir, True)
            for f in cdir.rglob("*"):
                if f.is_file() and not f.is_symlink():
                    os.chmod(f, 0o644)
            shutil.rmtree(cdir)
    if not dry_run:
        tmp = cache / (PROJECTS_FILE + ".tmp")
        if cache.is_dir():
            tmp.write_text(json.dumps(sorted(alive), indent=1) + "\n")
            tmp.replace(cache / PROJECTS_FILE)
    out(f"{'would free' if dry_run else 'freed'} {freed / 1024 ** 2:.1f} MB; "
        f"{len(alive)} project(s) keep {len(used)} commit(s)")
    return removed


def check(project: Path, settings: dict = None) -> list:
    """Findings: assets not fetched, not matching the lock, or not linked."""
    project = Path(project)
    if settings is None:
        settings = config.project_settings(project)
    cache = Path(config.host_settings(project)["cache"])
    datasets, paths, findings = parse_assets(project, settings)
    lf = config.LOCK_FILE
    try:
        lock = read_lock(project)
    except (AssetError, config.ConfigError) as e:
        return findings + [Finding(lf, str(e), FETCH, refused=True)]
    if not lock:
        if datasets or paths:
            findings.append(Finding(lf, "missing: the assets were never fetched", FETCH,
                                    refused=True))
        return findings
    locked = lock["datasets"]
    mine, local = local_sources(config.host_settings(project)), read_local(project)
    for spec in datasets:
        if spec.name in mine:
            if not mine[spec.name].is_dir():
                findings.append(Finding("host", f"{spec.name}: the host's copy {mine[spec.name]}"
                                        " is not a directory", "mount it, or drop it from the "
                                        "host settings' datasets:", refused=True))
            elif local.get(spec.name, {}).get("dir") != str(mine[spec.name]):
                findings.append(Finding(lf, f"{spec.name} is not linked from the host's copy",
                                        FETCH, refused=True))
            continue
        e = locked.get(spec.name)
        if e is None:
            findings.append(Finding(lf, f"{spec.name} is not fetched", FETCH, refused=True))
        elif not _entry_matches(e, spec):
            findings.append(Finding(lf, f"{spec.name}: {config.PROJECT_FILE} changed since the"
                                    " fetch (repo, take or revision)", FETCH, refused=True))
    for spec in paths:
        if not spec.resolved.is_dir():
            findings.append(Finding(config.PROJECT_FILE, f"path {spec.path}: no such directory",
                                    "make it, or drop the entry", refused=True))
        elif lock["paths"].get(spec.name) != {"path": spec.path}:
            findings.append(Finding(lf, f"path {spec.name} is not fetched", FETCH, refused=True))
    names = {s.name for s in datasets}
    for name in locked:
        if name not in names:
            findings.append(Finding(lf, f"{name} is locked but not in {config.PROJECT_FILE}",
                                    FETCH))
    for name, e in locked.items():
        if name in mine:
            continue
        cdir = commit_dir(cache, e["repo"], e["commit"])
        for f, h in (e.get("files") or {}).items():
            p = cdir / f
            if not p.is_file():
                findings.append(Finding(lf, f"{name}: {f} is missing from the cache ({cdir})",
                                        FETCH, refused=True))
            elif "sha256:" + sha256(p) != h:
                findings.append(Finding(lf, f"{name}: {f} differs from the lock",
                                        f"delete {cdir} (chmod -R u+w first), then {FETCH}"))
    try:
        want = plan_links(project, lock, cache,
                          {n: e for n, e in local.items() if n in mine})
    except AssetError as e:
        return findings + [Finding(config.PROJECT_FILE, str(e))]
    for key, target in sorted(want.items()):
        link = project / "assets" / key
        if not link.is_symlink() or Path(os.readlink(link)) != target:
            findings.append(Finding(f"assets/{key}", "not linked to the locked copy", FETCH,
                                    refused=True))
    return findings


def list_assets(project, out=print):
    out(f"Datasets on Hugging Face ({DEFAULT_NAMESPACE}/<name>):")
    w = max(len(n) for n, _, _ in KNOWN)
    for name, size, what in KNOWN:
        out(f"  {name:<{w}}  {size:>8}  {what}")
    if project is None:
        return
    settings = config.project_settings(project)
    datasets, paths, _ = parse_assets(project, settings)
    try:
        lock = read_lock(project)
    except AssetError:
        lock = {}
    out(f"This project ({project}):")
    for spec in datasets:
        e = (lock.get("datasets") or {}).get(spec.name)
        take = f"  take {', '.join(spec.take)}" if spec.take else ""
        if e:
            out(f"  {spec.name}  {e['commit'][:12]}{take}  {len(e.get('files') or {})} file(s)")
        else:
            out(f"  {spec.name}{take}  not fetched")
    for spec in paths:
        state = "" if spec.name in (lock.get("paths") or {}) else "  not fetched"
        out(f"  {spec.name}  path {spec.path}{state}")


def main(argv=None):
    parser = argparse.ArgumentParser(prog="ros2 run swb assets",
                                     description="A project's assets: fetch, list, check.")
    sub = parser.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("fetch", help="assets into the cache, the lock, links into assets/")
    f.add_argument("--update", action="store_true",
                   help="resolve every dataset's revision again instead of keeping the lock's")
    ls = sub.add_parser("list", help="the known datasets and what this project has fetched")
    c = sub.add_parser("check", help="re-hash the linked files against swb.lock")
    pr = sub.add_parser("prune", help="remove cached commits no project's swb.lock names")
    pr.add_argument("--dry-run", action="store_true", help="say what would go, remove nothing")
    add_project_option(parser, f, ls, c)
    args = parser.parse_args(ros_args(sys.argv[1:] if argv is None else argv))
    try:
        if args.cmd == "prune":
            prune(config.host_settings()["cache"], dry_run=args.dry_run)
            return 0
        if args.cmd == "list":
            try:
                project = config.find_project(args.project)
            except config.ConfigError:
                project = None
            list_assets(project)
            return 0
        project = config.find_project(args.project)
        if args.cmd == "fetch":
            config.writable_project(project)
            fetch(project, update=args.update)
            return 0
        findings = check(project)
        for x in findings:
            print(x)
        if not findings:
            print("assets: ok")
        return exit_code(findings)
    except HubUnreachable as e:
        print(f"error: {e}", file=sys.stderr)
        return INFRA
    except (AssetError, config.ConfigError) as e:
        print(f"problem: {e}", file=sys.stderr)
        return REFUSED


if __name__ == "__main__":
    sys.exit(main())
