"""``scensim assets fetch|list|check|prune``: a project's assets, by name, from a read-only cache.

docs/design/projects-and-pilots.md, "Assets". A dataset is a Hugging Face dataset (namespace
``jerry73204`` unless ``repo:`` names another). ``fetch``:

1. resolves each dataset to a commit (``revision:``, else the newest tag, else ``main``); a
   dataset already in ``scensim.lock`` with the same ``repo``, ``take`` and ``revision`` keeps its
   locked commit (runs read the lock, never the tag; ``--update`` resolves again);
2. downloads only the ``take:`` subpaths into ``<cache>/datasets/<repo>/<commit>/``; each file
   lands complete (downloaded beside, verified, renamed in) and read-only, and is never written
   again; the directories are read-only between fetches;
3. verifies each file against the dataset's ``MANIFEST.sha256`` (or ``SHA256SUMS``, both
   in sha256sum's format) when it has one;
4. writes ``scensim.lock``;
5. links ``<project>/assets/``: every map directory (one holding ``lanelet2_map.osm``) as
   ``assets/maps/<map>``, the CARLA USD set as ``assets/carla-usd``, any other dataset as
   ``assets/<dataset>``; a ``path:`` asset as ``assets/maps/<as>`` (a map) or ``assets/<as>``.

``prune`` removes from the cache every commit no project uses: fetch records each project in
``<cache>/projects.json``, and a commit is kept while a recorded project's ``scensim.lock`` names
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

from scensim import config
from scensim.findings import INFRA, REFUSED, Finding, add_project_option, exit_code, ros_args

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
FETCH = "scensim assets fetch"

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
        raise AssetError(f"{config.LOCK_FILE}: version {lock.get('version')!r}, this scensim reads"
                         f" {LOCK_VERSION} (fix: delete it and run {FETCH})")
    lock.setdefault("datasets", {})
    lock.setdefault("paths", {})
    return lock


def lock_text(lock: dict) -> str:
    return ("# written by `scensim assets fetch`; commit it\n"
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
        req = urllib.request.Request(url, headers={"User-Agent": "scensim-assets"})
        for attempt in range(self.retries):
            try:
                return urllib.request.urlopen(req, timeout=60)
            except urllib.error.HTTPError as e:
                if e.code in (429, 500, 502, 503, 504) and attempt + 1 < self.retries:
                    time.sleep(self.retry_wait * 2 ** attempt)
                    continue
                if e.code in (401, 403, 404):
                    raise AssetError(f"{url}: HTTP {e.code} (fix: check the dataset's name"
                                     " and revision; `scensim assets list`)") from None
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


BROTLI_FIX = "pip install 'brotli>=1.2'"


def _dist_version(name: str):
    """An installed distribution's version, or None."""
    try:
        from importlib.metadata import version
        return version(name)
    except Exception:  # noqa: BLE001 (not installed, or no metadata)
        return None


def _release(v) -> tuple:
    out = []
    for part in str(v or "").split("."):
        digits = "".join(c for c in part if c.isdigit()) if part[:1].isdigit() else ""
        if not digits:
            break
        out.append(int(digits))
    return tuple(out)


def brotli_problem(hf_version=None, brotli_version=None):
    """Why huggingface_hub cannot fetch here (issue 050), or None: huggingface_hub 2.x
    downloads through httpx2, which asks for brotli whenever ``brotli`` imports and decodes it
    with a keyword brotli added in 1.2 (``process() takes no keyword arguments`` before)."""
    hf = hf_version or _dist_version("huggingface_hub")
    if brotli_version is None:
        brotli_version = _dist_version("brotli") or _dist_version("Brotli")
        if brotli_version is None:
            try:
                import brotli
                brotli_version = getattr(brotli, "__version__", None) or "0"
            except ImportError:
                return None
    if not hf or _release(hf)[:1] < (2,) or _release(brotli_version) >= (1, 2):
        return None
    return (f"huggingface_hub {hf} decodes the Hub's brotli responses with brotli 1.2 or "
            f"newer, and brotli {brotli_version} is installed")


class HfHub:
    """The Hub through huggingface_hub (its auth, its rate-limit handling). A call it fails
    is made again over scensim's own HTTPS (``HttpHub``), which reports what still fails in
    its own words (issue 050: huggingface_hub 2.x with brotli older than 1.2 fails most
    downloads, and its error is no reason to blame the dataset)."""

    def __init__(self, fallback=None, err=None):
        from huggingface_hub import HfApi
        self.api = HfApi()
        self.fallback = fallback or HttpHub()
        self.err = err or sys.stderr
        self.noted = False

    def _note(self, what, e):
        """Say once why huggingface_hub is bypassed: the brotli mismatch, or its own error."""
        if self.noted:
            return
        self.noted = True
        problem = brotli_problem()
        cause = problem or f"{type(e).__name__}: {e}"
        fix = f" (fix: {BROTLI_FIX})" if problem else ""
        print(f"note: huggingface_hub failed on {what} ({cause}); fetching over plain HTTPS "
              f"instead{fix}", file=self.err)

    def tags(self, repo):
        try:
            refs = self.api.list_repo_refs(repo, repo_type="dataset")
        except Exception as e:  # noqa: BLE001 (huggingface_hub's errors vary by version)
            self._note(repo, e)
            return self.fallback.tags(repo)
        return [t.name for t in refs.tags]

    def revision(self, repo, rev):
        try:
            i = self.api.dataset_info(repo, revision=rev)
        except Exception as e:  # noqa: BLE001
            self._note(f"{repo}@{rev}", e)
            return self.fallback.revision(repo, rev)
        lm = i.last_modified.isoformat() if getattr(i, "last_modified", None) else None
        return RevisionInfo(i.sha, [s.rfilename for s in (i.siblings or [])], lm, rev)

    def download(self, repo, commit, rel, tmpd: Path) -> Path:
        from huggingface_hub import hf_hub_download
        try:
            return Path(hf_hub_download(repo_id=repo, filename=rel, repo_type="dataset",
                                        revision=commit, local_dir=str(tmpd)))
        except Exception as e:  # noqa: BLE001
            self._note(f"{repo}/{rel}", e)
            return self.fallback.download(repo, commit, rel, tmpd)


def hub_findings(project: Path, settings: dict) -> list:
    """A host warning when this project fetches from the Hub through a huggingface_hub that
    cannot decode its responses here (issue 050); the fetch still works, more slowly."""
    url = (config.host_settings(project).get("hub_url") or DEFAULT_HUB).rstrip("/")
    datasets = parse_assets(project, settings)[0]
    problem = brotli_problem() if datasets and url == DEFAULT_HUB else None
    if not problem:
        return []
    return [Finding("host", f"{problem}: `{FETCH}` falls back to plain HTTPS", BROTLI_FIX,
                    host=True)]


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


# --- what was hashed last (roadmap 077, G35) -----------------------------------------------
#
# Hashing a 19 GB dataset takes minutes on a spinning disk, and `assets fetch`, `assets check`,
# `project check` and the stack's start each did it on every call. A file is hashed once; its
# digest is kept beside the cache, ``<cache>/verified/<repo>/<commit>.json``, under the file's
# stat key (size, mtime, ctime, inode), and a later call hashes only a file whose key changed.
# Rewriting a file moves its mtime and ctime, and ctime cannot be set back by hand, so a
# changed file is still hashed and caught. ``assets check --hash`` and ``assets fetch
# --verify`` hash every file regardless.
VERIFIED_DIR = "verified"


def _stat_key(path: Path) -> list:
    st = os.stat(path)
    return [st.st_size, st.st_mtime_ns, st.st_ctime_ns, st.st_ino]


class Verified:
    """The digests of one commit's files, by stat key; ``digest()`` hashes only what changed."""

    def __init__(self, cache, repo, commit, force=False):
        self.path = Path(cache) / VERIFIED_DIR / repo / f"{commit}.json"
        self.force = force
        self.hashed = 0
        self.dirty = False
        try:
            data = json.loads(self.path.read_text())
            self.known = data if isinstance(data, dict) else {}
        except (OSError, ValueError):
            self.known = {}

    def digest(self, rel: str, path: Path, want: str = None) -> str:
        """``path``'s sha256 (hex). From the record when its stat key is unchanged and the
        recorded digest is ``want`` (if given); hashed otherwise, and recorded."""
        key = _stat_key(path)
        rec = self.known.get(rel)
        if (not self.force and isinstance(rec, dict) and rec.get("stat") == key
                and (want is None or rec.get("sha256") == want)):
            return rec["sha256"]
        d = sha256(path)
        self.hashed += 1
        if _stat_key(path) == key:   # not changed while it was read
            self.known[rel] = {"stat": key, "sha256": d}
            self.dirty = True
        return d

    def record(self, rel: str, path: Path, digest: str):
        """A digest computed elsewhere (a file just downloaded and checked)."""
        self.known[rel] = {"stat": _stat_key(path), "sha256": digest}
        self.dirty = True

    def save(self):
        if not self.dirty:
            return
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            try:   # another process's records of other files stay
                cur = json.loads(self.path.read_text())
                cur = cur if isinstance(cur, dict) else {}
            except (OSError, ValueError):
                cur = {}
            cur.update(self.known)
            tmp = self.path.with_name(f".{self.path.name}.{os.getpid()}")
            tmp.write_text(json.dumps(cur, sort_keys=True) + "\n")
            os.replace(tmp, self.path)
            self.dirty = False
        except OSError:
            pass   # a cache we cannot write to: the next call hashes again, nothing worse


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


def _differs(path: Path, cdir: Path) -> AssetError:
    return AssetError(f"{path}: differs from the manifest (fix: delete {cdir} after chmod -R"
                      f" u+w, then {FETCH})")


def fetch_dataset(hub, cache, spec: DatasetSpec, entry: dict, update=False, verify=False):
    """``(lock entry, what changed or None)`` for one dataset. ``verify``: hash every file
    already in the cache (otherwise only what its record says changed, G35)."""
    old_commit = entry.get("commit") if entry else None
    if entry and not update and _entry_matches(entry, spec):
        files = dict(entry.get("files") or {})
        cdir = commit_dir(cache, spec.repo, entry["commit"])
        missing = [f for f in files if not (cdir / f).is_file()]
        if verify:
            ver = Verified(cache, spec.repo, entry["commit"], force=True)
            try:
                for f in files:
                    if f not in missing and \
                            ver.digest(f, cdir / f) != files[f].split(":", 1)[-1]:
                        raise _differs(cdir / f, cdir)
            finally:
                ver.save()
        if not missing:
            return entry, (f"{len(files)} file(s) hashed, as locked" if verify else None)
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
    ver = Verified(cache, spec.repo, info.commit, force=verify)
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
                digest = None if verify else known.get(f)
                digest = digest or ver.digest(f, cdir / f, want)
                if want and digest != want:
                    ver.save()
                    raise _differs(cdir / f, cdir)
            else:
                digest = store.add(f, want)
                ver.record(f, cdir / f, digest)
                new += 1
            files[f] = "sha256:" + digest
    ver.save()
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
# scensim.lock keeps only what came from the Hub.
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
            raise AssetError(f"assets/{key}: a file or directory scensim did not make (fix: move it"
                             " away; assets/ holds only links)")
        link.symlink_to(target, target_is_directory=True)
        msgs.append(f"linked assets/{key} -> {target}")
    return msgs


# --- the commands: -------------------------------------------------------------------------

def fetch(project: Path, update=False, hub=None, out=print, verify=False) -> dict:
    """Fetch the project's assets, write its lock and links; returns the lock. ``verify``:
    hash every cached file, not only those whose record says they changed (G35)."""
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
                                    update, verify)
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
        # a testbench's lock links to its source (081 step 3): say where it was written
        out(f"{os.path.realpath(path)} written" if path.is_symlink() else
            f"{config.LOCK_FILE} written")
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
            (cache / VERIFIED_DIR / repo / f"{cdir.name}.json").unlink(missing_ok=True)
    if not dry_run:
        tmp = cache / (PROJECTS_FILE + ".tmp")
        if cache.is_dir():
            tmp.write_text(json.dumps(sorted(alive), indent=1) + "\n")
            tmp.replace(cache / PROJECTS_FILE)
    out(f"{'would free' if dry_run else 'freed'} {freed / 1024 ** 2:.1f} MB; "
        f"{len(alive)} project(s) keep {len(used)} commit(s)")
    return removed


def check(project: Path, settings: dict = None, full_hash=False) -> list:
    """Findings: assets not fetched, not matching the lock, or not linked. A file is hashed
    when its stat key differs from the one it was last hashed under (G35); ``full_hash``:
    every file."""
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
        ver = Verified(cache, e["repo"], e["commit"], force=full_hash)
        for f, h in (e.get("files") or {}).items():
            p = cdir / f
            if not p.is_file():
                findings.append(Finding(lf, f"{name}: {f} is missing from the cache ({cdir})",
                                        FETCH, refused=True))
            elif "sha256:" + ver.digest(f, p, h.split(":", 1)[-1]) != h:
                findings.append(Finding(lf, f"{name}: {f} differs from the lock",
                                        f"delete {cdir} (chmod -R u+w first), then {FETCH}"))
        ver.save()
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


def testbenches_work(d) -> bool:
    """``d`` is an installed testbench's work directory (``<logs>/.testbenches/<pkg>``)."""
    return Path(d).parent.name == ".testbenches"


def _home(p) -> str:
    p, home = str(p), str(Path.home())
    return "~" + p[len(home):] if p == home or p.startswith(home + "/") else p


def testbench_dir(name: str, source=None, fetching=False) -> Path:
    """The directory to work on for testbench ``name`` (081 step 3): its work directory when it
    is installed (whose lock links to its source), else the directory itself. A fetch of an
    installed testbench without a known source is refused, naming --source."""
    from scensim import testbenches
    try:
        sp = testbenches.split(name)
        if sp is None:
            return testbenches.at_path(name).share
        tb = testbenches.find(sp[0])
        if source:
            src = Path(source).expanduser().resolve()
            if not (src / config.PROJECT_FILE).is_file():
                raise config.ConfigError(f"--source {source}: no {config.PROJECT_FILE} there")
            tb.source = lambda: src
        elif fetching and tb.source() is None:
            raise config.ConfigError(
                f"testbench {tb.name} was installed without --symlink-install, so its source "
                f"is not known: name it (scensim assets fetch {tb.name} --source "
                f"src/{tb.name}), or rebuild with colcon build --symlink-install")
        return testbenches.work_dir(tb, config.host_settings())
    except testbenches.TestbenchRefused as e:
        raise config.ConfigError(str(e)) from None


def main(argv=None):
    from scensim import signals
    signals.take_sigint()   # a fetch of GBs stops on its group's SIGINT, even from a `&` job
    parser = argparse.ArgumentParser(prog="scensim assets",
                                     description="A project's assets: fetch, list, check.")
    sub = parser.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("fetch", help="assets into the cache, the lock, links into assets/")
    f.add_argument("--update", action="store_true",
                   help="resolve every dataset's revision again instead of keeping the lock's")
    ls = sub.add_parser("list", help="the known datasets and what this project has fetched")
    f.add_argument("--verify", action="store_true",
                   help="hash every cached file again (otherwise only files whose size or"
                        " times changed since they were last hashed)")
    c = sub.add_parser("check", help="the linked files against scensim.lock: each file whose size"
                                     " or times changed since it was last hashed, hashed again")
    c.add_argument("--hash", action="store_true", help="hash every file, changed or not")
    pr = sub.add_parser("prune", help="remove cached commits no project's scensim.lock names")
    pr.add_argument("--dry-run", action="store_true", help="say what would go, remove nothing")
    for p in (f, ls, c):
        p.add_argument("testbench", nargs="?",
                       help="a testbench by name (the sourced workspace's) or directory")
    f.add_argument("--source", help="the testbench's source directory, where its lock is "
                                    "written (needed when it was installed without "
                                    "--symlink-install)")
    add_project_option(parser, f, ls, c)
    args = parser.parse_args(ros_args(sys.argv[1:] if argv is None else argv))
    tb_name = getattr(args, "testbench", None)
    if tb_name:
        try:
            args.project = str(testbench_dir(tb_name, getattr(args, "source", None),
                                             fetching=args.cmd == "fetch"))
        except config.ConfigError as e:
            print(f"problem: {e}", file=sys.stderr)
            return REFUSED
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
        if tb_name and testbenches_work(project):
            # the lines below name assets/... of a directory the user never opens: say once where
            print(f"testbench {tb_name}: this host's assets are in {_home(project)}/assets")
        if args.cmd == "fetch":
            config.writable_project(project)
            fetch(project, update=args.update, verify=args.verify)
            return 0
        findings = check(project, full_hash=args.hash)
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
