"""A project's ground: its own per-map surface files, put where the world server reads them.

``ground:`` in ``testbench.yaml`` (docs/design/projects-and-pilots.md, "Ground")::

    ground:
      Town01:
        surfaces: ground/town01.yaml        # Isaac's surface map for Town01
        road_detail: ground/rough.yaml      # optional: the road's rocks and ruts

A file is the project's (a path in it) or another package's, ``pkg://<package>/<path>`` (081;
issue 054).

The Isaac world server takes ``<state dir>/<map>/surfaces.yaml`` and ``road_detail.yaml`` over
the (read-only, fetched) map directory's. ``apply`` links each named file there when a stack
starts, so the project's file is the one the world loads; a link it made before and the
project no longer names is removed. A file there that is not such a link (one a user put by
hand) is refused rather than replaced.
"""
import os
from pathlib import Path

KINDS = {"surfaces": "surfaces.yaml", "road_detail": "road_detail.yaml"}


class GroundError(RuntimeError):
    pass


def source(project, f: str) -> Path:
    """The file ``f`` names: ``pkg://<package>/<path>`` in that package's share, else a path
    in the project (unresolved: see ``apply``). GroundError when the package is not installed."""
    if f.startswith("pkg://"):
        from scensim import config
        try:
            return config.pkg_path(f)
        except config.ConfigError as e:
            raise GroundError(str(e)) from None
    return Path(project).resolve() / f


def problems(project, ground) -> list:
    """``(where, message)`` for every way ``ground:`` is wrong; with a ``project``, files
    that are not there too."""
    if ground is None:
        return []
    if not isinstance(ground, dict):
        return [("ground", "is not a mapping: {<map>: {surfaces: <file>}}")]
    out = []
    for m, files in ground.items():
        w = f"ground.{m}"
        if not isinstance(files, dict) or not files:
            out.append((w, "is not a mapping: {surfaces: <file>, road_detail: <file>}"))
            continue
        for k, f in files.items():
            if k not in KINDS:
                out.append((w, f"unknown key {k} (use only {', '.join(KINDS)})"))
            elif not isinstance(f, str) or not f:
                out.append((f"{w}.{k}", "a file in the project is required"))
            elif project is not None:
                try:
                    there = source(project, f).is_file()
                except GroundError as e:
                    out.append((f"{w}.{k}", str(e)))
                    continue
                if not there:
                    out.append((f"{w}.{k}", f"{f} is not a file in the project"
                                if not f.startswith("pkg://") else f"{f} is not a file"))
    return out


def _ours(path: Path, project: Path) -> bool:
    """A link scensim made: into the project, or into an installed package's share (a pkg://
    file)."""
    if not path.is_symlink():
        return False
    target = os.readlink(path)
    if target.startswith(str(Path(project).resolve()) + "/"):
        return True
    from scensim import config
    return bool(config.package_project(target))


def apply(project: Path, ground, state_dir: Path = None) -> list:
    """Link the project's ground files into ``<state>/<map>/``; the links made."""
    project = Path(project)
    state = Path(state_dir) if state_dir else project / "state"
    made, want = [], set()
    for m, files in (ground or {}).items():
        for k, f in (files or {}).items():
            if k not in KINDS:
                continue
            dst = state / m / KINDS[k]
            want.add(dst)
            # the link names the file's path in the project, not its resolved one: a file under
            # assets/ resolves into the shared dataset cache, outside the project, and the
            # next start would not know the link for its own (issue 043)
            src = source(project, f)
            if not src.is_file():
                raise GroundError(f"ground.{m}.{k}: {f} is not a file"
                                  + ("" if f.startswith("pkg://") else " in the project"))
            if dst.exists() or dst.is_symlink():
                # also ours: a link an older scensim made to the resolved file (issue 043)
                if not (_ours(dst, project) or
                        (dst.is_symlink() and Path(os.path.realpath(dst)) == src.resolve())):
                    raise GroundError(f"{dst} is not a link scensim made: move it into the project "
                                      f"and name it under ground.{m}.{k}")
                dst.unlink()
            dst.parent.mkdir(parents=True, exist_ok=True)
            dst.symlink_to(src)
            made.append(dst)
    # links of files the project no longer names
    if state.is_dir():
        for m_dir in state.iterdir():
            for name in KINDS.values():
                p = m_dir / name
                if p not in want and _ours(p, project):
                    p.unlink()
    return made
