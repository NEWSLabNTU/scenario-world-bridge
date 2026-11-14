"""The Autoware profile's own argument check, before a pilot starts.

autoware_sim_profile's ``check_launch_args`` refuses the arguments roadmap 051 renamed, a value
outside an argument's choices (``ndt:=``) and a pair that would launch without what one of
them asks for (``ndt:=none`` with a diagnostic graph that judges NDT). ROS launch drops an
undeclared name without a word, so these would otherwise surface as a wrong run, or not at
all. ``project check`` (and so the stack's plan) calls the profile's own ``problems()`` on
every ``autoware`` pilot's ``args`` and on the suites' ``pilot_args`` for it: the table is the
profile's, never a copy here.

The script is found installed (``lib/autoware_sim_profile/check_launch_args`` under the
package's prefix) or in this workspace's source tree; when neither is there the check is
skipped (the profile is not built, and no autoware pilot can start either).
"""
import importlib.machinery
import importlib.util
import os
from pathlib import Path

SCRIPT = "check_launch_args"
_cache = {}


def _candidates():
    try:
        from ament_index_python.packages import get_package_prefix
        yield Path(get_package_prefix("autoware_sim_profile")) / "lib" / \
            "autoware_sim_profile" / SCRIPT
    except Exception:  # noqa: BLE001 (not installed, no ament: the source tree)
        pass
    here = Path(__file__).resolve()
    for up in here.parents:
        p = up / "src" / "autoware_sim_vehicle" / "autoware_sim_profile" / "scripts" / SCRIPT
        if p.is_file():
            yield p
            break


def checker(path=None):
    """The profile's ``problems(args) -> [message]``, or None when it is not found."""
    key = str(path) if path else None
    if key in _cache:
        return _cache[key]
    fn = None
    for p in ([Path(path)] if path else _candidates()):
        if not p.is_file():
            continue
        try:
            loader = importlib.machinery.SourceFileLoader("_swb_check_launch_args", str(p))
            spec = importlib.util.spec_from_loader(loader.name, loader)
            mod = importlib.util.module_from_spec(spec)
            loader.exec_module(mod)
            fn = getattr(mod, "problems", None)
        except Exception:  # noqa: BLE001 (a broken script is the profile's to report)
            fn = None
        if fn is not None:
            break
    _cache[key] = fn
    return fn


def _text(v) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    return "" if v is None else str(v)


def arg_problems(args: dict, project: Path = None, check=None) -> list:
    """The profile's messages for ``args`` (launch arguments, name -> value). A value holding
    a run substitution (``{param:x}``, ``{seed}``, ...) is left out: it is known per run only.
    A relative file value is read against the project, as the stack's processes see it."""
    fn = check if check is not None else checker()
    if fn is None or not args:
        return []
    words = []
    for k, v in args.items():
        t = _text(v)
        if "{" in t and "}" in t:
            continue
        if project is not None and t and not os.path.isabs(t) and (Path(project) / t).is_file():
            t = str((Path(project) / t).resolve())
        words.append(f"{k}:={t}")
    try:
        return list(fn(words))
    except Exception as e:  # noqa: BLE001
        return [f"the profile's check_launch_args failed: {e}"]
