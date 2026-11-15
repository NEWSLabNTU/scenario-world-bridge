"""The configuration: conventions/<jurisdiction>.yaml, params.yaml, contracts.yaml (R12: conventions
are configuration, not code). Files ship inside the package (czp/config/); CZP_CONFIG names another
directory with the same layout."""

from __future__ import annotations

import copy
import os
from dataclasses import fields

import yaml

from .model import Capability

HERE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config")


def config_dir(path=None):
    return path or os.environ.get("CZP_CONFIG") or HERE


class Params:
    """params.yaml: every entry {value, unit, source, heldout}; attribute access gives the value.
    ``override(**kw)`` returns a copy with values replaced and their source set to the caller's."""

    def __init__(self, entries, classes):
        self._e = entries
        self.classes = classes

    def __getattr__(self, k):
        e = self.__dict__.get("_e", {})
        if k in e:
            return e[k]["value"]
        raise AttributeError(k)

    def source(self, k):
        return self._e[k].get("source", "")

    def names(self):
        return list(self._e)

    def override(self, source="override", **kw):
        e = copy.deepcopy(self._e)
        for k, v in kw.items():
            if k not in e:
                raise KeyError(f"unknown parameter {k}")
            e[k] = dict(e[k], value=v, source=source)
        return Params(e, copy.deepcopy(self.classes))

    def capability(self, cls="car", **kw):
        c = dict(self.classes.get(cls) or self.classes["car"])
        c.update(kw)
        names = {f.name for f in fields(Capability)}
        return Capability(**{k: float(v) for k, v in c.items() if k in names})


def load_params(path=None):
    p = path if path and path.endswith(".yaml") else os.path.join(config_dir(path), "params.yaml")
    raw = yaml.safe_load(open(p))
    classes = raw.pop("classes", {})
    classes.pop("source", None)
    for k, e in raw.items():
        if not isinstance(e, dict) or "value" not in e or "source" not in e:
            raise ValueError(f"params.yaml: {k} needs a value and a source")
    return Params(raw, classes)


def load_conventions(jurisdiction="generic", path=None):
    d = os.path.join(config_dir(path), "conventions")
    raw = yaml.safe_load(open(os.path.join(d, f"{jurisdiction}.yaml")))
    base = raw.pop("base", None)
    if base:
        merged = load_conventions(base, path)
        for k, v in raw.items():
            merged[k] = dict(merged.get(k) or {}, **v) if isinstance(v, dict) else v
        return merged
    return raw


def load_contracts(path=None):
    p = path if path and path.endswith(".yaml") else os.path.join(config_dir(path), "contracts.yaml")
    return yaml.safe_load(open(p))
