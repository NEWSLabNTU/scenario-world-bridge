"""Sampler plug-ins: what an OpenSCENARIO `UserDefinedDistribution type="scensim.sampler:<name>"`
names. A plug-in is found by name in the entry point group `scensim.sampler` (an installed
package declares `scensim.sampler = name = module:factory`); the four shipped here are also
known without an install, so the source tree works as it is.

A factory is called as `factory(content, base_dir=..., parameter=..., scenario=...)`:
`content` is the element's text (often a file path, relative to `base_dir`, the distribution
file's directory), `parameter` the parameterName it was written under, `scenario` the parsed
scenario (scensim_sampler.xosc.Scenario). It returns a Plugin.
"""
import importlib
import math
from importlib import metadata

GROUP = "scensim.sampler"
PREFIX = GROUP + ":"
BUILTIN = {
    "independent": "scensim_sampler.plugins.independent:from_content",
    "joint": "scensim_sampler.plugins.joint:Joint",
    "importance": "scensim_sampler.plugins.importance:Importance",
    "twise": "scensim_sampler.plugins.twise:TWise",
}


class PluginError(ValueError):
    pass


class Plugin:
    """What the sampler asks of a plug-in. Every method has a default but `draw` (or `rows`,
    for a deterministic one)."""
    name = "?"
    names = ()             # the values it assigns: scenario parameters or cell attributes
    deterministic = False  # True: rows() enumerates; draw(index) cycles through them
    base = None            # {"id", "sha256"}: the fitted model it draws from, if any
    proposal = None        # {"id", "sha256"}: the proposal it draws from instead, if any
    knobs = {}             # world knobs' defaults it brings (the sampler's override them)

    def derived_reads(self):
        """{derived name: the names its expression reads}: what a cell computed from a derived
        value may depend on is checked against this (scenario-description.md section 7)."""
        return {}

    def draw(self, rng, index, values):
        rows = self.rows()
        return dict(rows[index % len(rows)])

    def rows(self):
        raise PluginError(f"{self.name}: not a deterministic distribution")

    def log_weight(self, values):
        """log(base / proposal) at these values: 0 without a proposal; -inf outside the base's
        support (the draw is rejected)."""
        return 0.0

    def violations(self, values):
        return []

    def derived(self, values):
        return {}

    def cell(self, values):
        return {}


def known():
    """{name: "module:attr"}: the shipped plug-ins, then the installed entry points, which
    may replace one. Two installed packages giving one name different targets is refused."""
    out = dict(BUILTIN)
    seen = {}
    for ep in _entry_points():
        if ep.name in seen and seen[ep.name] != ep.value:
            raise PluginError(f"two plug-ins named {ep.name!r} in {GROUP}: {seen[ep.name]} "
                              f"and {ep.value}")
        seen[ep.name] = ep.value
    out.update(seen)
    return out


def _entry_points():
    eps = metadata.entry_points()
    if hasattr(eps, "select"):
        return list(eps.select(group=GROUP))
    return list(eps.get(GROUP, []))  # Python < 3.10


def load(type_name, content, *, base_dir, parameter, scenario):
    if not type_name.startswith(PREFIX):
        raise PluginError(f"UserDefinedDistribution type {type_name!r} for {parameter}: not "
                          f"a scensim sampler ({PREFIX}<name>)")
    name = type_name[len(PREFIX):]
    table = known()
    if name not in table:
        raise PluginError(f"no sampler plug-in {name!r} (for {parameter}); installed: "
                          f"{', '.join(sorted(table))}")
    module, _, attr = table[name].partition(":")
    factory = getattr(importlib.import_module(module), attr)
    plugin = factory(content, base_dir=base_dir, parameter=parameter, scenario=scenario)
    plugin.name = name
    return plugin


NEG_INF = -math.inf
