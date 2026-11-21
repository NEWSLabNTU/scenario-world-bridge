"""`scensim.sampler:independent`: one distribution per parameter, none conditioned on another.

The standard OpenSCENARIO distributions (NormalDistribution, UniformDistribution, ...) are this
plug-in's (scensim_sampler.xosc builds them); as a UserDefinedDistribution its content is a
file in the model format with `independent: 1`, no header, and no `given` or mixture."""
from . import Plugin
from ._modelplugin import ModelPlugin, model_path
from .model import Given, Group, Model, read_yaml
from ..dist import SpecError


class Independent(Plugin):
    """Distributions by name: {parameter: scensim_sampler.dist.Dist}."""
    name = "independent"

    def __init__(self, dists):
        self.dists = dict(dists)
        self.names = tuple(self.dists)

    def add(self, name, d):
        if name in self.dists:
            raise SpecError(f"{name} has two distributions")
        self.dists[name] = d
        self.names = tuple(self.dists)

    def draw(self, rng, index, values):
        return {n: d.sample(rng, values) for n, d in self.dists.items()}


def from_content(content, *, base_dir, parameter, scenario=None):
    written, path = model_path(content, base_dir, parameter, "independent")
    doc, sha = read_yaml(path)
    model = Model(doc, str(path), sha, kind="independent")
    for e in model.entries:
        if isinstance(e, (Given, Group)):
            raise SpecError(f"{path}: {', '.join(e.names)}: independent variables have no "
                            f"given and no mixture (use scensim.sampler:joint)")
    p = ModelPlugin(model, parameter)
    p.path = path
    return p
