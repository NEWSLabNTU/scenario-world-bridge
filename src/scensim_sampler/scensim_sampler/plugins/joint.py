"""`scensim.sampler:joint`: a fitted model of several parameters, drawn together with their
dependencies kept (scensim_sampler.plugins.model). Its header must name the data's dataset,
version and licence."""
from ._modelplugin import ModelPlugin, model_path
from .model import Model, read_yaml


class Joint(ModelPlugin):
    def __init__(self, content, *, base_dir, parameter, scenario=None):
        written, path = model_path(content, base_dir, parameter, "joint")
        doc, sha = read_yaml(path)
        super().__init__(Model(doc, str(path), sha, kind="model"), parameter)
        self.path = path
        self.base = {"id": written, "sha256": sha}
