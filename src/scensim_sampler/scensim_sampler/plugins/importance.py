"""`scensim.sampler:importance`: draw from a proposal, weighted by base / proposal.

Its content is the proposal file's path. The proposal names its base model (`base:`, relative
to the proposal) and replaces some of the base's variables, each by the same name (or group of
names), with the same discreteness; the rest are drawn as the base draws them. The base's
derived values, constraints and cell apply. The record carries both files' hashes and the
weight, with its logarithm (a small weight underflows a float). The weights are not
normalised: a rate is estimated self-normalised, sum(w * f) / sum(w)."""
import math
from pathlib import Path

from ._modelplugin import ModelPlugin, model_path
from .model import Model, entry, read_yaml
from ..dist import SpecError


def load_proposal(path):
    path = Path(path)
    doc, sha = read_yaml(path)
    where = str(path)
    if not isinstance(doc, dict) or doc.get("proposal") != 1:
        raise SpecError(f"{where}: `proposal: 1` (the format's version) is required")
    extra = set(doc) - {"proposal", "name", "base", "variables"}
    if extra:
        raise SpecError(f"{where}: unknown {sorted(extra)} (a proposal has proposal, name, "
                        f"base, variables)")
    for k in ("name", "base"):
        if not isinstance(doc.get(k), str) or not doc[k].strip():
            raise SpecError(f"{where}: {k} is required")
    variables = doc.get("variables")
    if not isinstance(variables, list) or not variables:
        raise SpecError(f"{where}: variables is a non-empty list (the base's it replaces)")
    entries = [entry(e, f"{where}: variables[{i}]") for i, e in enumerate(variables)]
    return doc, sha, entries


class Importance(ModelPlugin):
    def __init__(self, content=None, *, base_dir=".", parameter=None, scenario=None,
                 proposal_path=None, base_plugin=None):
        if proposal_path is None:
            _, proposal_path = model_path(content, base_dir, parameter, "importance")
        doc, sha, entries = load_proposal(proposal_path)
        base_path = (Path(proposal_path).parent / doc["base"]).resolve()
        if base_plugin is not None:
            if Path(base_plugin.path).resolve() != base_path:
                raise SpecError(f"{proposal_path}: its base is {base_path}, the scenario's "
                                f"model is {Path(base_plugin.path).resolve()}")
            base_model, self.base = base_plugin.model, base_plugin.base
        else:
            bdoc, bsha = read_yaml(base_path)
            base_model = Model(bdoc, str(base_path), bsha, kind="model")
            self.base = {"id": doc["base"], "sha256": bsha}
        super().__init__(base_model, parameter)
        self.base_model = base_model
        self.proposal_model = base_model.replace(entries)
        self.proposal = {"id": doc["name"], "sha256": sha}
        self.path = base_path

    def draw(self, rng, index, values):
        return self.proposal_model.sample(rng, values)

    def log_weight(self, values):
        lb = self.base_model.logpdf(values)
        if lb == -math.inf:
            return lb
        lq = self.proposal_model.logpdf(values)
        if lq == -math.inf:
            raise SpecError(f"{self.proposal['id']}: drew values its own density gives 0")
        return lb - lq
