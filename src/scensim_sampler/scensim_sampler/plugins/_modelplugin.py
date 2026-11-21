"""A plug-in that draws from a model file (scensim_sampler.plugins.model)."""
from pathlib import Path

from . import Plugin
from ..dist import SpecError


def model_path(content, base_dir, parameter, what):
    text = (content or "").strip()
    if not text or "\n" in text:
        raise SpecError(f"{parameter}: the {what} distribution's content is the path of its "
                        f"file, not {text!r}")
    return text, Path(base_dir) / text


class ModelPlugin(Plugin):
    def __init__(self, model, parameter):
        if parameter is not None and parameter not in model.names \
                and parameter not in model.derived:
            raise SpecError(f"{model.where} draws {', '.join(model.names)}, not {parameter} "
                            f"(the parameterName it is given under)")
        self.model = model
        self.names = tuple(model.names) + tuple(model.derived)
        self.knobs = dict(model.knobs)

    def draw(self, rng, index, values):
        return self.model.sample(rng, values)

    def derived(self, values):
        return self.model.eval_derived(values)

    def derived_reads(self):
        return {k: set(e.names) for k, e in self.model.derived.items()}

    def violations(self, values):
        return self.model.violations(values)

    def cell(self, values):
        return self.model.eval_cell(values)
