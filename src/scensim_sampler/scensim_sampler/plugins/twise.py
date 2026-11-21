"""`scensim.sampler:twise`: a covering array over discrete factors, so that every combination
of the levels of any t factors appears in at least one row (t = 2: every pair).

Content: inline YAML, or the path of a YAML file, `{t: 2, factors: {name: [levels]}}`. A factor
is a scenario parameter or a cell attribute. Deterministic: the rows are enumerated; under
Stochastic, run i takes row i mod rows (so N >= rows runs cover every t-combination)."""
import itertools
from pathlib import Path

import yaml

from . import Plugin
from ..dist import SpecError

MAX_CANDIDATES = 200000


def covering_array(factors, t):
    """Greedy: each row is the full combination that covers the most uncovered t-tuples
    (first one on a tie, in the factors' order), until every t-tuple is covered."""
    names = list(factors)
    t = min(t, len(names))
    levels = [list(factors[n]) for n in names]
    total = 1
    for lv in levels:
        total *= len(lv)
    if total > MAX_CANDIDATES:
        raise SpecError(f"twise: {total} full combinations, more than this builder takes "
                        f"({MAX_CANDIDATES})")
    combos = list(itertools.combinations(range(len(names)), t))
    uncovered = {(c, tuple(lv)) for c in combos
                 for lv in itertools.product(*(range(len(levels[i])) for i in c))}
    rows = []
    candidates = list(itertools.product(*(range(len(lv)) for lv in levels)))
    while uncovered:
        best, gain = None, -1
        for cand in candidates:
            g = sum((c, tuple(cand[i] for i in c)) in uncovered for c in combos)
            if g > gain:
                best, gain = cand, g
        uncovered -= {(c, tuple(best[i] for i in c)) for c in combos}
        rows.append({n: levels[i][best[i]] for i, n in enumerate(names)})
    return rows


class TWise(Plugin):
    deterministic = True

    def __init__(self, content, *, base_dir, parameter, scenario=None):
        text = (content or "").strip()
        if text.endswith((".yaml", ".yml")) and "\n" not in text:
            path = Path(base_dir) / text
            try:
                doc = yaml.safe_load(path.read_text())
            except OSError as e:
                raise SpecError(f"{path}: {e.strerror}") from None
            where = str(path)
        else:
            doc, where = yaml.safe_load(text), f"twise ({parameter})"
        if not isinstance(doc, dict) or set(doc) - {"t", "factors"}:
            raise SpecError(f"{where}: {{t: 2, factors: {{name: [levels]}}}}")
        t = doc.get("t", 2)
        if isinstance(t, bool) or not isinstance(t, int) or t < 1:
            raise SpecError(f"{where}: t is an integer >= 1, not {t!r}")
        factors = doc.get("factors")
        if (not isinstance(factors, dict) or not factors
                or any(not isinstance(v, list) or not v for v in factors.values())):
            raise SpecError(f"{where}: factors is {{name: [level, ...]}}, each non-empty")
        if parameter not in factors:
            raise SpecError(f"{where}: written under {parameter}, which is not a factor "
                            f"({', '.join(factors)})")
        self.t = t
        self.names = tuple(factors)
        self._rows = covering_array(factors, t)

    def rows(self):
        return self._rows
