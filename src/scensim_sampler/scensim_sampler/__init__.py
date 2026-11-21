"""scensim_sampler: OpenSCENARIO parameter distributions to concrete parameter sets, each with
its sample record (docs/design/sample-record.md). Pure Python (numpy, PyYAML); no simulator.

    from scensim_sampler import Sampler, check
    batch = Sampler.from_file("walker_dartout_dist.xosc").draw(n=100, seed=7)
    assert all(not check(r) for r in batch.records)
"""
__version__ = "0.1.0"

from .dist import SpecError  # noqa: E402
from .record import check, read, record_id  # noqa: E402
from .sampler import Batch, Sampler, SamplingError  # noqa: E402

__all__ = ["Batch", "Sampler", "SamplingError", "SpecError", "check", "read", "record_id",
           "__version__"]
