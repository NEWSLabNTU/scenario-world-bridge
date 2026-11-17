"""``colcon test`` on a testbench: nothing to run (its check runs at build)."""
from colcon_core.plugin_system import satisfies_version
from colcon_core.task import TaskExtensionPoint


class TestbenchTestTask(TaskExtensionPoint):
    """Test a ``scensim_testbench`` package: its check ran at build, so nothing here."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    async def test(self, *, additional_hooks=None):  # noqa: D102
        return 0
