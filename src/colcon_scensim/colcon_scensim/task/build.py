"""``colcon build`` of a testbench (scensim roadmap 081): a ROS package whose package.xml
exports ``<build_type>scensim_testbench</build_type>``, holding ``testbench.yaml`` and the files it
names. Nothing compiles. The build:

1. installs every file of the package to ``share/<pkg>/``, a copy, or a link per file under
   ``--symlink-install`` (an edit takes effect without a rebuild; adding or removing a file
   needs one), skipping what a host writes at run time (``assets/``, ``state/``, ``runs/``,
   ``.scensim/``) and version-control and cache directories;
2. registers it in the ament index: ``packages/<pkg>`` and ``scensim_testbenches/<pkg>``, the
   latter holding the scensim the testbench was built over (JSON), which ``scensim run``
   compares with the one it runs from;
3. writes the package's environment hooks (``AMENT_PREFIX_PATH``) as colcon does for every
   type;
4. runs the testbench check on the installed tree (``scensim.testbench.build_findings``): any
   finding fails this package's build, each printed as ``testbench check`` prints it; notes
   (no lock, assets not checked at build) are printed and do not fail it.

The check comes from scensim itself, so the build needs scensim on its Python path: sourcing
scensim's setup.bash puts both scensim and this plugin there.
"""
import json
import os
from pathlib import Path

from colcon_core.environment import create_environment_scripts
from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.shell import create_environment_hook
from colcon_core.task import create_file
from colcon_core.task import install
from colcon_core.task import TaskExtensionPoint

logger = colcon_logger.getChild(__name__)

BUILD_TYPE = 'scensim_testbench'
INDEX = 'share/ament_index/resource_index'
# what the host writes in a testbench at run time, never installed
HOST_DIRS = frozenset({'assets', 'state', 'runs', '.scensim', '.swb'})
# version control, caches and colcon's own markers
SKIP_DIRS = frozenset({'.git', '.hg', '.svn', '__pycache__', '.pytest_cache', '.mypy_cache'})
SKIP_FILES = frozenset({'COLCON_IGNORE', 'AMENT_IGNORE', 'CATKIN_IGNORE'})


def package_files(path):
    """The package's files to install, relative to ``path``, sorted."""
    root = Path(path)
    out = []
    for dirpath, dirnames, filenames in os.walk(root):
        rel_dir = Path(dirpath).relative_to(root)
        dirnames[:] = sorted(
            d for d in dirnames
            if d not in SKIP_DIRS and not (rel_dir == Path('.') and d in HOST_DIRS))
        for f in sorted(filenames):
            if f in SKIP_FILES or f.endswith('.pyc'):
                continue
            out.append((rel_dir / f).as_posix())
    return out


def _check_module():
    """scensim's testbench check, or None when scensim is not on this Python's path."""
    try:
        from scensim import testbench
    except ImportError:
        return None
    return testbench


class TestbenchBuildTask(TaskExtensionPoint):
    """Build a ``scensim_testbench`` package: install it, index it, check it."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    async def build(self, *, additional_hooks=None, skip_hook_creation=False):  # noqa: D102
        args = self.context.args
        pkg = self.context.pkg
        logger.info(
            f"Building ROS package in '{args.path}' with build type '{BUILD_TYPE}'")
        check = _check_module()
        if check is None:
            logger.error(
                f"{pkg.name}: the {BUILD_TYPE} build type needs scensim on this Python's "
                "path for its check: source scensim's setup.bash before colcon build")
            return 1

        share = f'share/{pkg.name}'
        for rel in package_files(args.path):
            install(args, rel, f'{share}/{rel}')
        create_file(args, f'{INDEX}/packages/{pkg.name}')
        create_file(
            args, f'{INDEX}/scensim_testbenches/{pkg.name}',
            content=json.dumps(check.built_over(), sort_keys=True) + '\n')

        hooks = list(additional_hooks or [])
        hooks += create_environment_hook(
            'ament_prefix_path', Path(args.install_base), pkg.name,
            'AMENT_PREFIX_PATH', '', mode='prepend')
        if not skip_hook_creation:
            create_environment_scripts(pkg, args, additional_hooks=hooks)

        installed = Path(args.install_base) / share
        findings = check.build_findings(installed, installed / 'package.xml')
        for note in check.build_notes(installed):
            logger.warning(f'{pkg.name}: {note}')
        if findings:
            for f in findings:
                logger.error(f'{pkg.name}: {f}')
            logger.error(
                f'{pkg.name}: the testbench check found {len(findings)} problem(s); '
                'the package is not built')
            return 1
        return 0
