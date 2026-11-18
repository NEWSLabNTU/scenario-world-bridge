"""Where the coop_pilots package installs scripts/coop (roadmap 081 step 4, group C): the
directory this testbench's judges, pilots and generator put on sys.path, and whose agents they
run. Never the checkout: without the package this fails, naming it."""
import sys
from pathlib import Path


def coop_dir() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        d = Path(get_package_share_directory("coop_pilots")) / "coop"
    except Exception as e:  # noqa: BLE001 (not installed, or no ROS sourced)
        sys.exit(f"coop_pilots is not installed ({type(e).__name__}: {e}): build it (just build) "
                 "and source the workspace's install/setup.bash")
    if not (d / "measures.py").is_file():
        sys.exit(f"coop_pilots is installed without its coop modules ({d}): rebuild it "
                 "(just build)")
    return d
