# pypi/: scensim's pip packages (roadmap 082 step 3)

- `scensim/`: the front end (`py3-none-any`, module `scensim_frontend`, console script
  `scensim`). `scensim --version` never downloads; `scensim install [--distro] [--autoware]`
  and the first command that needs scensim download the release tarball named by the release
  package, check its size and sha256, unpack it to `~/.local/share/scensim/<ver>-<distro>/sim/`
  (and `vehicle-autoware<rel>/`; `SCENSIM_HOME` overrides the root), point `current`, print
  the rosdep and source lines, then run the tarball's `bin/scensim`.
- `scensim-ros-jazzy/`: the same for ROS 2 Jazzy (Python 3.12), the simulation side only:
  the vehicle side is Autoware 1.5.0's, which is built for Humble only, and the front end
  refuses `--autoware` on Jazzy saying so.
- `scensim-ros-humble/`: only `manifest.json` (URL, size, sha256 per artefact), written by
  `make_manifest.py` from the release workflow's artefacts. The front end requires it by
  marker (`python_version == "3.10"`; Jazzy's, 3.12, requires scensim-ros-jazzy).

Tests (a fake release served on localhost): `cd scensim && uv run --no-project --with pytest
--with zstandard python -m pytest tests`.
