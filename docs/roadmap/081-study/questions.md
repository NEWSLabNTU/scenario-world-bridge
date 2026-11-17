# 081 and 082 on paper: design questions and rough edges

Found by 243 on 2026-11-17 while drafting [the install page](install-draft.md) and
[Tutorial 1 on a testbench](tutorial-1-testbench-draft.md) before either phase is built.
Each item names what the drafts assumed, so that a builder can confirm or change it. Most
important first.

## Decisions the user's path needs

1. **Who starts the stack.** 082's goal ends `scensim run my_first_drive`, with no stack
   step, but `swb run` today refuses when no stack is up (`--start-stack` opts in). Either
   the tutorial keeps a stack terminal (the draft does, with a new `scensim stack
   <testbench>`, since `ros2 launch scensim_launch scensim.launch.xml project:=` takes a
   directory or `pkg://` and the user now thinks in package names), or `scensim run` starts
   the stack when none serves the testbench and leaves it up. The first teaches what a
   stack is; the second has the fewest steps. 081 should decide, and name the command.
2. **What `scensim run <testbench>` runs.** 081 gives `<testbench>[/<suite>|/<scenario>]`.
   With no suffix: every suite, a default suite, or a refusal listing the choices? And a
   suite and a scenario may share a stem (`drive_to_goal.yaml` beside
   `scenarios/drive_to_goal.xosc`). The draft uses `my_first_drive/drive_to_goal` and assumes
   scenarios are found by stem; suites first, or `suites/<name>` and `scenarios/<name>`
   spelled out, would remove the ambiguity.
3. **How commands name a testbench's runs.** `runs list` and `runs show` today read the
   current directory's project. In a workspace the user is not inside a testbench. The
   draft assumes `scensim runs list <testbench>`; a run id alone should still work for
   `runs show`. Also, `<logs>/<pkg>/runs/` means two workspaces with a testbench of the same
   name share one runs directory: worth one sentence in the design (by design, or keyed by
   workspace).
4. **Sourcing Autoware.** 082's goal never sources Autoware, yet a `kind: autoware` pilot
   needs it, and the vehicle tarball is chosen by "a sourced Autoware". The draft's
   environment file sources ROS, Autoware, then scensim, and the workspace after. The order
   matters for the vehicle tarball's guard; 082 should state it.
5. **The first run downloads 65 MB on `scensim --version`.** A version query that downloads
   surprises users and CI. The draft documents `scensim install` as the explicit step, with
   the implicit download as a fallback; `--version` could print "not installed; run
   `scensim install`" instead.

## More root steps than "one rosdep line"

6. 082 says only the rosdep line needs root. A fresh apt ROS also needs `sudo rosdep init`
   (the ROS install guide does not always do it), and every host needs the socket-buffer
   sysctl (`99-cyclonedds-max.conf`), without which point clouds never arrive. The draft
   lists all three under "Once per machine"; 082's goal and exit criterion should count
   them, or `scensim install` could print all three lines together. The sysctl file must
   ship in the tarball (the draft assumes `share/scensim_launch/config/`).

## Python, pip and colcon

7. **The plugin and colcon must share a Python.** `colcon-scensim` is found through entry
   points of the Python that runs `colcon`. apt's colcon runs `/usr/bin/python3`; a
   `pip install --user` lands there, a plain venv does not, and the venv form for 24.04
   then fails with `unknown build type` (the draft's troubleshooting row). 082 should say
   which forms work, or ship `colcon-scensim` as the one thing installed with `--user`.
8. **Ubuntu 24.04 before Jazzy is released.** The `python_version == "3.12"` marker names
   `scensim-ros-jazzy`, which will not exist at the first release; pip's error would be a
   resolver failure, not a sentence. The draft assumes a clear message (`no scensim build
   for ROS 2 jazzy yet`); either the front end checks it, or the install page says 22.04
   only.
9. **A Python that is not ROS's** (a conda base, a pyenv 3.11) makes pip pick no release
   package, or the wrong one. The first-run guard catches a `ROS_DISTRO` mismatch; it should
   also name the Python actually used, since that is the usual cause.
10. **`current` moves under built workspaces.** Upgrading points `current` at the new
    version; a workspace built over the old one keeps its paths. The draft tells the user to
    rebuild; a guard in the workspace's setup (refuse a different scensim than it was built
    over, naming both) would make the rule unnecessary to remember.

## Testbench details the tutorial meets

11. **`package.xml`'s maintainer and licence.** `testbench new` must write something; the
    draft writes placeholders and asks the user to edit them. From `git config user.name`
    and `user.email`, when set, would save a step; the licence default needs a decision
    (Apache-2.0 like scensim's templates, or `TODO`).
12. **Package names.** A testbench is a ROS package, so `my-first-drive` is invalid.
    `testbench new` should refuse it, naming the valid form, before writing anything.
13. **Where `assets fetch` writes the lock without `--symlink-install`.** 081 finds the
    source through the symlink, else `--source <dir>`. A user who built without the flag
    (colcon's default) gets the second path silently or a refusal; the tutorial only works
    with the flag. Either `testbench new` notes it, or the lock goes next to
    `testbench.yaml` found by `colcon list`'s paths.
14. **`testbench check` at build cannot check assets**, so a fresh clone builds green and
    fails at the first run. The tutorial puts `assets fetch` after the build for that reason;
    the build could print one line when the lock names assets not yet fetched.
15. **Two command forms.** `scensim <cmd>` and `ros2 run scensim <cmd>` both work. The drafts
    use only `scensim`; every page should, with `ros2 run` mentioned once.

## Autoware

16. **NEWSLab's Autoware binaries and the link check.** 082 states NEWSLab's binaries "pass
    the same check", but the one host found on 2026-11-17 whose Autoware 1.5.0 had been
    reinstalled from a local repository (`/opt/autoware/1.5.0/repo`) failed it:
    `libautoware_compare_map_segmentation.so` uses `diagnostic_updater` without linking it,
    and Tutorial 1 could not pass there. If that local repository is the NEWSLab binaries
    082 means, the check fails today; if not, the two need different names in the docs. The
    current troubleshooting row (Tutorial 1, the reference) tells the user to "install
    Autoware 1.5.0's published packages", which must change once NEWSLab's binaries are the
    recommended ones.

## Wording

17. **"testbench" in the glossary**, and the move from "project": every page that says
    project (about 40) changes. A sentence for users of 0.1.0 is needed once
    (`swb.yaml` is refused naming `testbench.yaml`; their directory becomes a package with a
    `package.xml`), likely in the release notes rather than in each tutorial.

## Answers (2026-11-17)

133 decided items 1 to 8, 13 and 16; the drafts follow these answers. 243 decided the
others, as recorded; 133 folds both into 081 and 082 after the 080 landing.

| Item | Answer | By |
|---|---|---|
| 1 | `scensim run <tb>` starts the testbench's stack when none serves it, says so, and leaves it up (`--stop-stack` stops it after the run); `scensim stack <tb>` runs one in a terminal of its own. The tutorial uses plain `scensim run`. | 133 |
| 2 | `scensim run <tb>` runs the suite that `default:` in `testbench.yaml` names, and without one refuses, listing the suites. `<tb>/<name>` is a suite; a scenario is named by its path, `<tb>/scenarios/x.xosc`, so stems cannot clash. | 133 |
| 3 | Package names are unique in a sourced chain. Runs go to `<logs>/<pkg>/runs/`, with the workspace's path in `result.json`; two same-named testbenches in two workspaces share the directory, and `runs list` shows a workspace column. | 133 |
| 4 | Source order: ROS, the user's Autoware, scensim, then the workspace. | 133 |
| 5 | `scensim --version` never downloads. The first command that needs the binaries does, saying so with the size; `scensim install` does it on purpose. | 133 |
| 6 | One root block: `rosdep init` and `update` where never done, then the rosdep install line. No sysctl: the stack's localhost Cyclone profile does not need it; it is added only if a measurement asks. | 133 |
| 7 | The plugin ships in the tarball and is found through scensim's `setup.bash` (`PYTHONPATH` and its entry points), so apt's colcon sees it whatever pip form installed the front end; the venv form for 24.04 stays valid. 139 implements. | 133 |
| 8 | With no release package for the user's Python, the front end installs alone and its first command says `no prebuilt scensim for ROS <distro> yet; build from source`, with the link. | 133 |
| 9 | The front end's guard message names the Python it runs in beside the two ROS releases, since a conda or pyenv Python is the usual cause; the install draft's troubleshooting row shows it. A wording detail of 082 step 3. | 243 |
| 10 | The drafts keep "rebuild your workspaces after an update". A guard in a workspace's setup that refuses a different scensim than it was built over would remove the rule; proposed to 133 as a design change, not assumed. | 243 |
| 11 | `testbench new` takes the maintainer from `git config user.name` and `user.email` when set, else a placeholder, and writes `TODO: License declaration` as `ros2 pkg create` does: the licence is the user's choice. | 243 |
| 12 | `testbench new` refuses a name that is not a valid ROS package name before writing anything, naming the valid form (`my_first_drive`). | 243 |
| 13 | Without `--symlink-install`, `assets fetch` needs `--source <dir>` to write the lock; the build check warns when an installed testbench has no lock. | 133 |
| 14 | Covered by 13's warning: a testbench whose lock exists but whose assets are not fetched builds green and is refused at its first run, naming `scensim assets fetch <tb>`, as `project check` does today. | 243 |
| 15 | Every user page uses `scensim <cmd>`; `ros2 run scensim <cmd>` is named once, on the commands reference. | 243 |
| 16 | Asked of the user by 133; the troubleshooting row stays as it is until then. | 133 |
| 17 | The move from project to testbench is told once, in the release notes of the release that ships 081, with the `swb.yaml` refusal; the tutorials say only testbench. 243 writes it with 081 step 6. | 243 |

Open after these answers:

18. **Stopping a stack that `scensim run` left up.** 1 says how to stop it after one run
    (`--stop-stack`) but not later, from another command. The draft writes `scensim stack
    my_first_drive --stop`; 081 should name the form (today the stack is stopped by its
    process group, which a user cannot be expected to find).
