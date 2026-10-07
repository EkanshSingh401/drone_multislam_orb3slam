# DECISIONS (overnight run, started 2026-10-07 02:00 EDT)

Decisions made without the user, each with what / why / alternatives.

## Stage 0
- **Package built in its own Dockerfile step after the fork build** (`COPY active_slam_sim`, then
  `colcon build --packages-select active_slam_sim` with executable checks). Why: it links ov_msckf
  and active_slam_information, so it must come after them; a separate step keeps a failure local.
  Alt: adding it to the existing fork colcon invocation (couples an experimental package to the VIO build).
- **Closed-loop harness stays in `docker/tools/closed_loop/` and runs from `/out/cl/`** (copied),
  now using the image's binaries only. Alt: bake it into /opt/scripts (would trigger the md5 guard
  and require rebuilds for every harness edit during the night).
