
# Change Log

## [0.3.0] - 2025-06-03

### Added

### Changed

- Acados vendor ros2 v0.3.6 -> v0.4.0
- Comply with Acados Python API breaking changes:
  * `AcadosSimOpts` becomes `AcadosSimOptions`
  * `dims.N` is obsolete, now using `AcadosOcp.solver_options.N_horion`
- Update testing generated C-code

## [0.2.2] - 2025-01-19

### Added

### Changed

- Acados vendor ros2 v0.3.2 -> v0.3.6
- Update tests

## [0.2.1] - 2024-04-24

### Added

### Changed

- Acados vendor ros2 v0.3.0 -> v0.3.2

### Fixed

- [PR #7](https://github.com/ICube-Robotics/acados_solver_ros2/pull/7) Fixed `utils::get_stats_cost_value()` getter.

- [PR #6](https://github.com/ICube-Robotics/acados_solver_ros2/pull/6) Fixed `AcadosSolver::set_runtime_parameters()` getter.

- Fixed CI for doc generation.

## [0.2.0] - 2024-02-14

Note: the URL has changed from `https://github.com/tpoignonec/acados_controllers_ros2` to  `https://github.com/ICube-Robotics/acados_solver_ros2`. See [PR #1 (acados_solver_ros2)](https://github.com/tpoignonec/acados_solver_ros2/pull/1)

### Added

- [PR #32](https://github.com/tpoignonec/acados_controllers_ros2/pull/32) An example package for plugin export.
- Some more documentation in the "getting started" section.

### Changed

- [PR #2 (acados_solver_ros2)](https://github.com/tpoignonec/acados_solver_ros2/pull/2) Acados vendor ros2 v0.3.0
- [PR #32](https://github.com/tpoignonec/acados_controllers_ros2/pull/32) Package export namespace.

### Fixed

## [0.1.2] - 2023-11-15

### Added
- [PR #25](https://github.com/tpoignonec/acados_controllers_ros2/pull/25)
  Auto-regenerate CMAKE after plugin export.

### Changed

### Fixed

## [0.1.1] - 2023-11-14

First version after beta.

### Added

### Changed

### Fixed
