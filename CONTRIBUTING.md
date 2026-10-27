# Contributing

This repository is maintained by Studio Sanka members. We are not accepting pull requests or outside contributions at this time.

Found a bug, or have a question or idea? Please open an issue: https://github.com/studiosanka/sixeyes_ros2/issues. We read everything, though we cannot promise a reply to every message.

Licence: Apache-2.0 (see `LICENSE`).

## For maintainers

- Run `python scripts/check.py` before committing.
- Never edit `sixeyes_hardware/include/sixeyes/`, `sixeyes_hardware/include/sixeyes_protocol.h` or `sixeyes_hardware/src/host/`: change `sixeyes-host`, then `python scripts/sync-host-core.py`.
- Keep the README to one screen and technical detail in `docs/appendix/`. Keep the "untested" labels until CI has run on ROS 2.
