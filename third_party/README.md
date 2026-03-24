# Third-Party Cache

This directory is used as an optional local cache for build-time dependencies.

The repository does not need to commit the cached contents here. When the cache
is missing:

- `docker/Dockerfile.humble` falls back to the official upstream sources
- `scripts/bootstrap_third_party.sh` can pre-populate the cache on the host

Expected cache entries:

- `mujoco-3.3.6-linux-x86_64.tar.gz`
- `unitree_sdk2/`

Large local backup copies and extracted archives should stay untracked.
