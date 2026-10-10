# Devcontainer

System dependencies are installed by `rosdep` from the `package.xml` files under `src/`
(see `Dockerfile`). To add a dependency, declare it in the right `package.xml`, then
either rebuild the container or run inside it:

```bash
rosdep install --from-paths src --ignore-src -y
```

## Deferred optimization: manifest-only build context

**Current state:** `.dockerignore` only excludes `.git`, `build`, `install` and `log`, so
`COPY src /tmp/src` copies all sources. Any source edit invalidates the rosdep layer, so the
next rebuild reruns the full `rosdep install` (slower, but correct).

**Goal:** put only the `package.xml` files into the build context, so the rosdep layer is
rebuilt only when a dependency declaration changes.

**Implementation:** replace `.dockerignore` with

```
*
!src
src/**
!src/**/package.xml
```

and update the comment above `COPY src /tmp/src` in the `Dockerfile` accordingly.

**Prerequisite: BuildKit (buildx) must be installed on the host.** The legacy Docker
builder cannot re-include files below an excluded directory when the exception contains a
wildcard (`!src/**/package.xml`): it skips `src/<pkg>/` entirely. `/tmp/src` then arrives
empty, `rosdep` finds no packages, reports success, and the image silently lacks all
dependencies. This happened once already.

Check for BuildKit:

```bash
docker buildx version
# if missing: sudo apt install docker-buildx         (Ubuntu docker.io)
#             sudo apt install docker-buildx-plugin  (Docker CE repo)
```

**Verify before relying on it.** From the repo root, this must list every `package.xml`
under `src/` (including the ones inside the submodules):

```bash
printf 'FROM busybox\nCOPY src /tmp/src\nRUN find /tmp/src -type f\n' | \
  docker build --no-cache --progress=plain -f - . 2>&1 | grep package.xml
```

Then rebuild the devcontainer without cache and check inside it:

```bash
rosdep check --from-paths src --ignore-src   # "All system dependencies have been satisfied"
```

Consider also dropping `-r` from `rosdep install` in the `Dockerfile` so unresolvable keys
fail the build instead of being skipped. It would not have caught the empty-context case,
though, which is why the verification step above is required.
