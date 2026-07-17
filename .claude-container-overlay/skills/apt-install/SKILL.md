---
name: apt-install
description: Install apt packages inside claude-container so they both work NOW and persist across container relaunches. Use whenever an apt/system package is needed in the container — especially when apt-get fails with a permission error (no root/sudo) — instead of raw apt-get or hand-editing the overlay Dockerfile.
---

# apt-install: install now + persist in the container overlay

The claude-container image is ephemeral: packages installed at runtime vanish
on relaunch, and the runtime user usually has no root anyway. The `apt-install`
helper solves both halves at once — it installs the package via passwordless
sudo **and** appends a dated `RUN` block to the workspace's
`.claude-container-overlay/Dockerfile`, so the next image rebuild bakes the
package in permanently (see the `container-overlay` skill for the overlay
mechanics).

## Using it

If `apt-install` is on PATH (this workspace already bootstrapped it):

```sh
apt-install ffmpeg jq
```

That's the whole workflow. It runs `sudo apt-get update && apt-get install -y
--no-install-recommends <pkgs>`, then records any package not already listed in
the overlay Dockerfile as a new bottom-of-file `RUN` block (cache-friendly:
earlier layers stay valid). Re-recording is deduped by exact package-line
match, so running it twice, or on a package someone recorded by hand, is safe.

If it prints "not root and no sudo in this image", the workspace has the
overlay entries but the container predates them — ask the user to relaunch
claude-container, then retry.

## Bootstrapping a workspace that doesn't have it

Two additions to `<workspace>/.claude-container-overlay/` (create the
directory if missing):

1. Copy the `apt-install` script from this skill's directory (it sits next to
   this SKILL.md) into `.claude-container-overlay/apt-install` and make it
   executable.

2. Append to `.claude-container-overlay/Dockerfile` (dated per overlay
   convention):

   ```dockerfile
   # YYYY-MM-DD: passwordless sudo so `sudo apt-get install ...` works at
   # runtime. The rule matches ALL users because the runtime user is created
   # at container start with a host-mapped UID (no fixed name to grant).
   RUN apt-get update && apt-get install -y --no-install-recommends \
           sudo \
       && rm -rf /var/lib/apt/lists/* \
       && echo 'ALL ALL=(ALL:ALL) NOPASSWD:ALL' > /etc/sudoers.d/anyuser-nopasswd \
       && chmod 0440 /etc/sudoers.d/anyuser-nopasswd

   # YYYY-MM-DD: apt-install helper — `apt-install <pkg>...` installs via
   # sudo apt-get AND appends a dated RUN block to this Dockerfile, so
   # runtime installs survive the next image rebuild. Blocks append below
   # this line, keeping the layers above cached.
   COPY apt-install /usr/local/bin/apt-install
   RUN chmod 0755 /usr/local/bin/apt-install
   ```

3. Tell the user both features go live at the next claude-container launch
   (the sudo layer must exist before the script can install anything). If the
   package is needed *this* session, there is no root — everything must wait
   for the relaunch.

## Notes

- The script targets `/workspace/.claude-container-overlay` (claude-container's
  fixed mount point); `CLAUDE_OVERLAY_DIR` overrides it, mainly for testing.
- Packages installed by `apt-install` before a relaunch live on the container
  fs only until the relaunch — but because they're recorded, the rebuilt image
  has them; nothing is lost either way.
- Keep using plain `RUN` blocks (container-overlay skill) for non-apt state:
  binary downloads, `COPY` configs, Nix, etc. `apt-install` covers apt only.
