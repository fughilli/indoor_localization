# Overlay for the UWB indoor positioning (ESP32 + DWM3000) Bazel/Nix workspace.
# Host-level bootstrap only; compilers, Arduino cores, and esptool come from
# the third_party/embedded flake via rules_nixpkgs, NOT from here.
# Stanzas copied from fughilli/embedded's proven .claude-container-overlay.
# (Bazelisk already ships in this base image at /usr/local/bin/bazelisk.)

# 2026-07-17: openssh-client — the runtime user has no root, and git needs ssh
# for the private third_party/embedded submodule + pushes (repo-local deploy
# key at credentials/id_ed25519_personal_github).
RUN apt-get update && apt-get install -y --no-install-recommends \
        openssh-client \
    && rm -rf /var/lib/apt/lists/*

# 2026-07-17: Nix (flakes) via the Determinate installer. Needs no sudo and
# --init none skips the systemd daemon that has no place in a container.
# build-users-group is cleared so builds run as the calling user (no nixbld
# group); sandbox is off because the container can't set up build-sandbox user
# namespaces unprivileged.
RUN curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix \
        -o /tmp/nix-installer.sh \
    && sh /tmp/nix-installer.sh install linux \
        --init none \
        --no-confirm \
        --extra-conf "experimental-features = nix-command flakes" \
        --extra-conf "build-users-group =" \
        --extra-conf "sandbox = false" \
    && rm /tmp/nix-installer.sh
ENV PATH=/nix/var/nix/profiles/default/bin:$PATH

# 2026-07-17: make the single-user store writable by the dynamically-created
# runtime uid. Daemon-less Nix re-chmods per-user dirs to 0755 on every call and
# EPERMs if it does not own them, so delete them here; the runtime user
# recreates and owns them on first `nix` invocation.
RUN chmod -R go+rwX /nix \
    && rm -rf /nix/var/nix/profiles/per-user /nix/var/nix/gcroots/per-user

# 2026-07-17: `/nix/var/nix/profiles/default` points into per-user/root, which
# the line above deletes — so the default profile bin (which ENV PATH relies on)
# dangles and `nix` isn't found. Symlink the Determinate Nix binaries into
# /usr/local/bin (already on PATH, lives on the image fs) so every shell —
# login, non-login, or Bazel repo-rule subprocess — finds them.
RUN NIXBIN="$(find /nix/store -maxdepth 2 -type d -name bin -path '*-determinate-nix-*' | head -1)" \
    && for b in "$NIXBIN"/*; do ln -sf "$b" /usr/local/bin/"$(basename "$b")"; done \
    && nix --version
