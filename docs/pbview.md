# Protobuf for embedded transport structures (pbview)

Policy: every structure that crosses a link between embedded systems (radio frames,
stored blobs, anything a node sends or receives as bytes) is a protobuf message, defined in
a `.proto` under `swarm/proto/`, not a packed C struct or ad-hoc JSON.

**Reading** on the C++ firmware uses [pbview](https://github.com/fughilli/pbview) (extracted
from splanc's `oss/pbview`; MIT OR Apache-2.0): zero-copy views over the received bytes. A
view is a (pointer, length) pair; nothing is decoded into a struct, copied or allocated, so
a handler costs two words of stack whatever the schema. `Parse` validates once (framing,
wire types, UTF-8, nesting depth), so accessors never skip anything silently. **Writing**
uses `pbv::Writer`, which encodes into the frame buffer in place. Rust: `swarm_net` decodes with pbview views and
encodes with pbview's generated typed encoders (`FooEnc`; guardrails and compile-time size bounds from
a sidecar bounds file, see the pbview README); widget.v1 and the pose advert keep micropb from
`@firmware//proto`. All generate from the same `.proto`.

## Adding a message

```starlark
load("@firmware//proto:micropb.bzl", "proto_fdset")
load("@pbview//:defs.bzl", "pbview_cc_library")

proto_fdset(name = "foo_fdset", proto = "foo.proto")
pbview_cc_library(name = "foo_views", fdset = ":foo_fdset", header = "foo_views.h")
```

```cpp
#include "swarm/proto/foo_views.h"
foo::v1::BarView bar;
if (!foo::v1::BarView::Parse({rx, len}, &bar)) return;   // malformed: drop
use(bar.name());                                          // std::string_view into rx

pbv::Writer w(tx, sizeof tx);
w.Message(2, [&](pbv::Writer& m) { return m.Uint32(1, id) && m.Float(2, q); });
```

`swarm/proto:widget_views_test` is a worked example (a `PoseAdvertisement` and a
`WidgetConfig`). Naming: package `a.b` is `namespace a::b`; message `Foo` is `FooView`;
nested `Foo.Bar` is `Foo_BarView`; a oneof `kind` has `kind_case()`. Field accessors are
`snake_case`; absent fields read as their default, `optional` and message fields as
`std::optional`. The full list of differences from the Rust output is in the pbview README.

## Where this repo stands

- `widget.v1` (`swarm/proto/widget_config.proto`) is protobuf and now has C++ views.
  `PoseAdvertisement` is the over-the-air advert; the radio carries it as opaque bytes today.
- Migrated (docs/swarm-wire.md): the UWB link frames, the swarm_net mesh, settings and firmware
  formats, the manifest and the staging sector.
- **Not migrated:** the serial records between a node and its host (JSON; the sync-log and CIR
  experiment records among them). DS-TWR timing with the new frames is unmeasured on hardware.
- The C++ output has only been compiled on the host (this repo's `cc_test`), not under the
  xtensa toolchain.
- pbview is a `git_override` in `MODULE.bazel`, pinned by commit; for co-development use
  `bazel --override_module=pbview=/path/to/pbview`.
