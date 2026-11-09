# Protobuf for embedded transport structures (pbview)

Policy: every structure that crosses a link between embedded systems (radio frames,
stored blobs, anything a node sends or receives as bytes) is a protobuf message, defined in
a `.proto` under `swarm/proto/`, not a packed C struct or ad-hoc JSON.

**Reading** on the C++ firmware uses [pbview](https://github.com/fughilli/pbview) (extracted
from splanc's `oss/pbview`; MIT OR Apache-2.0): zero-copy views over the received bytes. A
view is a (pointer, length) pair; nothing is decoded into a struct, copied or allocated, so
a handler costs two words of stack whatever the schema. `Parse` validates once (framing,
wire types, UTF-8, nesting depth), so accessors never skip anything silently. **Writing**
uses `pbv::Writer`, which encodes into the frame buffer in place. Rust (the swarm core)
keeps using micropb from `@firmware//proto`; both generate from the same `.proto`.

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
- **Not yet migrated**, in the order I would do them: the `apps/common/protocol.h` link frames
  (`Header`, `Report`, `Result`; fixed-size packed structs, parsed with `memcpy`), the swarm_net
  mesh/firmware frames, and the serial records of the sync-log / CIR experiment (JSON today).
  Protobuf varints make frame lengths data-dependent, and the DS-TWR frames are on a tight
  timing budget (`kReplyTimeoutUs`, a 125-byte frame limit); measure before moving them.
- The C++ output has only been compiled on the host (this repo's `cc_test`), not under the
  xtensa toolchain.
- pbview is consumed through `local_path_override(path = "../pbview")` in `MODULE.bazel` until
  the repo is published; then switch to a `git_override` pinned by commit.
