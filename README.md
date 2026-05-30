# 3Draper

Cross-platform 3D kernel written in Rust with STEP file support and C FFI.

## Architecture

```
3Draper/
├── crates/
│   ├── draper-step/       Custom STEP (ISO 10303-21) parser
│   ├── draper-geometry/   Custom geometry kernel (curves, surfaces, transforms)
│   ├── draper-topology/   B-rep topology (Vertex/Edge/Wire/Face/Shell/Solid)
│   ├── draper-mesh/       Triangulation & mesh generation
│   ├── draper-core/       High-level API (Document, Scene, STEP I/O bridge)
│   ├── draper-ffi/        C FFI bindings for cross-language use
│   └── draper-viewer/     Minimal STEP file viewer
└── include/
    └── draper.h           Generated C header
```

## Key Principles

- **Own kernel** — no external 3D kernels (OpenCascade, etc.)
- **External triangulation** — Delaunay via `spade`, ear-clipping built-in
- **Cross-language** — C FFI allows use from C/C++, Python, C#, etc.
- **Cross-platform** — builds on Windows, macOS, Linux

## Building

```bash
cargo build --release
```

### Run the viewer

```bash
cargo run -p draper-viewer
```

### Build the FFI library

```bash
cargo build -p draper-ffi --release
```

This produces:
- `libdraper_ffi.a` (static library)
- `libdraper_ffi.so` (shared library on Linux)
- C header at `include/draper.h`

## Usage from C

```c
#include "draper.h"

DraperDocument* doc = draper_open_step("model.stp");
if (doc) {
    DraperStatistics stats = draper_get_statistics(doc);
    printf("Triangles: %lu\n", stats.triangle_count);

    DraperMesh* mesh = draper_get_mesh(doc);
    // Access vertices: draper_mesh_vertices(mesh)
    // Access indices: draper_mesh_indices(mesh)

    draper_mesh_free(mesh);
    draper_document_free(doc);
}
```

## STEP Support

Supports parsing all STEP AP versions (AP203, AP214, AP242, etc.) that use the
ISO 10303-21 exchange structure format. The parser handles:

- HEADER section (file description, name, schema)
- DATA section (entity instances with references)
- All parameter types (integer, real, string, enumeration, reference, typed, list, binary)
- Round-trip: parse → modify → write back

## Current Status

**v0.1.0** — Initial release with:
- STEP file parser (read/write)
- B-rep topology data structures
- Geometry primitives (curves, surfaces)
- Mesh generation for rendering
- C FFI bindings
- Minimal viewer (open, display structure tree, render, save)

## License

GNU General Public License v3.0 or later (GPL-3.0-or-later)

This project is free software: you can use, study, modify and redistribute it
under the terms of the **GNU GPLv3+**. Any derivative work — including use in
commercial products — **must also be released as open source under the same
license**. This ensures that no one can incorporate this kernel into proprietary
software without giving back to the community.

See [LICENSE](LICENSE) for the full license text.
