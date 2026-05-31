# 3Draper C API Reference

> Version 0.1.0 — Stable ABI with semantic versioning

This document describes the complete C API exposed by `libdraper_ffi`.
All symbols are exported with `extern "C"` linkage and `#[no_mangle]`,
making them suitable for consumption from C, C++, Python (ctypes/cffi),
C# (P/Invoke), and other FFI mechanisms.

---

## Table of Contents

- [Version & Feature Detection](#version--feature-detection)
- [Error Handling](#error-handling)
- [Document Functions](#document-functions)
- [Shape Builders](#shape-builders)
- [Triangulation & Mesh](#triangulation--mesh)
- [Analytical Queries](#analytical-queries)
- [Validation](#validation)
- [Export Functions](#export-functions)
- [Error Codes Reference](#error-codes-reference)
- [Thread Safety](#thread-safety)
- [Memory Management](#memory-management)
- [Version Compatibility](#version-compatibility)
- [Binding Examples](#binding-examples)

---

## Version & Feature Detection

### `draper_version_major`

```c
uint32_t draper_version_major(void);
```

Returns the major version number. Incremented on incompatible API changes.

### `draper_version_minor`

```c
uint32_t draper_version_minor(void);
```

Returns the minor version number. Incremented on backwards-compatible additions.

### `draper_version_patch`

```c
uint32_t draper_version_patch(void);
```

Returns the patch version number. Incremented on backwards-compatible bug fixes.

### `draper_version_string`

```c
const char* draper_version_string(void);
```

Returns a static string in `"major.minor.patch"` format (e.g. `"0.1.0"`).
The returned pointer is valid for the lifetime of the library and **must NOT be freed**.

### `draper_has_feature`

```c
bool draper_has_feature(const char* feature);
```

Check whether the library supports a named feature. Feature names are
case-sensitive ASCII strings. Returns `false` for unknown or `NULL` feature names.

| Feature name        | Meaning                                      |
|---------------------|----------------------------------------------|
| `step_import`       | STEP AP242 file import                       |
| `step_export`       | STEP AP242 file export                       |
| `stl_export`        | STL (ASCII & binary) export                  |
| `gltf_export`       | glTF 2.0 / GLB export                        |
| `obj_export`        | Wavefront OBJ export                         |
| `3mf_export`        | 3MF (3D Manufacturing Format) export         |
| `boolean_ops`       | Boolean union / subtract / intersect         |
| `healing`           | Geometry healing pipeline                    |
| `validation`        | Topology & STEP validation                   |
| `analytical_queries`| Volume, surface area, center of mass, inertia|
| `bvh`               | BVH acceleration for ray / proximity queries |

---

## Error Handling

### `draper_get_last_error`

```c
const char* draper_get_last_error(void);
```

Returns a pointer to a C string describing the last error that occurred on
the current thread. The pointer is valid **until the next FFI call on the
same thread**. Returns `NULL` if no error has occurred.

> **Important**: Copy the string immediately if you need it beyond the next
> FFI call. Do NOT free the returned pointer.

---

## Document Functions

### `draper_document_new`

```c
DraperDocument* draper_document_new(const char* name);
```

Create a new empty document. If `name` is `NULL`, the document is named
`"Untitled"`. Returns a pointer to the new document. The caller owns the
pointer and must free it with `draper_document_free`.

### `draper_document_free`

```c
void draper_document_free(DraperDocument* doc);
```

Free a document. Safe to call with `NULL` (no-op).

### `draper_document_solid_count`

```c
uint32_t draper_document_solid_count(const DraperDocument* doc);
```

Returns the number of solids in the document. Returns 0 if `doc` is `NULL`.

---

## Shape Builders

All shape builders add a new solid to the document and return `DraperResult::Success`
on success, or an error code on failure.

### `draper_document_add_box`

```c
DraperResult draper_document_add_box(DraperDocument* doc, double dx, double dy, double dz);
```

Add a box with dimensions `(dx, dy, dz)`. All dimensions must be positive.

### `draper_document_add_cylinder`

```c
DraperResult draper_document_add_cylinder(DraperDocument* doc, double radius, double height);
```

Add a cylinder with the given `radius` and `height`. Both must be positive.

### `draper_document_add_sphere`

```c
DraperResult draper_document_add_sphere(DraperDocument* doc, double radius);
```

Add a sphere with the given `radius`. Must be positive.

### `draper_document_add_cone`

```c
DraperResult draper_document_add_cone(DraperDocument* doc, double radius, double height);
```

Add a cone with the given `radius` and `height`. Both must be positive.

### `draper_document_add_torus`

```c
DraperResult draper_document_add_torus(DraperDocument* doc, double major_radius, double minor_radius);
```

Add a torus with the given radii. Both must be positive.

### `draper_document_add_engine`

```c
DraperResult draper_document_add_engine(DraperDocument* doc);
```

Add a built-in ICE engine model to the document.

---

## Triangulation & Mesh

### `draper_document_triangulate`

```c
DraperMesh* draper_document_triangulate(DraperDocument* doc);
```

Triangulate the document and return a mesh handle. Returns `NULL` on error
(use `draper_get_last_error()` for details). The caller owns the mesh and
must free it with `draper_mesh_free`.

### `draper_mesh_free`

```c
void draper_mesh_free(DraperMesh* mesh);
```

Free a mesh. Safe to call with `NULL` (no-op).

### `draper_mesh_vertex_count`

```c
uint32_t draper_mesh_vertex_count(const DraperMesh* mesh);
```

Returns the number of vertices. Returns 0 if `mesh` is `NULL`.

### `draper_mesh_triangle_count`

```c
uint32_t draper_mesh_triangle_count(const DraperMesh* mesh);
```

Returns the number of triangles. Returns 0 if `mesh` is `NULL`.

### `draper_mesh_get_vertices`

```c
uint32_t draper_mesh_get_vertices(const DraperMesh* mesh, double* out, uint32_t max_count);
```

Copy vertex positions into `out` as `(x, y, z)` triplets.
Caller must allocate a buffer of at least `max_count * 3` doubles.
Returns the number of vertices written, or 0 on error.

### `draper_mesh_get_triangles`

```c
uint32_t draper_mesh_get_triangles(const DraperMesh* mesh, uint32_t* out, uint32_t max_count);
```

Copy triangle indices into `out` as `(i, j, k)` triplets.
Caller must allocate a buffer of at least `max_count * 3` uint32 values.
Returns the number of triangles written, or 0 on error.

### `draper_mesh_export_stl`

```c
DraperResult draper_mesh_export_stl(const DraperMesh* mesh, const char* path, int binary);
```

Export mesh to STL file. Set `binary` to non-zero for binary STL, zero for ASCII.

---

## Analytical Queries

### `draper_solid_volume`

```c
double draper_solid_volume(const DraperDocument* doc);
```

Compute the total volume of all solids in the document using the divergence
theorem on a triangulated approximation. Returns 0.0 if `doc` is `NULL`
or the document has no solids.

### `draper_solid_surface_area`

```c
double draper_solid_surface_area(const DraperDocument* doc);
```

Compute the total surface area of all solids. Returns 0.0 if `doc` is `NULL`
or the document has no solids.

---

## Validation

### `draper_validate_step`

```c
DraperResult draper_validate_step(const DraperDocument* doc);
```

Run topology validation (shell closure, edge manifoldness, wire closure,
face orientation, Euler characteristic, etc.) on all solids in the document.

- Returns `DraperResult::Success` if no error-level issues are found
  (warnings and info are stored in the thread-local error message).
- Returns `DraperResult::TopologyError` if any error-level issues exist.
- Detailed issues are accessible via `draper_get_last_error()`.

---

## Export Functions

### `draper_document_export_step`

```c
DraperResult draper_document_export_step(DraperDocument* doc, const char* path);
```

Export the first solid in the document to STEP AP242 format.

### `draper_export_gltf`

```c
DraperResult draper_export_gltf(const DraperDocument* doc, const char* path);
```

Convenience function: triangulate the document and export to glTF 2.0
(GLB binary) in a single call.

### `draper_export_obj`

```c
DraperResult draper_export_obj(const DraperDocument* doc, const char* path);
```

Convenience function: triangulate and export to Wavefront OBJ.

### `draper_export_3mf`

```c
DraperResult draper_export_3mf(const DraperDocument* doc, const char* path);
```

Convenience function: triangulate and export to 3MF (3D Manufacturing Format).

---

## Error Codes Reference

| Code | Constant            | Meaning                                       |
|------|---------------------|-----------------------------------------------|
|  0   | `Success`           | Operation succeeded                           |
| -1   | `InvalidArgument`   | Null pointer, out-of-range value, etc.        |
| -2   | `FileNotFound`      | The requested file was not found              |
| -3   | `ParseError`        | STEP/file parsing failed                      |
| -4   | `GeometryError`     | Degenerate surface, NaN, etc.                 |
| -5   | `TopologyError`     | Broken B-Rep, non-manifold topology           |
| -6   | `TriangulationError`| Mesh generation failed                        |
| -7   | `OutOfMemory`       | Resource limit exceeded                       |
| -99  | `UnknownError`      | Unclassified error                            |

---

## Thread Safety

- **All FFI functions are thread-safe** with respect to different documents.
  Multiple threads may operate on different `DraperDocument*` handles
  concurrently without synchronisation.

- **The same document must not be accessed concurrently from multiple threads.**
  If you need shared access, use external synchronisation (mutex, etc.).

- **Error messages are thread-local.** Each thread has its own error string.
  `draper_get_last_error()` returns the last error for the *calling thread* only.

- **Version and feature functions** (`draper_version_*`, `draper_has_feature`)
  are fully thread-safe and can be called from any thread at any time.

---

## Memory Management

| Allocated by                     | Must be freed by               |
|----------------------------------|--------------------------------|
| `draper_document_new`            | `draper_document_free`         |
| `draper_document_triangulate`    | `draper_mesh_free`             |
| `draper_version_string`          | **Do NOT free** (static)       |
| `draper_get_last_error`          | **Do NOT free** (thread-local) |
| `draper_has_feature` input       | Caller (not consumed)          |
| All `path` parameters            | Caller (not consumed)          |

### Rules

1. Every `*_new` / `*_triangulate` call returns an owned pointer.
   The caller **must** free it with the corresponding `*_free` function.

2. All `const char*` path parameters are **borrowed** — the library reads
   them during the call and does not retain them afterward.

3. `draper_version_string()` returns a pointer to a static string.
   It is valid for the lifetime of the library and must not be freed.

4. `draper_get_last_error()` returns a pointer to thread-local storage.
   It is valid only until the next FFI call on the same thread.
   **Copy the string** if you need it beyond that point.

5. All `free` functions are safe to call with `NULL` (no-op).

---

## Version Compatibility

The 3Draper library follows [Semantic Versioning 2.0](https://semver.org/):

- **Major version 0** (`0.y.z`): Initial development. The API may change
  at any time. Clients should pin to exact versions.

- **Major version ≥ 1**: Backward-compatible guarantees apply:
  - New functions may be added (minor bump).
  - Existing function signatures will not change (only major bump).
  - Error codes will not be renumbered (only major bump for removal).

### Checking compatibility at runtime

```c
if (draper_version_major() != EXPECTED_MAJOR) {
    fprintf(stderr, "Incompatible 3Draper version\n");
}
```

Or use feature detection for optional functionality:

```c
if (draper_has_feature("gltf_export")) {
    draper_export_gltf(doc, "output.glb");
} else {
    fprintf(stderr, "glTF export not available\n");
}
```

---

## Binding Examples

### C

```c
#include "draper_ffi.h"

int main(void) {
    DraperDocument* doc = draper_document_new("Cube");
    draper_document_add_box(doc, 1.0, 2.0, 3.0);

    double vol = draper_solid_volume(doc);
    printf("Volume: %f\n", vol);

    draper_export_gltf(doc, "cube.glb");
    draper_document_free(doc);
    return 0;
}
```

### Python

```python
from draper import Document, version, has_feature

print(f"3Draper version: {version()}")
print(f"glTF export: {has_feature('gltf_export')}")

with Document("Cube") as doc:
    doc.add_box(1.0, 2.0, 3.0)
    print(f"Volume: {doc.volume()}")
    doc.export_gltf("cube.glb")
```

### C#

```csharp
using Draper;

using var doc = new DraperDocument("Cube");
doc.AddBox(1.0, 2.0, 3.0);
Console.WriteLine($"Volume: {doc.Volume()}");
doc.ExportGltf("cube.glb");
```

### JavaScript (WASM)

```javascript
const { Document, version, hasFeature } = require("./draper");

console.log(`3Draper version: ${version()}`);
console.log(`glTF export: ${hasFeature("gltf_export")}`);

const doc = new Document("Cube");
doc.addBox(1.0, 2.0, 3.0);
console.log(`Volume: ${doc.volume()}`);
doc.exportGltf("cube.glb");
doc.free();
```
