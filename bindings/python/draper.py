# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 KernelDev
"""
Python bindings for the 3Draper kernel via ctypes.

Usage:
    from draper import Document, DraperError

    doc = Document("MyModel")
    doc.add_box(1.0, 2.0, 3.0)
    vol = doc.volume()
    doc.export_gltf("output.glb")
    doc.close()

The shared library (libdraper_ffi.so / draper_ffi.dll) must be on the
library search path, or you can pass an explicit path:

    from draper import load_library
    load_library("/path/to/libdraper_ffi.so")
"""

import ctypes
import os
import platform
from typing import Optional

# ============================================================
# DraperResult error codes
# ============================================================

class DraperResult:
    """C-compatible result codes returned by every FFI function."""
    SUCCESS = 0
    INVALID_ARGUMENT = -1
    FILE_NOT_FOUND = -2
    PARSE_ERROR = -3
    GEOMETRY_ERROR = -4
    TOPOLOGY_ERROR = -5
    TRIANGULATION_ERROR = -6
    OUT_OF_MEMORY = -7
    UNKNOWN_ERROR = -99

    _MESSAGES = {
        0: "Success",
        -1: "Invalid argument",
        -2: "File not found",
        -3: "Parse error",
        -4: "Geometry error",
        -5: "Topology error",
        -6: "Triangulation error",
        -7: "Out of memory",
        -99: "Unknown error",
    }

    @staticmethod
    def message(code: int) -> str:
        return DraperResult._MESSAGES.get(code, f"Unknown result code: {code}")


# ============================================================
# Custom exception
# ============================================================

class DraperError(Exception):
    """Exception raised when a 3Draper FFI call returns an error."""

    def __init__(self, code: int, message: str = ""):
        self.code = code
        self.message = message or DraperResult.message(code)
        super().__init__(f"DraperError({code}): {self.message}")


# ============================================================
# Library loading
# ============================================================

_lib: Optional[ctypes.CDLL] = None


def _find_library_path() -> str:
    """Attempt to locate the shared library automatically."""
    system = platform.system()
    if system == "Linux":
        names = ["libdraper_ffi.so"]
    elif system == "Darwin":
        names = ["libdraper_ffi.dylib"]
    elif system == "Windows":
        names = ["draper_ffi.dll"]
    else:
        names = ["libdraper_ffi.so"]

    # Check relative to this file
    base_dir = os.path.dirname(os.path.abspath(__file__))
    for name in names:
        candidate = os.path.join(base_dir, name)
        if os.path.exists(candidate):
            return candidate

    # Check target directory (Rust build output)
    for name in names:
        candidate = os.path.join(base_dir, "..", "..", "target", "debug", name)
        if os.path.exists(candidate):
            return os.path.abspath(candidate)
        candidate = os.path.join(base_dir, "..", "..", "target", "release", name)
        if os.path.exists(candidate):
            return os.path.abspath(candidate)

    # Fall back to system library path
    return names[0]


def load_library(path: str) -> None:
    """Load the 3Draper shared library from the given path."""
    global _lib
    _lib = ctypes.CDLL(path)
    _setup_bindings()


def _get_lib() -> ctypes.CDLL:
    """Get or lazily load the shared library."""
    global _lib
    if _lib is None:
        load_library(_find_library_path())
    return _lib


def _check_result(code: int) -> None:
    """Raise DraperError if the result code is not Success."""
    if code != DraperResult.SUCCESS:
        lib = _get_lib()
        err_ptr = lib.draper_get_last_error()
        if err_ptr:
            msg = ctypes.cast(err_ptr, ctypes.c_char_p).value
            msg = msg.decode("utf-8", errors="replace") if msg else ""
        else:
            msg = ""
        raise DraperError(code, msg)


# ============================================================
# C function prototype declarations
# ============================================================

def _setup_bindings() -> None:
    """Set up ctypes argument and return types for all C API functions."""
    lib = _get_lib()

    # Version
    lib.draper_version_major.argtypes = []
    lib.draper_version_major.restype = ctypes.c_uint32
    lib.draper_version_minor.argtypes = []
    lib.draper_version_minor.restype = ctypes.c_uint32
    lib.draper_version_patch.argtypes = []
    lib.draper_version_patch.restype = ctypes.c_uint32
    lib.draper_version_string.argtypes = []
    lib.draper_version_string.restype = ctypes.c_char_p

    # Feature detection
    lib.draper_has_feature.argtypes = [ctypes.c_char_p]
    lib.draper_has_feature.restype = ctypes.c_bool

    # Error
    lib.draper_get_last_error.argtypes = []
    lib.draper_get_last_error.restype = ctypes.c_char_p

    # Document
    lib.draper_document_new.argtypes = [ctypes.c_char_p]
    lib.draper_document_new.restype = ctypes.c_void_p
    lib.draper_document_free.argtypes = [ctypes.c_void_p]
    lib.draper_document_free.restype = None
    lib.draper_document_solid_count.argtypes = [ctypes.c_void_p]
    lib.draper_document_solid_count.restype = ctypes.c_uint32

    # Shape builders
    lib.draper_document_add_box.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double]
    lib.draper_document_add_box.restype = ctypes.c_int32
    lib.draper_document_add_cylinder.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double]
    lib.draper_document_add_cylinder.restype = ctypes.c_int32
    lib.draper_document_add_sphere.argtypes = [ctypes.c_void_p, ctypes.c_double]
    lib.draper_document_add_sphere.restype = ctypes.c_int32
    lib.draper_document_add_cone.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double]
    lib.draper_document_add_cone.restype = ctypes.c_int32
    lib.draper_document_add_torus.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double]
    lib.draper_document_add_torus.restype = ctypes.c_int32
    lib.draper_document_add_engine.argtypes = [ctypes.c_void_p]
    lib.draper_document_add_engine.restype = ctypes.c_int32

    # Triangulation
    lib.draper_document_triangulate.argtypes = [ctypes.c_void_p]
    lib.draper_document_triangulate.restype = ctypes.c_void_p

    # Mesh
    lib.draper_mesh_free.argtypes = [ctypes.c_void_p]
    lib.draper_mesh_free.restype = None
    lib.draper_mesh_vertex_count.argtypes = [ctypes.c_void_p]
    lib.draper_mesh_vertex_count.restype = ctypes.c_uint32
    lib.draper_mesh_triangle_count.argtypes = [ctypes.c_void_p]
    lib.draper_mesh_triangle_count.restype = ctypes.c_uint32
    lib.draper_mesh_get_vertices.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_uint32]
    lib.draper_mesh_get_vertices.restype = ctypes.c_uint32
    lib.draper_mesh_get_triangles.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32]
    lib.draper_mesh_get_triangles.restype = ctypes.c_uint32
    lib.draper_mesh_export_stl.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int32]
    lib.draper_mesh_export_stl.restype = ctypes.c_int32

    # STEP export
    lib.draper_document_export_step.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lib.draper_document_export_step.restype = ctypes.c_int32

    # Analytical queries
    lib.draper_solid_volume.argtypes = [ctypes.c_void_p]
    lib.draper_solid_volume.restype = ctypes.c_double
    lib.draper_solid_surface_area.argtypes = [ctypes.c_void_p]
    lib.draper_solid_surface_area.restype = ctypes.c_double

    # Validation
    lib.draper_validate_step.argtypes = [ctypes.c_void_p]
    lib.draper_validate_step.restype = ctypes.c_int32

    # Export convenience
    lib.draper_export_gltf.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lib.draper_export_gltf.restype = ctypes.c_int32
    lib.draper_export_obj.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lib.draper_export_obj.restype = ctypes.c_int32
    lib.draper_export_3mf.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lib.draper_export_3mf.restype = ctypes.c_int32

    # STEP load (extended)
    lib.draper_document_load_step.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint8]
    lib.draper_document_load_step.restype = ctypes.c_int32

    # Editing ops
    lib.draper_solid_fillet_edge.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t, ctypes.c_double]
    lib.draper_solid_fillet_edge.restype = ctypes.c_int32
    lib.draper_solid_chamfer_edge.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t, ctypes.c_double]
    lib.draper_solid_chamfer_edge.restype = ctypes.c_int32
    lib.draper_solid_make_shell.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_double]
    lib.draper_solid_make_shell.restype = ctypes.c_int32

    # Transform ops
    lib.draper_document_translate.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double]
    lib.draper_document_translate.restype = ctypes.c_int32
    lib.draper_document_rotate.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double]
    lib.draper_document_rotate.restype = ctypes.c_int32
    lib.draper_document_rotate_around_point.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double]
    lib.draper_document_rotate_around_point.restype = ctypes.c_int32
    lib.draper_document_scale.argtypes = [ctypes.c_void_p, ctypes.c_double]
    lib.draper_document_scale.restype = ctypes.c_int32
    lib.draper_document_scale_around_point.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double]
    lib.draper_document_scale_around_point.restype = ctypes.c_int32
    lib.draper_document_mirror.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double]
    lib.draper_document_mirror.restype = ctypes.c_int32

    # Boolean ops
    lib.draper_document_boolean_union.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t, ctypes.POINTER(ctypes.c_uint32)]
    lib.draper_document_boolean_union.restype = ctypes.c_int32
    lib.draper_document_boolean_subtract.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t, ctypes.POINTER(ctypes.c_uint32)]
    lib.draper_document_boolean_subtract.restype = ctypes.c_int32
    lib.draper_document_boolean_intersect.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t, ctypes.POINTER(ctypes.c_uint32)]
    lib.draper_document_boolean_intersect.restype = ctypes.c_int32
    lib.draper_document_delete_solid.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
    lib.draper_document_delete_solid.restype = ctypes.c_int32

    # Patterns
    lib.draper_document_circular_pattern.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t, ctypes.c_double, ctypes.c_double, ctypes.c_double]
    lib.draper_document_circular_pattern.restype = ctypes.c_int32
    lib.draper_document_linear_pattern.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double]
    lib.draper_document_linear_pattern.restype = ctypes.c_int32

    # Holes
    lib.draper_solid_add_circular_hole.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double]
    lib.draper_solid_add_circular_hole.restype = ctypes.c_int32
    lib.draper_solid_remove_hole.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t, ctypes.c_size_t]
    lib.draper_solid_remove_hole.restype = ctypes.c_int32
    lib.draper_solid_clear_holes.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t]
    lib.draper_solid_clear_holes.restype = ctypes.c_uint32

    # Face management
    lib.draper_solid_delete_face.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t]
    lib.draper_solid_delete_face.restype = ctypes.c_int32
    lib.draper_solid_reverse_face.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t]
    lib.draper_solid_reverse_face.restype = ctypes.c_int32

    # GDT
    class _GdtResult(ctypes.Structure):
        _fields_ = [
            ("tolerance_value", ctypes.c_double),
            ("actual_deviation", ctypes.c_double),
            ("passed", ctypes.c_uint8),
            ("status_code", ctypes.c_uint8),
        ]
    lib._GdtResult = _GdtResult
    lib.draper_solid_gdt_check.argtypes = [
        ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int32, ctypes.c_double,
        ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_uint8,
        ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_uint8,
        ctypes.c_double, ctypes.c_uint8,
    ]
    lib.draper_solid_gdt_check.restype = _GdtResult
    lib.draper_solid_gdt_check_all.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_char_p]
    lib.draper_solid_gdt_check_all.restype = ctypes.c_void_p

    # String free + edge listing + bbox
    lib.draper_free_string.argtypes = [ctypes.c_void_p]
    lib.draper_free_string.restype = None
    lib.draper_solid_list_edges.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
    lib.draper_solid_list_edges.restype = ctypes.c_void_p
    lib.draper_document_bbox.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double)]
    lib.draper_document_bbox.restype = ctypes.c_int32

    # STEP → USDA
    lib.draper_export_step_to_usda.argtypes = [
        ctypes.c_char_p, ctypes.c_char_p, ctypes.c_double,
        ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8,
    ]
    lib.draper_export_step_to_usda.restype = ctypes.c_int32


# ============================================================
# Public API
# ============================================================

def version() -> str:
    """Return the library version as a string (e.g. '0.1.0')."""
    lib = _get_lib()
    raw = lib.draper_version_string()
    return raw.decode("utf-8") if raw else "unknown"


def version_tuple() -> tuple:
    """Return the library version as a (major, minor, patch) tuple."""
    lib = _get_lib()
    return (lib.draper_version_major(),
            lib.draper_version_minor(),
            lib.draper_version_patch())


def has_feature(feature: str) -> bool:
    """Check whether the library supports a named feature.

    Feature names: step_import, step_export, stl_export, gltf_export,
    obj_export, 3mf_export, boolean_ops, healing, validation,
    analytical_queries, bvh.
    """
    lib = _get_lib()
    return lib.draper_has_feature(feature.encode("utf-8"))


# ============================================================
# Mesh class
# ============================================================

class Mesh:
    """Wrapper around a DraperMesh opaque handle.

    Obtain a Mesh via Document.triangulate().  Call close() or use
    as a context manager to free the underlying handle.
    """

    def __init__(self, handle: int):
        self._handle = handle

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def close(self) -> None:
        """Free the mesh handle."""
        if self._handle:
            _get_lib().draper_mesh_free(self._handle)
            self._handle = None

    @property
    def vertex_count(self) -> int:
        """Number of vertices in the mesh."""
        return _get_lib().draper_mesh_vertex_count(self._handle)

    @property
    def triangle_count(self) -> int:
        """Number of triangles in the mesh."""
        return _get_lib().draper_mesh_triangle_count(self._handle)

    def get_vertices(self) -> list:
        """Return vertex positions as a list of (x, y, z) tuples."""
        n = self.vertex_count
        if n == 0:
            return []
        buf = (ctypes.c_double * (n * 3))()
        written = _get_lib().draper_mesh_get_vertices(self._handle, buf, n)
        result = []
        for i in range(written):
            result.append((buf[i * 3], buf[i * 3 + 1], buf[i * 3 + 2]))
        return result

    def get_triangles(self) -> list:
        """Return triangle indices as a list of (i, j, k) tuples."""
        n = self.triangle_count
        if n == 0:
            return []
        buf = (ctypes.c_uint32 * (n * 3))()
        written = _get_lib().draper_mesh_get_triangles(self._handle, buf, n)
        result = []
        for i in range(written):
            result.append((buf[i * 3], buf[i * 3 + 1], buf[i * 3 + 2]))
        return result

    def export_stl(self, path: str, binary: bool = True) -> None:
        """Export mesh to STL file."""
        _check_result(_get_lib().draper_mesh_export_stl(
            self._handle, path.encode("utf-8"), 1 if binary else 0
        ))


# ============================================================
# Document class
# ============================================================

class Document:
    """High-level Python wrapper for a 3Draper document.

    Usage:
        doc = Document("MyModel")
        doc.add_box(1.0, 2.0, 3.0)
        doc.export_gltf("output.glb")
        doc.close()

    Or as a context manager:
        with Document("MyModel") as doc:
            doc.add_sphere(1.0)
            vol = doc.volume()
    """

    def __init__(self, name: str = "Untitled"):
        lib = _get_lib()
        self._handle = lib.draper_document_new(name.encode("utf-8"))
        if not self._handle:
            raise DraperError(DraperResult.UNKNOWN_ERROR, "Failed to create document")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def close(self) -> None:
        """Free the document handle."""
        if self._handle:
            _get_lib().draper_document_free(self._handle)
            self._handle = None

    # ----------------------------------------------------------
    # Shape builders
    # ----------------------------------------------------------

    def add_box(self, dx: float, dy: float, dz: float) -> None:
        """Add a box primitive with dimensions (dx, dy, dz)."""
        _check_result(_get_lib().draper_document_add_box(
            self._handle, dx, dy, dz
        ))

    def add_cylinder(self, radius: float, height: float) -> None:
        """Add a cylinder primitive."""
        _check_result(_get_lib().draper_document_add_cylinder(
            self._handle, radius, height
        ))

    def add_sphere(self, radius: float) -> None:
        """Add a sphere primitive."""
        _check_result(_get_lib().draper_document_add_sphere(
            self._handle, radius
        ))

    def add_cone(self, radius: float, height: float) -> None:
        """Add a cone primitive."""
        _check_result(_get_lib().draper_document_add_cone(
            self._handle, radius, height
        ))

    def add_torus(self, major_radius: float, minor_radius: float) -> None:
        """Add a torus primitive."""
        _check_result(_get_lib().draper_document_add_torus(
            self._handle, major_radius, minor_radius
        ))

    def add_engine(self) -> None:
        """Add a built-in ICE engine model."""
        _check_result(_get_lib().draper_document_add_engine(self._handle))

    # ----------------------------------------------------------
    # Properties
    # ----------------------------------------------------------

    @property
    def solid_count(self) -> int:
        """Number of solids in the document."""
        return _get_lib().draper_document_solid_count(self._handle)

    # ----------------------------------------------------------
    # Triangulation
    # ----------------------------------------------------------

    def triangulate(self) -> Mesh:
        """Triangulate the document and return a Mesh object."""
        handle = _get_lib().draper_document_triangulate(self._handle)
        if not handle:
            raise DraperError(DraperResult.TRIANGULATION_ERROR,
                              _get_last_error_message())
        return Mesh(handle)

    # ----------------------------------------------------------
    # Analytical queries
    # ----------------------------------------------------------

    def volume(self) -> float:
        """Compute the total volume of all solids in the document."""
        return _get_lib().draper_solid_volume(self._handle)

    def surface_area(self) -> float:
        """Compute the total surface area of all solids in the document."""
        return _get_lib().draper_solid_surface_area(self._handle)

    # ----------------------------------------------------------
    # Validation
    # ----------------------------------------------------------

    def validate(self) -> None:
        """Run topology validation on all solids.

        Raises DraperError if any error-level issues are found.
        Warnings and info are available in the exception message.
        """
        code = _get_lib().draper_validate_step(self._handle)
        _check_result(code)

    # ----------------------------------------------------------
    # Export
    # ----------------------------------------------------------

    def export_step(self, path: str) -> None:
        """Export the document to STEP AP242 format."""
        _check_result(_get_lib().draper_document_export_step(
            self._handle, path.encode("utf-8")
        ))

    def export_gltf(self, path: str) -> None:
        """Export the document to glTF 2.0 (GLB binary) format."""
        _check_result(_get_lib().draper_export_gltf(
            self._handle, path.encode("utf-8")
        ))

    def export_obj(self, path: str) -> None:
        """Export the document to Wavefront OBJ format."""
        _check_result(_get_lib().draper_export_obj(
            self._handle, path.encode("utf-8")
        ))

    def export_3mf(self, path: str) -> None:
        """Export the document to 3MF (3D Manufacturing Format)."""
        _check_result(_get_lib().draper_export_3mf(
            self._handle, path.encode("utf-8")
        ))

    # ----------------------------------------------------------
    # Editing operations (fillet / chamfer / shell)
    # ----------------------------------------------------------

    def fillet_edge(self, solid_index: int, edge_index: int, radius: float) -> None:
        """Fillet (round) an edge of a solid.

        Parameters
        ----------
        solid_index : int
            Index of the solid in the document.
        edge_index : int
            TopoId of the edge. Pass 0 to auto-pick the first manifold edge.
        radius : float
            Fillet radius in mm.
        """
        _check_result(_get_lib().draper_solid_fillet_edge(
            self._handle, solid_index, edge_index, radius
        ))

    def chamfer_edge(self, solid_index: int, edge_index: int, distance: float) -> None:
        """Chamfer (bevel) an edge of a solid.

        Parameters
        ----------
        solid_index : int
            Index of the solid.
        edge_index : int
            TopoId of the edge. Pass 0 to auto-pick.
        distance : float
            Chamfer distance in mm.
        """
        _check_result(_get_lib().draper_solid_chamfer_edge(
            self._handle, solid_index, edge_index, distance
        ))

    def make_shell(self, solid_index: int, thickness: float) -> None:
        """Shell a solid (inward offset by `thickness` mm).

        Parameters
        ----------
        solid_index : int
            Index of the solid.
        thickness : float
            Shell thickness in mm.
        """
        _check_result(_get_lib().draper_solid_make_shell(
            self._handle, solid_index, thickness
        ))

    # ----------------------------------------------------------
    # Transform operations
    # ----------------------------------------------------------

    def translate(self, dx: float, dy: float, dz: float) -> None:
        """Translate every solid in the document by (dx, dy, dz)."""
        _check_result(_get_lib().draper_document_translate(
            self._handle, dx, dy, dz
        ))

    def rotate(self, ax: float, ay: float, az: float, angle: float) -> None:
        """Rotate every solid about axis (ax, ay, az) by `angle` radians."""
        _check_result(_get_lib().draper_document_rotate(
            self._handle, ax, ay, az, angle
        ))

    def rotate_around_point(self, ax: float, ay: float, az: float,
                            cx: float, cy: float, cz: float,
                            angle: float) -> None:
        """Rotate every solid about an axis through (cx, cy, cz) by `angle` radians."""
        _check_result(_get_lib().draper_document_rotate_around_point(
            self._handle, ax, ay, az, cx, cy, cz, angle
        ))

    def scale(self, factor: float) -> None:
        """Uniformly scale every solid by `factor` about the origin."""
        _check_result(_get_lib().draper_document_scale(self._handle, factor))

    def scale_around_point(self, factor: float,
                           cx: float, cy: float, cz: float) -> None:
        """Uniformly scale every solid by `factor` about (cx, cy, cz)."""
        _check_result(_get_lib().draper_document_scale_around_point(
            self._handle, factor, cx, cy, cz
        ))

    def mirror(self, ox: float, oy: float, oz: float,
               nx: float, ny: float, nz: float) -> None:
        """Mirror every solid about the plane through (ox, oy, oz) with normal (nx, ny, nz)."""
        _check_result(_get_lib().draper_document_mirror(
            self._handle, ox, oy, oz, nx, ny, nz
        ))

    # ----------------------------------------------------------
    # Boolean operations
    # ----------------------------------------------------------

    def boolean_union(self, a_index: int, b_index: int) -> int:
        """Boolean union of two solids. Returns the index of the new solid."""
        out = ctypes.c_uint32(0)
        _check_result(_get_lib().draper_document_boolean_union(
            self._handle, a_index, b_index, ctypes.byref(out)
        ))
        return out.value

    def boolean_subtract(self, a_index: int, b_index: int) -> int:
        """Boolean subtract (A - B). Returns the index of the new solid."""
        out = ctypes.c_uint32(0)
        _check_result(_get_lib().draper_document_boolean_subtract(
            self._handle, a_index, b_index, ctypes.byref(out)
        ))
        return out.value

    def boolean_intersect(self, a_index: int, b_index: int) -> int:
        """Boolean intersect (A ∩ B). Returns the index of the new solid."""
        out = ctypes.c_uint32(0)
        _check_result(_get_lib().draper_document_boolean_intersect(
            self._handle, a_index, b_index, ctypes.byref(out)
        ))
        return out.value

    def delete_solid(self, index: int) -> None:
        """Delete the solid at `index`."""
        _check_result(_get_lib().draper_document_delete_solid(self._handle, index))

    # ----------------------------------------------------------
    # Pattern operations
    # ----------------------------------------------------------

    def circular_pattern(self, solid_index: int, count: int,
                         ax: float, ay: float, az: float) -> None:
        """Create a circular pattern of `count` copies around axis (ax, ay, az)."""
        _check_result(_get_lib().draper_document_circular_pattern(
            self._handle, solid_index, count, ax, ay, az
        ))

    def linear_pattern(self, solid_index: int, count: int,
                       dx: float, dy: float, dz: float, step: float) -> None:
        """Create a linear pattern of `count` copies along (dx, dy, dz) with `step` mm spacing."""
        _check_result(_get_lib().draper_document_linear_pattern(
            self._handle, solid_index, count, dx, dy, dz, step
        ))

    # ----------------------------------------------------------
    # Hole operations
    # ----------------------------------------------------------

    def add_circular_hole(self, solid_index: int, face_index: int,
                          cx: float, cy: float, cz: float,
                          radius: float) -> None:
        """Add a circular hole of `radius` mm at (cx, cy, cz) on the face."""
        _check_result(_get_lib().draper_solid_add_circular_hole(
            self._handle, solid_index, face_index, cx, cy, cz, radius
        ))

    def remove_hole(self, solid_index: int, face_index: int, hole_index: int) -> None:
        """Remove the i-th inner wire (hole) from a face."""
        _check_result(_get_lib().draper_solid_remove_hole(
            self._handle, solid_index, face_index, hole_index
        ))

    def clear_holes(self, solid_index: int, face_index: int) -> int:
        """Clear all holes from a face. Returns the number of holes removed."""
        return _get_lib().draper_solid_clear_holes(self._handle, solid_index, face_index)

    # ----------------------------------------------------------
    # Face management
    # ----------------------------------------------------------

    def delete_face(self, solid_index: int, face_index: int) -> None:
        """Delete a face from a solid. WARNING: breaks watertightness."""
        _check_result(_get_lib().draper_solid_delete_face(
            self._handle, solid_index, face_index
        ))

    def reverse_face(self, solid_index: int, face_index: int) -> None:
        """Reverse the orientation of a face."""
        _check_result(_get_lib().draper_solid_reverse_face(
            self._handle, solid_index, face_index
        ))

    # ----------------------------------------------------------
    # GDT checks
    # ----------------------------------------------------------

    GDT_FLATNESS = 0
    GDT_STRAIGHTNESS = 1
    GDT_CIRCULARITY = 2
    GDT_CYLINDRICITY = 3
    GDT_POSITION = 4
    GDT_PARALLELISM = 5
    GDT_PERPENDICULARITY = 6
    GDT_ANGULARITY = 7
    GDT_RUNOUT = 8
    GDT_PROFILE_OF_LINE = 9
    GDT_PROFILE_OF_SURFACE = 10

    def gdt_check(self, solid_index: int, check_type: int, tolerance_value: float,
                  datum_axis=None, nominal_position=None, nominal_angle_deg=None) -> dict:
        """Run a single GDT check on the solid's mesh.

        Returns a dict with keys: tolerance_value, actual_deviation, passed, status_code.
        """
        use_da = 1 if datum_axis is not None else 0
        use_np = 1 if nominal_position is not None else 0
        use_na = 1 if nominal_angle_deg is not None else 0
        da = datum_axis or (0.0, 0.0, 0.0)
        np_ = nominal_position or (0.0, 0.0, 0.0)
        na = nominal_angle_deg or 0.0

        lib = _get_lib()
        r = lib.draper_solid_gdt_check(
            self._handle, solid_index, check_type, tolerance_value,
            da[0], da[1], da[2], use_da,
            np_[0], np_[1], np_[2], use_np,
            na, use_na,
        )
        return {
            "tolerance_value": r.tolerance_value,
            "actual_deviation": r.actual_deviation,
            "passed": bool(r.passed),
            "status_code": r.status_code,
        }

    def gdt_check_all(self, solid_index: int, json_specs: str) -> str:
        """Run all GDT checks from a JSON array of specs. Returns results as JSON string."""
        ptr = _get_lib().draper_solid_gdt_check_all(
            self._handle, solid_index, json_specs.encode("utf-8")
        )
        if not ptr:
            raise DraperError(DraperResult.UNKNOWN_ERROR, _get_last_error_message())
        try:
            return ctypes.cast(ptr, ctypes.c_char_p).value.decode("utf-8")
        finally:
            _get_lib().draper_free_string(ptr)

    # ----------------------------------------------------------
    # Edge listing
    # ----------------------------------------------------------

    def list_edges(self, solid_index: int) -> str:
        """List all edges in a solid as a JSON array string."""
        ptr = _get_lib().draper_solid_list_edges(self._handle, solid_index)
        if not ptr:
            raise DraperError(DraperResult.UNKNOWN_ERROR, _get_last_error_message())
        try:
            return ctypes.cast(ptr, ctypes.c_char_p).value.decode("utf-8")
        finally:
            _get_lib().draper_free_string(ptr)

    # ----------------------------------------------------------
    # STEP I/O
    # ----------------------------------------------------------

    def load_step(self, path: str, heal: bool = True) -> None:
        """Load a STEP file and append all its solids to the document."""
        _check_result(_get_lib().draper_document_load_step(
            self._handle, path.encode("utf-8"), 1 if heal else 0
        ))

    # ----------------------------------------------------------
    # Bounding box
    # ----------------------------------------------------------

    def bounding_box(self) -> tuple:
        """Return (min_x, min_y, min_z, max_x, max_y, max_z)."""
        buf = (ctypes.c_double * 6)()
        _check_result(_get_lib().draper_document_bbox(self._handle, buf))
        return tuple(buf)


# ============================================================
# Module-level STEP → USDA helper
# ============================================================

def export_step_to_usda(step_path: str, output_path: str,
                        chord_tolerance: float = 0.1,
                        smooth_normals: bool = True,
                        include_camera: bool = True,
                        include_light: bool = True) -> None:
    """Convert a STEP file to a USDA (USD ASCII) file."""
    _check_result(_get_lib().draper_export_step_to_usda(
        step_path.encode("utf-8"),
        output_path.encode("utf-8"),
        chord_tolerance,
        1 if smooth_normals else 0,
        1 if include_camera else 0,
        1 if include_light else 0,
    ))


# ============================================================
# Helpers
# ============================================================

def _get_last_error_message() -> str:
    """Retrieve the last error message from thread-local storage."""
    lib = _get_lib()
    err_ptr = lib.draper_get_last_error()
    if err_ptr:
        raw = ctypes.cast(err_ptr, ctypes.c_char_p).value
        return raw.decode("utf-8", errors="replace") if raw else ""
    return ""
