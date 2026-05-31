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
