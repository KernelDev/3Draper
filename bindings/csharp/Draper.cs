// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//
// C# bindings for the 3Draper kernel via P/Invoke.
//
// Usage:
//     using Draper;
//
//     using var doc = new DraperDocument("MyModel");
//     doc.AddBox(1.0, 2.0, 3.0);
//     double vol = doc.Volume();
//     doc.ExportGltf("output.glb");
//
// The native library (libdraper_ffi.so / draper_ffi.dll) must be on the
// DLL search path.  On Linux/macOS you may need to set LD_LIBRARY_PATH.

using System;
using System.Runtime.InteropServices;
using System.Text;

namespace Draper
{
    // ================================================================
    // DraperResult enum
    // ================================================================

    /// <summary>
    /// C-compatible result codes returned by every FFI function.
    /// </summary>
    public enum DraperResult : int
    {
        /// <summary>Operation succeeded.</summary>
        Success = 0,

        /// <summary>An invalid argument was passed.</summary>
        InvalidArgument = -1,

        /// <summary>The requested file was not found.</summary>
        FileNotFound = -2,

        /// <summary>Parsing of a file or data structure failed.</summary>
        ParseError = -3,

        /// <summary>A geometry evaluation failed.</summary>
        GeometryError = -4,

        /// <summary>A topology error occurred.</summary>
        TopologyError = -5,

        /// <summary>Triangulation / mesh generation failed.</summary>
        TriangulationError = -6,

        /// <summary>Out of memory or resource limit exceeded.</summary>
        OutOfMemory = -7,

        /// <summary>An unclassified error occurred.</summary>
        UnknownError = -99,
    }

    // ================================================================
    // DraperException
    // ================================================================

    /// <summary>
    /// Exception thrown when a 3Draper FFI call returns an error.
    /// </summary>
    public class DraperException : Exception
    {
        /// <summary>The FFI result code.</summary>
        public DraperResult ResultCode { get; }

        public DraperException(DraperResult code, string message)
            : base($"DraperException({(int)code}): {message}")
        {
            ResultCode = code;
        }

        public DraperException(DraperResult code)
            : this(code, ResultMessage(code))
        {
        }

        private static string ResultMessage(DraperResult code) => code switch
        {
            DraperResult.Success => "Success",
            DraperResult.InvalidArgument => "Invalid argument",
            DraperResult.FileNotFound => "File not found",
            DraperResult.ParseError => "Parse error",
            DraperResult.GeometryError => "Geometry error",
            DraperResult.TopologyError => "Topology error",
            DraperResult.TriangulationError => "Triangulation error",
            DraperResult.OutOfMemory => "Out of memory",
            _ => $"Unknown error ({(int)code})",
        };
    }

    // ================================================================
    // Native P/Invoke declarations
    // ================================================================

    internal static class Native
    {
        const string DLL = "draper_ffi";

        // -- Version --

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint draper_version_major();

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint draper_version_minor();

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint draper_version_patch();

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr draper_version_string();

        // -- Feature detection --

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.U1)]
        public static extern bool draper_has_feature([MarshalAs(UnmanagedType.LPStr)] string feature);

        // -- Error --

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr draper_get_last_error();

        // -- Document --

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr draper_document_new([MarshalAs(UnmanagedType.LPStr)] string name);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void draper_document_free(IntPtr doc);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint draper_document_solid_count(IntPtr doc);

        // -- Shape builders --

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern DraperResult draper_document_add_box(IntPtr doc, double dx, double dy, double dz);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern DraperResult draper_document_add_cylinder(IntPtr doc, double radius, double height);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern DraperResult draper_document_add_sphere(IntPtr doc, double radius);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern DraperResult draper_document_add_cone(IntPtr doc, double radius, double height);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern DraperResult draper_document_add_torus(IntPtr doc, double majorRadius, double minorRadius);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern DraperResult draper_document_add_engine(IntPtr doc);

        // -- Triangulation --

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr draper_document_triangulate(IntPtr doc);

        // -- Mesh --

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void draper_mesh_free(IntPtr mesh);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint draper_mesh_vertex_count(IntPtr mesh);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint draper_mesh_triangle_count(IntPtr mesh);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint draper_mesh_get_vertices(IntPtr mesh, double[] outBuf, uint maxCount);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint draper_mesh_get_triangles(IntPtr mesh, uint[] outBuf, uint maxCount);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern DraperResult draper_mesh_export_stl(IntPtr mesh, [MarshalAs(UnmanagedType.LPStr)] string path, int binary);

        // -- STEP export --

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern DraperResult draper_document_export_step(IntPtr doc, [MarshalAs(UnmanagedType.LPStr)] string path);

        // -- Analytical queries --

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern double draper_solid_volume(IntPtr doc);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern double draper_solid_surface_area(IntPtr doc);

        // -- Validation --

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern DraperResult draper_validate_step(IntPtr doc);

        // -- Export convenience --

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern DraperResult draper_export_gltf(IntPtr doc, [MarshalAs(UnmanagedType.LPStr)] string path);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern DraperResult draper_export_obj(IntPtr doc, [MarshalAs(UnmanagedType.LPStr)] string path);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern DraperResult draper_export_3mf(IntPtr doc, [MarshalAs(UnmanagedType.LPStr)] string path);
    }

    // ================================================================
    // DraperMesh class
    // ================================================================

    /// <summary>
    /// Wrapper around a DraperMesh opaque handle.
    /// Obtain via DraperDocument.Triangulate().  Dispose to free the
    /// underlying handle.
    /// </summary>
    public class DraperMesh : IDisposable
    {
        internal IntPtr _handle;
        private bool _disposed;

        internal DraperMesh(IntPtr handle)
        {
            _handle = handle;
        }

        /// <summary>Number of vertices in the mesh.</summary>
        public uint VertexCount => Native.draper_mesh_vertex_count(_handle);

        /// <summary>Number of triangles in the mesh.</summary>
        public uint TriangleCount => Native.draper_mesh_triangle_count(_handle);

        /// <summary>
        /// Get vertex positions as an array of (x, y, z) triplets.
        /// The returned array has length VertexCount * 3.
        /// </summary>
        public double[] GetVertices()
        {
            uint n = VertexCount;
            if (n == 0) return Array.Empty<double>();
            double[] buf = new double[n * 3];
            Native.draper_mesh_get_vertices(_handle, buf, n);
            return buf;
        }

        /// <summary>
        /// Get triangle indices as an array of (i, j, k) triplets.
        /// The returned array has length TriangleCount * 3.
        /// </summary>
        public uint[] GetTriangles()
        {
            uint n = TriangleCount;
            if (n == 0) return Array.Empty<uint>();
            uint[] buf = new uint[n * 3];
            Native.draper_mesh_get_triangles(_handle, buf, n);
            return buf;
        }

        /// <summary>Export mesh to STL file.</summary>
        public void ExportStl(string path, bool binary = true)
        {
            var result = Native.draper_mesh_export_stl(_handle, path, binary ? 1 : 0);
            DraperDocument.CheckResult(result);
        }

        public void Dispose()
        {
            if (!_disposed && _handle != IntPtr.Zero)
            {
                Native.draper_mesh_free(_handle);
                _handle = IntPtr.Zero;
            }
            _disposed = true;
        }
    }

    // ================================================================
    // DraperDocument class
    // ================================================================

    /// <summary>
    /// High-level C# wrapper for a 3Draper document.
    /// </summary>
    /// <example>
    /// <code>
    /// using var doc = new DraperDocument("MyModel");
    /// doc.AddBox(1.0, 2.0, 3.0);
    /// double vol = doc.Volume();
    /// doc.ExportGltf("output.glb");
    /// </code>
    /// </example>
    public class DraperDocument : IDisposable
    {
        internal IntPtr _handle;
        private bool _disposed;

        // -- Static helpers --

        /// <summary>Library version as a string (e.g. "0.1.0").</summary>
        public static string Version
        {
            get
            {
                IntPtr ptr = Native.draper_version_string();
                return Marshal.PtrToStringUTF8(ptr) ?? "unknown";
            }
        }

        /// <summary>Library version as a (major, minor, patch) tuple.</summary>
        public static (uint Major, uint Minor, uint Patch) VersionTuple =>
            (Native.draper_version_major(),
             Native.draper_version_minor(),
             Native.draper_version_patch());

        /// <summary>Check whether the library supports a named feature.</summary>
        public static bool HasFeature(string feature) => Native.draper_has_feature(feature);

        // -- Constructor / Dispose --

        public DraperDocument(string name = "Untitled")
        {
            _handle = Native.draper_document_new(name);
            if (_handle == IntPtr.Zero)
                throw new DraperException(DraperResult.UnknownError, "Failed to create document");
        }

        public void Dispose()
        {
            if (!_disposed && _handle != IntPtr.Zero)
            {
                Native.draper_document_free(_handle);
                _handle = IntPtr.Zero;
            }
            _disposed = true;
        }

        // -- Shape builders --

        /// <summary>Add a box primitive with dimensions (dx, dy, dz).</summary>
        public void AddBox(double dx, double dy, double dz) =>
            CheckResult(Native.draper_document_add_box(_handle, dx, dy, dz));

        /// <summary>Add a cylinder primitive.</summary>
        public void AddCylinder(double radius, double height) =>
            CheckResult(Native.draper_document_add_cylinder(_handle, radius, height));

        /// <summary>Add a sphere primitive.</summary>
        public void AddSphere(double radius) =>
            CheckResult(Native.draper_document_add_sphere(_handle, radius));

        /// <summary>Add a cone primitive.</summary>
        public void AddCone(double radius, double height) =>
            CheckResult(Native.draper_document_add_cone(_handle, radius, height));

        /// <summary>Add a torus primitive.</summary>
        public void AddTorus(double majorRadius, double minorRadius) =>
            CheckResult(Native.draper_document_add_torus(_handle, majorRadius, minorRadius));

        /// <summary>Add a built-in ICE engine model.</summary>
        public void AddEngine() =>
            CheckResult(Native.draper_document_add_engine(_handle));

        // -- Properties --

        /// <summary>Number of solids in the document.</summary>
        public uint SolidCount => Native.draper_document_solid_count(_handle);

        // -- Triangulation --

        /// <summary>Triangulate the document and return a DraperMesh.</summary>
        public DraperMesh Triangulate()
        {
            IntPtr meshHandle = Native.draper_document_triangulate(_handle);
            if (meshHandle == IntPtr.Zero)
                throw new DraperException(DraperResult.TriangulationError, GetLastError());
            return new DraperMesh(meshHandle);
        }

        // -- Analytical queries --

        /// <summary>Compute the total volume of all solids in the document.</summary>
        public double Volume() => Native.draper_solid_volume(_handle);

        /// <summary>Compute the total surface area of all solids in the document.</summary>
        public double SurfaceArea() => Native.draper_solid_surface_area(_handle);

        // -- Validation --

        /// <summary>
        /// Run topology validation on all solids.
        /// Throws DraperException if any error-level issues are found.
        /// </summary>
        public void Validate() =>
            CheckResult(Native.draper_validate_step(_handle));

        // -- Export --

        /// <summary>Export the document to STEP AP242 format.</summary>
        public void ExportStep(string path) =>
            CheckResult(Native.draper_document_export_step(_handle, path));

        /// <summary>Export the document to glTF 2.0 (GLB binary) format.</summary>
        public void ExportGltf(string path) =>
            CheckResult(Native.draper_export_gltf(_handle, path));

        /// <summary>Export the document to Wavefront OBJ format.</summary>
        public void ExportObj(string path) =>
            CheckResult(Native.draper_export_obj(_handle, path));

        /// <summary>Export the document to 3MF (3D Manufacturing Format).</summary>
        public void Export3mf(string path) =>
            CheckResult(Native.draper_export_3mf(_handle, path));

        // -- Internal helpers --

        internal static void CheckResult(DraperResult result)
        {
            if (result != DraperResult.Success)
            {
                string msg = GetLastError();
                throw new DraperException(result, msg);
            }
        }

        private static string GetLastError()
        {
            IntPtr ptr = Native.draper_get_last_error();
            if (ptr == IntPtr.Zero) return "";
            return Marshal.PtrToStringUTF8(ptr) ?? "";
        }
    }
}
