// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # Digital Twin & IoT Integration
//!
//! Real-time geometry versioning for IoT (Internet of Things) and Digital Twin
//! applications (ROADMAP_VISION_2036 §4.3).
//!
//! This module provides:
//! - **Geometry versioning**: Track geometry changes over time with semantic
//!   version stamps and rollback support.
//! - **IoT telemetry binding**: Bind sensor data streams to geometric parameters
//!   (e.g., temperature → thermal expansion offset).
//! - **Snapshot management**: Capture, diff, and restore geometry states.
//! - **Event journal**: Append-only log of all geometry mutations for audit trail.
//!
//! ## Architecture
//!
//! ```text
//! IoT Device → TelemetryStream → ParameterBinding → GeometryVersion
//!                                                      ↓
//!                                              VersionJournal
//!                                                      ↓
//!                                              Snapshot
//! ```
//!
//! ## Usage
//!
//! ```
//! use draper_core::digital_twin::*;
//!
//! // Create a digital twin manager
//! let mut twin = DigitalTwin::new("pump-001");
//!
//! // Bind a temperature sensor to a thermal expansion parameter
//! twin.bind_parameter(ParameterBinding {
//!     source: TelemetrySource::Sensor("temp_sensor_1".into()),
//!     target: ParameterTarget::Extrude { feature_id: 1, param: "distance".into() },
//!     transform: TransformFormula::Linear { scale: 0.000012, offset: 0.0 },
//!     min_value: Some(-100.0),
//!     max_value: Some(200.0),
//! });
//!
//! // Feed telemetry data
//! twin.update_telemetry("temp_sensor_1", 75.0);
//!
//! // Snapshot the current geometry state
//! let snapshot = twin.snapshot();
//! ```

use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

// ============================================================
// 1. Telemetry Sources & Bindings
// ============================================================

/// A source of real-time telemetry data (sensor, API, computed value).
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum TelemetrySource {
    /// A named sensor (e.g., "temp_sensor_1").
    Sensor(String),
    /// An external API endpoint (e.g., "weather_api:temperature").
    Api(String),
    /// A computed value from another module (e.g., "fea:max_stress").
    Computed(String),
}

/// A target parameter that telemetry data drives.
#[derive(Clone, Debug, PartialEq)]
pub enum ParameterTarget {
    /// Drive a feature history parameter (feature_id, param_name).
    Extrude { feature_id: u64, param: String },
    /// Drive a sketch dimension (sketch_id, dimension_name).
    Sketch { sketch_id: u64, param: String },
    /// Drive a VP node parameter (node_id, param_name).
    VpNode { node_id: u64, param: String },
    /// Drive a material property (instance_id, property).
    Material { instance_id: u64, property: String },
}

/// Transform formula applied to telemetry value before binding.
#[derive(Clone, Debug)]
pub enum TransformFormula {
    /// linear: y = scale * x + offset
    Linear { scale: f64, offset: f64 },
    /// piecewise linear interpolation through (input, output) pairs
    PiecewiseLinear { points: Vec<(f64, f64)> },
    /// constant regardless of input
    Constant(f64),
    /// exponential: y = a * exp(b * x)
    Exponential { a: f64, b: f64 },
}

impl TransformFormula {
    /// Apply the transform to a telemetry value.
    pub fn apply(&self, x: f64) -> f64 {
        match self {
            TransformFormula::Linear { scale, offset } => scale * x + offset,
            TransformFormula::Constant(v) => *v,
            TransformFormula::Exponential { a, b } => a * (b * x).exp(),
            TransformFormula::PiecewiseLinear { points } => {
                if points.is_empty() {
                    return 0.0;
                }
                if points.len() == 1 {
                    return points[0].1;
                }
                // Find the bracketing pair
                if x <= points[0].0 {
                    return points[0].1;
                }
                if x >= points.last().unwrap().0 {
                    return points.last().unwrap().1;
                }
                for i in 0..points.len() - 1 {
                    let (x0, y0) = points[i];
                    let (x1, y1) = points[i + 1];
                    if x >= x0 && x <= x1 {
                        let t = if (x1 - x0).abs() < 1e-10 { 0.0 } else { (x - x0) / (x1 - x0) };
                        return y0 + t * (y1 - y0);
                    }
                }
                points.last().unwrap().1
            }
        }
    }
}

/// A binding from a telemetry source to a geometric parameter.
#[derive(Clone, Debug)]
pub struct ParameterBinding {
    /// Source of telemetry data.
    pub source: TelemetrySource,
    /// Target geometric parameter.
    pub target: ParameterTarget,
    /// Transform applied to telemetry before binding.
    pub transform: TransformFormula,
    /// Optional minimum clamp value.
    pub min_value: Option<f64>,
    /// Optional maximum clamp value.
    pub max_value: Option<f64>,
}

impl ParameterBinding {
    /// Compute the parameter value from a raw telemetry reading.
    pub fn compute_value(&self, raw: f64) -> f64 {
        let mut val = self.transform.apply(raw);
        if let Some(min) = self.min_value {
            val = val.max(min);
        }
        if let Some(max) = self.max_value {
            val = val.min(max);
        }
        val
    }
}

// ============================================================
// 2. Geometry Versioning
// ============================================================

/// A semantic version stamp for geometry state.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct GeometryVersion {
    /// Major version (incremented on topology changes).
    pub major: u32,
    /// Minor version (incremented on parameter changes).
    pub minor: u32,
    /// Patch version (incremented on display changes).
    pub patch: u32,
    /// Unix timestamp when this version was created.
    pub timestamp: u64,
    /// Human-readable description of what changed.
    pub description: String,
}

impl GeometryVersion {
    /// Create version 0.0.0 at time zero.
    pub fn initial() -> Self {
        Self {
            major: 0,
            minor: 0,
            patch: 0,
            timestamp: 0,
            description: "Initial state".to_string(),
        }
    }

    /// Create a new version with the current timestamp.
    pub fn new(major: u32, minor: u32, patch: u32, description: &str) -> Self {
        Self {
            major,
            minor,
            patch,
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_secs())
                .unwrap_or(0),
            description: description.to_string(),
        }
    }

    /// Bump major version (topology change).
    pub fn bump_major(&self, description: &str) -> Self {
        Self::new(self.major + 1, 0, 0, description)
    }

    /// Bump minor version (parameter change).
    pub fn bump_minor(&self, description: &str) -> Self {
        Self::new(self.major, self.minor + 1, 0, description)
    }

    /// Bump patch version (display change).
    pub fn bump_patch(&self, description: &str) -> Self {
        Self::new(self.major, self.minor, self.patch + 1, description)
    }

    /// Format as "M.m.p".
    pub fn version_string(&self) -> String {
        format!("{}.{}.{}", self.major, self.minor, self.patch)
    }
}

impl std::fmt::Display for GeometryVersion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} ({})", self.version_string(), self.description)
    }
}

// ============================================================
// 3. Event Journal (append-only log)
// ============================================================

/// An event in the geometry mutation journal.
#[derive(Clone, Debug)]
pub struct TwinEvent {
    /// Version stamp after this event.
    pub version: GeometryVersion,
    /// What kind of mutation occurred.
    pub event_type: TwinEventType,
    /// Which telemetry source triggered this (if any).
    pub triggered_by: Option<TelemetrySource>,
    /// The raw telemetry value that triggered this (if any).
    pub raw_value: Option<f64>,
    /// The computed parameter value after transform.
    pub computed_value: Option<f64>,
}

/// Type of geometry mutation event.
#[derive(Clone, Debug, PartialEq)]
pub enum TwinEventType {
    /// A parameter was updated via telemetry.
    TelemetryUpdate,
    /// A feature was added to the history.
    FeatureAdded,
    /// A feature was removed.
    FeatureRemoved,
    /// A manual parameter edit.
    ManualEdit,
    /// A snapshot was taken.
    SnapshotTaken,
    /// A rollback was performed.
    Rollback,
}

// ============================================================
// 4. Snapshot Management
// ============================================================

/// A snapshot of the digital twin's state at a point in time.
#[derive(Clone, Debug)]
pub struct TwinSnapshot {
    /// Version when the snapshot was taken.
    pub version: GeometryVersion,
    /// All telemetry values at this point.
    pub telemetry_values: HashMap<String, f64>,
    /// All computed parameter values at this point.
    pub parameter_values: HashMap<String, f64>,
    /// Timestamp of the snapshot.
    pub timestamp: u64,
    /// Optional label for the snapshot.
    pub label: Option<String>,
}

impl TwinSnapshot {
    /// Create a new snapshot.
    pub fn new(
        version: GeometryVersion,
        telemetry: &HashMap<String, f64>,
        params: &HashMap<String, f64>,
    ) -> Self {
        Self {
            version,
            telemetry_values: telemetry.clone(),
            parameter_values: params.clone(),
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_secs())
                .unwrap_or(0),
            label: None,
        }
    }

    /// Label the snapshot for easy identification.
    pub fn labeled(mut self, label: &str) -> Self {
        self.label = Some(label.to_string());
        self
    }

    /// Compute the diff between two snapshots.
    /// Returns a list of (parameter_name, old_value, new_value) for changed params.
    pub fn diff(&self, other: &TwinSnapshot) -> Vec<(String, f64, f64)> {
        let mut changes = Vec::new();
        for (key, &new_val) in &other.parameter_values {
            if let Some(&old_val) = self.parameter_values.get(key) {
                if (old_val - new_val).abs() > 1e-10 {
                    changes.push((key.clone(), old_val, new_val));
                }
            } else {
                changes.push((key.clone(), 0.0, new_val));
            }
        }
        // Also find removed keys
        for (key, &old_val) in &self.parameter_values {
            if !other.parameter_values.contains_key(key) {
                changes.push((key.clone(), old_val, 0.0));
            }
        }
        changes
    }
}

// ============================================================
// 5. Digital Twin Manager
// ============================================================

/// The main Digital Twin manager.
///
/// Tracks geometry versions, manages telemetry bindings, and maintains
/// an event journal for audit trail.
pub struct DigitalTwin {
    /// Unique identifier for this digital twin (e.g., "pump-001").
    pub id: String,
    /// Current geometry version.
    pub current_version: GeometryVersion,
    /// Active parameter bindings.
    pub bindings: Vec<ParameterBinding>,
    /// Latest telemetry values (source_name → raw_value).
    pub telemetry_values: HashMap<String, f64>,
    /// Latest computed parameter values (target_key → computed_value).
    pub parameter_values: HashMap<String, f64>,
    /// Append-only event journal.
    pub journal: Vec<TwinEvent>,
    /// Saved snapshots.
    pub snapshots: Vec<TwinSnapshot>,
}

impl DigitalTwin {
    /// Create a new digital twin with the given ID.
    pub fn new(id: &str) -> Self {
        Self {
            id: id.to_string(),
            current_version: GeometryVersion::initial(),
            bindings: Vec::new(),
            telemetry_values: HashMap::new(),
            parameter_values: HashMap::new(),
            journal: Vec::new(),
            snapshots: Vec::new(),
        }
    }

    /// Bind a telemetry source to a geometric parameter.
    pub fn bind_parameter(&mut self, binding: ParameterBinding) {
        let source_name = match &binding.source {
            TelemetrySource::Sensor(s) | TelemetrySource::Api(s) | TelemetrySource::Computed(s) => s.clone(),
        };
        let target_key = format_target_key(&binding.target);
        // If telemetry already has a value for this source, compute immediately
        if let Some(&raw) = self.telemetry_values.get(&source_name) {
            let computed = binding.compute_value(raw);
            self.parameter_values.insert(target_key.clone(), computed);
        }
        self.bindings.push(binding);
    }

    /// Update a telemetry value and trigger all dependent parameter updates.
    pub fn update_telemetry(&mut self, source_name: &str, raw_value: f64) {
        self.telemetry_values.insert(source_name.to_string(), raw_value);

        // Process all bindings that use this source
        let mut changed_params = Vec::new();
        for binding in &self.bindings {
            let binding_source = match &binding.source {
                TelemetrySource::Sensor(s) | TelemetrySource::Api(s) | TelemetrySource::Computed(s) => s,
            };
            if binding_source == source_name {
                let computed = binding.compute_value(raw_value);
                let target_key = format_target_key(&binding.target);
                self.parameter_values.insert(target_key.clone(), computed);
                changed_params.push((target_key, computed));
            }
        }

        // Log event
        if !changed_params.is_empty() {
            self.current_version = self.current_version.bump_minor(&format!(
                "Telemetry update: {} = {}",
                source_name, raw_value
            ));
            self.journal.push(TwinEvent {
                version: self.current_version.clone(),
                event_type: TwinEventType::TelemetryUpdate,
                triggered_by: Some(TelemetrySource::Sensor(source_name.to_string())),
                raw_value: Some(raw_value),
                computed_value: changed_params.first().map(|(_, v)| *v),
            });
        }
    }

    /// Take a snapshot of the current state.
    pub fn snapshot(&mut self) -> TwinSnapshot {
        let snap = TwinSnapshot::new(
            self.current_version.clone(),
            &self.telemetry_values,
            &self.parameter_values,
        );
        self.snapshots.push(snap.clone());
        self.current_version = self.current_version.bump_patch("Snapshot taken");
        self.journal.push(TwinEvent {
            version: self.current_version.clone(),
            event_type: TwinEventType::SnapshotTaken,
            triggered_by: None,
            raw_value: None,
            computed_value: None,
        });
        snap
    }

    /// Take a labeled snapshot.
    pub fn snapshot_labeled(&mut self, label: &str) -> TwinSnapshot {
        let snap = TwinSnapshot::new(
            self.current_version.clone(),
            &self.telemetry_values,
            &self.parameter_values,
        ).labeled(label);
        self.snapshots.push(snap.clone());
        self.current_version = self.current_version.bump_patch(&format!("Snapshot: {}", label));
        self.journal.push(TwinEvent {
            version: self.current_version.clone(),
            event_type: TwinEventType::SnapshotTaken,
            triggered_by: None,
            raw_value: None,
            computed_value: None,
        });
        snap
    }

    /// Rollback to a specific snapshot.
    /// Returns true if rollback succeeded.
    pub fn rollback(&mut self, snapshot_index: usize) -> bool {
        if snapshot_index >= self.snapshots.len() {
            return false;
        }
        let snapshot = self.snapshots[snapshot_index].clone();
        self.telemetry_values = snapshot.telemetry_values;
        self.parameter_values = snapshot.parameter_values;
        self.current_version = self.current_version.bump_major(&format!(
            "Rollback to {}",
            snapshot.version.version_string()
        ));
        self.journal.push(TwinEvent {
            version: self.current_version.clone(),
            event_type: TwinEventType::Rollback,
            triggered_by: None,
            raw_value: None,
            computed_value: None,
        });
        // Remove all snapshots after the one we rolled back to
        self.snapshots.truncate(snapshot_index + 1);
        true
    }

    /// Get the event journal (read-only).
    pub fn journal(&self) -> &[TwinEvent] {
        &self.journal
    }

    /// Get all snapshots (read-only).
    pub fn snapshots(&self) -> &[TwinSnapshot] {
        &self.snapshots
    }

    /// Get the current parameter value for a target.
    pub fn get_parameter(&self, target: &ParameterTarget) -> Option<f64> {
        let key = format_target_key(target);
        self.parameter_values.get(&key).copied()
    }

    /// Number of telemetry sources currently tracked.
    pub fn telemetry_count(&self) -> usize {
        self.telemetry_values.len()
    }

    /// Number of parameter bindings.
    pub fn binding_count(&self) -> usize {
        self.bindings.len()
    }

    /// Number of events in the journal.
    pub fn journal_len(&self) -> usize {
        self.journal.len()
    }

    /// Number of saved snapshots.
    pub fn snapshot_count(&self) -> usize {
        self.snapshots.len()
    }
}

/// Format a ParameterTarget as a unique string key for HashMap storage.
fn format_target_key(target: &ParameterTarget) -> String {
    match target {
        ParameterTarget::Extrude { feature_id, param } => {
            format!("extrude:{}:{}", feature_id, param)
        }
        ParameterTarget::Sketch { sketch_id, param } => {
            format!("sketch:{}:{}", sketch_id, param)
        }
        ParameterTarget::VpNode { node_id, param } => {
            format!("vp:{}:{}", node_id, param)
        }
        ParameterTarget::Material { instance_id, property } => {
            format!("material:{}:{}", instance_id, property)
        }
    }
}

// ============================================================
// 6. Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transform_linear() {
        let t = TransformFormula::Linear { scale: 2.0, offset: 1.0 };
        assert!((t.apply(5.0) - 11.0).abs() < 1e-10);
    }

    #[test]
    fn test_transform_constant() {
        let t = TransformFormula::Constant(42.0);
        assert!((t.apply(100.0) - 42.0).abs() < 1e-10);
    }

    #[test]
    fn test_transform_exponential() {
        let t = TransformFormula::Exponential { a: 1.0, b: 1.0 };
        assert!((t.apply(0.0) - 1.0).abs() < 1e-10);
        assert!((t.apply(1.0) - std::f64::consts::E).abs() < 1e-6);
    }

    #[test]
    fn test_transform_piecewise_linear() {
        let t = TransformFormula::PiecewiseLinear {
            points: vec![(0.0, 10.0), (50.0, 60.0), (100.0, 110.0)],
        };
        assert!((t.apply(0.0) - 10.0).abs() < 1e-10);
        assert!((t.apply(25.0) - 35.0).abs() < 1e-10);
        assert!((t.apply(50.0) - 60.0).abs() < 1e-10);
        assert!((t.apply(75.0) - 85.0).abs() < 1e-10);
        assert!((t.apply(100.0) - 110.0).abs() < 1e-10);
        // Extrapolation
        assert!((t.apply(-10.0) - 10.0).abs() < 1e-10);
        assert!((t.apply(200.0) - 110.0).abs() < 1e-10);
    }

    #[test]
    fn test_transform_piecewise_empty() {
        let t = TransformFormula::PiecewiseLinear { points: vec![] };
        assert!((t.apply(42.0) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_parameter_binding_clamp() {
        let binding = ParameterBinding {
            source: TelemetrySource::Sensor("temp".into()),
            target: ParameterTarget::Extrude { feature_id: 1, param: "distance".into() },
            transform: TransformFormula::Linear { scale: 1.0, offset: 0.0 },
            min_value: Some(0.0),
            max_value: Some(100.0),
        };
        assert!((binding.compute_value(-5.0) - 0.0).abs() < 1e-10);
        assert!((binding.compute_value(50.0) - 50.0).abs() < 1e-10);
        assert!((binding.compute_value(150.0) - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_geometry_version_bump() {
        let v0 = GeometryVersion::initial();
        assert_eq!(v0.version_string(), "0.0.0");
        let v1 = v0.bump_minor("param change");
        assert_eq!(v1.version_string(), "0.1.0");
        let v2 = v1.bump_major("topology change");
        assert_eq!(v2.version_string(), "1.0.0");
        let v3 = v2.bump_patch("display change");
        assert_eq!(v3.version_string(), "1.0.1");
    }

    #[test]
    fn test_digital_twin_create() {
        let twin = DigitalTwin::new("pump-001");
        assert_eq!(twin.id, "pump-001");
        assert_eq!(twin.current_version.version_string(), "0.0.0");
        assert_eq!(twin.binding_count(), 0);
        assert_eq!(twin.telemetry_count(), 0);
        assert_eq!(twin.journal_len(), 0);
    }

    #[test]
    fn test_digital_twin_telemetry_update() {
        let mut twin = DigitalTwin::new("sensor-test");
        twin.bind_parameter(ParameterBinding {
            source: TelemetrySource::Sensor("temp".into()),
            target: ParameterTarget::Extrude { feature_id: 1, param: "distance".into() },
            transform: TransformFormula::Linear { scale: 0.01, offset: 0.0 },
            min_value: None,
            max_value: None,
        });

        // Initially no parameter value
        assert!(twin.get_parameter(&ParameterTarget::Extrude { feature_id: 1, param: "distance".into() }).is_none());

        // Update telemetry
        twin.update_telemetry("temp", 100.0);

        // Parameter should be computed: 0.01 * 100 = 1.0
        let param = twin.get_parameter(&ParameterTarget::Extrude { feature_id: 1, param: "distance".into() });
        assert!(param.is_some());
        assert!((param.unwrap() - 1.0).abs() < 1e-10);

        // Version should have bumped
        assert_eq!(twin.current_version.minor, 1);

        // Journal should have an entry
        assert_eq!(twin.journal_len(), 1);
        assert_eq!(twin.journal()[0].event_type, TwinEventType::TelemetryUpdate);
    }

    #[test]
    fn test_digital_twin_snapshot() {
        let mut twin = DigitalTwin::new("snap-test");
        twin.bind_parameter(ParameterBinding {
            source: TelemetrySource::Sensor("temp".into()),
            target: ParameterTarget::Extrude { feature_id: 1, param: "distance".into() },
            transform: TransformFormula::Linear { scale: 0.01, offset: 0.0 },
            min_value: None,
            max_value: None,
        });
        twin.update_telemetry("temp", 50.0);

        let snap = twin.snapshot();
        assert_eq!(snap.version.minor, 1); // snapshot taken after telemetry update
        assert_eq!(twin.snapshot_count(), 1);

        // Change telemetry
        twin.update_telemetry("temp", 100.0);
        let param = twin.get_parameter(&ParameterTarget::Extrude { feature_id: 1, param: "distance".into() }).unwrap();
        assert!((param - 1.0).abs() < 1e-10);

        // Snapshot diff
        let snap2 = twin.snapshot();
        let diff = snap.diff(&snap2);
        assert_eq!(diff.len(), 1);
        assert!((diff[0].1 - 0.5).abs() < 1e-10); // old value was 0.5
        assert!((diff[0].2 - 1.0).abs() < 1e-10);  // new value is 1.0
    }

    #[test]
    fn test_digital_twin_rollback() {
        let mut twin = DigitalTwin::new("rollback-test");
        twin.bind_parameter(ParameterBinding {
            source: TelemetrySource::Sensor("temp".into()),
            target: ParameterTarget::Extrude { feature_id: 1, param: "distance".into() },
            transform: TransformFormula::Linear { scale: 1.0, offset: 0.0 },
            min_value: None,
            max_value: None,
        });

        twin.update_telemetry("temp", 42.0);
        twin.snapshot_labeled("initial");

        twin.update_telemetry("temp", 99.0);
        let param = twin.get_parameter(&ParameterTarget::Extrude { feature_id: 1, param: "distance".into() }).unwrap();
        assert!((param - 99.0).abs() < 1e-10);

        // Rollback to snapshot 0
        assert!(twin.rollback(0));

        let param = twin.get_parameter(&ParameterTarget::Extrude { feature_id: 1, param: "distance".into() }).unwrap();
        assert!((param - 42.0).abs() < 1e-10);

        // Major version should have bumped
        assert!(twin.current_version.major > 0);

        // Journal should have rollback event
        let last_event = twin.journal().last().unwrap();
        assert_eq!(last_event.event_type, TwinEventType::Rollback);
    }

    #[test]
    fn test_digital_twin_multiple_bindings() {
        let mut twin = DigitalTwin::new("multi-bind");

        // Two sensors driving the same feature differently
        twin.bind_parameter(ParameterBinding {
            source: TelemetrySource::Sensor("temp".into()),
            target: ParameterTarget::Extrude { feature_id: 1, param: "distance".into() },
            transform: TransformFormula::Linear { scale: 0.01, offset: 0.0 },
            min_value: None,
            max_value: None,
        });
        twin.bind_parameter(ParameterBinding {
            source: TelemetrySource::Sensor("pressure".into()),
            target: ParameterTarget::Extrude { feature_id: 2, param: "distance".into() },
            transform: TransformFormula::Linear { scale: 0.1, offset: 0.0 },
            min_value: None,
            max_value: None,
        });

        twin.update_telemetry("temp", 100.0);
        twin.update_telemetry("pressure", 50.0);

        let p1 = twin.get_parameter(&ParameterTarget::Extrude { feature_id: 1, param: "distance".into() }).unwrap();
        let p2 = twin.get_parameter(&ParameterTarget::Extrude { feature_id: 2, param: "distance".into() }).unwrap();
        assert!((p1 - 1.0).abs() < 1e-10);   // 0.01 * 100
        assert!((p2 - 5.0).abs() < 1e-10);    // 0.1 * 50
    }

    #[test]
    fn test_snapshot_diff() {
        let mut v1 = HashMap::new();
        v1.insert("a".to_string(), 1.0);
        v1.insert("b".to_string(), 2.0);
        let s1 = TwinSnapshot::new(GeometryVersion::initial(), &HashMap::new(), &v1);

        let mut v2 = HashMap::new();
        v2.insert("a".to_string(), 1.5);  // changed
        v2.insert("b".to_string(), 2.0);  // unchanged
        v2.insert("c".to_string(), 3.0);  // added
        let s2 = TwinSnapshot::new(GeometryVersion::initial(), &HashMap::new(), &v2);

        let diff = s1.diff(&s2);
        assert_eq!(diff.len(), 2); // "a" changed, "c" added
        // "a" should be in the diff (1.0 → 1.5)
        let a_change = diff.iter().find(|(k, _, _)| k == "a");
        assert!(a_change.is_some());
        assert!((a_change.unwrap().1 - 1.0).abs() < 1e-10);
        assert!((a_change.unwrap().2 - 1.5).abs() < 1e-10);
    }

    #[test]
    fn test_thermal_expansion_binding() {
        // Real-world example: thermal expansion of a steel rod.
        // Coefficient of thermal expansion (steel): ~12e-6 /°C
        // At 20°C (reference), length = 100mm.
        // At temp T, length = 100 * (1 + 12e-6 * (T - 20))
        // Linear transform: scale = 12e-6 * 100 = 0.0012, offset = 100 - 0.0012 * 20 = 99.976
        let mut twin = DigitalTwin::new("steel-rod-001");
        twin.bind_parameter(ParameterBinding {
            source: TelemetrySource::Sensor("temp".into()),
            target: ParameterTarget::Extrude { feature_id: 1, param: "distance".into() },
            transform: TransformFormula::Linear { scale: 0.0012, offset: 99.976 },
            min_value: Some(0.0),
            max_value: None,
        });

        // At 20°C: length = 0.0012 * 20 + 99.976 = 100.0
        twin.update_telemetry("temp", 20.0);
        let length = twin.get_parameter(&ParameterTarget::Extrude { feature_id: 1, param: "distance".into() }).unwrap();
        assert!((length - 100.0).abs() < 1e-3);

        // At 100°C: length = 0.0012 * 100 + 99.976 = 100.096
        twin.update_telemetry("temp", 100.0);
        let length = twin.get_parameter(&ParameterTarget::Extrude { feature_id: 1, param: "distance".into() }).unwrap();
        assert!((length - 100.096).abs() < 1e-3);
    }

    #[test]
    fn test_format_target_key() {
        let t1 = ParameterTarget::Extrude { feature_id: 1, param: "distance".into() };
        assert_eq!(format_target_key(&t1), "extrude:1:distance");

        let t2 = ParameterTarget::Sketch { sketch_id: 5, param: "width".into() };
        assert_eq!(format_target_key(&t2), "sketch:5:width");

        let t3 = ParameterTarget::VpNode { node_id: 42, param: "radius".into() };
        assert_eq!(format_target_key(&t3), "vp:42:radius");

        let t4 = ParameterTarget::Material { instance_id: 3, property: "density".into() };
        assert_eq!(format_target_key(&t4), "material:3:density");
    }
}
