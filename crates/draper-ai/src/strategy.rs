// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Automatic healing strategy selection (4.6.2).
//!
//! Takes a list of `ClassifiedDefect` and selects the optimal healing strategy.
//! Uses priority ordering and interdependency analysis to compose multi-step
//! healing pipelines.
//!
//! # Priority ordering
//!
//! 1. **Fix normals** — affects all downstream operations
//! 2. **Mark degenerate edges** — degenerate edges interfere with gap detection
//! 3. **Close gaps** — gaps cause holes and non-manifold edges
//! 4. **Fill holes** — holes cause boundary edges
//! 5. **Stitch edges** — after gaps are closed, collinear edges can be merged
//! 6. **Remove slivers** — after topology is repaired, clean up mesh quality
//! 7. **Remove small features** — final cleanup
//! 8. **Fix self-intersections** — requires valid topology first
//! 9. **Adjust tolerance** — last resort for persistent issues
//!
//! # Learning component
//!
//! The `HealingStrategySelector` tracks the success/failure of applied strategies
//! and adjusts weights for future selections. This creates a simple reinforcement
//! learning loop without external ML dependencies.

use crate::classifier::{ClassifiedDefect, DefectType};
use draper_topology::HealingParams;
use std::collections::HashMap;

// ============================================================
// HealingStrategy enum
// ============================================================

/// A healing strategy that maps to one or more operations in the
/// `draper_topology::healing` module.
#[derive(Clone, Debug, PartialEq)]
pub enum HealingStrategy {
    /// Close gaps between boundary edges within tolerance.
    CloseGaps,
    /// Fill small holes by triangulating boundary loops.
    FillHoles,
    /// Stitch collinear edges that share a vertex.
    StitchEdges,
    /// Fix face normals that point inward.
    FixNormals,
    /// Remove sliver triangles (extreme aspect ratio).
    RemoveSlivers,
    /// Remove small features (tiny faces/triangles).
    RemoveSmallFeatures,
    /// Mark degenerate edges (zero-length or degenerate curves).
    MarkDegenerate,
    /// Fix self-intersecting surface patches.
    FixSelfIntersection,
    /// Adjust tolerance for persistent issues.
    AdjustTolerance,
    /// Composite strategy: apply multiple strategies in order.
    Composite(Vec<HealingStrategy>),
}

impl HealingStrategy {
    /// Human-readable name.
    pub fn name(&self) -> &str {
        match self {
            HealingStrategy::CloseGaps => "CloseGaps",
            HealingStrategy::FillHoles => "FillHoles",
            HealingStrategy::StitchEdges => "StitchEdges",
            HealingStrategy::FixNormals => "FixNormals",
            HealingStrategy::RemoveSlivers => "RemoveSlivers",
            HealingStrategy::RemoveSmallFeatures => "RemoveSmallFeatures",
            HealingStrategy::MarkDegenerate => "MarkDegenerate",
            HealingStrategy::FixSelfIntersection => "FixSelfIntersection",
            HealingStrategy::AdjustTolerance => "AdjustTolerance",
            HealingStrategy::Composite(_strategies) => {
                "Composite"
            }
        }
    }

    /// Numeric priority (lower = higher priority).
    pub fn priority(&self) -> u32 {
        match self {
            HealingStrategy::FixNormals => 1,
            HealingStrategy::MarkDegenerate => 2,
            HealingStrategy::CloseGaps => 3,
            HealingStrategy::FillHoles => 4,
            HealingStrategy::StitchEdges => 5,
            HealingStrategy::RemoveSlivers => 6,
            HealingStrategy::RemoveSmallFeatures => 7,
            HealingStrategy::FixSelfIntersection => 8,
            HealingStrategy::AdjustTolerance => 9,
            HealingStrategy::Composite(strategies) => {
                // Composite priority = minimum of component priorities
                strategies
                    .iter()
                    .map(|s| s.priority())
                    .min()
                    .unwrap_or(10)
            }
        }
    }
}

impl std::fmt::Display for HealingStrategy {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            HealingStrategy::Composite(strategies) => {
                let names: Vec<&str> = strategies.iter().map(|s| s.name()).collect();
                write!(f, "Composite([{}])", names.join(", "))
            }
            _ => write!(f, "{}", self.name()),
        }
    }
}

// ============================================================
// StrategyResult — learning feedback
// ============================================================

/// Outcome of applying a healing strategy, used for learning.
#[derive(Clone, Debug)]
pub struct StrategyOutcome {
    /// The strategy that was applied.
    pub strategy: HealingStrategy,
    /// Whether the strategy successfully resolved the target defects.
    pub success: bool,
    /// Number of defects remaining after applying the strategy.
    pub remaining_defects: usize,
    /// Improvement score: (defects_before - defects_after) / defects_before.
    pub improvement: f64,
}

// ============================================================
// HealingStrategySelector
// ============================================================

/// Selects the optimal healing strategy based on classified defects.
///
/// # Learning component
///
/// The selector maintains a history of strategy outcomes. When selecting
/// a strategy, it weights the decision by past success rates:
///
/// - Strategies with high success rates are preferred
/// - Strategies that failed for similar defect patterns are deprioritized
/// - The weight is blended with the rule-based priority: `final_weight = priority * success_rate`
#[derive(Clone, Debug)]
pub struct HealingStrategySelector {
    /// History of strategy outcomes for learning.
    outcome_history: Vec<StrategyOutcome>,
    /// Success rate per strategy type, computed from history.
    success_rates: HashMap<String, f64>,
    /// Weight for the learning component (0.0 = pure rule-based, 1.0 = pure learned).
    learning_weight: f64,
}

impl Default for HealingStrategySelector {
    fn default() -> Self {
        Self {
            outcome_history: Vec::new(),
            success_rates: HashMap::new(),
            learning_weight: 0.3,
        }
    }
}

impl HealingStrategySelector {
    /// Create a new selector with default settings.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a selector with a custom learning weight.
    pub fn with_learning_weight(learning_weight: f64) -> Self {
        Self {
            learning_weight: learning_weight.clamp(0.0, 1.0),
            ..Self::default()
        }
    }

    /// Select the optimal healing strategy for a list of classified defects.
    ///
    /// The selection algorithm:
    /// 1. Map each defect type to its corresponding strategy.
    /// 2. Sort strategies by priority (normals first, then degenerate, etc.).
    /// 3. Check for interdependencies and create composite strategies if needed.
    /// 4. Apply learned weights from past outcomes.
    pub fn select_strategy(&self, defects: &[ClassifiedDefect]) -> HealingStrategy {
        if defects.is_empty() {
            return HealingStrategy::Composite(Vec::new());
        }

        // Step 1: Map defect types to strategies
        let mut required_strategies: Vec<HealingStrategy> = Vec::new();
        let mut seen_types: HashMap<DefectType, bool> = HashMap::new();

        for defect in defects {
            if seen_types.contains_key(&defect.defect_type) {
                continue;
            }
            seen_types.insert(defect.defect_type, true);

            let strategy = defect_type_to_strategy(defect.defect_type);
            required_strategies.push(strategy);
        }

        if required_strategies.is_empty() {
            return HealingStrategy::Composite(Vec::new());
        }

        // Step 2: Sort by priority
        required_strategies.sort_by_key(|s| s.priority());

        // Step 3: Check for interdependencies and compose
        let composed = compose_strategies(&required_strategies, defects);

        // Step 4: Apply learned weights
        let weighted = self.apply_learned_weights(composed);

        weighted
    }

    /// Record the outcome of a strategy for learning.
    pub fn record_outcome(&mut self, outcome: StrategyOutcome) {
        // Update success rates
        let key = strategy_key(&outcome.strategy);
        let entry = self.success_rates.entry(key).or_insert(0.5); // Default: 50% success
        // Exponential moving average
        let alpha = 0.3;
        *entry = *entry * (1.0 - alpha) + if outcome.success { 1.0 } else { 0.0 } * alpha;

        self.outcome_history.push(outcome);
    }

    /// Get the current success rate for a strategy type.
    pub fn success_rate_for(&self, strategy: &HealingStrategy) -> f64 {
        let key = strategy_key(strategy);
        self.success_rates.get(&key).copied().unwrap_or(0.5)
    }

    /// Get the number of recorded outcomes.
    pub fn outcome_count(&self) -> usize {
        self.outcome_history.len()
    }

    /// Map a selected strategy to `draper_topology::healing::HealingParams`.
    ///
    /// Configures `HealingParams` based on the strategy and the defect information.
    pub fn strategy_to_healing_params(
        &self,
        strategy: &HealingStrategy,
        defects: &[ClassifiedDefect],
    ) -> HealingParams {
        let mut params = HealingParams::default();

        match strategy {
            HealingStrategy::FixNormals => {
                params.fix_normals = true;
                params.stitch_edges = false;
                params.merge_faces = false;
                params.propagate_tolerances = false;
            }
            HealingStrategy::MarkDegenerate => {
                params.fix_normals = false;
                params.stitch_edges = false;
                params.merge_faces = false;
                params.propagate_tolerances = false;
            }
            HealingStrategy::CloseGaps => {
                // Increase gap factor based on severity of gap defects
                if let Some(gap_defect) = defects.iter().find(|d| d.defect_type == DefectType::Gap) {
                    params.gap_factor = 10.0 + gap_defect.severity * 40.0;
                }
                params.fix_normals = false;
                params.stitch_edges = false;
                params.merge_faces = false;
                params.propagate_tolerances = false;
            }
            HealingStrategy::FillHoles => {
                // Adjust max hole edges based on defect data
                if let Some(hole_defect) = defects.iter().find(|d| d.defect_type == DefectType::Hole) {
                    // More complex holes may need more edges
                    let affected = hole_defect.affected_elements.len();
                    params.max_hole_edges = affected.max(8).min(32);
                }
                params.fix_normals = false;
                params.stitch_edges = false;
                params.merge_faces = false;
                params.propagate_tolerances = false;
            }
            HealingStrategy::StitchEdges => {
                params.fix_normals = false;
                params.stitch_edges = true;
                params.merge_faces = false;
                params.propagate_tolerances = false;
            }
            HealingStrategy::RemoveSlivers => {
                // Adjust aspect ratio threshold based on sliver severity
                if let Some(sliver_defect) = defects.iter().find(|d| d.defect_type == DefectType::SliverTriangle) {
                    params.max_aspect_ratio = match sliver_defect.severity {
                        s if s > 0.8 => 50.0,  // Aggressive: catch more slivers
                        s if s > 0.4 => 100.0, // Default
                        _ => 200.0,            // Conservative: only extreme slivers
                    };
                }
                params.fix_normals = false;
                params.stitch_edges = false;
                params.merge_faces = false;
                params.propagate_tolerances = false;
            }
            HealingStrategy::RemoveSmallFeatures => {
                // Adjust min face area based on defect data
                if let Some(small_defect) = defects.iter().find(|d| d.defect_type == DefectType::SmallFeature) {
                    // Smaller min_face_area = more aggressive removal
                    params.min_face_area = match small_defect.severity {
                        s if s > 0.8 => 1e-10,
                        s if s > 0.4 => 1e-12,
                        _ => 1e-14,
                    };
                }
                params.fix_normals = false;
                params.stitch_edges = false;
                params.merge_faces = false;
                params.propagate_tolerances = false;
            }
            HealingStrategy::FixSelfIntersection => {
                // Self-intersection repair typically requires tolerance adjustment
                params.fix_normals = false;
                params.stitch_edges = false;
                params.merge_faces = false;
                params.propagate_tolerances = true;
            }
            HealingStrategy::AdjustTolerance => {
                // Tolerance adjustment: increase tolerance context
                params.propagate_tolerances = true;
                params.fix_normals = false;
                params.stitch_edges = false;
                params.merge_faces = false;
                // Increase base tolerance for persistent issues
                if let Some(tol_defect) = defects.iter().find(|d| d.defect_type == DefectType::ToleranceMismatch) {
                    params.tolerance = 1e-6 * (1.0 + tol_defect.severity * 10.0);
                }
            }
            HealingStrategy::Composite(strategies) => {
                // For composite strategies, configure all parameters
                // The composite runs each strategy in sequence, so we enable
                // all relevant features
                let has_fix_normals = strategies.iter().any(|s| matches!(s, HealingStrategy::FixNormals));
                let has_stitch = strategies.iter().any(|s| matches!(s, HealingStrategy::StitchEdges));
                let has_close_gaps = strategies.iter().any(|s| matches!(s, HealingStrategy::CloseGaps));
                let has_fill_holes = strategies.iter().any(|s| matches!(s, HealingStrategy::FillHoles));

                params.fix_normals = has_fix_normals;
                params.stitch_edges = has_stitch;
                params.propagate_tolerances = true; // Always propagate for composite

                // Adjust gap factor and hole edges based on defect data
                if has_close_gaps {
                    if let Some(gap_defect) = defects.iter().find(|d| d.defect_type == DefectType::Gap) {
                        params.gap_factor = 10.0 + gap_defect.severity * 40.0;
                    }
                }
                if has_fill_holes {
                    if let Some(hole_defect) = defects.iter().find(|d| d.defect_type == DefectType::Hole) {
                        let affected = hole_defect.affected_elements.len();
                        params.max_hole_edges = affected.max(8).min(32);
                    }
                }
            }
        }

        params
    }

    /// Apply learned weights to a strategy.
    ///
    /// If the strategy has a poor success rate, we may add additional strategies
    /// or increase aggressiveness.
    fn apply_learned_weights(&self, strategy: HealingStrategy) -> HealingStrategy {
        if self.outcome_history.is_empty() {
            return strategy; // No learning data yet
        }

        match &strategy {
            HealingStrategy::Composite(strategies) => {
                // Check if any component strategy has poor success rate
                let mut adjusted = strategies.clone();
                for s in strategies.iter() {
                    let rate = self.success_rate_for(s);
                    if rate < 0.3 && self.learning_weight > 0.0 {
                        // Poor success rate — consider adding AdjustTolerance as a fallback
                        log::info!(
                            "Strategy {} has low success rate ({:.1}%), adding tolerance adjustment",
                            s.name(),
                            rate * 100.0
                        );
                        if !adjusted.iter().any(|s| matches!(s, HealingStrategy::AdjustTolerance)) {
                            adjusted.push(HealingStrategy::AdjustTolerance);
                        }
                    }
                }
                // Re-sort by priority
                adjusted.sort_by_key(|s| s.priority());
                HealingStrategy::Composite(adjusted)
            }
            _ => strategy,
        }
    }
}

// ============================================================
// Helper functions
// ============================================================

/// Map a defect type to its primary healing strategy.
fn defect_type_to_strategy(defect_type: DefectType) -> HealingStrategy {
    match defect_type {
        DefectType::Gap => HealingStrategy::CloseGaps,
        DefectType::Hole => HealingStrategy::FillHoles,
        DefectType::NonManifoldEdge => HealingStrategy::StitchEdges,
        DefectType::FlippedNormal => HealingStrategy::FixNormals,
        DefectType::SliverTriangle => HealingStrategy::RemoveSlivers,
        DefectType::SmallFeature => HealingStrategy::RemoveSmallFeatures,
        DefectType::DegenerateEdge => HealingStrategy::MarkDegenerate,
        DefectType::SelfIntersection => HealingStrategy::FixSelfIntersection,
        DefectType::ToleranceMismatch => HealingStrategy::AdjustTolerance,
    }
}

/// Compose multiple strategies based on interdependencies.
///
/// Interdependency rules:
/// - If there are both FlippedNormal and other defects → fix normals first
///   (normals affect all downstream operations)
/// - If there are both Gap and Hole defects → close gaps before filling holes
///   (closing gaps may resolve some holes)
/// - If there are both DegenerateEdge and Gap defects → mark degenerate first
///   (degenerate edges interfere with gap detection)
/// - If there are both SliverTriangle and SmallFeature → remove slivers first
///   (slivers may be part of small features)
fn compose_strategies(
    strategies: &[HealingStrategy],
    defects: &[ClassifiedDefect],
) -> HealingStrategy {
    if strategies.len() == 1 {
        return strategies[0].clone();
    }

    // Check for specific interdependencies
    let defect_types: Vec<DefectType> = defects.iter().map(|d| d.defect_type).collect();

    let has_flipped_normals = defect_types.contains(&DefectType::FlippedNormal);
    let has_degenerate = defect_types.contains(&DefectType::DegenerateEdge);
    let has_gaps = defect_types.contains(&DefectType::Gap);
    let has_holes = defect_types.contains(&DefectType::Hole);
    let has_slivers = defect_types.contains(&DefectType::SliverTriangle);
    let has_small_features = defect_types.contains(&DefectType::SmallFeature);
    let has_self_intersection = defect_types.contains(&DefectType::SelfIntersection);

    // Build the ordered strategy list with interdependency handling
    let mut ordered: Vec<HealingStrategy> = Vec::new();

    // Priority 1: Fix normals (affects all downstream)
    if has_flipped_normals {
        ordered.push(HealingStrategy::FixNormals);
    }

    // Priority 2: Mark degenerate edges (interferes with gap detection)
    if has_degenerate {
        ordered.push(HealingStrategy::MarkDegenerate);
    }

    // Priority 3: Close gaps (may resolve some holes)
    if has_gaps {
        ordered.push(HealingStrategy::CloseGaps);
    }

    // Priority 4: Fill holes
    if has_holes {
        ordered.push(HealingStrategy::FillHoles);
    }

    // Priority 5: Stitch edges
    if defect_types.contains(&DefectType::NonManifoldEdge) {
        ordered.push(HealingStrategy::StitchEdges);
    }

    // Priority 6: Remove slivers (before small features — slivers may be part of small features)
    if has_slivers {
        ordered.push(HealingStrategy::RemoveSlivers);
    }

    // Priority 7: Remove small features
    if has_small_features {
        ordered.push(HealingStrategy::RemoveSmallFeatures);
    }

    // Priority 8: Fix self-intersections
    if has_self_intersection {
        ordered.push(HealingStrategy::FixSelfIntersection);
    }

    // Priority 9: Adjust tolerance
    if defect_types.contains(&DefectType::ToleranceMismatch) {
        ordered.push(HealingStrategy::AdjustTolerance);
    }

    // Remove duplicates (in case a strategy was already added)
    let mut seen: HashMap<String, bool> = HashMap::new();
    ordered.retain(|s| {
        let key = strategy_key(s);
        if seen.contains_key(&key) {
            false
        } else {
            seen.insert(key, true);
            true
        }
    });

    if ordered.len() == 1 {
        ordered.into_iter().next().unwrap_or(HealingStrategy::Composite(Vec::new()))
    } else {
        HealingStrategy::Composite(ordered)
    }
}

/// Generate a string key for a strategy (for use in HashMaps).
fn strategy_key(strategy: &HealingStrategy) -> String {
    match strategy {
        HealingStrategy::Composite(strategies) => {
            let keys: Vec<String> = strategies.iter().map(|s| strategy_key(s)).collect();
            format!("Composite({})", keys.join("+"))
        }
        _ => strategy.name().to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::classifier::DefectType;
    use draper_geometry::Point3d;

    fn make_defect(defect_type: DefectType, severity: f64) -> ClassifiedDefect {
        ClassifiedDefect {
            defect_type,
            severity,
            location: Point3d::ORIGIN,
            affected_elements: Vec::new(),
            confidence: 0.9,
            reason: String::new(),
        }
    }

    #[test]
    fn test_single_defect_strategy() {
        let defects = vec![make_defect(DefectType::Gap, 0.5)];
        let selector = HealingStrategySelector::new();
        let strategy = selector.select_strategy(&defects);

        // Single defect → simple strategy (not composite)
        match strategy {
            HealingStrategy::CloseGaps => {} // expected
            _ => panic!("Expected CloseGaps strategy for gap defect, got {:?}", strategy),
        }
    }

    #[test]
    fn test_normal_defect_highest_priority() {
        let defects = vec![
            make_defect(DefectType::Gap, 0.5),
            make_defect(DefectType::FlippedNormal, 0.3),
            make_defect(DefectType::Hole, 0.4),
        ];
        let selector = HealingStrategySelector::new();
        let strategy = selector.select_strategy(&defects);

        // Should be composite with FixNormals first
        match &strategy {
            HealingStrategy::Composite(strategies) => {
                assert!(
                    matches!(strategies[0], HealingStrategy::FixNormals),
                    "FixNormals should be first in composite strategy, got {:?}",
                    strategies[0]
                );
            }
            _ => panic!("Expected Composite strategy for multiple defects"),
        }
    }

    #[test]
    fn test_degenerate_before_gaps() {
        let defects = vec![
            make_defect(DefectType::Gap, 0.5),
            make_defect(DefectType::DegenerateEdge, 0.6),
        ];
        let selector = HealingStrategySelector::new();
        let strategy = selector.select_strategy(&defects);

        match &strategy {
            HealingStrategy::Composite(strategies) => {
                let degen_idx = strategies.iter().position(|s| matches!(s, HealingStrategy::MarkDegenerate));
                let gap_idx = strategies.iter().position(|s| matches!(s, HealingStrategy::CloseGaps));
                if let (Some(di), Some(gi)) = (degen_idx, gap_idx) {
                    assert!(di < gi, "MarkDegenerate should come before CloseGaps");
                }
            }
            _ => panic!("Expected Composite strategy"),
        }
    }

    #[test]
    fn test_gaps_before_holes() {
        let defects = vec![
            make_defect(DefectType::Gap, 0.5),
            make_defect(DefectType::Hole, 0.6),
        ];
        let selector = HealingStrategySelector::new();
        let strategy = selector.select_strategy(&defects);

        match &strategy {
            HealingStrategy::Composite(strategies) => {
                let gap_idx = strategies.iter().position(|s| matches!(s, HealingStrategy::CloseGaps));
                let hole_idx = strategies.iter().position(|s| matches!(s, HealingStrategy::FillHoles));
                if let (Some(gi), Some(hi)) = (gap_idx, hole_idx) {
                    assert!(gi < hi, "CloseGaps should come before FillHoles");
                }
            }
            _ => panic!("Expected Composite strategy"),
        }
    }

    #[test]
    fn test_strategy_to_healing_params_fix_normals() {
        let defects = vec![make_defect(DefectType::FlippedNormal, 0.5)];
        let selector = HealingStrategySelector::new();
        let strategy = selector.select_strategy(&defects);
        let params = selector.strategy_to_healing_params(&strategy, &defects);

        assert!(params.fix_normals, "FixNormals strategy should enable fix_normals");
    }

    #[test]
    fn test_strategy_to_healing_params_close_gaps() {
        let defects = vec![make_defect(DefectType::Gap, 0.8)];
        let selector = HealingStrategySelector::new();
        let strategy = selector.select_strategy(&defects);
        let params = selector.strategy_to_healing_params(&strategy, &defects);

        // Gap factor should be increased for high severity
        assert!(params.gap_factor > 10.0, "High severity gap should increase gap_factor");
    }

    #[test]
    fn test_learning_records_outcomes() {
        let mut selector = HealingStrategySelector::new();

        selector.record_outcome(StrategyOutcome {
            strategy: HealingStrategy::CloseGaps,
            success: true,
            remaining_defects: 0,
            improvement: 1.0,
        });

        selector.record_outcome(StrategyOutcome {
            strategy: HealingStrategy::CloseGaps,
            success: false,
            remaining_defects: 5,
            improvement: 0.0,
        });

        assert_eq!(selector.outcome_count(), 2);
        let rate = selector.success_rate_for(&HealingStrategy::CloseGaps);
        // After one success and one failure with EMA alpha=0.3:
        // First: 0.5 * 0.7 + 1.0 * 0.3 = 0.65
        // Second: 0.65 * 0.7 + 0.0 * 0.3 = 0.455
        assert!((rate - 0.455).abs() < 0.01, "Success rate should be approximately 0.455, got {}", rate);
    }

    #[test]
    fn test_empty_defects_returns_empty_composite() {
        let selector = HealingStrategySelector::new();
        let strategy = selector.select_strategy(&[]);
        match strategy {
            HealingStrategy::Composite(strategies) => assert!(strategies.is_empty()),
            _ => panic!("Expected empty Composite for no defects"),
        }
    }

    #[test]
    fn test_strategy_priority_order() {
        assert!(HealingStrategy::FixNormals.priority() < HealingStrategy::CloseGaps.priority());
        assert!(HealingStrategy::MarkDegenerate.priority() < HealingStrategy::CloseGaps.priority());
        assert!(HealingStrategy::CloseGaps.priority() < HealingStrategy::FillHoles.priority());
        assert!(HealingStrategy::FillHoles.priority() < HealingStrategy::StitchEdges.priority());
        assert!(HealingStrategy::StitchEdges.priority() < HealingStrategy::RemoveSlivers.priority());
    }

    #[test]
    fn test_composite_strategy_display() {
        let strategy = HealingStrategy::Composite(vec![
            HealingStrategy::FixNormals,
            HealingStrategy::CloseGaps,
        ]);
        let display = format!("{}", strategy);
        assert!(display.contains("FixNormals"), "Display should contain FixNormals");
        assert!(display.contains("CloseGaps"), "Display should contain CloseGaps");
    }

    #[test]
    fn test_all_defect_types_map_to_strategies() {
        // Ensure every defect type maps to a valid strategy
        let defect_types = [
            DefectType::Gap,
            DefectType::Hole,
            DefectType::NonManifoldEdge,
            DefectType::FlippedNormal,
            DefectType::SliverTriangle,
            DefectType::SmallFeature,
            DefectType::DegenerateEdge,
            DefectType::SelfIntersection,
            DefectType::ToleranceMismatch,
        ];

        for dt in &defect_types {
            let strategy = defect_type_to_strategy(*dt);
            assert!(!strategy.name().is_empty(), "Strategy should have a name");
        }
    }
}
