//! Static RV2 render-pass dependency graph.
//!
//! The graph owns metadata and a deterministic schedule only. Physical wgpu
//! resources remain explicitly owned by the renderer, and graph compilation
//! happens once during V2 initialization.

use std::collections::HashSet;
use thiserror::Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum PassId {
    ShadowNear,
    ShadowMid,
    ShadowFar,
    Scene,
    Postprocess,
}

impl PassId {
    pub(crate) const COUNT: usize = 5;

    pub(crate) const fn index(self) -> usize {
        match self {
            Self::ShadowNear => 0,
            Self::ShadowMid => 1,
            Self::ShadowFar => 2,
            Self::Scene => 3,
            Self::Postprocess => 4,
        }
    }

    pub(crate) const fn label(self) -> &'static str {
        match self {
            Self::ShadowNear => "G3E near cascade shadow depth pass",
            Self::ShadowMid => "G3E mid cascade shadow depth pass",
            Self::ShadowFar => "G3E far cascade shadow depth pass",
            Self::Scene => "G1C scene pass",
            Self::Postprocess => "G3B HDR postprocess pass",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct ResourceId(usize);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ResourceClass {
    Persistent,
    ResizeDependent,
    Transient,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ResourceDecl {
    class: ResourceClass,
    imported: bool,
}

#[derive(Debug, Clone)]
struct PassDecl {
    id: PassId,
    reads: Vec<ResourceId>,
    writes: Vec<ResourceId>,
    explicit_dependencies: Vec<PassId>,
}

#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub(crate) enum GraphError {
    #[error("duplicate render pass ID {0:?}")]
    DuplicatePass(PassId),
    #[error("render pass {pass:?} references unknown resource {resource:?}")]
    UnknownResource { pass: PassId, resource: ResourceId },
    #[error("render pass {pass:?} has conflicting access to resource {resource:?}")]
    ConflictingAccess { pass: PassId, resource: ResourceId },
    #[error("resource {resource:?} has multiple writers")]
    MultipleWriters { resource: ResourceId },
    #[error("resource {resource:?} is read but has no writer and is not imported")]
    ReadBeforeWrite { resource: ResourceId },
    #[error("render pass {pass:?} depends on unknown pass {dependency:?}")]
    UnknownPassDependency { pass: PassId, dependency: PassId },
    #[error("render graph contains a dependency cycle")]
    Cycle,
}

#[derive(Debug, Default)]
pub(crate) struct RenderGraphBuilder {
    resources: Vec<ResourceDecl>,
    passes: Vec<PassDecl>,
}

impl RenderGraphBuilder {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    pub(crate) fn add_resource(&mut self, class: ResourceClass, imported: bool) -> ResourceId {
        let id = ResourceId(self.resources.len());
        self.resources.push(ResourceDecl { class, imported });
        id
    }

    pub(crate) fn add_pass(
        &mut self,
        id: PassId,
        reads: &[ResourceId],
        writes: &[ResourceId],
        explicit_dependencies: &[PassId],
    ) -> Result<(), GraphError> {
        if self.passes.iter().any(|pass| pass.id == id) {
            return Err(GraphError::DuplicatePass(id));
        }
        for resource in reads.iter().chain(writes) {
            if resource.0 >= self.resources.len() {
                return Err(GraphError::UnknownResource {
                    pass: id,
                    resource: *resource,
                });
            }
        }
        let mut seen = HashSet::with_capacity(reads.len() + writes.len());
        for resource in reads.iter().chain(writes) {
            if !seen.insert(*resource) {
                return Err(GraphError::ConflictingAccess {
                    pass: id,
                    resource: *resource,
                });
            }
        }
        self.passes.push(PassDecl {
            id,
            reads: reads.to_vec(),
            writes: writes.to_vec(),
            explicit_dependencies: explicit_dependencies.to_vec(),
        });
        Ok(())
    }

    pub(crate) fn compile(self) -> Result<CompiledGraph, GraphError> {
        let pass_count = self.passes.len();
        let mut writer_by_resource = vec![None; self.resources.len()];
        for (pass_index, pass) in self.passes.iter().enumerate() {
            for resource in &pass.writes {
                if writer_by_resource[resource.0].replace(pass_index).is_some() {
                    return Err(GraphError::MultipleWriters {
                        resource: *resource,
                    });
                }
            }
        }

        let mut edges = vec![vec![false; pass_count]; pass_count];
        for (reader_index, pass) in self.passes.iter().enumerate() {
            for resource in &pass.reads {
                match writer_by_resource[resource.0] {
                    Some(writer_index) => edges[writer_index][reader_index] = true,
                    None if self.resources[resource.0].imported => {}
                    None => {
                        return Err(GraphError::ReadBeforeWrite {
                            resource: *resource,
                        });
                    }
                }
            }
            for dependency in &pass.explicit_dependencies {
                let dependency_index = self
                    .passes
                    .iter()
                    .position(|candidate| candidate.id == *dependency)
                    .ok_or(GraphError::UnknownPassDependency {
                        pass: pass.id,
                        dependency: *dependency,
                    })?;
                edges[dependency_index][reader_index] = true;
            }
        }

        let mut indegree = vec![0usize; pass_count];
        for row in &edges {
            for (target, present) in row.iter().enumerate() {
                if *present {
                    indegree[target] += 1;
                }
            }
        }
        let mut emitted = vec![false; pass_count];
        let mut execution_order = Vec::with_capacity(pass_count);
        for _ in 0..pass_count {
            let next = (0..pass_count).find(|index| !emitted[*index] && indegree[*index] == 0);
            let Some(next) = next else {
                return Err(GraphError::Cycle);
            };
            emitted[next] = true;
            execution_order.push(self.passes[next].id);
            for target in 0..pass_count {
                if edges[next][target] {
                    indegree[target] -= 1;
                }
            }
        }

        Ok(CompiledGraph {
            execution_order: execution_order.into_boxed_slice(),
            resources: self.resources.into_boxed_slice(),
        })
    }
}

#[derive(Debug)]
pub(crate) struct CompiledGraph {
    execution_order: Box<[PassId]>,
    resources: Box<[ResourceDecl]>,
}

impl CompiledGraph {
    pub(crate) fn execution_order(&self) -> &[PassId] {
        &self.execution_order
    }

    pub(crate) fn resource_class_counts(&self) -> [usize; 3] {
        let mut counts = [0; 3];
        for resource in &self.resources {
            let index = match resource.class {
                ResourceClass::Persistent => 0,
                ResourceClass::ResizeDependent => 1,
                ResourceClass::Transient => 2,
            };
            counts[index] += 1;
        }
        counts
    }

    #[cfg(test)]
    fn resource_class(&self, resource: ResourceId) -> ResourceClass {
        self.resources[resource.0].class
    }
}

pub(crate) fn build_v2_render_graph() -> Result<CompiledGraph, GraphError> {
    let mut builder = RenderGraphBuilder::new();
    let shadow_near = builder.add_resource(ResourceClass::Persistent, false);
    let shadow_mid = builder.add_resource(ResourceClass::Persistent, false);
    let shadow_far = builder.add_resource(ResourceClass::Persistent, false);
    let hdr_scene = builder.add_resource(ResourceClass::ResizeDependent, false);
    let depth = builder.add_resource(ResourceClass::ResizeDependent, false);
    let surface_output = builder.add_resource(ResourceClass::Transient, false);

    builder.add_pass(PassId::ShadowNear, &[], &[shadow_near], &[])?;
    builder.add_pass(PassId::ShadowMid, &[], &[shadow_mid], &[PassId::ShadowNear])?;
    builder.add_pass(PassId::ShadowFar, &[], &[shadow_far], &[PassId::ShadowMid])?;
    builder.add_pass(
        PassId::Scene,
        &[shadow_near, shadow_mid, shadow_far],
        &[hdr_scene, depth],
        &[PassId::ShadowFar],
    )?;
    builder.add_pass(
        PassId::Postprocess,
        &[hdr_scene],
        &[surface_output],
        &[PassId::Scene],
    )?;
    builder.compile()
}

#[cfg(test)]
mod tests {
    use super::*;

    const EXPECTED_ORDER: [PassId; PassId::COUNT] = [
        PassId::ShadowNear,
        PassId::ShadowMid,
        PassId::ShadowFar,
        PassId::Scene,
        PassId::Postprocess,
    ];

    #[test]
    fn production_graph_has_exact_execution_order() {
        let graph = build_v2_render_graph().unwrap();
        assert_eq!(graph.execution_order(), EXPECTED_ORDER);
    }

    #[test]
    fn independent_passes_keep_declaration_order() {
        let mut builder = RenderGraphBuilder::new();
        let first = builder.add_resource(ResourceClass::Persistent, false);
        let second = builder.add_resource(ResourceClass::Persistent, false);
        builder
            .add_pass(PassId::ShadowMid, &[], &[first], &[])
            .unwrap();
        builder
            .add_pass(PassId::ShadowNear, &[], &[second], &[])
            .unwrap();
        let graph = builder.compile().unwrap();
        assert_eq!(
            graph.execution_order(),
            [PassId::ShadowMid, PassId::ShadowNear]
        );
    }

    #[test]
    fn duplicate_pass_is_rejected() {
        let mut builder = RenderGraphBuilder::new();
        builder.add_pass(PassId::Scene, &[], &[], &[]).unwrap();
        assert_eq!(
            builder.add_pass(PassId::Scene, &[], &[], &[]),
            Err(GraphError::DuplicatePass(PassId::Scene))
        );
    }

    #[test]
    fn unknown_resource_and_conflicting_access_are_rejected() {
        let mut builder = RenderGraphBuilder::new();
        let resource = builder.add_resource(ResourceClass::Transient, false);
        assert!(matches!(
            builder.add_pass(PassId::Scene, &[ResourceId(99)], &[], &[]),
            Err(GraphError::UnknownResource { .. })
        ));
        assert!(matches!(
            builder.add_pass(PassId::Scene, &[resource], &[resource], &[]),
            Err(GraphError::ConflictingAccess { .. })
        ));
    }

    #[test]
    fn non_imported_read_without_writer_is_rejected() {
        let mut builder = RenderGraphBuilder::new();
        let resource = builder.add_resource(ResourceClass::Transient, false);
        builder
            .add_pass(PassId::Scene, &[resource], &[], &[])
            .unwrap();
        assert!(matches!(
            builder.compile(),
            Err(GraphError::ReadBeforeWrite { .. })
        ));
    }

    #[test]
    fn dependency_cycle_is_rejected() {
        let mut builder = RenderGraphBuilder::new();
        builder
            .add_pass(PassId::ShadowNear, &[], &[], &[PassId::ShadowMid])
            .unwrap();
        builder
            .add_pass(PassId::ShadowMid, &[], &[], &[PassId::ShadowNear])
            .unwrap();
        assert!(matches!(builder.compile(), Err(GraphError::Cycle)));
    }

    #[test]
    fn production_resource_classes_are_explicit() {
        let mut builder = RenderGraphBuilder::new();
        let persistent = builder.add_resource(ResourceClass::Persistent, true);
        let resize = builder.add_resource(ResourceClass::ResizeDependent, true);
        let transient = builder.add_resource(ResourceClass::Transient, true);
        let graph = builder.compile().unwrap();
        assert_eq!(graph.resource_class(persistent), ResourceClass::Persistent);
        assert_eq!(graph.resource_class(resize), ResourceClass::ResizeDependent);
        assert_eq!(graph.resource_class(transient), ResourceClass::Transient);
    }
}
