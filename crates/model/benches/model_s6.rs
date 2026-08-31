use criterion::{Criterion, criterion_group, criterion_main};
use model::AircraftModelLoader;
use std::hint::black_box;

const ACRO_ELECTRIC_01_JSON: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../models/acro_electric_01/model.json"
));

fn benchmarks(criterion: &mut Criterion) {
    criterion.bench_function("B15/parse_validate_acro_electric_01", |bencher| {
        bencher.iter(|| {
            black_box(AircraftModelLoader::from_json_str(black_box(ACRO_ELECTRIC_01_JSON)).unwrap())
        });
    });

    let model = AircraftModelLoader::from_json_str(ACRO_ELECTRIC_01_JSON).unwrap();
    criterion.bench_function("B16/model_semantic_fingerprint", |bencher| {
        bencher.iter(|| black_box(black_box(&model).physics_fingerprint()));
    });
}

criterion_group!(model_s6, benchmarks);
criterion_main!(model_s6);
