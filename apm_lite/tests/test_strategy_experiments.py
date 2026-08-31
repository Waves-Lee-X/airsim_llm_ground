from runpy import run_path


MODULE = run_path("tools/run_strategy_experiments.py", run_name="strategy_experiments")


def test_strategy_experiment_matrix_is_complete_and_reproducible():
    payload = MODULE["run_matrix"](vehicles=8, duration=30, seeds=[11, 12], workers=2)

    assert payload["matrix_runs"] == 4 * 5 * 2
    assert payload["duplicate_verification_runs"] == payload["matrix_runs"]
    assert payload["all_duplicate_hashes_match"] is True
    assert len(payload["summary"]) == 4 * 5
    assert all(row["runs"] == 2 for row in payload["summary"])
