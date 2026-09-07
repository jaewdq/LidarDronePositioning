"""Regression tests for trajectory-based hover inference."""

from competition_score.compet_score import Sample, infer_marker_hovers


def test_last_discovered_marker_can_complete_its_first_hover():
    """SEARCH must include the complete first visit to the fourth marker."""
    markers = {
        1: (0.0, 0.0),
        2: (10.0, 0.0),
        3: (20.0, 0.0),
        4: (30.0, 0.0),
    }
    samples = []
    time_s = 0.0
    for marker_id in (1, 2, 3, 4):
        x, y = markers[marker_id]
        for _ in range(51):
            samples.append(Sample(
                index=len(samples),
                t=time_s,
                x=x,
                y=y,
                z=2.0,
                speed=0.0,
            ))
            time_s += 0.1

    preliminary = infer_marker_hovers(
        samples, markers, 1.0, 1.0, 0.25, 4.0, 0.5, 0.35
    )
    search_end_index = max(
        result.visit_end_index for result in preliminary.values()
    )
    results = infer_marker_hovers(
        samples, markers, 1.0, 1.0, 0.25, 4.0, 0.5, 0.35,
        max_hover_index=search_end_index,
    )

    assert all(result.valid for result in results.values())
    assert results[4].duration >= 4.0
