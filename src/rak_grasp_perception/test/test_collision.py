import numpy as np
import pytest

from rak_grasp_perception.collision import occupied_boxes


def test_mixed_roi_voxel_preserves_points_without_solid_overlap():
    roi = {'min': [.006, .004, .003], 'max': [.014, .016, .017]}
    target = np.array([[.01, .01, .01]])
    # All six sides of the same 2 cm cell contain observed obstacles.
    obstacles = np.array([[.002, .01, .01], [.018, .01, .01],
                          [.01, .002, .01], [.01, .018, .01],
                          [.01, .01, .001], [.01, .01, .019]])
    a = occupied_boxes(target, roi, True)
    b = occupied_boxes(obstacles, roi, False)
    assert len(a) == 1 and len(b) == 6
    for center_a, size_a in a:
        for center_b, size_b in b:
            assert np.any(abs(center_a-center_b) > (size_a+size_b)*.5)
    for points, boxes in ((target, a), (obstacles, b)):
        for point in points:
            assert any(np.all(abs(point-center) <= size*.5+1e-6)
                       for center, size in boxes)


def test_collision_proxy_empty_and_bounded():
    roi = {'min': [0, 0, 0], 'max': [.01, .01, .01]}
    assert occupied_boxes([], roi, True) == []
    with pytest.raises(ValueError, match='voxels'):
        occupied_boxes([[.1, 0, 0], [.2, 0, 0]], roi, False, limit=1)
    with pytest.raises(ValueError, match='boxes'):
        occupied_boxes([[0, 0, 0]], roi, False, limit=1)
