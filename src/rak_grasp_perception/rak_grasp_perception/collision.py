"""ROI-partitioned occupancy boxes for the software planning scene.

Grid and ROI share the depth optical frame. Clip cells at the ROI boundary so
independently occupied target/obstacle cells cannot invent overlapping solids.
This is a visible-surface proxy, not an object mesh or contact dynamics model.
"""
import numpy as np

from .core import roi_mask


def occupied_boxes(points, roi, target, voxel=0.02, limit=4000):
    """Return optical-frame (center, dimensions) pairs, retaining both partitions.

One micrometre per face separates coincident numerical boundaries; it is not
physical clearance. All observed points remain covered within this tolerance.
ROI membership is assumed to be the operator's target segmentation.
"""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    roi_mask(points, roi)  # Validate the same ROI contract used for membership.
    lower, upper = np.asarray(roi['min']), np.asarray(roi['max'])
    cells = np.unique(np.floor(points / voxel).astype(np.int64), axis=0)
    if len(cells) > limit:
        raise ValueError('scene exceeds 4000 occupied voxels; reduce capture workspace')
    boxes = []
    for cell in cells:
        lo, hi = cell*voxel, (cell+1)*voxel
        cut_lo, cut_hi = np.maximum(lo, lower), np.minimum(hi, upper)
        if np.any(cut_hi <= cut_lo):
            fragments = [] if target else [(lo, hi)]
        elif target:
            fragments = [(cut_lo, cut_hi)]
        else:
            # Disjoint six-slab complement of the ROI intersection. Never
            # remove a mixed cell wholesale: its obstacle portion is retained.
            fragments = []
            lo, hi = lo.copy(), hi.copy()
            for axis in range(3):
                if lo[axis] < cut_lo[axis]:
                    end = hi.copy(); end[axis] = cut_lo[axis]
                    fragments.append((lo.copy(), end))
                    lo[axis] = cut_lo[axis]
                if hi[axis] > cut_hi[axis]:
                    start = lo.copy(); start[axis] = cut_hi[axis]
                    fragments.append((start, hi.copy()))
                    hi[axis] = cut_hi[axis]
        for lo, hi in fragments:
            size = hi-lo
            # Keep thin fragments too; never shrink by more than 10% per face.
            inset = np.minimum(1e-6, size*0.1)
            boxes.append(((lo+hi)*0.5, size-2*inset))
            if len(boxes) > limit:
                raise ValueError('scene exceeds 4000 collision boxes; reduce capture workspace')
    return boxes
