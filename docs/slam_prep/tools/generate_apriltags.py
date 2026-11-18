"""Generate printable AprilTag 36h11 tags for the fixed venue.

Why 36h11 / 0.16 m: docs/design/vision_nav_v1.md §3.1c — 0.16 m gives ~64 px at
2 m with fx≈800, and 36h11 is the apt apriltag_ros default family.

Print at 100% / 300 DPI. Each PNG contains a white quiet zone, a label, and a
100 mm scale-check bar. Measure the printed black edge with calipers and update
`src/rak_bringup/config/apriltag.yaml` if it is not exactly 0.16 m.

Output:
  slam_prep/tags/tag36h11_id0.png ... id9.png  (300 DPI, print each at 100%)
  slam_prep/tags/layout_template.json
"""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "tags"
DPI = 300
EDGE_M = 0.16
EDGE_PX = int(round(EDGE_M / 0.0254 * DPI))  # metres -> inches -> pixels
QUIET_PX = int(round(0.02 / 0.0254 * DPI))   # 20 mm quiet zone
LABEL_PX = 260
SCALE_BAR_M = 0.10
SCALE_BAR_PX = int(round(SCALE_BAR_M / 0.0254 * DPI))
TAG_IDS = list(range(10))


def make_page(tag_id: int) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.generateImageMarker(dictionary, tag_id, EDGE_PX)
    marker = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

    width = EDGE_PX + 2 * QUIET_PX
    # the bottom area only needs the scale bar thickness + caption, not its length;
    # keeping the page <= A4 height means the tag prints at 100% without scaling.
    height = LABEL_PX + EDGE_PX + 2 * QUIET_PX + 220
    page = np.full((height, width, 3), 255, dtype=np.uint8)

    y0 = LABEL_PX + QUIET_PX
    x0 = QUIET_PX
    page[y0:y0 + EDGE_PX, x0:x0 + EDGE_PX] = marker

    label = f"APRILTAG 36h11  ID {tag_id}  edge {EDGE_M*1000:.0f} mm  print 100% / {DPI} DPI"
    cv2.putText(page, label, (QUIET_PX, 80), cv2.FONT_HERSHEY_SIMPLEX,
                1.2, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(page, "measure the black edge after printing and update apriltag.yaml",
                (QUIET_PX, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2, cv2.LINE_AA)

    bar_y = y0 + EDGE_PX + QUIET_PX + 80
    bar_x0 = QUIET_PX
    bar_x1 = bar_x0 + SCALE_BAR_PX
    cv2.line(page, (bar_x0, bar_y), (bar_x1, bar_y), (0, 0, 0), 4)
    cv2.line(page, (bar_x0, bar_y - 20), (bar_x0, bar_y + 20), (0, 0, 0), 4)
    cv2.line(page, (bar_x1, bar_y - 20), (bar_x1, bar_y + 20), (0, 0, 0), 4)
    cv2.putText(page, "100 mm check", (bar_x0, bar_y + 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2, cv2.LINE_AA)
    return page


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    for tag_id in TAG_IDS:
        page = make_page(tag_id)
        rgb = cv2.cvtColor(page, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(rgb)
        image.save(OUT / f"tag36h11_id{tag_id}.png", dpi=(DPI, DPI))

    layout = {
        "family": "36h11",
        "edge_m": EDGE_M,
        "print_dpi": DPI,
        "scale_bar_m": SCALE_BAR_M,
        "tag_ids": TAG_IDS,
        "positions": {str(tag_id): None for tag_id in TAG_IDS},
        "position_format": "tag_id -> [x_m, y_m, yaw_rad] in the fixed-venue frame",
        "note": "Fill positions after placing tags at corners/junctions; measure the printed black edge and update apriltag.yaml size if needed.",
        "detection_topic": "/apriltag/detections",
    }
    (OUT / "layout_template.json").write_text(
        json.dumps(layout, indent=2) + "\n", encoding="utf-8"
    )
    print("wrote", OUT)


if __name__ == "__main__":
    main()
