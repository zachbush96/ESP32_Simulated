from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

from src.vision.detector import Detection


@dataclass
class Track:
    id: int
    bbox: tuple  # (x1, y1, x2, y2)
    label: str
    missed_frames: int = 0


class SimpleTracker:
    def __init__(self, max_missed_frames: int = 5, distance_threshold: float = 50.0) -> None:
        self.next_id = 0
        self.tracks: Dict[int, Track] = {}
        self.max_missed_frames = max_missed_frames
        self.distance_threshold = distance_threshold

    def update(self, detections: List[Detection]) -> List[Track]:
        """
        Update tracks with new detections using a simple greedy Euclidean distance matching.
        """
        # List of (track_id, center_x, center_y)
        track_centers = []
        for t_id, track in self.tracks.items():
            cx = (track.bbox[0] + track.bbox[2]) / 2
            cy = (track.bbox[1] + track.bbox[3]) / 2
            track_centers.append((t_id, cx, cy))

        # List of (detection_index, center_x, center_y)
        det_centers = []
        for i, det in enumerate(detections):
            cx = (det.bbox[0] + det.bbox[2]) / 2
            cy = (det.bbox[1] + det.bbox[3]) / 2
            det_centers.append((i, cx, cy))

        matched_track_ids = set()
        matched_det_indices = set()

        # Calculate distances and match greedily
        # A real implementation might use Hungarian algorithm (scipy.optimize.linear_sum_assignment)
        # But for "quick and snappy" and low number of objects, greedy is fine.
        potential_matches = []
        for t_idx, (t_id, t_cx, t_cy) in enumerate(track_centers):
            for d_idx, (d_i, d_cx, d_cy) in enumerate(det_centers):
                dist = np.sqrt((t_cx - d_cx) ** 2 + (t_cy - d_cy) ** 2)
                if dist < self.distance_threshold:
                    potential_matches.append((dist, t_id, d_i))

        # Sort by distance (closest first)
        potential_matches.sort(key=lambda x: x[0])

        for dist, t_id, d_i in potential_matches:
            if t_id in matched_track_ids or d_i in matched_det_indices:
                continue

            # Update track
            det = detections[d_i]
            self.tracks[t_id].bbox = det.bbox
            self.tracks[t_id].label = det.label # Update label just in case
            self.tracks[t_id].missed_frames = 0

            matched_track_ids.add(t_id)
            matched_det_indices.add(d_i)

        # Handle unmatched tracks (increment missed_frames)
        files_to_remove = []
        for t_id in self.tracks:
            if t_id not in matched_track_ids:
                self.tracks[t_id].missed_frames += 1
                if self.tracks[t_id].missed_frames > self.max_missed_frames:
                    files_to_remove.append(t_id)

        for t_id in files_to_remove:
            del self.tracks[t_id]

        # Handle unmatched detections (create new tracks)
        for i, det in enumerate(detections):
            if i not in matched_det_indices:
                new_track = Track(
                    id=self.next_id,
                    bbox=det.bbox,
                    label=det.label
                )
                self.tracks[self.next_id] = new_track
                self.next_id += 1

        return list(self.tracks.values())
