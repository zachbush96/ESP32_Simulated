from dataclasses import dataclass
from typing import List

from src.vision.detector import Detection


# Re-exporting Track from simple_tracker to maintain compatibility with other modules imports
# if they were importing Track from here.
# However, the previous Track definition had 'detection: Detection'
# The new Track has 'bbox', 'label'.
# We need to adapt or replace.

# Since I can control the whole codebase, I will replace the implementation
# and aliasing the new class.

from src.tracking.simple_tracker import SimpleTracker, Track

# Alias for backward compatibility if needed, but we should use SimpleTracker
Tracker = SimpleTracker
