import cv2
import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional


@dataclass
class DetectedObject:
    """Represents a detected object with its bounding box and metadata."""
    object_type: str  # "goal_yellow", "goal_blue", "ball", etc.
    x: int            # Top-left x coordinate
    y: int            # Top-left y coordinate
    width: int        # Bounding box width
    height: int       # Bounding box height
    confidence: float # 0.0 to 1.0, optional confidence metric
    color: Tuple[int, int, int] = (255, 255, 255)  # BGR color for drawing


class DetectionVisualizer:
    def __init__(self):
        self.object_types: Dict[str, Dict] = {}

    def register_object_type(
        self,
        type_name: str,
        color: Tuple[int, int, int],
        label: str | None = None,
        thickness: int = 2
    ) -> None:
        """
        Register a new detection object type.

        Args:
            type_name: Unique identifier for the object type (e.g., "goal_yellow")
            color: RGB color tuple
            label: Human-readable label for the object (defaults to type_name)
            thickness: Line thickness for drawing rectangles
        """
        if label is None:
            label = type_name

        color_bgr = (color[2], color[1], color[0])

        self.object_types[type_name] = {
            "color": color_bgr,
            "label": label,
            "thickness": thickness,
        }

    def draw_detections(
        self,
        frame: np.ndarray,
        detections: List[DetectedObject],
        draw_labels: bool = True,
        alpha: float = 1.0
    ) -> np.ndarray:
        """
        Draw detection rectangles on the frame.

        Args:
            frame: Input frame (BGR or RGB, will be modified in-place)
            detections: List of DetectedObject instances to draw
            draw_labels: Whether to draw text labels on rectangles
            alpha: Opacity of rectangles (0.0 to 1.0). 1.0 = fully opaque

        Returns:
            Modified frame with rectangles drawn
        """
        if not detections or frame is None:
            return frame

        frame = frame.copy()
        
        for detection in detections:
            if detection.object_type not in self.object_types:
                continue

            obj_config = self.object_types[detection.object_type]
            # Use detection's color if specified, otherwise use registered type color
            color = tuple(detection.color) if detection.color != (255, 255, 255) else obj_config["color"]
            thickness = obj_config["thickness"]
            label = obj_config["label"]

            x1, y1 = detection.x, detection.y
            x2, y2 = detection.x + detection.width, detection.y + detection.height

            if alpha < 1.0:
                overlay = frame.copy()
                cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)
                frame = cv2.addWeighted(overlay, alpha * 0.3, frame, 1 - alpha * 0.3, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
            else:
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)

            if draw_labels:
                self._draw_label(frame, label, x1, y1, color, detection.confidence)

        return frame

    def _draw_label(
        self,
        frame: np.ndarray,
        label: str,
        x: int,
        y: int,
        color: Tuple[int, int, int],
        confidence: float | None = None
    ) -> None:
        font = cv2.FONT_HERSHEY_DUPLEX
        font_scale = 0.5
        thickness = 1
        text_color = color

        if confidence is not None and confidence > 0:
            label = f"{label} ({confidence:.0%})"

        text_size = cv2.getTextSize(label, font, font_scale, thickness)[0]
        text_x = x
        text_y = max(y - 5, 20)

        bg_x1 = text_x - 2
        bg_y1 = text_y - text_size[1] - 2
        bg_x2 = text_x + text_size[0] + 2
        bg_y2 = text_y + 2

        cv2.rectangle(frame, (bg_x1, bg_y1), (bg_x2, bg_y2), (0, 0, 0), -1)
        cv2.putText(
            frame,
            label,
            (text_x, text_y),
            font,
            font_scale,
            text_color,
            thickness
        )

    def get_registered_types(self) -> List[str]:
        """Get list of all registered object types."""
        return list(self.object_types.keys())

    def get_object_type_config(self, type_name: str) -> Optional[Dict]:
        """Get configuration for a specific object type."""
        return self.object_types.get(type_name)


_visualizer_instance = None


def get_visualizer() -> DetectionVisualizer:
    """Get or create the global visualizer instance."""
    global _visualizer_instance
    if _visualizer_instance is None:
        _visualizer_instance = DetectionVisualizer()
    return _visualizer_instance


def register_detection_type(
    type_name: str,
    color: Tuple[int, int, int],
    label: str | None = None,
    thickness: int = 2
) -> None:
    """
    Convenience function to register a new detection type globally.

    Example:
        register_detection_type("obstacle", color=(128, 128, 128), label="Obstacle")
    """
    visualizer = get_visualizer()
    visualizer.register_object_type(type_name, color, label, thickness)


def draw_detections_on_frame(
    frame: np.ndarray,
    detections: List[DetectedObject],
    draw_labels: bool = True,
    alpha: float = 1.0
) -> np.ndarray:
    """
    Convenience function to draw detections on a frame using global visualizer.

    Example:
        frame = draw_detections_on_frame(frame, [DetectedObject(...)])
    """
    visualizer = get_visualizer()
    return visualizer.draw_detections(frame, detections, draw_labels, alpha)

