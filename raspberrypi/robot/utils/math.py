import math



def normalize_angle_deg(angle: float) -> float:
    """Normalize angle to range [-180, 180)."""
    return ((angle + 180) % 360) - 180

def normalize_angle_rad(angle: float) -> float:
    """Normalize angle to range [-pi, pi)."""
    return ((angle + math.pi) % (2 * math.pi)) - math.pi

