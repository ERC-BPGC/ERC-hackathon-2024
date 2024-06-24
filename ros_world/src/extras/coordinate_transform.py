import numpy as np
from typing import List, Tuple

def find_transform(points_old: List[Tuple[float, float]], points_new: List[Tuple[float, float]]) -> np.ndarray:
    """
    Find the transform between two coordinate frames using least squares method.
    
    Args:
    points_old (List[Tuple[float, float]]): List of points in the old coordinate frame.
    points_new (List[Tuple[float, float]]): List of points in the new coordinate frame.
    
    Returns:
    np.ndarray: 3x3 transformation matrix
    """
    if len(points_old) != len(points_new) or len(points_old) < 2:
        raise ValueError("Need at least 2 pairs of corresponding points")
    
    A = np.zeros((2*len(points_old), 6))
    b = np.zeros((2*len(points_old), 1))
    
    for i, ((x_old, y_old), (x_new, y_new)) in enumerate(zip(points_old, points_new)):
        A[2*i] = [x_old, y_old, 1, 0, 0, 0]
        A[2*i+1] = [0, 0, 0, x_old, y_old, 1]
        b[2*i] = x_new
        b[2*i+1] = y_new
    
    x, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    
    transform = np.array([
        [x[0][0], x[1][0], x[2][0]],
        [x[3][0], x[4][0], x[5][0]],
        [0, 0, 1]
    ])
    
    return transform

def apply_transform(transform: np.ndarray, x_old: float, y_old: float) -> Tuple[float, float]:
    """
    Apply the transformation to convert coordinates from old frame to new frame.
    
    Args:
    transform (np.ndarray): 3x3 transformation matrix
    x_old (float): x-coordinate in old frame
    y_old (float): y-coordinate in old frame
    
    Returns:
    Tuple[float, float]: (x_new, y_new) coordinates in new frame
    """
    point_old = np.array([x_old, y_old, 1])
    point_new = np.dot(transform, point_old)
    return (point_new[0], point_new[1])

# Example usage:
if __name__ == "__main__":
    # Example points in old and new coordinate frames
    points_new = [(5.843359, 6.065896), (8.674840, 3.101610), (-5.946200, -5.879850), (-3.069760, -8.558200),(-5.881630,-8.526410),(-12.421200,-21.419900),(-24.246500,-5.305690),(26.660300,1.204140)]
    points_old = [(1068, 392), (1174, 502), (920, 661), (972, 708),(921,706),(808, 929),(603, 650),(1486, 536)]
    
    # Find the transform
    transform = find_transform(points_old, points_new)
    print("Transformation matrix:")
    print(transform)
    
    # Test the transform
    test_point = (603, 650)
    result = apply_transform(transform, *test_point)
    print(f"Original point: {test_point}")
    print(f"Transformed point: {result}")