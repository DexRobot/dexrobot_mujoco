import numpy as np

def adjust_angles(current_angles, previous_angles):
    """
    Adjusts an array of current angles based on previous angles to ensure continuity.

    Parameters:
        current_angles (np.ndarray): The current angles measurement in radians.
        previous_angles (np.ndarray): The previous angles measurement in radians.

    Returns:
        np.ndarray: Adjusted angles in radians that are continuous with the previous measurements.
    """
    # Concatenate the previous and current angles to use them for unwrapping
    concatenated_angles = np.concatenate(([previous_angles[-1]], current_angles))

    # Unwrap the concatenated angles to correct discontinuities
    unwrapped_angles = np.unwrap(concatenated_angles)

    # The first angle is from the previous data, so we start from the second element
    return unwrapped_angles[1:]
