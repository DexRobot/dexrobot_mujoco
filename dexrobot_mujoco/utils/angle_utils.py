import numpy as np

def adjust_angles(current_angles, previous_angles):
    """
    Adjusts an array of current angles based on previous angles using numpy's unwrap function to ensure continuity.

    Parameters:
        current_angles (np.ndarray): The current angles measurement in radians.
        previous_angles (np.ndarray): The previous angles measurement in radians.

    Returns:
        np.ndarray: Adjusted angles in radians that are continuous with the previous measurements.
    """
    # Concatenate the previous and current angles to handle them together
    concatenated_angles = np.vstack((previous_angles, current_angles))

    # Apply unwrap along the column axis
    unwrapped_angles = np.unwrap(concatenated_angles, axis=0)

    # Return only the second row which corresponds to the adjusted current angles
    return unwrapped_angles[1]
