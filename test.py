import numpy as np

# Example: translate a set of 3D points by a vector
points = np.array([[1, 2, 3],
                   [4, 5, 6],
                   [7, 8, 9]])

translation_vector = [10, 20, 30]

# Apply translation
points += translation_vector

print(points)