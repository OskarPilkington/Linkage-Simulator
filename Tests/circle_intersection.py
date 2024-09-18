import math

def circle_intersection(x1, y1, r1, x2, y2, r2):
    # Calculate the distance between the centers
    d = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    # Check if there is an intersection
    if d > r1 + r2:  # No intersection: the circles are too far apart
        return None
    if d < abs(r1 - r2):  # No intersection: one circle is inside the other
        return None
    if d == 0 and r1 == r2:  # Infinite number of intersection points: circles are identical
        return None

    # Find the point where the line through the circle intersection points crosses the line between the circle centers
    a = (r1**2 - r2**2 + d**2) / (2 * d)
    h = math.sqrt(r1**2 - a**2)

    # Point P2 is the point where the line through the intersection points crosses the line between the centers
    x3 = x1 + a * (x2 - x1) / d
    y3 = y1 + a * (y2 - y1) / d

    # Calculate the offset of the intersection points from point P2
    x4_1 = x3 + h * (y2 - y1) / d
    y4_1 = y3 - h * (x2 - x1) / d

    x4_2 = x3 - h * (y2 - y1) / d
    y4_2 = y3 + h * (x2 - x1) / d

    return (x4_1, y4_1), (x4_2, y4_2)

# Example usage
x1, y1, r1 = 0, 15, 5
x2, y2, r2 = 0, 0, 5

intersection_points = circle_intersection(x1, y1, r1, x2, y2, r2)
if intersection_points:
    print(f"Intersection points: {intersection_points}")
else:
    print("No intersection or circles are coincident.")
