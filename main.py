import cv2
import numpy as np
import math
import itertools

# --- CONFIGURATION PARAMETERS ---
# It's crucial to separate configuration from the main logic for easier tuning.
CONFIG = {
    # --- Camera and Object Parameters ---
    # IMPORTANT: Calibrate this value for your specific camera.
    # To calibrate, place the robot at a known distance (e.g., 50 cm),
    # run the script, and note the average 'pixel_width' of the marker.
    # Then, calculate: FOCAL_LENGTH_PX = (pixel_width * KNOWN_DISTANCE_CM) / REAL_TOTAL_WIDTH_CM
    "FOCAL_LENGTH_PX": 1400.0,  # PRE-CALIBRATED FOCAL LENGTH (NEEDS YOUR CALIBRATION)

    # Real-world distance between the centers of the two outer dots (in cm).
    # Since adjacent dots are 5cm apart, the total width is 2 * 5cm.
    "REAL_TOTAL_WIDTH_CM": 10.0,

    # --- Color Detection Parameters (HSV Color Space) ---
    # Red color can wrap around the hue spectrum (0-180 in OpenCV).
    # These two ranges define the lower and upper bounds for red.
    "HSV_RED_LOWER_1": (0, 120, 70),
    "HSV_RED_UPPER_1": (10, 255, 255),
    "HSV_RED_LOWER_2": (170, 120, 70),
    "HSV_RED_UPPER_2": (180, 255, 255),

    # --- Candidate Filtering Parameters ---
    # These filters help remove noise and false positives (like a red finger).
    "MIN_CONTOUR_AREA": 25,       # Minimum area of a contour to be considered a dot.
    "MIN_CIRCULARITY": 0.75,      # Filters out non-circular shapes. 1.0 is a perfect circle.
    "MIN_SOLIDITY": 0.8,          # Filters out shapes with indentations. 1.0 is a solid shape.

    # --- Geometric Validation Tolerances ---
    # These tolerances account for real-world imperfections like perspective distortion.
    "COLLINEARITY_TOLERANCE": 3.5, # How much a point can deviate from the line formed by the other two.
    "EQUIDISTANT_TOL_RATIO": 0.25, # Allowed relative difference between segments (e.g., 0.25 = 25%).
    "HORIZONTAL_ANGLE_TOL_DEG": 20.0, # Allowed deviation from a perfectly horizontal line.
}


class RobotTracker:
    """
    A class to encapsulate the entire robot tracking and distance estimation pipeline.
    """
    def __init__(self, config):
        self.config = config
        self.focal_length_px = config["FOCAL_LENGTH_PX"]
        self.real_width_cm = config["REAL_TOTAL_WIDTH_CM"]

    def _detect_candidates(self, frame):
        """
        Stage 1: Detects all potential red, circular objects in the frame.
        Returns a list of candidate dictionaries and the binary mask for debugging.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.config["HSV_RED_LOWER_1"], self.config["HSV_RED_UPPER_1"])
        mask2 = cv2.inRange(hsv, self.config["HSV_RED_LOWER_2"], self.config["HSV_RED_UPPER_2"])
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.config["MIN_CONTOUR_AREA"]:
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0: continue
            
            circularity = (4 * math.pi * area) / (perimeter * perimeter)
            
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            solidity = float(area) / hull_area if hull_area > 0 else 0

            if circularity > self.config["MIN_CIRCULARITY"] and solidity > self.config["MIN_SOLIDITY"]:
                M = cv2.moments(cnt)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    candidates.append({'contour': cnt, 'center': (cx, cy)})
        
        return candidates, mask

    def _find_valid_triplet(self, candidates):
        """
        Stage 2: Finds a valid triplet of dots from the candidates that satisfies
        geometric constraints (collinear, equidistant, horizontal).
        """
        if len(candidates) < 3:
            return None

        possible_triplets = list(itertools.combinations(candidates, 3))
        valid_triplets = []

        for triplet in possible_triplets:
            centers = [c['center'] for c in triplet]
            
            # Sort points by x-coordinate to easily check distances and orientation.
            centers.sort(key=lambda p: p[0])
            p1, p2, p3 = centers[0], centers[1], centers[2]

            # 1. Collinearity Check: Calculate the perpendicular distance of p2 from the line p1-p3
            # This is more intuitive than the cross-product area.
            distance_from_line = np.abs(np.cross(np.array(p3)-np.array(p1), np.array(p1)-np.array(p2))) / np.linalg.norm(np.array(p3)-np.array(p1))
            if distance_from_line > self.config["COLLINEARITY_TOLERANCE"]:
                continue

            # 2. Equidistance Check
            dist1 = math.dist(p1, p2)
            dist2 = math.dist(p2, p3)
            if dist1 == 0 or dist2 == 0: continue
            
            ratio_diff = abs(dist1 - dist2) / ((dist1 + dist2) / 2.0)
            if ratio_diff > self.config["EQUIDISTANT_TOL_RATIO"]:
                continue

            # 3. Horizontal Orientation Check
            angle_rad = math.atan2(p3[1] - p1[1], p3[0] - p1[0])
            angle_deg = abs(math.degrees(angle_rad))
            
            if angle_deg > self.config["HORIZONTAL_ANGLE_TOL_DEG"] and \
               (180 - angle_deg) > self.config["HORIZONTAL_ANGLE_TOL_DEG"]:
                continue

            pixel_width = math.dist(p1, p3)
            valid_triplets.append({'points': centers, 'pixel_width': pixel_width})

        if not valid_triplets:
            return None

        # If multiple valid triplets are found, choose the one with the largest apparent size.
        best_triplet = max(valid_triplets, key=lambda t: t['pixel_width'])
        return best_triplet

    def process_frame(self, frame):
        """
        Main processing pipeline for a single frame.
        """
        candidates, mask = self._detect_candidates(frame)
        valid_triplet = self._find_valid_triplet(candidates)

        if valid_triplet is None:
            return None, mask

        pixel_width = valid_triplet['pixel_width']
        if pixel_width <= 0:
            return None, mask

        distance_cm = (self.real_width_cm * self.focal_length_px) / pixel_width

        points = valid_triplet['points']
        centroid_x = sum(p[0] for p in points) / 3.0
        frame_center_x = frame.shape[1] / 2.0
        offset_x = centroid_x - frame_center_x

        result = {
            "distance_cm": distance_cm,
            "offset_x": offset_x,
            "points": points,
            "pixel_width": pixel_width
        }
        return result, mask


def main():
    """
    Main function to run the robot tracker.
    """
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    tracker = RobotTracker(CONFIG)
    print("Starting robot detection. Press 'q' to quit.")
    print("-" * 30)
    print("IMPORTANT: You may need to calibrate 'FOCAL_LENGTH_PX' in the CONFIG.")
    print("-" * 30)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to grab frame.")
            break

        result, mask = tracker.process_frame(frame)

        if result:
            points = result["points"]
            for point in points:
                cv2.circle(frame, tuple(map(int, point)), 7, (0, 255, 0), -1)
            
            cv2.line(frame, tuple(map(int, points[0])), tuple(map(int, points[2])), (255, 255, 0), 2)

            dist_text = f"Distance: {result['distance_cm']:.1f} cm"
            offset_text = f"Offset X: {result['offset_x']:.1f} px"
            
            cv2.putText(frame, dist_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(frame, offset_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            print(f"Distance: {result['distance_cm']:.1f} cm, Offset: {result['offset_x']:.1f} px")

        cv2.imshow("Robot Detection", frame)
        cv2.imshow("Red Mask (for Debugging)", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Program terminated.")

if __name__ == "__main__":
    main()
