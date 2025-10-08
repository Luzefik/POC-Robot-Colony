import cv2
import numpy as np
import math
import itertools


CONFIG = {
    "FOCAL_LENGTH_PX": 1400.0,  # pre adjusted parameter for my laptop (Oleksii)
    "REAL_TOTAL_WIDTH_CM": 10.0,
    # color stuff (red)
    "HSV_RED_LOWER_1": (0, 120, 70),
    "HSV_RED_UPPER_1": (10, 255, 255),
    "HSV_RED_LOWER_2": (170, 120, 70),
    "HSV_RED_UPPER_2": (180, 255, 255),
    "MIN_CONTOUR_AREA": 25,
    "MIN_CIRCULARITY": 0.75,
    "MIN_SOLIDITY": 0.8,
    "COLLINEARITY_TOLERANCE": 3.5,
    "EQUIDISTANT_TOL_RATIO": 0.25,
    "HORIZONTAL_ANGLE_TOL_DEG": 20.0,
}


class RobotTracker:

    def __init__(self, config):
        self.config = config
        self.focal_length_px = config["FOCAL_LENGTH_PX"]
        self.real_width_cm = config["REAL_TOTAL_WIDTH_CM"]

    def _detect_candidates(self, frame):
        """
        Detects all potential red, circular objects in the frame.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(
            hsv, self.config["HSV_RED_LOWER_1"], self.config["HSV_RED_UPPER_1"]
        )
        mask2 = cv2.inRange(
            hsv, self.config["HSV_RED_LOWER_2"], self.config["HSV_RED_UPPER_2"]
        )
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
            if perimeter == 0:
                continue

            circularity = (4 * math.pi * area) / (perimeter * perimeter)

            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            solidity = float(area) / hull_area if hull_area > 0 else 0

            if (
                circularity > self.config["MIN_CIRCULARITY"]
                and solidity > self.config["MIN_SOLIDITY"]
            ):
                M = cv2.moments(cnt)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    candidates.append({"contour": cnt, "center": (cx, cy)})

        return candidates, mask

    def _find_valid_triplet(self, candidates):
        """
        Finds a valid triplet of dots from the candidates
        """
        if len(candidates) < 3:
            return None

        possible_triplets = list(itertools.combinations(candidates, 3))
        valid_triplets = []

        for triplet in possible_triplets:
            centers = [c["center"] for c in triplet]

            centers.sort(key=lambda p: p[0])
            p1, p2, p3 = centers[0], centers[1], centers[2]

            distance_from_line = np.abs(
                np.cross(np.array(p3) - np.array(p1), np.array(p1) - np.array(p2))
            ) / np.linalg.norm(np.array(p3) - np.array(p1))
            if distance_from_line > self.config["COLLINEARITY_TOLERANCE"]:
                continue

            dist1 = math.dist(p1, p2)
            dist2 = math.dist(p2, p3)
            if dist1 == 0 or dist2 == 0:
                continue

            ratio_diff = abs(dist1 - dist2) / ((dist1 + dist2) / 2.0)
            if ratio_diff > self.config["EQUIDISTANT_TOL_RATIO"]:
                continue

            angle_rad = math.atan2(p3[1] - p1[1], p3[0] - p1[0])
            angle_deg = abs(math.degrees(angle_rad))

            if (
                angle_deg > self.config["HORIZONTAL_ANGLE_TOL_DEG"]
                and (180 - angle_deg) > self.config["HORIZONTAL_ANGLE_TOL_DEG"]
            ):
                continue

            pixel_width = math.dist(p1, p3)
            valid_triplets.append({"points": centers, "pixel_width": pixel_width})

        if not valid_triplets:
            return None

        best_triplet = max(valid_triplets, key=lambda t: t["pixel_width"])
        return best_triplet

    def process_frame(self, frame):
        """
        Main processing pipeline for a single frame.
        """
        candidates, mask = self._detect_candidates(frame)
        valid_triplet = self._find_valid_triplet(candidates)

        if valid_triplet is None:
            return None, mask

        pixel_width = valid_triplet["pixel_width"]
        if pixel_width <= 0:
            return None, mask

        distance_cm = (self.real_width_cm * self.focal_length_px) / pixel_width

        points = valid_triplet["points"]
        centroid_x = sum(p[0] for p in points) / 3.0
        frame_center_x = frame.shape[1] / 2.0
        offset_x = centroid_x - frame_center_x

        result = {
            "distance_cm": distance_cm,
            "offset_x": offset_x,
            "points": points,
            "pixel_width": pixel_width,
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
    print("Start of the program")
    #   i used it for calibration once:
    # print("calibrate 'FOCAL_LENGTH_PX' in the CONFIG")

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

            cv2.line(
                frame,
                tuple(map(int, points[0])),
                tuple(map(int, points[2])),
                (255, 255, 0),
                2,
            )

            dist_text = f"Distance: {result['distance_cm']:.1f} cm"
            offset_text = f"Offset X: {result['offset_x']:.1f} px"

            cv2.putText(
                frame,
                dist_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                frame,
                offset_text,
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

            print(
                f"Distance: {result['distance_cm']:.1f} cm, Offset: {result['offset_x']:.1f} px"
            )

        cv2.imshow("Robot Detection", frame)
        cv2.imshow("Red Mask (for Debugging)", mask)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Program terminated.")


if __name__ == "__main__":
    main()
