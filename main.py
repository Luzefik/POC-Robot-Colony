# Програма виявляє три точки в реальному часі використовуючи камеру вашого пристрою (ноутбук в даному випадку)
# Повертає 3 параметри, координати точок, відстань до машини в сантиметрах та зміщення від центру кадру
# поки (обʼєктивно) код трохи захардкоджений, але він +- працює


import cv2
import numpy as np
import math

REAL_MARKER_CM = 5.0
KNOWN_DIST_CM = 50.0
FOCAL_PX = None 

def detect_three_red_circles(img, min_area=50, max_area=5000, circularity_thresh=0.7):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower1 = np.array([0, 100, 100])
    upper1 = np.array([10, 255, 255])
    lower2 = np.array([160, 100, 100])
    upper2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv, lower1, upper1)
    mask2 = cv2.inRange(hsv, lower2, upper2)
    mask = cv2.bitwise_or(mask1, mask2)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area or area > max_area:
            continue
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        radius = float(radius)
        if radius <= 0:
            continue
        circularity = area / (math.pi * radius * radius)
        if circularity < circularity_thresh:
            continue
        candidates.append({'center': (x, y), 'radius': radius,
                          'area': area, 'contour': cnt})

    if len(candidates) < 3:
        return None, mask

    candidates = sorted(candidates, key=lambda c: c['area'], reverse=True)
    best3 = candidates[:3]
    return best3, mask


def pairwise_distances(centers):
    d = []
    for i in range(3):
        for j in range(i+1, 3):
            d.append(math.dist(centers[i], centers[j]))
    return d

def centroid(centers):
    xs = [c[0] for c in centers]
    ys = [c[1] for c in centers]
    return (sum(xs)/3.0, sum(ys)/3.0)


cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("камера не відкривається")
    exit()

print("Старт")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Не вдалося зчитати кадр")
        break

    best3, mask = detect_three_red_circles(frame)

    if best3 is None:
        print("Не знайшов три відповідні кола")
    else:
        centers = [c['center'] for c in best3]
        for c in best3:
            x, y = int(c['center'][0]), int(c['center'][1])
            r = int(c['radius'])
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 3, (255, 0, 0), -1)

        dists_px = pairwise_distances(centers)
        mean_px = sum(dists_px)/len(dists_px)

        if FOCAL_PX is None:
            FOCAL_PX = (mean_px * KNOWN_DIST_CM) / REAL_MARKER_CM
            print(f"Калібрування focal_px = {FOCAL_PX:.1f}")
            distance_cm = KNOWN_DIST_CM
        else:

            distance_cm = (REAL_MARKER_CM * FOCAL_PX) / mean_px

        cx, cy = centroid(centers)
        dx = cx - frame.shape[1]/2
        dy = cy - frame.shape[0]/2

        distance_cm = distance_cm/2
        print(f"Координати точок: {[ (int(c[0]), int(c[1])) for c in centers ]}")
        print(f"Середня відстань між точками (px): {mean_px:.1f}")
        print(f"Відстань до машини (см): {distance_cm:.1f}")
        print(f"Центр трикутника: ({cx:.1f}, {cy:.1f}), dx={dx:.1f}, dy={dy:.1f}")

    cv2.imshow("mask", mask)
    cv2.imshow("result", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
