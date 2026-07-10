// Precision harness for the detection algorithm. Not part of the firmware
// build: link it into a host-side test binary together with dot_detection.c
// and recorded frames to measure accuracy without flashing the board.

#include "dot_detection.h"
#include <cmath>
#include <vector>

// Mean distance (px) between the detected center dot and the ground-truth
// centers for a set of recorded frames.
float test_precision(std::vector<camera_fb_t> &fbs,
                     const std::vector<DetectedDot> &centers,
                     BlobResult (*process)(camera_fb_t *fb)) {
    if (fbs.empty())
        return 0.0f;

    float sum = 0.0f;
    for (size_t i = 0; i < fbs.size(); i++) {
        BlobResult r = process(&fbs[i]);
        sum += std::hypot(r.dots[1].x - centers[i].x,
                          r.dots[1].y - centers[i].y);
    }

    return sum / fbs.size();
}
