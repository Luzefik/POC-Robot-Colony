#include <vector>
#include "esp_camera.h"
#include "dot_detection.h"
#include "dots_algo.h"
#include <math.h>

//standard deviation of the distance from the actual central point tot he detected central point, in pixels
float test_precision(std::vector<camera_fb_t> fbs, std::vector<Point> centers, BlobResult (*process_image)(camera_fb_t *fb))
{
    std::vector<float> diff;
    int div = 0;
    for (size_t i = 0; i < fbs.size(); i++) {
        Point detected = process_image(&fbs[i]).blobs[1];
        float d = sqrt(pow(detected.x - centers[i].x, 2) + pow(detected.y - centers[i].y, 2));
        diff.push_back(d);
        div++;
    }
    
    return std::accumulate(diff.begin(), diff.end(), 0.0f) / div;
}