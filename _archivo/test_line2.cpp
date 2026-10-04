#include <iostream>
#include <cmath>
#include <algorithm>

using namespace std;

static bool isOnLineSegment(float px, float py, float x1, float y1, float x2, float y2) {
    if (x1 < 0 || y1 < 0 || x2 < 0 || y2 < 0) return false;
    float L2 = pow(x2 - x1, 2) + pow(y2 - y1, 2);
    if (L2 == 0.0f) return false;
    float t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / L2;
    t = max(0.0f, min(1.0f, t));
    float proj_x = x1 + t * (x2 - x1);
    float proj_y = y1 + t * (y2 - y1);
    return sqrt(pow(px - proj_x, 2) + pow(py - proj_y, 2)) <= 0.6f; // Tolerancia reducida
}

int main() {
    float x1 = 14, y1 = 28;
    float x2 = 28, y2 = 38;
    
    for (int y = 14; y < 20; y++) {
        for (int x = 6; x < 16; x++) {
            float cx = x * 2.0f;
            float cy = y * 2.0f;
            if (isOnLineSegment(cx, cy, x1, y1, x2, y2)) {
                cout << "* ";
            } else {
                cout << ". ";
            }
        }
        cout << endl;
    }
    return 0;
}
