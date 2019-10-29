#pragma once

#include <string>

typedef struct BBox {
    int x;
    int y;
    int w;
    int h;
    float prob;
    std::string obj_class;
} BBox;

