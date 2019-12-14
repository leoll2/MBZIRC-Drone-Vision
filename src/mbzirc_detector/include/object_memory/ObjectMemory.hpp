#pragma once

#include <iostream>
#include <list>
#include <string>
#include <vector>

#include "BBox.hpp"

typedef struct ObjHistory {
    int counter;
    BBox bbox;
} ObjHistory;

class ObjectMemory {
    unsigned max_objects;
    unsigned max_dist;
    unsigned inc, dec;
    unsigned min_counter, max_counter, thr_counter;
    std::list<ObjHistory> histories;

    unsigned int computeBBoxDistance(const BBox& b1, const BBox& b2,
        const unsigned res_w, const unsigned res_h);
    std::list<ObjHistory>::iterator findClosestHistory(const BBox& b,
        const unsigned res_w, const unsigned res_h);
    bool addNewHistory(BBox b);
    void decreaseAllCounters();
    void cleanExpiredHistories();

public:
    ObjectMemory(unsigned max_objects, unsigned max_dist, unsigned inc, unsigned dec,
        unsigned min_counter, unsigned max_counter, unsigned thr_counter);
    ~ObjectMemory();
    std::vector<BBox> getObjects();
    void putObjects(std::vector<BBox>, const unsigned res_w, const unsigned res_h);

    friend std::ostream& operator<<(std::ostream& os, const ObjectMemory& om);
};
