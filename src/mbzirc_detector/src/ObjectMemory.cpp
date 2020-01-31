#include <algorithm>
#include <climits>
#include <cstdlib>
#include <iostream>

#include "object_memory/ObjectMemory.hpp"


int clamp (int n, int lower, int upper)
{
    return std::max(lower, std::min(n, upper));
}

/* Compute a heuristic distance between two bounding bboxes */
unsigned int ObjectMemory::computeBBoxDistance(const BBox& b1, const BBox& b2,
    const unsigned res_w, const unsigned res_h)
{
    const int x1_mid = b1.x + b1.w/2;
    const int x2_mid = b2.x + b2.w/2;
    const int y1_mid = b1.y + b1.h/2;
    const int y2_mid = b2.y + b2.h/2;
    const int r1 = std::max(b1.w, b1.h);
    const int r2 = std::max(b2.w, b2.h);
    const int r_min = std::min(r1, r2);
    const double r_min_perc = (double)r_min / res_w;
    const double off_x_perc = (double)std::abs(x1_mid - x2_mid) / res_w;
    const double off_y_perc = (double)std::abs(y1_mid - y2_mid) / res_h;
    const double limit_off_xy = (r_min_perc <= 0.05) ? 0.095 :
                                (r_min_perc >= 0.20) ? 0.5 :
                                50 - 0.5*(60*r_min_perc - 12)*(60*r_min_perc - 12);
    const double limit_scale_r = (r_min_perc <= 0.02) ? 2.0 :
                                 (r_min_perc >= 0.22) ? 0.5 :
                                 2.0 - 7.5*(r_min_perc - 0.02);
    const double scale_r = (double)std::abs(r1-r2) / (double)r_min;

    //std::cout << "r_min_perc " << r_min_perc << std::endl;
    //std::cout << "offx% " << off_x_perc << "  offy% " << off_y_perc << "  scaleR " << scale_r << std::endl;
    //std::cout << "limxy% " << limit_off_xy << "  limscaleR " << limit_scale_r << std::endl;

    if (scale_r > limit_scale_r)
        return UINT_MAX;
    if (off_x_perc > limit_off_xy)
        return UINT_MAX;
    if (off_y_perc > limit_off_xy)
        return UINT_MAX;

    //std::cout << "BBox distance: " << (unsigned)(100*(off_x_perc + off_y_perc)) << std::endl;
    return (unsigned)(100*(off_x_perc + off_y_perc));
    
}


/* Decrease the counter of all histories */
void ObjectMemory::decreaseAllCounters()
{
    for (auto &h : histories)
        h.counter = clamp(h.counter-dec, min_counter, max_counter);
}


/* Return true if the history counter is 0 or less */
bool counter_expired(const ObjHistory& h)
{
    return (h.counter <= 0);
}


/* Cleanup histories with counter <= 0 */
void ObjectMemory::cleanExpiredHistories()
{
    histories.remove_if(counter_expired);
}


/* Returns an iterator to the closest history w.r.t object b.
 * Returns histories.end() if distance is larger than max value. */
std::list<ObjHistory>::iterator ObjectMemory::findClosestHistory(const BBox& b,
    const unsigned res_w, const unsigned res_h)
{
    unsigned int closest_dist = UINT_MAX;
    std::list<ObjHistory>::iterator closest = histories.end();
    std::list<ObjHistory>::iterator h;

    // For each history
    for (h=histories.begin(); h != histories.end(); h++) {
        // If type matches and minimum distance so far, update closest iterator
        if (b.obj_class.compare(h->bbox.obj_class) == 0) {
            unsigned int hist_dist = computeBBoxDistance(b, h->bbox, res_w, res_h);
            if (hist_dist < closest_dist) {
                closest_dist = hist_dist;
                closest = h;
            }
        }
    }

    // Return closest (if not too far)
    if (closest_dist  < max_dist) {
        return closest;
    } else {
        return histories.end();
    }
}


bool ObjectMemory::addNewHistory(BBox b)
{
    if (histories.size() < max_objects) {
        ObjHistory oh = {(int)inc, b};
        histories.push_back(oh);
        return true;
    } else {
        return false;
    }
}


ObjectMemory::ObjectMemory(unsigned max_objects, unsigned max_dist, unsigned inc, unsigned dec,
    unsigned min_counter, unsigned max_counter, unsigned thr_counter)
{
    this->max_objects = max_objects;
    this->max_dist = max_dist;
    this->inc = inc;
    this->dec = dec;
    this->min_counter = min_counter;
    this->max_counter = max_counter;
    this->thr_counter = thr_counter;
}


ObjectMemory::~ObjectMemory() {}


/* Return a vector containing the bounding boxes from histories */
std::vector<BBox> ObjectMemory::getObjects() {

    std::vector<BBox> ret_bboxes;

    for (const auto& h: histories) {
        if (h.counter >= thr_counter)
            ret_bboxes.push_back(h.bbox);
    }

    return ret_bboxes;
}


BBox ObjectMemory::mobileMeanBBoxes(BBox& new_bbox, BBox& mobmean_bbox)
{
    int d_old, d_new;
    double mu;
    BBox res_bbox;

    d_old = std::max(mobmean_bbox.w, mobmean_bbox.h);
    d_new = std::max(new_bbox.w, new_bbox.h);

    if (d_new > d_old)
        mu = 0.3;
    else
        mu = 0.05;

    res_bbox.w = mu*new_bbox.w + (1-mu)*old_bbox.w;
    res_bbox.h = mu*new_bbox.h + (1-mu)*old_bbox.h;
    res_bbox.x = new_bbox.x + new_bbox.w/2 -res_bbox.w;
    res_bbox.y = new_bbox.y + new_bbox.h/2 -res_bbox.h;
    res_bbox.prob = new_bbox.prob;
    res_bbox.obj_class = new_bbox.obj_class;

    return res_bbox;
}


void ObjectMemory::putObjects(std::vector<BBox> new_bboxes, const unsigned res_w, const unsigned res_h)
{
    std::list<ObjHistory>::iterator closest;
    BBox old_bbox, new_bbox;

    // Decrement the counter of each history
    decreaseAllCounters();

    // For each detected bounding box
    for (const auto &b : new_bboxes) {
        // Find the closest history
        closest = findClosestHistory(b, res_w, res_h);
        if (closest != histories.end()) {
            // if a close history exists
            old_bbox = closest->bbox;
            closest->bbox = mobileMeanBBoxes(new_bbox, old_bbox);
            closest->counter = clamp(closest->counter + inc + dec, min_counter, max_counter);
        } else {
            // if no close history exists, add a new one (if enough space)
            addNewHistory(b);
        }
    }

    // Remove histories whose counter is 0
    cleanExpiredHistories();
}


std::ostream& operator<<(std::ostream& os, const ObjectMemory& om)
{
    os << "Histories: " << om.histories.size() << std::endl;
    for (const auto &h : om.histories) {
        os << "  cnt: " << h.counter << " (x,y,w,h): " 
            << h.bbox.x << ", "  << h.bbox.y << ", "
            << h.bbox.w << ", "  << h.bbox.h << ", "
            << "class: " << h.bbox.obj_class << std::endl;
    }
    return os;
}
