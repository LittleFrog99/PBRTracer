#ifndef UTILITY_KDTREE
#define UTILITY_KDTREE

#include "core/primitive.h"

class KDTree : public Aggregate {
public:
    KDTree();

private:
    const int isectCost, travCost, maxPrims;
    const Float emptyBonus;
    vector<shared_ptr<Primitive>> primitives;
};

#endif // UTILITY_KDTREE
