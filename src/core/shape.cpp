#include "shape.h"
#include "core/stats.h"

STAT_COUNTER("Scene/Shapes created", nShapesCreated);

Shape::Shape(const Transform *ObjectToWorld, const Transform *WorldToObject, bool revOrient)
    : objectToWorld(ObjectToWorld), worldToObject(WorldToObject), reverseOrientation(revOrient),
    transformSwapsHandedness(objectToWorld->swapsHandedness())
{
    ++nShapesCreated;
}

