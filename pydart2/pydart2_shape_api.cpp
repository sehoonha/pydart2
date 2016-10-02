#include <iostream>
#include <string>
#include <vector>
#include <map>
using std::cout;
using std::cerr;
using std::endl;

// Boost headers
#include <boost/algorithm/string.hpp>

#include "pydart2_manager.h"
#include "pydart2_api.h"
#include "pydart2_shape_api.h"
#include "pydart2_draw.h"

#ifdef DART6_NEW_SHAPE_API
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/ConeShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/MultiSphereShape.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"
#include "dart/dynamics/LineSegmentShape.hpp"
#include "dart/dynamics/Marker.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/collision/CollisionDetector.hpp"
#endif

using namespace pydart;


////////////////////////////////////////////////////////////////////////////////
// ShapeNode and Shape
void SHAPENODE(getOffset)(int wid, int skid, int bid, int sid, double outv3[3]) {
    dart::dynamics::ShapeNode* shapenode = GET_SHAPENODE(wid, skid, bid, sid);
    write(shapenode->getOffset(), outv3);
}


void SHAPENODE(setOffset)(int wid, int skid, int bid, int sid, double inv3[3]) {
    dart::dynamics::ShapeNode* shapenode = GET_SHAPENODE(wid, skid, bid, sid);
    shapenode->setOffset(read(inv3, 3));
}


void SHAPENODE(getRelativeTransform)(int wid, int skid, int bid, int sid, double outv44[4][4]) {
    dart::dynamics::ShapeNode* shapenode = GET_SHAPENODE(wid, skid, bid, sid);
    write_isometry(shapenode->getRelativeTransform(), outv44);
}


void SHAPENODE(setRelativeTransform)(int wid, int skid, int bid, int sid, double inv44[4][4]) {
    dart::dynamics::ShapeNode* shapenode = GET_SHAPENODE(wid, skid, bid, sid);
    shapenode->setRelativeTransform(read_isometry(inv44));
}

bool SHAPENODE(hasVisualAspect)(int wid, int skid, int bid, int sid) {
    dart::dynamics::ShapeNode* shapenode = GET_SHAPENODE(wid, skid, bid, sid);
    return shapenode->has<dart::dynamics::VisualAspect>();
}

bool SHAPENODE(hasCollisionAspect)(int wid, int skid, int bid, int sid) {
    dart::dynamics::ShapeNode* shapenode = GET_SHAPENODE(wid, skid, bid, sid);
    return shapenode->has<dart::dynamics::CollisionAspect>();
}

void SHAPENODE(getVisualAspectRGBA)(int wid, int skid, int bid, int sid, double outv4[4]) {
  dart::dynamics::ShapeNode* shapenode = GET_SHAPENODE(wid, skid, bid, sid);
  auto visualAspect = shapenode->getVisualAspect();
  write(visualAspect->getRGBA(), outv4);

}
void SHAPENODE(setVisualAspectRGBA)(int wid, int skid, int bid,
  int sid, double inv4[4]) {
  dart::dynamics::ShapeNode* shapenode = GET_SHAPENODE(wid, skid, bid, sid);
  auto visualAspect = shapenode->getVisualAspect();
  visualAspect->setRGBA(read(inv4, 4));
}


////////////////////////////////////////
// Shape Functions

double SHAPE(getVolume)(int wid, int skid, int bid, int sid) {
    dart::dynamics::Shape* shape = GET_SHAPE(wid, skid, bid, sid);
    return shape->getVolume();
}

#ifdef DART6_NEW_SHAPE_API
int SHAPE(getShapeType)(int wid, int skid, int bid, int sid) {
    dart::dynamics::Shape* shape = GET_SHAPE(wid, skid, bid, sid);
    using dart::dynamics::Shape;
    using dart::dynamics::SphereShape;
    using dart::dynamics::BoxShape;
    using dart::dynamics::EllipsoidShape;
    using dart::dynamics::CylinderShape;
    using dart::dynamics::CapsuleShape;
    using dart::dynamics::ConeShape;
    using dart::dynamics::PlaneShape;
    using dart::dynamics::MultiSphereShape;
    using dart::dynamics::MeshShape;
    using dart::dynamics::SoftMeshShape;
    using dart::dynamics::LineSegmentShape;

    if (shape->is<BoxShape>()) return 1;
    else if (shape->is<EllipsoidShape>()) return 2;
    else if (shape->is<CylinderShape>()) return 3;
    else if (shape->is<MeshShape>()) return 4;
    else if (shape->is<SoftMeshShape>()) return 5;
    else if (shape->is<LineSegmentShape>()) return 6;
    return -1;
}
#else
int SHAPE(getShapeType)(int wid, int skid, int bid, int sid) {
    dart::dynamics::Shape* shape = GET_SHAPE(wid, skid, bid, sid);
    return (int)shape->getShapeType();
}
#endif

void SHAPE(render)(int wid, int skid, int bid, int sid) {
    dart::dynamics::Shape* shape = GET_SHAPE(wid, skid, bid, sid);
    dart::gui::RenderInterface* ri = Manager::getRI();
    drawShape(ri, shape);
}

void SHAPE(getBoundingBoxMin)(int wid, int skid, int bid, int sid, double outv3[3]) {
    dart::dynamics::Shape* shape = GET_SHAPE(wid, skid, bid, sid);
    write(shape->getBoundingBox().getMin(), outv3);
}

void SHAPE(getBoundingBoxMax)(int wid, int skid, int bid, int sid, double outv3[3]) {
    dart::dynamics::Shape* shape = GET_SHAPE(wid, skid, bid, sid);
    write(shape->getBoundingBox().getMax(), outv3);
}
