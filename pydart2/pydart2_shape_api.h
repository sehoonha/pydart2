#ifndef PYDART2_SHAPE_API_H
#define PYDART2_SHAPE_API_H

////////////////////////////////////////////////////////////////////////////////
// ShapeNode and Shape
#define SHAPENODE(funcname) shapenode__##funcname
#define GET_SHAPENODE(wid, skid, bid, sid) (Manager::skeleton(wid, skid)->getBodyNode(bid)->getShapeNodes()[sid]);
#define SHAPE(funcname) shape__##funcname
#define GET_SHAPE(wid, skid, bid, sid) (Manager::skeleton(wid, skid)->getBodyNode(bid)->getShapeNodes()[sid])->getShape().get();

////////////////////////////////////////
// ShapeNode Functions
void SHAPENODE(getOffset)(int wid, int skid, int bid, int sid, double outv3[3]);
void SHAPENODE(setOffset)(int wid, int skid, int bid, int sid, double inv3[3]);
void SHAPENODE(getRelativeTransform)(int wid, int skid, int bid, int sid, double outv44[4][4]);
void SHAPENODE(setRelativeTransform)(int wid, int skid, int bid, int sid, double inv44[4][4]);

bool SHAPENODE(hasVisualAspect)(int wid, int skid, int bid, int sid);
bool SHAPENODE(hasCollisionAspect)(int wid, int skid, int bid, int sid);

void SHAPENODE(getVisualAspectRGBA)(int wid, int skid, int bid, int sid, double outv4[4]);
void SHAPENODE(setVisualAspectRGBA)(int wid, int skid, int bid, int sid, double inv4[4]);

////////////////////////////////////////
// Shape Functions
double SHAPE(getVolume)(int wid, int skid, int bid, int sid);
int SHAPE(getShapeType)(int wid, int skid, int bid, int sid);
// const char* SHAPE(getType)(int wid, int skid, int bid, int sid);
void SHAPE(render)(int wid, int skid, int bid, int sid);
void SHAPE(getBoundingBoxMin)(int wid, int skid, int bid, int sid, double outv3[3]);
void SHAPE(getBoundingBoxMax)(int wid, int skid, int bid, int sid, double outv3[3]);

////////////////////////////////////////
// BoxShape Functions
#define BOX_SHAPE(funcname) box_shape__##funcname
#define GET_BOX_SHAPE(wid, skid, bid, sid) dynamic_cast<dart::dynamics::BoxShape*>((Manager::skeleton(wid, skid)->getBodyNode(bid)->getShapeNodes()[sid])->getShape().get());

void BOX_SHAPE(getSize)(int wid, int skid, int bid, int sid, double outv3[3]);
void BOX_SHAPE(setSize)(int wid, int skid, int bid, int sid, double inv3[3]);

////////////////////////////////////////
// EllipsoidShape Functions
#define ELLIPSOID_SHAPE(funcname) ellipsoid_shape__##funcname
#define GET_ELLIPSOID_SHAPE(wid, skid, bid, sid) dynamic_cast<dart::dynamics::EllipsoidShape*>((Manager::skeleton(wid, skid)->getBodyNode(bid)->getShapeNodes()[sid])->getShape().get());

void ELLIPSOID_SHAPE(getSize)(int wid, int skid, int bid, int sid, double outv3[3]);
void ELLIPSOID_SHAPE(setSize)(int wid, int skid, int bid, int sid, double inv3[3]);

////////////////////////////////////////
// MeshShape Functions
#define MESH_SHAPE(funcname) mesh_shape__##funcname
#define GET_MESH_SHAPE(wid, skid, bid, sid) dynamic_cast<dart::dynamics::MeshShape*>((Manager::skeleton(wid, skid)->getBodyNode(bid)->getShapeNodes()[sid])->getShape().get());

void MESH_SHAPE(getScale)(int wid, int skid, int bid, int sid, double outv3[3]);
const char* MESH_SHAPE(getMeshPath)(int wid, int skid, int bid, int sid);


#endif // #ifndef PYDART2_SHAPE_API_H
