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

#endif // #ifndef PYDART2_SHAPE_API_H
