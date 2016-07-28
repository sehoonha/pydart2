#ifndef PYDART2_PYDART2_API_H
#define PYDART2_PYDART2_API_H

////////////////////////////////////////////////////////////////////////////////
// Init Functions
void init(bool verbose=true);
void destroy();

void setVerbose(bool verbose=true);
bool getVerbose();


////////////////////////////////////////////////////////////////////////////////
// World
#define WORLD(funcname) world__##funcname
#define GET_WORLD(wid) Manager::world(wid)

int createWorld(double timestep);
int createWorldFromSkel(const char* const path);
void destroyWorld(int wid);

int WORLD(addSkeleton)(int wid, const char* const path);
int WORLD(getNumSkeletons)(int wid);

void WORLD(reset)(int wid);
void WORLD(step)(int wid);
void WORLD(render)(int wid);

////////////////////////////////////////
// World::Time Functions
void WORLD(setTimeStep)(int wid, double _timeStep);
double WORLD(getTimeStep)(int wid);
void WORLD(setTime)(int wid, double _time);
double WORLD(getTime)(int wid);
int WORLD(getSimFrames)(int wid);
int WORLD(getIndex)(int wid, int _index);

////////////////////////////////////////
// World::Property Functions
void WORLD(setGravity)(int wid, double inv3[3]);
void WORLD(getGravity)(int wid, double outv3[3]);

////////////////////////////////////////////////////////////////////////////////
// Skeleton
#define SKEL(funcname) skeleton__##funcname
#define GET_SKELETON(wid, skid) Manager::skeleton(wid, skid);

void SKEL(render)(int wid, int skid);
void SKEL(renderWithColor)(int wid, int skid, double inv4[4]);
const char* SKEL(getName)(int wid, int skid);
double SKEL(getMass)(int wid, int skid);

////////////////////////////////////////
// Skeleton::Property Functions
bool SKEL(isMobile)(int wid, int skid);
void SKEL(setMobile)(int wid, int skid, bool mobile);
bool SKEL(getSelfCollisionCheck)(int wid, int skid);
void SKEL(setSelfCollisionCheck)(int wid, int skid, int enable);
bool SKEL(getAdjacentBodyCheck)(int wid, int skid);
void SKEL(setAdjacentBodyCheck)(int wid, int skid, int enable);
void SKEL(setRootJointToTransAndEuler)(int wid, int skid);

////////////////////////////////////////
// Skeleton::Structure Information Functions
int SKEL(getNumBodyNodes)(int wid, int skid);
int SKEL(getNumDofs)(int wid, int skid);

////////////////////////////////////////
// Skeleton::Pose Functions
void SKEL(getPositions)(int wid, int skid, double* outv, int ndofs);
void SKEL(setPositions)(int wid, int skid, double* inv, int ndofs);
void SKEL(getVelocities)(int wid, int skid, double* outv, int ndofs);
void SKEL(setVelocities)(int wid, int skid, double* inv, int ndofs);
void SKEL(setForces)(int wid, int skid, double* inv, int ndofs);

////////////////////////////////////////
// Skeleton::Difference Functions
void SKEL(getPositionDifferences)(int wid, int skid,
                                  double* inv1, int indofs1,
                                  double* inv2, int indofs2,
                                  double* outv, int ndofs);
void SKEL(getVelocityDifferences)(int wid, int skid,
                                  double* inv1, int indofs1,
                                  double* inv2, int indofs2,
                                  double* outv, int ndofs);

////////////////////////////////////////
// Skeleton::Limit Functions
void SKEL(getPositionLowerLimits)(int wid, int skid, double* outv, int ndofs);
void SKEL(getPositionUpperLimits)(int wid, int skid, double* outv, int ndofs);
void SKEL(getForceLowerLimits)(int wid, int skid, double* outv, int ndofs);
void SKEL(getForceUpperLimits)(int wid, int skid, double* outv, int ndofs);

////////////////////////////////////////
// Skeleton::Momentum Functions
void SKEL(getCOM)(int wid, int skid, double outv3[3]);
void SKEL(getCOMLinearVelocity)(int wid, int skid, double outv3[3]);
void SKEL(getCOMLinearAcceleration)(int wid, int skid, double outv3[3]);

////////////////////////////////////////
// Skeleton::Lagrangian Functions
void SKEL(getMassMatrix)(int wid, int skid, double* outm, int nrows, int ncols);
void SKEL(getCoriolisAndGravityForces)(int wid, int skid, double* outv, int ndofs);
void SKEL(getConstraintForces)(int wid, int skid, double* outv, int ndofs);

////////////////////////////////////////////////////////////////////////////////
// BodyNode
#define BODY(funcname) bodynode__##funcname
#define GET_BODY(wid, skid, bid) Manager::skeleton(wid, skid)->getBodyNode(bid)

const char* BODY(getName)(int wid, int skid, int bid);

////////////////////////////////////////
// BodyNode::Structure Functions
int BODY(getParentBodyNode)(int wid, int skid, int bid);
int BODY(getNumChildBodyNodes)(int wid, int skid, int bid);
int BODY(getChildBodyNode)(int wid, int skid, int bid, int _index);

////////////////////////////////////////
// BodyNode::Property Functions
double BODY(getMass)(int wid, int skid, int bid);
void BODY(getInertia)(int wid, int skid, int bid, double outv33[3][3]);

////////////////////////////////////////////////////////////////////////////////
// DegreeOfFreedom
#define DOF(funcname) dof__##funcname
#define GET_DOF(wid, skid, dofid) Manager::skeleton(wid, skid)->getDof(dofid)

const char* DOF(getName)(int wid, int skid, int dofid);


#endif // #ifndef PYDART2_PYDART2_API_H
