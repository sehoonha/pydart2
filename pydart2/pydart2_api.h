#ifndef PYDART2_PYDART2_API_H
#define PYDART2_PYDART2_API_H

#include <vector>

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
int SKEL(getNumJoints)(int wid, int skid);
int SKEL(getNumDofs)(int wid, int skid);
int SKEL(getNumMarkers)(int wid, int skid);

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
// BodyNode::Joint and Dof Functions
int BODY(getParentJoint)(int wid, int skid, int bid);
int BODY(getNumChildJoints)(int wid, int skid, int bid);
int BODY(getChildJoint)(int wid, int skid, int bid, int _index);
int BODY(getNumDependentDofs)(int wid, int skid, int bid);
int BODY(getDependentDof)(int wid, int skid, int bid, int _index);

////////////////////////////////////////
// BodyNode::Shape
int BODY(getNumShapeNodes)(int wid, int skid, int bid);

////////////////////////////////////////
// BodyNode::Index Functions
int BODY(getIndexInSkeleton)(int wid, int skid, int bid);
int BODY(getIndexInTree)(int wid, int skid, int bid);
int BODY(getTreeIndex)(int wid, int skid, int bid);

////////////////////////////////////////
// BodyNode::Property Functions
void BODY(setGravityMode)(int wid, int skid, int bid, bool _gravityMode);
bool BODY(getGravityMode)(int wid, int skid, int bid);
bool BODY(isCollidable)(int wid, int skid, int bid);
void BODY(setCollidable)(int wid, int skid, int bid, bool _isCollidable);

////////////////////////////////////////
// BodyNode::Inertia Functions
double BODY(getMass)(int wid, int skid, int bid);
void BODY(getInertia)(int wid, int skid, int bid, double outv33[3][3]);

////////////////////////////////////////
// BodyNode::Momentum Functions
void BODY(getLocalCOM)(int wid, int skid, int bid, double outv3[3]);
void BODY(getCOM)(int wid, int skid, int bid, double outv3[3]);
void BODY(getCOMLinearVelocity)(int wid, int skid, int bid, double outv3[3]);
void BODY(getCOMSpatialVelocity)(int wid, int skid, int bid, double outv6[6]);
void BODY(getCOMLinearAcceleration)(int wid, int skid, int bid, double outv3[3]);
void BODY(getCOMSpatialAcceleration)(int wid, int skid, int bid, double outv6[6]);

////////////////////////////////////////
// BodyNode::Friction and Restitution Functions
void BODY(setFrictionCoeff)(int wid, int skid, int bid, double _coeff);
double BODY(getFrictionCoeff)(int wid, int skid, int bid);
void BODY(setRestitutionCoeff)(int wid, int skid, int bid, double _coeff);
double BODY(getRestitutionCoeff)(int wid, int skid, int bid);

////////////////////////////////////////
// BodyNode::Transforms
void BODY(getTransform)(int wid, int skid, int bid, double outv44[4][4]);
void BODY(getWorldTransform)(int wid, int skid, int bid, double outv44[4][4]);
void BODY(getRelativeTransform)(int wid, int skid, int bid, double outv44[4][4]);

////////////////////////////////////////
// BodyNode::Ext Force and Torque
void BODY(addExtForce)(int wid, int skid, int bid, double inv3[3], double inv3_2[3], bool _isForceLocal, bool _isOffsetLocal);
void BODY(setExtForce)(int wid, int skid, int bid, double inv3[3], double inv3_2[3], bool _isForceLocal, bool _isOffsetLocal);
void BODY(addExtTorque)(int wid, int skid, int bid, double inv3[3], bool _isLocal);
void BODY(setExtTorque)(int wid, int skid, int bid, double inv3[3], bool _isLocal);

////////////////////////////////////////
// BodyNode::Jacobian Functions
void BODY(getJacobian)(int wid, int skid, int bid, double inv3[3], double* outm, int nrows, int ncols);
void BODY(getLinearJacobian)(int wid, int skid, int bid, double inv3[3], double* outm, int nrows, int ncols);
void BODY(getAngularJacobian)(int wid, int skid, int bid, double* outm, int nrows, int ncols);
void BODY(getWorldJacobian)(int wid, int skid, int bid, double inv3[3], double* outm, int nrows, int ncols);
void BODY(getLinearJacobianDeriv)(int wid, int skid, int bid, double inv3[3], double* outm, int nrows, int ncols);
void BODY(getAngularJacobianDeriv)(int wid, int skid, int bid, double* outm, int nrows, int ncols);

////////////////////////////////////////////////////////////////////////////////
// DegreeOfFreedom
#define DOF(funcname) dof__##funcname
#define GET_DOF(wid, skid, dofid) Manager::skeleton(wid, skid)->getDof(dofid)

const char* DOF(getName)(int wid, int skid, int dofid);

////////////////////////////////////////
// Dof::Index Functions
int DOF(getIndexInSkeleton)(int wid, int skid, int dofid);
int DOF(getIndexInTree)(int wid, int skid, int dofid);
int DOF(getIndexInJoint)(int wid, int skid, int dofid);
int DOF(getTreeIndex)(int wid, int skid, int dofid);

////////////////////////////////////////
// Dof::Position Functions
double DOF(getPosition)(int wid, int skid, int dofid);
void DOF(setPosition)(int wid, int skid, int dofid, double _position);
double DOF(getInitialPosition)(int wid, int skid, int dofid);
void DOF(setInitialPosition)(int wid, int skid, int dofid, double _initial);
bool DOF(hasPositionLimit)(int wid, int skid, int dofid);
double DOF(getPositionLowerLimit)(int wid, int skid, int dofid);
void DOF(setPositionLowerLimit)(int wid, int skid, int dofid, double _limit);
double DOF(getPositionUpperLimit)(int wid, int skid, int dofid);
void DOF(setPositionUpperLimit)(int wid, int skid, int dofid, double _limit);

////////////////////////////////////////
// Dof::Velocity Functions
double DOF(getVelocity)(int wid, int skid, int dofid);
void DOF(setVelocity)(int wid, int skid, int dofid, double _velocity);
double DOF(getInitialVelocity)(int wid, int skid, int dofid);
void DOF(setInitialVelocity)(int wid, int skid, int dofid, double _initial);
double DOF(getVelocityLowerLimit)(int wid, int skid, int dofid);
void DOF(setVelocityLowerLimit)(int wid, int skid, int dofid, double _limit);
double DOF(getVelocityUpperLimit)(int wid, int skid, int dofid);
void DOF(setVelocityUpperLimit)(int wid, int skid, int dofid, double _limit);

////////////////////////////////////////
// Dof::Passive Force Functions
double DOF(getSpringStiffness)(int wid, int skid, int dofid);
void DOF(setSpringStiffness)(int wid, int skid, int dofid, double _k);
double DOF(getRestPosition)(int wid, int skid, int dofid);
void DOF(setRestPosition)(int wid, int skid, int dofid, double _q0);
double DOF(getDampingCoefficient)(int wid, int skid, int dofid);
void DOF(setDampingCoefficient)(int wid, int skid, int dofid, double _coeff);
double DOF(getCoulombFriction)(int wid, int skid, int dofid);
void DOF(setCoulombFriction)(int wid, int skid, int dofid, double _friction);

////////////////////////////////////////////////////////////////////////////////
// Joint
#define JOINT(funcname) joint__##funcname
#define GET_JOINT(wid, skid, jid) Manager::skeleton(wid, skid)->getJoint(jid)

////////////////////////////////////////
// Joint::Property Functions
const char* JOINT(getName)(int wid, int skid, int jid);
const char* JOINT(setName)(int wid, int skid, int jid, const char* _name, bool _renameDofs);
bool JOINT(isKinematic)(int wid, int skid, int jid);
bool JOINT(isDynamic)(int wid, int skid, int jid);

////////////////////////////////////////
// Joint::Parent and child bodynode Functions
int JOINT(getParentBodyNode)(int wid, int skid, int jid);
int JOINT(getChildBodyNode)(int wid, int skid, int jid);
void JOINT(setTransformFromParentBodyNode)(int wid, int skid, int jid, double inv44[4][4]);
void JOINT(setTransformFromChildBodyNode)(int wid, int skid, int jid, double inv44[4][4]);
void JOINT(getTransformFromParentBodyNode)(int wid, int skid, int jid, double outv44[4][4]);
void JOINT(getTransformFromChildBodyNode)(int wid, int skid, int jid, double outv44[4][4]);

////////////////////////////////////////
// Joint::Limit Functions
void JOINT(setPositionLimitEnforced)(int wid, int skid, int jid, bool _isPositionLimitEnforced);
bool JOINT(isPositionLimitEnforced)(int wid, int skid, int jid);
bool JOINT(hasPositionLimit)(int wid, int skid, int jid, int _index);
double JOINT(getPositionLowerLimit)(int wid, int skid, int jid, int _index);
void JOINT(setPositionLowerLimit)(int wid, int skid, int jid, int _index, double _position);
double JOINT(getPositionUpperLimit)(int wid, int skid, int jid, int _index);
void JOINT(setPositionUpperLimit)(int wid, int skid, int jid, int _index, double _position);

////////////////////////////////////////
// Joint::Dof Functions
int JOINT(getDof)(int wid, int skid, int jid, int _index);
int JOINT(getNumDofs)(int wid, int skid, int jid);

////////////////////////////////////////
// Joint::Passive Force Functions
double JOINT(getSpringStiffness)(int wid, int skid, int jid, int _index);
void JOINT(setSpringStiffness)(int wid, int skid, int jid, int _index, double _k);
double JOINT(getRestPosition)(int wid, int skid, int jid, int _index);
void JOINT(setRestPosition)(int wid, int skid, int jid, int _index, double _q0);
double JOINT(getDampingCoefficient)(int wid, int skid, int jid, int _index);
void JOINT(setDampingCoefficient)(int wid, int skid, int jid, int _index, double _coeff);
double JOINT(getCoulombFriction)(int wid, int skid, int jid, int _index);
void JOINT(setCoulombFriction)(int wid, int skid, int jid, int _index, double _friction);

////////////////////////////////////////////////////////////////////////////////
// Marker
#define MARKER(funcname) marker__##funcname
#define GET_MARKER(wid, skid, mid) Manager::skeleton(wid, skid)->getMarker(mid)

int MARKER(getBodyNode)(int wid, int skid, int mid);
void MARKER(getLocalPosition)(int wid, int skid, int mid, double outv3[3]);
void MARKER(setLocalPosition)(int wid, int skid, int mid, double inv3[3]);
void MARKER(getWorldPosition)(int wid, int skid, int mid, double outv3[3]);
void MARKER(render)(int wid, int skid, int mid);

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

////////////////////////////////////////
// Shape Functions
double SHAPE(getVolume)(int wid, int skid, int bid, int sid);
int SHAPE(getShapeType)(int wid, int skid, int bid, int sid);
void SHAPE(render)(int wid, int skid, int bid, int sid);
void SHAPE(getBoundingBoxMin)(int wid, int skid, int bid, int sid, double outv3[3]);
void SHAPE(getBoundingBoxMax)(int wid, int skid, int bid, int sid, double outv3[3]);

////////////////////////////////////////////////////////////////////////////////
// Contacts
#define COLLISION_RESULT(funcname) collisionresult__##funcname
#define GET_COLLISION_RESULT(wid) Manager::world(wid)->getConstraintSolver()->getLastCollisionResult();

int COLLISION_RESULT(getNumContacts)(int wid);
void COLLISION_RESULT(getContacts)(int wid, double* outv, int nout);
std::vector<int> COLLISION_RESULT(getCollidingBodyNodes)(int wid);
void COLLISION_RESULT(renderContact)(double inv6[6], double size, double scale);

////////////////////////////////////////////////////////////////////////////////
// Constraints
int addBallJointConstraint(int wid, int skid1, int bid1, int skid2, int bid2,
                           double inv3[3]);
                       

#endif // #ifndef PYDART2_PYDART2_API_H
