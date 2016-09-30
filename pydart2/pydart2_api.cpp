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
#include "pydart2_world_api.h"
#include "pydart2_api.h"


#include "pydart2_draw.h"

using namespace pydart;

////////////////////////////////////////////////////////////////////////////////
// Init Functions
void init(bool verbose) {
    setVerbose(verbose);
    if (Manager::getInstance()) {
        Manager::destroy();
    }
    Manager::init();
}

void destroy() {
    Manager::destroy();
}

void setVerbose(bool verbose) {
    Manager::g_verbose = verbose;
}

bool getVerbose() {
    return Manager::g_verbose;
}

////////////////////////////////////////////////////////////////////////////////
// Skeleton
void SKEL(render)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    dart::gui::RenderInterface* ri = Manager::getRI();
    drawSkeleton(ri, skel.get());
}

void SKEL(renderWithColor)(int wid, int skid, double inv4[4]) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    dart::gui::RenderInterface* ri = Manager::getRI();
    Eigen::Vector4d color(inv4);
    // MSG << "color = " << color.transpose() << "\n";
    drawSkeleton(ri, skel.get(), color, false);
}

const char* SKEL(getName)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getName().c_str();
}

double SKEL(getMass)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getMass();
}

////////////////////////////////////////
// Skeleton::Property Functions
bool SKEL(isMobile)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->isMobile();
}

void SKEL(setMobile)(int wid, int skid, bool mobile) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setMobile(mobile);
}

bool SKEL(getSelfCollisionCheck)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getSelfCollisionCheck();
}

void SKEL(setSelfCollisionCheck)(int wid, int skid, int enable) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setSelfCollisionCheck(enable);
}

bool SKEL(getAdjacentBodyCheck)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getAdjacentBodyCheck();
}

void SKEL(setAdjacentBodyCheck)(int wid, int skid, int enable) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setAdjacentBodyCheck(enable);
}

void SKEL(setRootJointToTransAndEuler)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    // change the joint type to euler
    dart::dynamics::BodyNode* oldRoot = skel->getRootBodyNode();
    oldRoot->changeParentJointType<dart::dynamics::EulerJoint>();
    oldRoot->getParentJoint()->setName("root_r");
    // create a new root
    std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> ret =
        skel->createJointAndBodyNodePair
        <dart::dynamics::TranslationalJoint, dart::dynamics::BodyNode>();
    dart::dynamics::Joint* newJoint = ret.first;
    newJoint->setName("root_t");
    dart::dynamics::BodyNode* newBody = ret.second;
    // rearrange the root joints
    oldRoot->moveTo(newBody);
}

////////////////////////////////////////
// Skeleton::Structure Information Functions
int SKEL(getNumBodyNodes)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getNumBodyNodes();
}

int SKEL(getNumJoints)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getNumJoints();
}

int SKEL(getNumDofs)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getNumDofs();
}

int SKEL(getNumMarkers)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getNumMarkers();
}

////////////////////////////////////////
// Skeleton::Pose Functions
void SKEL(getPositions)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getPositions(), outv);
}

void SKEL(setPositions)(int wid, int skid, double* inv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setPositions(read(inv, ndofs));
}

void SKEL(getVelocities)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getVelocities(), outv);
}

void SKEL(setVelocities)(int wid, int skid, double* inv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setVelocities(read(inv, ndofs));
}

void SKEL(getAccelerations)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getAccelerations(), outv);
}

void SKEL(setForces)(int wid, int skid, double* inv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setForces(read(inv, ndofs));
}

////////////////////////////////////////
// Skeleton::Difference Functions
void SKEL(getPositionDifferences)(int wid, int skid,
                                  double* inv1, int indofs1,
                                  double* inv2, int indofs2,
                                  double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    Eigen::VectorXd dq1 = read(inv1, indofs1);
    Eigen::VectorXd dq2 = read(inv2, indofs2);
    Eigen::VectorXd dq_diff = skel->getVelocityDifferences(dq1, dq2);
    write(dq_diff, outv);
}

void SKEL(getVelocityDifferences)(int wid, int skid,
                                  double* inv1, int indofs1,
                                  double* inv2, int indofs2,
                                  double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    Eigen::VectorXd q1 = read(inv1, indofs1);
    Eigen::VectorXd q2 = read(inv2, indofs2);
    Eigen::VectorXd q_diff = skel->getPositionDifferences(q1, q2);
    write(q_diff, outv);
}

////////////////////////////////////////
// Skeleton::Limit Functions
void SKEL(getPositionLowerLimits)(int wid, int skid, double* outv, int ndofs) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getPositionLowerLimits(), outv);
}

void SKEL(getPositionUpperLimits)(int wid, int skid, double* outv, int ndofs) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getPositionUpperLimits(), outv);
}

void SKEL(getForceLowerLimits)(int wid, int skid, double* outv, int ndofs) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getForceLowerLimits(), outv);
}

void SKEL(getForceUpperLimits)(int wid, int skid, double* outv, int ndofs) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getForceUpperLimits(), outv);
}

////////////////////////////////////////
// Skeleton::Momentum Functions
void SKEL(getCOM)(int wid, int skid, double outv3[3]) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getCOM(), outv3);
}

void SKEL(getCOMLinearVelocity)(int wid, int skid, double outv3[3]) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getCOMLinearVelocity(), outv3);
}

void SKEL(getCOMLinearAcceleration)(int wid, int skid, double outv3[3]) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getCOMLinearAcceleration(), outv3);
}

////////////////////////////////////////
// Skeleton::Lagrangian Functions
void SKEL(getMassMatrix)(int wid, int skid, double* outm, int nrows, int ncols) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write_matrix(skel->getMassMatrix(), outm);
}

void SKEL(getCoriolisAndGravityForces)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getCoriolisAndGravityForces(), outv);
}

void SKEL(getConstraintForces)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getConstraintForces(), outv);
}

////////////////////////////////////////////////////////////////////////////////
// BodyNode
const char* BODY(getName)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getName().c_str();
}

////////////////////////////////////////
// BodyNode::Structure Functions
int BODY(getParentBodyNode)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    dart::dynamics::BodyNode* parent = body->getParentBodyNode();
    if (parent == NULL) {
        return -1;
    } else {
        return parent->getIndexInSkeleton();
    }
}

int BODY(getNumChildBodyNodes)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getNumChildBodyNodes();
}

int BODY(getChildBodyNode)(int wid, int skid, int bid, int _index) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    dart::dynamics::BodyNode* child = body->getChildBodyNode(_index);
    if (child == NULL) {
        return -1;
    } else {
        return child->getIndexInSkeleton();
    }
}

////////////////////////////////////////
// BodyNode::Joint and Dof Functions
int BODY(getParentJoint)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    if (body->getParentJoint()) {
        return body->getParentJoint()->getJointIndexInSkeleton();
    } else {
        return -1;
    }
}


int BODY(getNumChildJoints)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getNumChildJoints();
}


int BODY(getChildJoint)(int wid, int skid, int bid, int _index) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getChildJoint(_index)->getJointIndexInSkeleton();
}


int BODY(getNumDependentDofs)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getNumDependentDofs();
}


int BODY(getDependentDof)(int wid, int skid, int bid, int _index) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getDependentDof(_index)->getIndexInSkeleton();
}

////////////////////////////////////////
// BodyNode::Shape
int BODY(getNumShapeNodes)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    // dart::dynamics::ShapePtr shape = body->getVisualizationShape(0);
    return body->getShapeNodes().size();
}

////////////////////////////////////////
// BodyNode::Index Functions
int BODY(getIndexInSkeleton)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getIndexInSkeleton();
}


int BODY(getIndexInTree)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getIndexInTree();
}


int BODY(getTreeIndex)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getTreeIndex();
}



////////////////////////////////////////
// BodyNode::Property Functions
void BODY(setGravityMode)(int wid, int skid, int bid, bool _gravityMode) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    body->setGravityMode(_gravityMode);
}


bool BODY(getGravityMode)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getGravityMode();
}


bool BODY(isCollidable)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->isCollidable();
}


void BODY(setCollidable)(int wid, int skid, int bid, bool _isCollidable) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    body->setCollidable(_isCollidable);
}


////////////////////////////////////////
// BodyNode::Inertia Functions
double BODY(getMass)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getMass();
}

void BODY(getInertia)(int wid, int skid, int bid, double outv33[3][3]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
    body->getMomentOfInertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
    outv33[0][0] = Ixx;    outv33[1][1] = Iyy;    outv33[2][2] = Izz;
    outv33[0][1] = Ixy;    outv33[1][0] = Ixy;
    outv33[0][2] = Ixz;    outv33[2][0] = Ixz;
    outv33[1][2] = Iyz;    outv33[2][1] = Iyz;
}

////////////////////////////////////////
// BodyNode::Momentum Functions
void BODY(getLocalCOM)(int wid, int skid, int bid, double outv3[3]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write(body->getLocalCOM(), outv3);
}


void BODY(getCOM)(int wid, int skid, int bid, double outv3[3]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write(body->getCOM(), outv3);
}


void BODY(getCOMLinearVelocity)(int wid, int skid, int bid, double outv3[3]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write(body->getCOMLinearVelocity(), outv3);
}


void BODY(getCOMSpatialVelocity)(int wid, int skid, int bid, double outv6[6]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write(body->getCOMSpatialVelocity(), outv6);
}


void BODY(getCOMLinearAcceleration)(int wid, int skid, int bid, double outv3[3]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write(body->getCOMLinearAcceleration(), outv3);
}


void BODY(getCOMSpatialAcceleration)(int wid, int skid, int bid, double outv6[6]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write(body->getCOMSpatialAcceleration(), outv6);
}

////////////////////////////////////////
// BodyNode::Friction and Restitution Functions
void BODY(setFrictionCoeff)(int wid, int skid, int bid, double _coeff) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    body->setFrictionCoeff(_coeff);
}


double BODY(getFrictionCoeff)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getFrictionCoeff();
}


void BODY(setRestitutionCoeff)(int wid, int skid, int bid, double _coeff) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    body->setRestitutionCoeff(_coeff);
}


double BODY(getRestitutionCoeff)(int wid, int skid, int bid) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    return body->getRestitutionCoeff();
}


////////////////////////////////////////
// BodyNode::Transforms
void BODY(getTransform)(int wid, int skid, int bid, double outv44[4][4]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write_isometry(body->getTransform(), outv44);
}


void BODY(getWorldTransform)(int wid, int skid, int bid, double outv44[4][4]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write_isometry(body->getWorldTransform(), outv44);
}


void BODY(getRelativeTransform)(int wid, int skid, int bid, double outv44[4][4]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write_isometry(body->getRelativeTransform(), outv44);
}

////////////////////////////////////////
// BodyNode::Ext Force and Torque
void BODY(addExtForce)(int wid, int skid, int bid, double inv3[3], double inv3_2[3], bool _isForceLocal, bool _isOffsetLocal) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    body->addExtForce(read(inv3, 3), read(inv3_2, 3), _isForceLocal, _isOffsetLocal);
}

void BODY(setExtForce)(int wid, int skid, int bid, double inv3[3], double inv3_2[3], bool _isForceLocal, bool _isOffsetLocal) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    body->setExtForce(read(inv3, 3), read(inv3_2, 3), _isForceLocal, _isOffsetLocal);
}


void BODY(addExtTorque)(int wid, int skid, int bid, double inv3[3], bool _isLocal) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    body->addExtTorque(read(inv3, 3), _isLocal);
}


void BODY(setExtTorque)(int wid, int skid, int bid, double inv3[3], bool _isLocal) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    body->setExtTorque(read(inv3, 3), _isLocal);
}

////////////////////////////////////////
// BodyNode::Jacobian Functions
void BODY(getJacobian)(int wid, int skid, int bid, double inv3[3], double* outm, int nrows, int ncols) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write_matrix(body->getJacobian(read(inv3, 3)), outm);
}

void BODY(getLinearJacobian)(int wid, int skid, int bid, double inv3[3], double* outm, int nrows, int ncols) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write_matrix(body->getLinearJacobian(read(inv3, 3)), outm);
}


void BODY(getAngularJacobian)(int wid, int skid, int bid, double* outm, int nrows, int ncols) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write_matrix(body->getAngularJacobian(), outm);
}


void BODY(getWorldJacobian)(int wid, int skid, int bid, double inv3[3], double* outm, int nrows, int ncols) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write_matrix(body->getWorldJacobian(read(inv3, 3)), outm);
}


void BODY(getLinearJacobianDeriv)(int wid, int skid, int bid, double inv3[3], double* outm, int nrows, int ncols) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write_matrix(body->getLinearJacobianDeriv(read(inv3, 3)), outm);
}


void BODY(getAngularJacobianDeriv)(int wid, int skid, int bid, double* outm, int nrows, int ncols) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    write_matrix(body->getAngularJacobianDeriv(), outm);
}


////////////////////////////////////////////////////////////////////////////////
// DegreeOfFreedom
const char* DOF(getName)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getName().c_str();
}

////////////////////////////////////////
// Dof::Index Functions
int DOF(getIndexInSkeleton)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getIndexInSkeleton();
}


int DOF(getIndexInTree)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getIndexInTree();
}


int DOF(getIndexInJoint)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getIndexInJoint();
}


int DOF(getTreeIndex)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getTreeIndex();
}

////////////////////////////////////////
// Dof::Position Functions
double DOF(getPosition)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getPosition();
}


void DOF(setPosition)(int wid, int skid, int dofid, double _position) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setPosition(_position);
}


double DOF(getInitialPosition)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getInitialPosition();
}


void DOF(setInitialPosition)(int wid, int skid, int dofid, double _initial) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setInitialPosition(_initial);
}


bool DOF(hasPositionLimit)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->hasPositionLimit();
}


double DOF(getPositionLowerLimit)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getPositionLowerLimit();
}


void DOF(setPositionLowerLimit)(int wid, int skid, int dofid, double _limit) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setPositionLowerLimit(_limit);
}


double DOF(getPositionUpperLimit)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getPositionUpperLimit();
}


void DOF(setPositionUpperLimit)(int wid, int skid, int dofid, double _limit) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setPositionUpperLimit(_limit);
}

////////////////////////////////////////
// Dof::Velocity Functions
double DOF(getVelocity)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getVelocity();
}


void DOF(setVelocity)(int wid, int skid, int dofid, double _velocity) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setVelocity(_velocity);
}


double DOF(getInitialVelocity)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getInitialVelocity();
}


void DOF(setInitialVelocity)(int wid, int skid, int dofid, double _initial) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setInitialVelocity(_initial);
}


double DOF(getVelocityLowerLimit)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getVelocityLowerLimit();
}


void DOF(setVelocityLowerLimit)(int wid, int skid, int dofid, double _limit) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setVelocityLowerLimit(_limit);
}


double DOF(getVelocityUpperLimit)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getVelocityUpperLimit();
}


void DOF(setVelocityUpperLimit)(int wid, int skid, int dofid, double _limit) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setVelocityUpperLimit(_limit);
}

////////////////////////////////////////
// Dof::Passive Force Functions
double DOF(getSpringStiffness)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getSpringStiffness();
}


void DOF(setSpringStiffness)(int wid, int skid, int dofid, double _k) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setSpringStiffness(_k);
}


double DOF(getRestPosition)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getRestPosition();
}


void DOF(setRestPosition)(int wid, int skid, int dofid, double _q0) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setRestPosition(_q0);
}


double DOF(getDampingCoefficient)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getDampingCoefficient();
}


void DOF(setDampingCoefficient)(int wid, int skid, int dofid, double _coeff) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setDampingCoefficient(_coeff);
}


double DOF(getCoulombFriction)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getCoulombFriction();
}


void DOF(setCoulombFriction)(int wid, int skid, int dofid, double _friction) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    dof->setCoulombFriction(_friction);
}


////////////////////////////////////////
// Dof::Passive Force Functions
double DOF(getConstraintImpulse)(int wid, int skid, int dofid) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->getConstraintImpulse();
}


void DOF(setConstraintImpulse)(int wid, int skid, int dofid, double _impulse) {
    dart::dynamics::DegreeOfFreedom* dof = GET_DOF(wid, skid, dofid);
    return dof->setConstraintImpulse(_impulse);
}


////////////////////////////////////////////////////////////////////////////////
// Joint

////////////////////////////////////////
// Joint::Property Functions
const char* JOINT(getName)(int wid, int skid, int jid) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getName().c_str();
}


const char* JOINT(setName)(int wid, int skid, int jid, const char* _name, bool _renameDofs) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->setName(_name, _renameDofs).c_str();
}


bool JOINT(isKinematic)(int wid, int skid, int jid) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->isKinematic();
}


bool JOINT(isDynamic)(int wid, int skid, int jid) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->isDynamic();
}


const char* JOINT(getType)(int wid, int skid, int jid) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getType().c_str();
}


void JOINT(setActuatorType)(int wid, int skid, int jid, int actuator_type) {
  dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
  joint->setActuatorType(static_cast<dart::dynamics::Joint::ActuatorType>(actuator_type));
}


int JOINT(getActuatorType)(int wid, int skid, int jid) {
  dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
  return (int)joint->getActuatorType();
}

////////////////////////////////////////
// Joint::Parent and child Functions
int JOINT(getParentBodyNode)(int wid, int skid, int jid) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    if (joint->getParentBodyNode()) {
        return joint->getParentBodyNode()->getIndexInSkeleton();
    } else {
        return -1;
    }
}


int JOINT(getChildBodyNode)(int wid, int skid, int jid) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getChildBodyNode()->getIndexInSkeleton();
}


void JOINT(setTransformFromParentBodyNode)(int wid, int skid, int jid, double inv44[4][4]) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    joint->setTransformFromParentBodyNode(read_isometry(inv44));
}


void JOINT(setTransformFromChildBodyNode)(int wid, int skid, int jid, double inv44[4][4]) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    joint->setTransformFromChildBodyNode(read_isometry(inv44));
}


void JOINT(getTransformFromParentBodyNode)(int wid, int skid, int jid, double outv44[4][4]) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    write_isometry(joint->getTransformFromParentBodyNode(), outv44);
}


void JOINT(getTransformFromChildBodyNode)(int wid, int skid, int jid, double outv44[4][4]) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    write_isometry(joint->getTransformFromChildBodyNode(), outv44);
}

////////////////////////////////////////
// Joint::Limit Functions
void JOINT(setPositionLimitEnforced)(int wid, int skid, int jid, bool _isPositionLimitEnforced) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    joint->setPositionLimitEnforced(_isPositionLimitEnforced);
}


bool JOINT(isPositionLimitEnforced)(int wid, int skid, int jid) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->isPositionLimitEnforced();
}


bool JOINT(hasPositionLimit)(int wid, int skid, int jid, int _index) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->hasPositionLimit(_index);
}


double JOINT(getPositionLowerLimit)(int wid, int skid, int jid, int _index) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getPositionLowerLimit(_index);
}


void JOINT(setPositionLowerLimit)(int wid, int skid, int jid, int _index, double _position) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    joint->setPositionLowerLimit(_index, _position);
}


double JOINT(getPositionUpperLimit)(int wid, int skid, int jid, int _index) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getPositionUpperLimit(_index);
}


void JOINT(setPositionUpperLimit)(int wid, int skid, int jid, int _index, double _position) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    joint->setPositionUpperLimit(_index, _position);
}

////////////////////////////////////////
// Joint::Dof Functions
int JOINT(getDof)(int wid, int skid, int jid, int _index) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getDof(_index)->getIndexInSkeleton();
}


int JOINT(getNumDofs)(int wid, int skid, int jid) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getNumDofs();
}

////////////////////////////////////////
// Joint::Passive Force Functions
double JOINT(getSpringStiffness)(int wid, int skid, int jid, int _index) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getSpringStiffness(_index);
}


void JOINT(setSpringStiffness)(int wid, int skid, int jid, int _index, double _k) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    joint->setSpringStiffness(_index, _k);
}


double JOINT(getRestPosition)(int wid, int skid, int jid, int _index) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getRestPosition(_index);
}


void JOINT(setRestPosition)(int wid, int skid, int jid, int _index, double _q0) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    joint->setRestPosition(_index, _q0);
}


double JOINT(getDampingCoefficient)(int wid, int skid, int jid, int _index) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getDampingCoefficient(_index);
}


void JOINT(setDampingCoefficient)(int wid, int skid, int jid, int _index, double _coeff) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    joint->setDampingCoefficient(_index, _coeff);
}


double JOINT(getCoulombFriction)(int wid, int skid, int jid, int _index) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getCoulombFriction(_index);
}


void JOINT(setCoulombFriction)(int wid, int skid, int jid, int _index, double _friction) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    joint->setCoulombFriction(_index, _friction);
}


////////////////////////////////////////
// Joint::REVOLUTE_JOINT Functions
void REVOLUTE_JOINT(getAxis)(int wid, int skid, int jid, double outv3[3]) {
    dart::dynamics::RevoluteJoint* joint = GET_REVOLUTE_JOINT(wid, skid, jid);
    if (joint == NULL) {
      ERR << " [pydart2_api] joint is not RevoluteJoint\n";
      return;
    }
    write(joint->getAxis(), outv3);
}


void REVOLUTE_JOINT(setAxis)(int wid, int skid, int jid, double inv3[3]) {
  dart::dynamics::RevoluteJoint* joint = GET_REVOLUTE_JOINT(wid, skid, jid);
  if (joint == NULL) {
    ERR << " [pydart2_api] joint is not RevoluteJoint\n";
    return;
  }
  joint->setAxis(read(inv3, 3));
}


////////////////////////////////////////
// Joint::PRISMATIC_JOINT Functions
void PRISMATIC_JOINT(getAxis)(int wid, int skid, int jid, double outv3[3]) {
    dart::dynamics::PrismaticJoint* joint = GET_PRISMATIC_JOINT(wid, skid, jid);
    if (joint == NULL) {
      ERR << " [pydart2_api] joint is not PrismaticJoint\n";
      return;
    }
    write(joint->getAxis(), outv3);
}


void PRISMATIC_JOINT(setAxis)(int wid, int skid, int jid, double inv3[3]) {
  dart::dynamics::PrismaticJoint* joint = GET_PRISMATIC_JOINT(wid, skid, jid);
  if (joint == NULL) {
    ERR << " [pydart2_api] joint is not PrismaticJoint\n";
    return;
  }
  joint->setAxis(read(inv3, 3));
}


////////////////////////////////////////
// Joint::UNIVERSAL_JOINT Functions
void UNIVERSAL_JOINT(getAxis1)(int wid, int skid, int jid, double outv3[3]) {
    dart::dynamics::UniversalJoint* joint = GET_UNIVERSAL_JOINT(wid, skid, jid);
    if (joint == NULL) {
      ERR << " [pydart2_api] joint is not UniversalJoint\n";
      return;
    }
    write(joint->getAxis1(), outv3);
}


void UNIVERSAL_JOINT(setAxis1)(int wid, int skid, int jid, double inv3[3]) {
  dart::dynamics::UniversalJoint* joint = GET_UNIVERSAL_JOINT(wid, skid, jid);
  if (joint == NULL) {
    ERR << " [pydart2_api] joint is not UniversalJoint\n";
    return;
  }
  joint->setAxis1(read(inv3, 3));
}


void UNIVERSAL_JOINT(getAxis2)(int wid, int skid, int jid, double outv3[3]) {
    dart::dynamics::UniversalJoint* joint = GET_UNIVERSAL_JOINT(wid, skid, jid);
    if (joint == NULL) {
      ERR << " [pydart2_api] joint is not UniversalJoint\n";
      return;
    }
    write(joint->getAxis2(), outv3);
}


void UNIVERSAL_JOINT(setAxis2)(int wid, int skid, int jid, double inv3[3]) {
  dart::dynamics::UniversalJoint* joint = GET_UNIVERSAL_JOINT(wid, skid, jid);
  if (joint == NULL) {
    ERR << " [pydart2_api] joint is not UniversalJoint\n";
    return;
  }
  joint->setAxis2(read(inv3, 3));
}


////////////////////////////////////////
// Joint::EULER_JOINT Functions
const char* EULER_JOINT(getAxisOrder)(int wid, int skid, int jid) {
    dart::dynamics::EulerJoint* joint = GET_EULER_JOINT(wid, skid, jid);
    if (joint == NULL) {
      ERR << " [pydart2_api] joint is not EulerJoint\n";
      return "Wrong Joint Type";
    }
    auto ordering = joint->getAxisOrder();
    if (ordering == dart::dynamics::EulerJoint::AxisOrder::XYZ) {
        return "XYZ";
    }
    if (ordering == dart::dynamics::EulerJoint::AxisOrder::ZYX) {
        return "ZYX";
    }
    return "Invalid Order";
}


void EULER_JOINT(setAxisOrder)(int wid, int skid, int jid, const char* axisorder) {
  dart::dynamics::EulerJoint* joint = GET_EULER_JOINT(wid, skid, jid);
  if (joint == NULL) {
    ERR << " [pydart2_api] joint is not EulerJoint\n";
    return;
  }
  std::string ordering(axisorder);
  if (ordering == "XYZ") {
    joint->setAxisOrder(dart::dynamics::EulerJoint::AxisOrder::XYZ);
  } else if (ordering == "ZYX"){
    joint->setAxisOrder(dart::dynamics::EulerJoint::AxisOrder::ZYX);
  } else {
    ERR << " [pydart2_api] invalid EulerJoint AxisOrder" << ordering << "\n";
  }
}


////////////////////////////////////////////////////////////////////////////////
// Marker

int MARKER(getBodyNode)(int wid, int skid, int mid) {
    dart::dynamics::Marker* marker = GET_MARKER(wid, skid, mid);
    return marker->getBodyNodePtr()->getIndexInSkeleton();
}


void MARKER(getLocalPosition)(int wid, int skid, int mid, double outv3[3]) {
    dart::dynamics::Marker* marker = GET_MARKER(wid, skid, mid);
    write(marker->getLocalPosition(), outv3);
}


void MARKER(setLocalPosition)(int wid, int skid, int mid, double inv3[3]) {
    dart::dynamics::Marker* marker = GET_MARKER(wid, skid, mid);
    marker->setLocalPosition(read(inv3, 3));
}


void MARKER(getWorldPosition)(int wid, int skid, int mid, double outv3[3]) {
    dart::dynamics::Marker* marker = GET_MARKER(wid, skid, mid);
    write(marker->getWorldPosition(), outv3);
}

void MARKER(render)(int wid, int skid, int mid) {
    dart::dynamics::Marker* marker = GET_MARKER(wid, skid, mid);
    dart::gui::RenderInterface* ri = Manager::getRI();
    drawMarker(ri, marker);
}

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

////////////////////////////////////////
// Shape Functions

double SHAPE(getVolume)(int wid, int skid, int bid, int sid) {
    dart::dynamics::Shape* shape = GET_SHAPE(wid, skid, bid, sid);
    return shape->getVolume();
}

int SHAPE(getShapeType)(int wid, int skid, int bid, int sid) {
    dart::dynamics::Shape* shape = GET_SHAPE(wid, skid, bid, sid);
    return (int)shape->getShapeType();
}

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

////////////////////////////////////////////////////////////////////////////////
// Collision Result
int COLLISION_RESULT(getNumContacts)(int wid) {
    const auto result = GET_COLLISION_RESULT(wid);
    return result.getNumContacts();
}

void COLLISION_RESULT(getContacts)(int wid, double* outv, int nout) {
    const auto result = GET_COLLISION_RESULT(wid);
    const auto nContacts = static_cast<int>(result.getNumContacts());

    // Construct the skeleton index map
    std::map<const dart::dynamics::Skeleton*, int> indices;
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    for (size_t i = 0; i < world->getNumSkeletons(); ++i) {
        indices[world->getSkeleton(i).get()] = i;
    }

    Eigen::VectorXd state(10 * nContacts);
    for (auto i = 0; i < nContacts; ++i) {
        auto begin = i * 10;
        auto contact = result.getContact(i);

        state.segment(begin, 3)     = contact.point;
        state.segment(begin + 3, 3) = contact.force;
        state(begin + 6) = -1;
        state(begin + 7) = -1;
        state(begin + 8) = -1;
        state(begin + 9) = -1;

        auto shapeNode1 = contact.collisionObject1->getShapeFrame()->asShapeNode();
        if (shapeNode1) {
            auto b = shapeNode1->getBodyNodePtr();
            state(begin + 6) = indices[b->getSkeleton().get()];
            state(begin + 7) = b->getIndexInSkeleton();
        }

        auto shapeNode2 = contact.collisionObject2->getShapeFrame()->asShapeNode();
        if (shapeNode2) {
            auto b = shapeNode2->getBodyNodePtr();
            state(begin + 8) = indices[b->getSkeleton().get()];
            state(begin + 9) = b->getIndexInSkeleton();
        }
    }
    write(state, outv);
}

std::vector<int> COLLISION_RESULT(getCollidingBodyNodes)(int wid) {
    const auto result = GET_COLLISION_RESULT(wid);
    const auto bodynodes = result.getCollidingBodyNodes();
    std::vector<int> ret;
    std::map<const dart::dynamics::Skeleton*, int> indices;
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    for (size_t i = 0; i < world->getNumSkeletons(); ++i) {
        indices[world->getSkeleton(i).get()] = i;
    }

    for (auto b: bodynodes) {
        ret.push_back(indices[b->getSkeleton().get()]);
        ret.push_back(b->getIndexInSkeleton());
    }
    return ret;
}

void COLLISION_RESULT(renderContact)(double inv6[6], double size, double scale) {
    dart::gui::RenderInterface* ri = Manager::getRI();
    drawContact(ri, read(inv6, 6), size, scale);
}

////////////////////////////////////////////////////////////////////////////////
// Constraints
int addBallJointConstraint(int wid, int skid1, int bid1, int skid2, int bid2,
                           double inv3[3]) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    dart::dynamics::BodyNodePtr bd1 = GET_BODY(wid, skid1, bid1);
    dart::dynamics::BodyNodePtr bd2 = GET_BODY(wid, skid2, bid2);
    Eigen::Vector3d jointPos = read(inv3, 3);
    // MSG << bd1->getName() << "\n";
    // MSG << bd2->getName() << "\n";
    // MSG << jointPos << "\n";
    dart::constraint::BallJointConstraintPtr cl =
        std::make_shared<dart::constraint::BallJointConstraint>(bd1, bd2, jointPos);
    world->getConstraintSolver()->addConstraint(cl);
    return 0;
}
