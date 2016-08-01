#include "pydart2_api.h"
#include <iostream>
#include <string>
#include <vector>
#include <map>
using std::cout;
using std::cerr;
using std::endl;

// Boost headers
#include <boost/algorithm/string.hpp>    

// Dart headers
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/utils/utils.hpp"

#include "pydart2_draw.h"

#define MSG if (Manager::g_verbose) dtmsg
#define DBG if (Manager::g_verbose) dtdbg
#define WARN if (Manager::g_verbose) dtwarn
#define ERR if (Manager::g_verbose) dterr

namespace pydart {

////////////////////////////////////////////////////////////
// class Manager
class Manager {
public:
    static void init();
    static void destroy();
    static Manager* getInstance() { return g_manager; }
    static dart::gui::RenderInterface* getRI() {
        return g_ri;
    }

    static dart::simulation::WorldPtr world(int index = 0);
    static dart::dynamics::SkeletonPtr skeleton(int index);
    static dart::dynamics::SkeletonPtr skeleton(int wid, int skid);
    static int createWorld(double timestep);
    static int createWorldFromSkel(const char* const path);
    static void destroyWorld(int id);
    static bool g_verbose;
protected:
    static Manager* g_manager;
    static dart::gui::RenderInterface* g_ri;

    // std::vector<dart::simulation::WorldPtr> worlds;
    int next_id;
    std::map<int, dart::simulation::WorldPtr> worlds;

};

Manager* Manager::g_manager = NULL;
bool Manager::g_verbose = true;
dart::gui::RenderInterface* Manager::g_ri = NULL;

void Manager::init() {
    g_manager = new Manager();
    g_ri = new dart::gui::OpenGLRenderInterface();
    g_ri->initialize();
    g_manager->next_id = 0;
    // if (verbose) dtmsg << "Hello!";
    MSG << " [pydart2_api] Initialize pydart manager OK\n";
}

void Manager::destroy() {
    if (g_manager) {
        delete g_manager;
        g_manager = NULL;
    }
    if (g_ri) {
        delete g_ri;
        g_ri = NULL;
    }
    MSG << " [pydart2_api] Destroy pydart manager OK\n";
}

dart::simulation::WorldPtr Manager::world(int index) {
    Manager* manager = getInstance();
    return manager->worlds[index];
}

dart::dynamics::SkeletonPtr Manager::skeleton(int index) {
    return world()->getSkeleton(index);
}

dart::dynamics::SkeletonPtr Manager::skeleton(int wid, int skid) {
    return world(wid)->getSkeleton(skid);
}


int Manager::createWorld(double timestep) {
    Manager* manager = getInstance();

    dart::simulation::WorldPtr w(new dart::simulation::World());
    w->setTimeStep(timestep);
    w->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
    // int id = manager->worlds.size();
    // manager->worlds.push_back(w);
    int id = manager->next_id++; 
    manager->worlds[id] = w;
    return id;
}

int Manager::createWorldFromSkel(const char* const path) {
    Manager* manager = getInstance();

    dart::simulation::WorldPtr w(dart::utils::SkelParser::readWorld(path));
    // w->setTimeStep(timestep);
    // w->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
    int id = manager->next_id++;
    manager->worlds[id] = w;
    // int id = manager->worlds.size();
    // manager->worlds.push_back(w);
    MSG << " [pydart2_api] worlds.size = " << manager->worlds.size() << "\n";
    MSG << " [pydart2_api] worlds.# skeletons = " << w->getNumSkeletons() << "\n";
    return id;
}

void Manager::destroyWorld(int id) {
    Manager* manager = getInstance();
    dart::simulation::WorldPtr w = manager->worlds[id];
    manager->worlds.erase(id);
    MSG << " [pydart2_api] worlds.size = " << manager->worlds.size() << "\n";
    // w.reset();
    MSG << " [pydart2_api] Destroy world OK: " << id << "\n";
}

// class Manager
////////////////////////////////////////////////////////////

} // namespace pydart

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
// Helper functions 
void write(const Eigen::VectorXd& src, double* dst) {
    for (int i = 0; i < src.size(); i++) {
        dst[i] = src(i);
    }
}

void write_matrix(const Eigen::MatrixXd& src, double* dst) {
    int ptr = 0;
    for (int i = 0; i < src.rows(); i++) {
        for (int j = 0; j < src.cols(); j++) {
            dst[ptr++] = src(i, j);
        }
    }
}

void write_isometry(const Eigen::Isometry3d& src, double dst[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            dst[i][j] = src(i, j);
        }
    }
}

Eigen::VectorXd read(double* src, int n) {
    Eigen::VectorXd dst(n);
    for (int i =0; i < n; i++) {
        dst(i) = src[i];
    }
    return dst;
}

Eigen::Isometry3d read_isometry(double src[4][4]) {
    Eigen::Isometry3d dst;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            dst(i, j) = src[i][j];
        }
    }
    return dst;
}


////////////////////////////////////////////////////////////////////////////////
// World 
int createWorld(double timestep) {
    return Manager::createWorld(timestep);
}

int createWorldFromSkel(const char* const path) {
    int wid = Manager::createWorldFromSkel(path);
    // MSG << " [pydart2_api] # Skeletons in " << path << " = " << numSkeletons(wid) << "\n";
    return wid;
}

void destroyWorld(int wid) {
    Manager::destroyWorld(wid);
}

int WORLD(addSkeleton)(int wid, const char* const path) {
    using namespace dart::simulation;
    using namespace dart::dynamics;
    std::string strpath(path);
    std::string ext = strpath.substr(strpath.length() - 4);
    boost::algorithm::to_lower(ext);

    SkeletonPtr skel = NULL;
    if (ext == ".sdf") {
        MSG << " [pydart_api] parse as SDF (ext: " << ext << ")" << endl;
        skel = dart::utils::SdfParser::readSkeleton(path);
    } else if (ext == "urdf") {
        MSG << " [pydart_api] parse as URDF (ext: " << ext << ")" << endl;
        dart::utils::DartLoader urdfLoader;
        skel = urdfLoader.parseSkeleton(path);
    } else if (ext == ".vsk") {
        MSG << " [pydart_api] parse as VSK (ext: " << ext << ")" << endl;
        skel = dart::utils::VskParser::readSkeleton(path);
    } else {
        ERR << " [pydart_api] bad extension (ext: " << ext << ")" << endl;
        return -1;
    }

    MSG << " [pydart_api] skel [" << path << "]" << endl;
    
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    int id = world->getNumSkeletons();
    world->addSkeleton(skel);
    return id;
}

int WORLD(getNumSkeletons)(int wid) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    return world->getNumSkeletons();
}

void WORLD(reset)(int wid) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    world->reset();
    for (size_t skid = 0; skid < world->getNumSkeletons(); skid++) {
        dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
        skel->resetCommands();
        skel->resetPositions();
        skel->resetVelocities();
        skel->resetAccelerations();
        skel->resetGeneralizedForces();
        skel->clearExternalForces();
        skel->clearConstraintImpulses();
    }
}

void WORLD(step)(int wid) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    world->step();
}

void WORLD(render)(int wid) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    dart::gui::RenderInterface* ri = Manager::getRI();
    drawWorld(ri, world);
}

////////////////////////////////////////
// World::Time Functions

void WORLD(setTimeStep)(int wid, double _timeStep) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    world->setTimeStep(_timeStep);
}

double WORLD(getTimeStep)(int wid) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    return world->getTimeStep();
}

void WORLD(setTime)(int wid, double _time) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    world->setTime(_time);
}

double WORLD(getTime)(int wid) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    return world->getTime();
}

int WORLD(getSimFrames)(int wid) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    return world->getSimFrames();
}

int WORLD(getIndex)(int wid, int _index) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    return world->getIndex(_index);
}

////////////////////////////////////////
// World::Property Functions
void WORLD(setGravity)(int wid, double inv3[3]) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    world->setGravity(read(inv3, 3));
}

void WORLD(getGravity)(int wid, double outv3[3]) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    write(world->getGravity(), outv3);
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
    MSG << "color = " << color.transpose() << "\n";
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
// Joint::Dof Functions
int JOINT(getDof)(int wid, int skid, int jid, int _index) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getDof(_index)->getIndexInSkeleton();
}


int JOINT(getNumDofs)(int wid, int skid, int jid) {
    dart::dynamics::JointPtr joint = GET_JOINT(wid, skid, jid);
    return joint->getNumDofs();
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
