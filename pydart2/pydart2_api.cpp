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

////////////////////////////////////////////////////////////////////////////////
// Skeleton
void write(const Eigen::VectorXd& src, double* dst) {
    for (int i = 0; i < src.size(); i++) {
        dst[i] = src(i);
    }
}

Eigen::VectorXd read(double* src, int n) {
    Eigen::VectorXd dst(n);
    for (int i =0; i < n; i++) {
        dst(i) = src[i];
    }
    return dst;
}


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

bool SKEL(isMobile)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->isMobile();
}

void SKEL(setMobile)(int wid, int skid, bool mobile) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setMobile(mobile);
}


////////////////////////////////////////
// Structure Information Functions
int SKEL(getNumBodies)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getNumBodyNodes();
}

int SKEL(getNumDofs)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getNumDofs();
}

////////////////////////////////////////
// Pose Functions
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
