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
#include "dart/utils/utils.hpp"

namespace pydart {

////////////////////////////////////////////////////////////
// class Manager
class Manager {
public:
    static void init();
    static void destroy();
    static Manager* getInstance() { return g_manager; }
    // static dart::renderer::RenderInterface* getRI() {
    //     return g_ri;
    // }

    static dart::simulation::WorldPtr world(int index = 0);
    static dart::dynamics::SkeletonPtr skeleton(int index);
    static dart::dynamics::SkeletonPtr skeleton(int wid, int skid);
    static int createWorld(double timestep);
    static int createWorldFromSkel(const char* const path);
    static void destroyWorld(int id);
    
protected:
    static Manager* g_manager;
    // static dart::renderer::RenderInterface* g_ri;

    // std::vector<dart::simulation::WorldPtr> worlds;
    int next_id;
    std::map<int, dart::simulation::WorldPtr> worlds;

};

Manager* Manager::g_manager = NULL;
// dart::renderer::RenderInterface* Manager::g_ri = NULL;

void Manager::init() {
    g_manager = new Manager();
    // g_ri = new dart::renderer::OpenGLRenderInterface();
    //   g_ri->initialize();
    g_manager->next_id = 0;
    cout << " [pydart_api] Initialize pydart manager OK" << endl;
}

void Manager::destroy() {
    if (g_manager) {
        delete g_manager;
        g_manager = NULL;
    }
    // if (g_ri) {
    //     delete g_ri;
    //     g_ri = NULL;
    // }
    cout << " [pydart_api] Destroy pydart manager OK" << endl;
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
    return id;
}

void Manager::destroyWorld(int id) {
    Manager* manager = getInstance();
    dart::simulation::WorldPtr w = manager->worlds[id];
    manager->worlds.erase(id);
    cout << " [pydart_api] worlds.size = " << manager->worlds.size() << endl;
    // w.reset();
    cout << " [pydart_api] Destroy world OK: " << id << endl;
}

// class Manager
////////////////////////////////////////////////////////////

} // namespace pydart

using namespace pydart;

////////////////////////////////////////////////////////////////////////////////
// Init Functions
void init() {
    if (Manager::getInstance()) {
        Manager::destroy();
    }
    Manager::init();
}

void destroy() {
    Manager::destroy();
}

