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

void WORLD(setTimeStep)(int wid, double _timeStep);
double WORLD(getTimeStep)(int wid);
void WORLD(setTime)(int wid, double _time);
double WORLD(getTime)(int wid);
int WORLD(getSimFrames)(int wid);
int WORLD(getIndex)(int wid, int _index);

////////////////////////////////////////////////////////////////////////////////
// Skeleton
#define SKEL(funcname) skeleton__##funcname
#define GET_SKELETON(wid, skid) Manager::skeleton(wid, skid);

void SKEL(render)(int wid, int skid);
void SKEL(renderWithColor)(int wid, int skid, double inv4[4]);
const char* SKEL(getName)(int wid, int skid);
double SKEL(getMass)(int wid, int skid);
bool SKEL(isMobile)(int wid, int skid);
void SKEL(setMobile)(int wid, int skid, bool mobile);

////////////////////////////////////////
// Structure Information Functions
int SKEL(getNumBodies)(int wid, int skid);
int SKEL(getNumDofs)(int wid, int skid);

////////////////////////////////////////
// Pose Functions
void SKEL(getPositions)(int wid, int skid, double* outv, int ndofs);
void SKEL(setPositions)(int wid, int skid, double* inv, int ndofs);
void SKEL(getVelocities)(int wid, int skid, double* outv, int ndofs);
void SKEL(setVelocities)(int wid, int skid, double* inv, int ndofs);
void SKEL(setForces)(int wid, int skid, double* inv, int ndofs);

#endif // #ifndef PYDART2_PYDART2_API_H
