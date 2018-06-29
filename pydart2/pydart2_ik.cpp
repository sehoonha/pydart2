#include <nlopt.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <map>
using std::cout;
using std::cerr;
using std::endl;

#include "pydart2_ik.h"
#include <Eigen/Dense>
#include "dart/dart.hpp"

#include "pydart2_manager.h"
#include "pydart2_api.h"
#include "pydart2_skeleton_api.h"
#include "pydart2_bodynode_api.h"
using namespace pydart;


struct Pydart2IK_imp {
    bool verbose = false;
    int num = 0;
    int eval_counter = 0;
    dart::dynamics::SkeletonPtr skel = NULL;
    std::vector<dart::dynamics::BodyNodePtr> bodynodes;
    std::vector<Eigen::Vector3d> offsets;
    std::vector<Eigen::Vector3d> targets;
};

Pydart2IK::Pydart2IK() {
    cout << "Pydart2IK" << endl;
    this->imp = new Pydart2IK_imp();
}

Pydart2IK::~Pydart2IK() {
    cout << "~Pydart2IK" << endl;
    if (this->imp) {
        delete this->imp;
    }
}

void Pydart2IK::set_verbose(bool verbose) {
    this->imp->verbose = verbose;
}


int Pydart2IK::num() {
    return this->imp->num;
}


void Pydart2IK::set_skeleton(int wid, int skid) {
    this->imp->skel = GET_SKELETON(wid, skid);
}

const char* Pydart2IK::skel_name() {
    return this->imp->skel->getName().c_str();
}


void Pydart2IK::add_body_and_offset(int wid, int skid, int bid, double inv3[3]) {
    dart::dynamics::BodyNodePtr body = GET_BODY(wid, skid, bid);
    Eigen::Vector3d offset(inv3[0], inv3[1], inv3[2]);
    this->imp->num++;
    this->imp->bodynodes.push_back(body);
    this->imp->offsets.push_back(offset);
    this->imp->targets.push_back(Eigen::Vector3d::Zero());
}

const char* Pydart2IK::body_name(int index) {
    return this->imp->bodynodes[index]->getName().c_str();
}

void Pydart2IK::offset(int index, double outv3[3]) {
    write(this->imp->offsets[index], outv3);
}

void Pydart2IK::position(int index, double outv3[3]) {
    auto& body = this->imp->bodynodes[index];
    const auto& o = this->imp->offsets[index];
    const Eigen::Isometry3d& T = body->getWorldTransform();
    write(T.rotation() * o + T.translation(), outv3);
}


void Pydart2IK::set_target(int index, double inv3[3]) {
    this->imp->targets[index] << inv3[0], inv3[1], inv3[2];
}

void Pydart2IK::set_targets(double* v, int dim1, int dim2) {
    int ptr = 0;
    for (int i = 0; i < dim1; i++) {
        this->imp->targets[i] << v[ptr], v[ptr + 1], v[ptr + 2];
        ptr += 3;
    }
}

void Pydart2IK::target(int index, double outv3[3]) {
    write(this->imp->targets[index], outv3);
}

double Pydart2IK::f() {
    double ret = 0.0;
    int N = this->imp->num;
    for (int index = 0; index < N; ++index) {
        auto& body = this->imp->bodynodes[index];
        const auto& o = this->imp->offsets[index];
        const Eigen::Isometry3d& T = body->getWorldTransform();
        const Eigen::Vector3d& lhs = T.rotation() * o + T.translation();
        const Eigen::Vector3d& rhs = this->imp->targets[index];
        double err_i = (lhs - rhs).squaredNorm();
        ret += err_i;
        // cout << err_i << " " << lhs.transpose() << " " << rhs.transpose() << endl;
    }
    return ret / N;
}

void Pydart2IK::g(double* outv, int nout) {
    double ret = 0.0;
    int N = this->imp->num;
    auto& skel = this->imp->skel;
    int nDofs = skel->getNumDofs();
    Eigen::VectorXd g = Eigen::VectorXd::Zero(nDofs);
    for (int index = 0; index < N; ++index) {
        auto& body = this->imp->bodynodes[index];
        const auto& o = this->imp->offsets[index];
        const Eigen::Isometry3d& T = body->getWorldTransform();
        const Eigen::Vector3d& lhs = T.rotation() * o + T.translation();
        const Eigen::Vector3d& rhs = this->imp->targets[index];
        const Eigen::MatrixXd& J = body->getLinearJacobian(o);
        const Eigen::VectorXd& g_i = J.transpose() * 2.0 * (lhs - rhs);
        // cout << "[J]" << J.rows() << " " << J.cols() << endl;
        // cout << J << endl;
        // cout << lhs.transpose() << " " << rhs.transpose() << endl;
        // cout << "g_i.size = " << g_i.size() << endl;
        // cout << "g_i = " << g_i.transpose() << endl;
        for (int j = 0; j < body->getNumDependentDofs(); ++j) {
            int dof = body->getDependentDof(j)->getIndexInSkeleton();
            // cout << dof << " <- " << j << endl;
            g[dof] += g_i[j];
        }
    }
    g /= N;
    // cout << "g = " << g.transpose() << endl;
    write(g, outv);
}

double func(const std::vector<double> &x,
            std::vector<double> &grad,
            void *my_func_data) {
    Pydart2IK* ik = static_cast<Pydart2IK*>(my_func_data);
    auto& skel = ik->imp->skel;
    int nDofs = skel->getNumDofs();
    ik->imp->eval_counter++;

    Eigen::VectorXd x_in_eigen(x.size());
    for (int i = 0; i < x.size(); ++i) x_in_eigen[i] = x[i];
    skel->setPositions(x_in_eigen);
    double ret = ik->f();

    if (ik->imp->verbose) {
        printf("[f] %d: %.8f\n", ik->imp->eval_counter, ret);
    }

    if (!grad.empty()) {
        double* temp_g = new double[nDofs];
        ik->g(temp_g, nDofs);
        for (int i = 0; i < nDofs; i++) grad[i] = temp_g[i];
        // cout << "\ttemp_g OK" << endl;
        // for (int i = 0; i < nDofs; i++) cout << grad[i] << " "; cout << endl;
        delete[] temp_g;
    }
    return ret;
}

int Pydart2IK::solve(double xtol_rel) {
    auto& skel = this->imp->skel;
    int nDofs = skel->getNumDofs();
    // nlopt::opt opt(nlopt::LD_MMA, nDofs);
    nlopt::opt opt(nlopt::LD_SLSQP, nDofs);
    opt.set_min_objective(func, this);
    opt.set_xtol_rel(xtol_rel);
    double minf;

    Eigen::VectorXd q = skel->getPositions();
    std::vector<double> x(q.data(), q.data() + q.size());

    this->imp->eval_counter = 0;
    try{
        nlopt::result result = opt.optimize(x, minf);
        // std::cout << "found minimum at f(" <<  skel->getPositions().transpose() << ") = "
        //     << std::setprecision(10) << minf << std::endl;
        return 0;
    }
    catch(std::exception &e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
        return -1;
    }
}
