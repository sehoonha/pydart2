#ifndef PYDART2_PYDART2_IK_H
#define PYDART2_PYDART2_IK_H

struct Pydart2IK_imp;

class Pydart2IK {
public:
    Pydart2IK();
    ~Pydart2IK();
    void set_verbose(bool verbose);
    int num();
    void set_skeleton(int wid, int skid);
    const char* skel_name();

    void add_body_and_offset(int wid, int skid, int bid, double inv3[3]);
    const char* body_name(int index);
    void offset(int index, double outv3[3]);
    void position(int index, double outv3[3]);

    void set_target(int index, double inv3[3]);
    void set_targets(double* v, int dim1, int dim2);
    void target(int index, double outv3[3]);

    double f();
    void g(double* outv, int nout);

    int solve(double xtol_rel);

    Pydart2IK_imp* imp;
};


#endif
