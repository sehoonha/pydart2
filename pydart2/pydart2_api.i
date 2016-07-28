%module pydart2_api

%{
  #define SWIG_FILE_WITH_INIT
  #include "pydart2_api.h"
%}

/* Include the NumPy typemaps library */
%include "numpy.i"

%init %{
  import_array();
%}

%apply (double IN_ARRAY1[ANY]) {(double inv3[3])};
%apply (double IN_ARRAY1[ANY]) {(double inv4[4])};

%apply (double* IN_ARRAY1, int DIM1) {(double* inv, int ndofs)};
%apply (double* IN_ARRAY1, int DIM1) {(double* inv1, int indofs1)};
%apply (double* IN_ARRAY1, int DIM1) {(double* inv2, int indofs2)};
%apply (double* ARGOUT_ARRAY1, int DIM1) {(double* outv, int ndofs)};
%apply (double* ARGOUT_ARRAY1, int DIM1) {(double* outv1, int ondofs1)};
%apply (double* ARGOUT_ARRAY1, int DIM1) {(double* outv2, int ondofs2)};
%apply (double* INPLACE_ARRAY2, int DIM1, int DIM2) {(double* outm, int nrows, int ncols)};

%include "pydart2_api.h"

