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
%apply (double* ARGOUT_ARRAY1, int DIM1) {(double* outv, int ndofs)};

%include "pydart2_api.h"

