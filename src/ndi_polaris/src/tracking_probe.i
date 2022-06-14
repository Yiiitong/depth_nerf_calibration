%module tracking_probe
%{
/* Includes the header in the wrapper code */
#include <ndi_polaris/blas.hpp>
#include <ndi_polaris/convert.hpp>
#include <ndi_polaris/convert_string.hpp>
#include <ndi_polaris/SerialComm.hpp>
#include <ndi_polaris/NDIComm.hpp>
#include <ndi_polaris/TrackerFrame.hpp>
#include <ndi_polaris/TrackerTarget.hpp>
%}

/* Parse the header file to generate wrappers */
%include "std_string.i"
%include "std_vector.i"
%include <ndi_polaris/blas.hpp>
%include <ndi_polaris/convert.hpp>
%include <ndi_polaris/convert_string.hpp>
%include <ndi_polaris/SerialComm.hpp>
%include <ndi_polaris/NDIComm.hpp>
%include <ndi_polaris/TrackerFrame.hpp>
%include <ndi_polaris/TrackerTarget.hpp>

namespace std {
    %template(TargetVector) vector<TrackerTarget>;
    %template(FloatVector) vector<float>;
}
