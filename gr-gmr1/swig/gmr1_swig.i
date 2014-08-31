/* -*- c++ -*- */

#define GR_GMR1_API

%include "gnuradio.i"                   // the common stuff

//load generated python docstrings
%include "gmr1_swig_doc.i"


%{
#include "gnuradio/gmr1/gsmtap_sink.h"
#include "gnuradio/gmr1/rach_demod.h"
#include "gnuradio/gmr1/rach_detect.h"
#include "gnuradio/gmr1/rach_detect_core.h"
%}

%include "gnuradio/gmr1/gsmtap_sink.h"
GR_SWIG_BLOCK_MAGIC2(gmr1, gsmtap_sink);

%include "gnuradio/gmr1/rach_demod.h"
GR_SWIG_BLOCK_MAGIC2(gmr1, rach_demod);

%include "gnuradio/gmr1/rach_detect.h"
GR_SWIG_BLOCK_MAGIC2(gmr1, rach_detect);

%include "gnuradio/gmr1/rach_detect_core.h"
GR_SWIG_BLOCK_MAGIC2(gmr1, rach_detect_core);
