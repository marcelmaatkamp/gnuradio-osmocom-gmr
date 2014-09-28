/* -*- c++ -*- */
/*
 * Copyright 2014 Sylvain Munaut <tnt@246tNt.com>
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */


#ifndef INCLUDED_GR_GMR1_RACH_DETECT_FFT_IMPL_H
#define INCLUDED_GR_GMR1_RACH_DETECT_FFT_IMPL_H

#include <gnuradio/gmr1/rach_detect_fft.h>

#include <gnuradio/fft/fft.h>

namespace gr {
  namespace gmr1 {

    /*!
     * \brief
     * \ingroup gmr1
     */
    class rach_detect_fft_impl : public rach_detect_fft
    {
     private:
      float d_threshold;
      int d_overlap_ratio;
      int d_fft_size;
      gr::fft::fft_complex *d_fft;
      gr_complex *d_buf;
      float *d_win;
      float *d_pwr;

      int d_pos;

      void peak_detect(uint64_t position);

     public:
      rach_detect_fft_impl();
      virtual ~rach_detect_fft_impl();

      int general_work(int noutput_items,
                       gr_vector_int& ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items);
    };

  } // namespace gmr1
} // namespace gr

#endif /* INCLUDED_GR_GMR1_RACH_DETECT_FFT_IMPL_H */
