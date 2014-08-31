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


#ifndef INCLUDED_GR_GMR1_RACH_DETECT_IMPL_H
#define INCLUDED_GR_GMR1_RACH_DETECT_IMPL_H

#include <gnuradio/gmr1/rach_detect.h>

#include <gnuradio/blocks/complex_to_mag_squared.h>
#include <gnuradio/blocks/moving_average_ff.h>
#include <gnuradio/filter/fft_filter_ccc.h>
#include <gnuradio/gmr1/rach_detect_core.h>


namespace gr {
  namespace gmr1 {

    /*!
     * \brief
     * \ingroup gmr1
     */
    class rach_detect_impl : public rach_detect
    {
     private:
      int d_sps;
      int d_burst_window;
      float d_freq_bias;

      gr::blocks::complex_to_mag_squared::sptr	d_ref_pwr;
      gr::blocks::moving_average_ff::sptr	d_ref_pwr_avg;
      gr::filter::fft_filter_ccc::sptr		d_corr;
      gr::blocks::complex_to_mag_squared::sptr	d_corr_pwr;
      gr::gmr1::rach_detect_core::sptr		d_detect;

      int gen_taps(std::vector<gr_complex> &taps);

     public:
      rach_detect_impl(int sps, int burst_window, float freq_bias,
                       const std::string& len_tag_key);
      virtual ~rach_detect_impl();

      int sps() const;
      int burst_window() const;
    };

  } // namespace gmr1
} // namespace gr

#endif /* INCLUDED_GR_GMR1_RACH_DETECT_IMPL_H */
