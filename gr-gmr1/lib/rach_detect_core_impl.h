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


#ifndef INCLUDED_GR_GMR1_RACH_DETECT_CORE_IMPL_H
#define INCLUDED_GR_GMR1_RACH_DETECT_CORE_IMPL_H

#include <gnuradio/gmr1/rach_detect_core.h>

namespace gr {
  namespace gmr1 {

    /*!
     * \brief
     * \ingroup gmr1
     */
    class rach_detect_core_impl : public rach_detect_core
    {
     private:
      pmt::pmt_t d_len_tag_key;
      pmt::pmt_t d_burst_length_pmt;

      int d_burst_offset;
      int d_burst_length;
      int d_scan_window;

      gr_complex *d_burst;

      int      d_cnt;
      uint64_t d_max_pos;
      float    d_max_corr;

     public:
      rach_detect_core_impl(int burst_offset, int burst_length,
                            int scan_window,
                            const std::string& len_tag_key);
      virtual ~rach_detect_core_impl();

      int burst_offset() const;
      int burst_length() const;
      int scan_window() const;

      int general_work(int noutput_items,
                       gr_vector_int& ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items);

    };

  } // namespace gmr1
} // namespace gr

#endif /* INCLUDED_GR_GMR1_RACH_DETECT_CORE_IMPL_H */
