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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>

#include <gnuradio/io_signature.h>

#include "rach_detect_impl.h"


#include "cxvec_compat.h"

extern "C" {
#include <osmocom/gmr1/sdr/pi4cxpsk.h>
#include <osmocom/gmr1/sdr/nb.h>
}


namespace gr {
  namespace gmr1 {

rach_detect::sptr
rach_detect::make(int sps, int burst_window, float freq_bias,
                  const std::string& len_tag_key)
{
	return gnuradio::get_initial_sptr(
		new rach_detect_impl(
			sps, burst_window, freq_bias, len_tag_key
		)
	);
}


rach_detect_impl::rach_detect_impl(int sps, int burst_window, float freq_bias,
                                   const std::string& len_tag_key)
    : gr::hier_block2("rach_detect",
                io_signature::make(1, 1, sizeof(gr_complex)),
                io_signature::make(1, 1, sizeof(gr_complex))),
      d_sps(sps), d_burst_window(burst_window), d_freq_bias(freq_bias)
{
	const struct gmr1_pi4cxpsk_burst *rach = &gmr1_rach_burst;
	std::vector<gr_complex> taps;
	int delay, avg_len, burst_offset, burst_length, scan_window;

	/* Build the taps from RACH burst description */
	delay = this->gen_taps(taps);

	/* Compute parameters */
	avg_len = taps.size();
	burst_length = rach->len * this->d_sps + burst_window;
	burst_offset = - (delay + ( burst_window / 2));
	scan_window = (rach->len * this->d_sps) / 2;

	/* Build the sub-blocks */
	this->d_ref_pwr  = gr::blocks::complex_to_mag_squared::make();
	this->d_ref_pwr_avg = gr::blocks::moving_average_ff::make(avg_len, 1.0f, avg_len * 10);

	this->d_corr = gr::filter::fft_filter_ccc::make(1, taps);
	this->d_corr_pwr = gr::blocks::complex_to_mag_squared::make();

	this->d_detect = gr::gmr1::rach_detect_core::make(burst_offset, burst_length, scan_window, len_tag_key);

	/* Connect */
	this->connect(this->self(), 0, this->d_ref_pwr, 0);
	this->connect(this->d_ref_pwr, 0, this->d_ref_pwr_avg, 0);

	this->connect(this->self(), 0, this->d_corr, 0);
	this->connect(this->d_corr, 0, this->d_corr_pwr, 0);

	this->connect(this->self(), 0, this->d_detect, 0);
	this->connect(this->d_ref_pwr_avg, 0, this->d_detect, 1);
	this->connect(this->d_corr_pwr, 0, this->d_detect, 2);

	this->connect(this->d_detect, 0, this->self(), 0);
}

rach_detect_impl::~rach_detect_impl()
{
	/* Nothing to do */
}


int
rach_detect_impl::sps() const
{
	return this->d_sps;
}

int
rach_detect_impl::burst_window() const
{
	return this->d_burst_window;
}


int
rach_detect_impl::gen_taps(std::vector<gr_complex> &taps)
{
	const struct gmr1_pi4cxpsk_burst *rach = &gmr1_rach_burst;
	struct osmo_cxvec *sync;
	int i, j, k;
	int first, last;

	/* Generate reference signal */
	sync = osmo_cxvec_alloc(rach->len);
	sync->len = rach->len;

	for (i=0,j=0,k=0; i<rach->len; i++)
	{
		struct gmr1_pi4cxpsk_sync *sp = &rach->sync[0][j];

		sync->data[i] = 0.0f;

		if ((sp->pos + k) == i) {
			sync->data[i] = rach->mod->syms[sp->syms[k++]].mod_val;
			if (k == sp->len) {
				j++;
				k = 0;
			}
		}
	}

	osmo_cxvec_rotate(sync, M_PIf / 4.0f + this->d_freq_bias, sync);

	/* First and last tap (ignore the final 1 symbol sync) */
	first = rach->sync[0][0].pos;
	last  = rach->sync[0][j-2].pos + rach->sync[0][j-2].len - 1;

	/* Create reverse conjugate vector */
	for (i=last; i>=first; i--)
	{
		taps.push_back(conjf(sync->data[i]));

		if (i != last)
			for (j=1; j<this->d_sps; j++)
				taps.push_back(0.0f);
	}

	/* Done */
	osmo_cxvec_free(sync);

	/* Return filter delay (wrt to burst start) */
	return last * this->d_sps;
}

  } // namespace gmr1
} // namespace gr
