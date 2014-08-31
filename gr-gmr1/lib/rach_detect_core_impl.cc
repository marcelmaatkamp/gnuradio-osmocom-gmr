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

#include "rach_detect_core_impl.h"


namespace gr {
  namespace gmr1 {

rach_detect_core::sptr
rach_detect_core::make(int burst_offset, int burst_length, int scan_window,
                       const std::string& len_tag_key)
{
	return gnuradio::get_initial_sptr(
		new rach_detect_core_impl(
			burst_offset, burst_length, scan_window,
			len_tag_key
		)
	);
}

rach_detect_core_impl::rach_detect_core_impl(
	int burst_offset, int burst_length, int scan_window,
	const std::string& len_tag_key)
    : gr::block("rach_detect_core",
                io_signature::make2(3, 3, sizeof(gr_complex), sizeof(float)),
                io_signature::make (1, 1, sizeof(gr_complex))),
      d_len_tag_key(pmt::string_to_symbol(len_tag_key)),
      d_burst_length_pmt(pmt::from_long(burst_length)),
      d_burst_offset(burst_offset), d_burst_length(burst_length),
      d_scan_window(scan_window),
      d_cnt(0), d_max_pos(0), d_max_corr(0.0f)
{
	/* Allocate buffer for burst */
	this->d_burst = new gr_complex[d_burst_length];

	/* History setup */
	if (this->d_burst_offset < 0) {
		this->set_history(1 + this->d_burst_length);
	} else {
		this->set_history(1 + this->d_burst_length + this->d_burst_offset);
	}
}

rach_detect_core_impl::~rach_detect_core_impl()
{
	/* Release buffer */
	delete[] this->d_burst;
}


int
rach_detect_core_impl::burst_offset() const
{
	return this->d_burst_offset;
}

int
rach_detect_core_impl::burst_length() const
{
	return this->d_burst_length;
}

int
rach_detect_core_impl::scan_window() const
{
	return this->d_scan_window;
}


int
rach_detect_core_impl::general_work(
	int noutput_items,
	gr_vector_int& ninput_items,
	gr_vector_const_void_star &input_items,
	gr_vector_void_star &output_items)
{
	/* Cast buffers */
	const gr_complex *in_sig    = reinterpret_cast<const gr_complex *>(input_items[0]);
	const float      *in_pwr    = reinterpret_cast<const float *>(input_items[1]);
	const float      *in_corr   = reinterpret_cast<const float *>(input_items[2]);
	gr_complex       *out_burst = reinterpret_cast<gr_complex *>(output_items[0]);

	/* Skip over history */
	if (this->d_burst_offset < 0)
	{
		in_sig  -= this->d_burst_offset;
		in_pwr  -= this->d_burst_offset;
		in_corr -= this->d_burst_offset;
	}

	/* Position */
	uint64_t pos = this->nitems_read(0);

	/* Just skip the start because the trigger point is not ready yet */
	if (pos < history()) {
		this->consume_each(noutput_items);
		return 0;
	}

	/* Main scan loop */
	int i, max_i = -1;

	for (i=0; i<noutput_items; i++,pos++)
	{
		/* Trigger condition */
		if (in_corr[i] > 1.5f * in_pwr[i])
		{
			/* Reset scan window */
			this->d_cnt = this->d_scan_window;

			/* New maxima ? */
			if (in_corr[i] > d_max_corr)
			{
				this->d_max_corr = in_corr[i];
				this->d_max_pos  = pos;
				max_i = i;
			}
		}

		/* Within a window ? */
		else if (this->d_cnt > 0)
		{
			/* No match here */
			this->d_cnt--;

			/* Is it the end ? */
			if (!this->d_cnt)
			{
				printf("%llu\n", (long long unsigned)this->d_max_pos);

				this->d_max_corr = 0.0f;
				this->d_max_pos  = 0;

				memcpy(
					out_burst,
					(max_i >= 0) ?
						&in_sig[max_i + this->d_burst_offset] :
						this->d_burst,
					this->d_burst_length * sizeof(gr_complex)
				);

				add_item_tag(
					0,
					this->nitems_written(0),
					this->d_len_tag_key,
					this->d_burst_length_pmt
				);

				this->consume_each(i);
				return this->d_burst_length;
			}
		}
	}

	/* Save the burst for later */
	if (max_i >= 0) {
		memcpy(
			this->d_burst,
			&in_sig[max_i + this->d_burst_offset],
			this->d_burst_length * sizeof(gr_complex)
		);
	}

	/* We scanned it all */
	this->consume_each(noutput_items);

	return 0;
}

  } // namespace gmr1
} // namespace gr
