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
#include <gnuradio/fft/window.h>
#include <volk/volk.h>

#include "rach_detect_fft_impl.h"


namespace gr {
  namespace gmr1 {

rach_detect_fft::sptr
rach_detect_fft::make()
{
	return gnuradio::get_initial_sptr(
		new rach_detect_fft_impl()
	);
}

rach_detect_fft_impl::rach_detect_fft_impl()
    : gr::block("rach_detect_fft",
                io_signature::make(1, 1, sizeof(gr_complex)),
                io_signature::make(1, 1, sizeof(gr_complex)))
{
	this->d_threshold = 8.5f; // 7.5f;
	this->d_overlap_ratio = 2;
	this->d_fft_size = 512;
	this->d_fft = new gr::fft::fft_complex(this->d_fft_size, true, 1);

	this->d_buf = (gr_complex *) volk_malloc(this->d_fft_size * sizeof(gr_complex), 128);
	this->d_win = (float *) volk_malloc(this->d_fft_size * sizeof(float), 128);
	this->d_pwr = (float *) volk_malloc(this->d_fft_size * sizeof(float), 128);

	this->d_pos = 0;

	std::vector<float> win = gr::fft::window::blackmanharris(this->d_fft_size);
	memcpy(this->d_win, &win[0], this->d_fft_size * sizeof(float));
}

rach_detect_fft_impl::~rach_detect_fft_impl()
{
	volk_free(this->d_pwr);
	volk_free(this->d_win);
	volk_free(this->d_buf);
	delete this->d_fft;
}


void
rach_detect_fft_impl::peak_detect(uint64_t position)
{
	const int avg_hwin = 15;
	const int avg_win = (avg_hwin * 2) + 1;
	float sum;
	int i;

	/* Prime the moving average */
	sum = 0.0f;
	for (i=0; i<avg_win; i++)
		sum += this->d_pwr[i];

	/* Do a scan and compare with avg with peak */
	for (i=avg_hwin; i<this->d_fft_size-avg_hwin; i++)
	{
		if (this->d_pwr[i] > (this->d_threshold * sum / (float)avg_win))
			printf("(%lld, %d)\n", (long long)position, i);
		sum += this->d_pwr[i+avg_hwin+1] - this->d_pwr[i-avg_hwin];
	}
}


int
rach_detect_fft_impl::general_work(
	int noutput_items,
	gr_vector_int& ninput_items,
	gr_vector_const_void_star &input_items,
	gr_vector_void_star &output_items)
{
	int read;

	/* Buffers pointer */
	const gr_complex *sig_in = reinterpret_cast<const gr_complex *>(input_items[0]);
	gr_complex *burst_out = reinterpret_cast<gr_complex *>(output_items[0]);
	gr_complex *fft_in  = this->d_fft->get_inbuf();
	gr_complex *fft_out = this->d_fft->get_outbuf();

	/* Process as much as we can */
	for (read=0; read<noutput_items;)
	{
		int n_adv = this->d_fft_size / this->d_overlap_ratio;
		int n_reuse = this->d_fft_size - n_adv;
		int n_fill;

		/* Fill our internal buffer */
		n_fill = this->d_fft_size - this->d_pos;
		if (n_fill > noutput_items - read)
			n_fill = noutput_items - read;

		memcpy(this->d_buf + this->d_pos,
		       sig_in + read,
		       n_fill * sizeof(gr_complex));

		read += n_fill;
		this->d_pos += n_fill;

		if (this->d_pos != this->d_fft_size)
			break;

		/* Apply window */
		volk_32fc_32f_multiply_32fc(
			fft_in,
			this->d_buf,
			this->d_win,
			this->d_fft_size
		);

		/* Compute FFT */
		this->d_fft->execute();

		/* Compute the squared power */
		volk_32fc_magnitude_squared_32f(this->d_pwr, fft_out, this->d_fft_size);

		/* Run the peak detection */
		this->peak_detect(
			this->nitems_read(0) + read - (this->d_fft_size / 2)
		);

		/* Handle overlap */
		if (this->d_overlap_ratio > 1) {
			memmove(this->d_buf, this->d_buf + n_adv, n_reuse * sizeof(gr_complex));
			this->d_pos = n_reuse;
		} else {
			this->d_pos = 0;
		}
	}

	/* We read some stuff */
	this->consume_each(read);

	return 0;
}

  } // namespace gmr1
} // namespace gr
