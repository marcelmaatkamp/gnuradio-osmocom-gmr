/* GMR-1 Demo RX live application */

/* (C) 2012 by Sylvain Munaut <tnt@246tNt.com>
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <complex.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <osmocom/core/gsmtap.h>
#include <osmocom/core/gsmtap_util.h>

#include <osmocom/dsp/cxvec.h>
#include <osmocom/dsp/cxvec_math.h>

#include <osmocom/gmr1/gsmtap.h>
#include <osmocom/gmr1/l1/a5.h>
#include <osmocom/gmr1/l1/bcch.h>
#include <osmocom/gmr1/l1/ccch.h>
#include <osmocom/gmr1/l1/facch3.h>
#include <osmocom/gmr1/l1/facch9.h>
#include <osmocom/gmr1/l1/tch3.h>
#include <osmocom/gmr1/l1/tch9.h>
#include <osmocom/gmr1/l1/interleave.h>
#include <osmocom/gmr1/sdr/defs.h>
#include <osmocom/gmr1/sdr/dkab.h>
#include <osmocom/gmr1/sdr/fcch.h>
#include <osmocom/gmr1/sdr/pi4cxpsk.h>
#include <osmocom/gmr1/sdr/nb.h>

#include "sampbuf.h"
#include "sa_file.h"


struct app_state
{
	/* Sample source */
	struct sample_buf *buf;

	int arfcn[MAX_CHANS];
	char *filename[MAX_CHANS];

	/* Params */
	int n_chans;
	int sps;

	/* GSMTap */
	struct gsmtap_inst *gti;
};

#define TCH3_MARGIN	10
#define TCH9_MARGIN	50
#define BCCH_MARGIN	100


/* ------------------------------------------------------------------------ */

static int
win_map(struct osmo_cxvec *win,
        float complex *data, int data_len,
        int begin, int win_len)
{
	if ((begin + win_len) > data_len)
		return -1;

	osmo_cxvec_init_from_data(win, &data[begin], win_len);

	return 0;
}

static int
burst_map(struct osmo_cxvec *burst,
          float complex *data, int data_len, int base_align, int sps,
          struct gmr1_pi4cxpsk_burst *burst_type, int tn, int win)
{
	int begin, len;
	int etoa;

	etoa  = win >> 1;
	begin = base_align + (sps * tn * 39) - etoa;
	len   = (burst_type->len * sps) + win;

	if ((begin < 0) || ((begin + len) > data_len))
		return -EIO;

	osmo_cxvec_init_from_data(burst, &data[begin], len);

	return etoa;
}

static float
burst_energy(struct osmo_cxvec *burst)
{
	int i;
	int b = (burst->len >> 5); /* exclude the borders */
	float e = 0.0f;
	for (i=b; i<burst->len-b; i++)
		e += osmo_normsqf(burst->data[i]);
	e /= burst->len;
	return e;
}

static inline float
to_hz(float f_rps)
{
	return (GMR1_SYM_RATE * f_rps) / (2.0f * M_PIf);
}

static inline float
to_db(float v)
{
	return 10.0f * log10f(v);
}


/* TCH9 ------------------------------------------------------------------- */

struct tch9_sink_params {
	struct app_state *as;
	int chan_id;
	uint32_t fn;
	uint64_t align;
	float freq_err;
	int tn;
	int ciph;
	uint8_t kc[8];
};

struct tch9_sink_priv {
	/* App */
	struct app_state *as;
	int chan_id;

	/* Alignement time/freq */
	uint32_t fn;
	uint64_t align;
	int align_err;
	float freq_err;

	int tn;

	int aligned;

	/* End detection */
	int bad_crc;

	/* Cipher */
	int ciph;
	uint8_t kc[8];

	/* Interleaver */
	struct gmr1_interleaver il;

	/* Output data */
	FILE *data;
};

static int
tch9_sink_init(struct sample_actor *sa, void *params_ptr)
{
	struct tch9_sink_priv *priv = sa->priv;
	struct tch9_sink_params *params = params_ptr;

	priv->as = params->as;
	priv->chan_id = params->chan_id;

	priv->fn = params->fn;
	priv->align = params->align;
	priv->freq_err = params->freq_err;

	priv->tn = params->tn;

	priv->ciph = params->ciph;
	memcpy(priv->kc, params->kc, 8);

	gmr1_interleaver_init(&priv->il, 3, 648);

	return 0;
}

static void
tch9_sink_fini(struct sample_actor *sa)
{
	struct tch9_sink_priv *priv = sa->priv;

	gmr1_interleaver_fini(&priv->il);

	if (priv->data)
		fclose(priv->data);
}

static int
tch9_sink_work(struct sample_actor *sa,
               float complex *data, unsigned int data_len)
{
	struct tch9_sink_priv *priv = sa->priv;
	struct osmo_cxvec _burst, *burst = &_burst;
	int sps, base_align, frame_len;
	int e_toa, rv, sync_id, crc, conv;
	sbit_t ebits[662], bits_sacch[10], bits_status[4];
	ubit_t ciph[658];
	float toa, freq_err;

	/* Get quick params */
	sps = priv->as->sps;
	frame_len = sps * 39 * 24;
	base_align = sps * TCH9_MARGIN;

	/* If not aligned ... do that first */
	if (!priv->aligned) {
		/* Basically we want to have :
		 * |#|Frame|#| with the # being margin.
		 */

		uint64_t target = priv->align - sps * TCH9_MARGIN;
		int discard;

		if (target < sa->time) {
			target += frame_len;
			priv->fn += 1;
			priv->align += frame_len;
		}

		discard = target - sa->time;

		if (discard > data_len)
			return data_len;

		priv->aligned = 1;

		return discard;
	}

	/* Check we have enough data (frame_len) */
	if (data_len < (frame_len + 2*TCH9_MARGIN))
		return 0;

	/* Map potential burst */
	e_toa = burst_map(burst, data, data_len, base_align, sps,
	                  &gmr1_nt9_burst, priv->tn, sps + (sps/2));
	if (e_toa < 0)
		return e_toa;

	/* Demodulate burst */
	rv = gmr1_pi4cxpsk_demod(
		&gmr1_nt9_burst,
		burst, sps, -priv->freq_err,
		ebits, &sync_id, &toa, &freq_err
	);
	if (rv < 0)
		return rv;

	fprintf(stderr, "[.]   %s\n", sync_id ? "TCH9" : "FACCH9");
	fprintf(stderr, "toa=%.1f, sync_id=%d\n", toa, sync_id);

	/* Track frequency */
	priv->freq_err += freq_err;

	/* Process depending on type */
	if (!sync_id) { /* FACCH9 */
		uint8_t l2[38];

		/* Generate cipher stream */
		gmr1_a5(priv->ciph, priv->kc, priv->fn, 658, ciph, NULL);

		/* Decode */
		crc = gmr1_facch9_decode(l2, bits_sacch, bits_status, ebits, ciph, &conv);
		fprintf(stderr, "crc=%d, conv=%d\n", crc, conv);

		/* Send to GSMTap if correct */
		if (!crc)
			gsmtap_sendmsg(priv->as->gti, gmr1_gsmtap_makemsg(
				GSMTAP_GMR1_TCH9 | GSMTAP_GMR1_FACCH,
				priv->as->arfcn[priv->chan_id],
				priv->fn, priv->tn, l2, 38));

		/* Detect end */
		if (crc)
			if (priv->bad_crc++ > 10)
				return -1;
	} else { /* TCH9 */
		uint8_t l2[60];
		int i, s = 0;

		/* Generate cipher stream */
		gmr1_a5(priv->ciph, priv->kc, priv->fn, 658, ciph, NULL);

		for (i=0; i<662; i++)
			s += ebits[i] < 0 ? -ebits[i] : ebits[i];
		s /= 662;

		/* Decode */
		gmr1_tch9_decode(l2, bits_sacch, bits_status, ebits, GMR1_TCH9_9k6, ciph, &priv->il, &conv);
		fprintf(stderr, "fn=%d, conv9=%d, avg=%d\n", priv->fn, conv, s);

		/* Forward to GSMTap (no CRC to validate :( ) */
		gsmtap_sendmsg(priv->as->gti, gmr1_gsmtap_makemsg(
			GSMTAP_GMR1_TCH9,
			priv->as->arfcn[priv->chan_id],
			priv->fn, priv->tn, l2, 60));

		/* Save to file */
		{
			if (!priv->data) {
				char fname[256];
				sprintf(fname, "/tmp/gmr1_csd_%d_%d_%d.dat", priv->as->arfcn[priv->chan_id], priv->tn, priv->fn);
				priv->data = fopen(fname, "wb");
			}
			fwrite(l2, 60, 1, priv->data);
		}
	}

	/* Accumulate alignement error */
	fprintf(stderr, "toa=%f | %d %d %d\n", toa, e_toa, ((int)roundf(toa)) - e_toa, priv->align_err);
	priv->align_err += ((int)roundf(toa)) - e_toa;

	/* Take align_err into account */
	if (priv->align_err > 4) {
		frame_len++;
		priv->align_err -= 4;
		fprintf(stderr, ">>>> REALIGN +++ %d\n", priv->align_err);
	} else if (priv->align_err < -4) {
		frame_len--;
		priv->align_err += 4;
		fprintf(stderr, ">>>> REALIGN --- %d\n", priv->align_err);
	}

	/* Done, go to next frame */
	priv->fn += 1;

	return frame_len;
}

const struct sample_actor_desc tch9_sink = {
	.init = tch9_sink_init,
	.fini = tch9_sink_fini,
	.work = tch9_sink_work,
	.priv_size = sizeof(struct tch9_sink_priv),
};


/* TCH3 ------------------------------------------------------------------- */

struct tch3_sink_params {
	struct app_state *as;
	int chan_id;
	uint32_t fn;
	uint64_t align;
	float freq_err;
	int tn;
	int dkab_pos;
	float ref_energy;
};

struct tch3_sink_priv {
	/* App */
	struct app_state *as;
	int chan_id;

	/* Alignement time/freq */
	uint32_t fn;
	uint64_t align;
	int align_err;
	float freq_err;

	int tn;
	int dkab_pos;

	int aligned;

	/* Energy thresholds */
	float energy_dkab;
	float energy_burst;

	int weak_cnt;

	/* FAACH state */
	sbit_t ebits[104*4];
	uint32_t bi_fn[4];
	int sync_id;
	int burst_cnt;

	int followed;

	/* Cipher */
	int ciph;
	uint8_t kc[8];

	/* Output data */
	FILE *data;
};

static int
tch3_sink_init(struct sample_actor *sa, void *params_ptr)
{
	struct tch3_sink_priv *priv = sa->priv;
	struct tch3_sink_params *params = params_ptr;

	priv->as = params->as;
	priv->chan_id = params->chan_id;

	priv->fn = params->fn;
	priv->align = params->align;
	priv->freq_err = params->freq_err;

	priv->tn = params->tn;
	priv->dkab_pos = params->dkab_pos;

	priv->energy_burst = params->ref_energy * 0.75f;
	priv->energy_dkab  = priv->energy_burst / 8.0f; /* ~ 8 times less pwr */

	priv->weak_cnt = 0;

	priv->ciph = 0;

	return 0;
}

static void
tch3_sink_fini(struct sample_actor *sa)
{
	struct tch3_sink_priv *priv = sa->priv;

	if (priv->data)
		fclose(priv->data);
}

static int
_rx_tch3_dkab(struct sample_actor *sa, struct osmo_cxvec *burst, float *toa)
{
	struct tch3_sink_priv *priv = sa->priv;
	sbit_t ebits[8];
	int rv;

	fprintf(stderr, "[.]   DKAB\n");

	rv = gmr1_dkab_demod(burst, priv->as->sps, -priv->freq_err, priv->dkab_pos, ebits, toa);

	fprintf(stderr, "toa=%f\n", *toa);

	return rv;
}

static inline int
_facch3_is_ass_cmd_1(const uint8_t *l2)
{
	return (l2[3] == 0x06) && (l2[4] == 0x2e);
}

static void
_facch3_ass_cmd_1_parse(const uint8_t *l2, int *arfcn, int *rx_tn)
{
	*rx_tn = ((l2[5] & 0x03) << 3) | (l2[6] >> 5);
	*arfcn = ((l2[6] & 0x1f) << 6) | (l2[7] >> 2);
}

static int
_rx_tch3_facch_flush(struct sample_actor *sa)
{
	struct tch3_sink_priv *priv = sa->priv;
	ubit_t ciph[96*4];
	uint8_t l2[10];
	ubit_t sbits[8*4];
	int sps, base_align;
	int i, crc, conv;

	/* Get quick params */
	sps = priv->as->sps;
	base_align = sps * TCH3_MARGIN;

	/* Cipher stream */
	for (i=0; i<4; i++)
		gmr1_a5(priv->ciph, priv->kc, priv->bi_fn[i], 96, ciph+(96*i), NULL);

	/* Decode the burst */
	crc = gmr1_facch3_decode(l2, sbits, priv->ebits, ciph, &conv);

	fprintf(stderr, "crc=%d, conv=%d\n", crc, conv);

	/* Retry with ciphering ? */
#if 0
	if (!st->ciph && crc) {
		ciph = _ciph;
		for (i=0; i<4; i++)
			gmr1_a5(1, cd->kc, st->bi_fn[i], 96, ciph+(96*i), NULL);

		crc = gmr1_facch3_decode(l2, sbits, st->ebits, ciph, &conv);

		fprintf(stderr, "crc=%d, conv=%d\n", crc, conv);

		if (!crc)
			st->ciph = 1;
	}
#endif

	/* Send to GSMTap if correct */
	if (!crc)
		gsmtap_sendmsg(priv->as->gti, gmr1_gsmtap_makemsg(
			GSMTAP_GMR1_TCH3 | GSMTAP_GMR1_FACCH,
			priv->as->arfcn[priv->chan_id],
			priv->fn-3, priv->tn, l2, 10));

	/* Parse for assignement */
	if (!crc && _facch3_is_ass_cmd_1(l2) && !priv->followed)
	{
		struct tch9_sink_params p;
		struct sample_actor *nsa;
		int arfcn, tn;
		int i;

		/* Extract TN & ARFCN */
		_facch3_ass_cmd_1_parse(l2, &arfcn, &tn);

		/* Debug print */
		fprintf(stderr, "[+] TCH9 assigned on ARFCN %d TN %d\n",
			arfcn, tn);

		/* Find matching channel ID */
		for (i=0; i<priv->as->n_chans; i++)
			if (priv->as->arfcn[i] == arfcn)
				break;

		if (i == priv->as->n_chans) {
			fprintf(stderr, "No data stream available for that ARFCN\n");
			goto nofollow;
		}

		/* Start TCH9 task */
		p.as       = priv->as;
		p.chan_id  = i;
		p.fn       = priv->fn;
		p.align    = sa->time + base_align;
		p.freq_err = priv->freq_err;
		p.tn       = tn;
		p.ciph     = priv->ciph;
		memcpy(p.kc, priv->kc, 8);

		nsa = sbuf_add_consumer(priv->as->buf, p.chan_id, &tch9_sink, &p);
		if (!nsa) {
			fprintf(stderr, "[!] Failed to create TCH9 sink for stream #%d\n", p.chan_id);
			return -ENOMEM;
		}

		/* Stop current TCH3 task */
			/* FIXME should only happen later, ass message spans several
			 * FACCH3 ... */
		priv->followed = 1;
	}
nofollow:

	/* Clear state */
	priv->sync_id ^= 1;
	priv->burst_cnt = 0;
	memset(priv->bi_fn, 0xff, sizeof(uint32_t) * 4);
	memset(priv->ebits, 0x00, sizeof(sbit_t) * 104 * 4);

	/* Done */
	return 0;
}

static int
_rx_tch3_facch(struct sample_actor *sa, struct osmo_cxvec *burst, float *toa)
{
	struct tch3_sink_priv *priv = sa->priv;
	sbit_t ebits[104];
	int rv, bi, sync_id;

	/* Burst index */
	bi = priv->fn & 3;

	/* Debug */
	fprintf(stderr, "[.]   FACCH3 (bi=%d)\n", bi);

	/* Demodulate burst */
	rv = gmr1_pi4cxpsk_demod(
		&gmr1_nt3_facch_burst,
		burst, priv->as->sps, -priv->freq_err,
		ebits, &sync_id, toa, NULL
	);

	fprintf(stderr, "toa=%.1f, sync_id=%d\n", *toa, sync_id);

	/* Does this burst belong with previous ones ? */
	if (sync_id != priv->sync_id)
		_rx_tch3_facch_flush(sa);

	/* Store this burst */
	memcpy(&priv->ebits[104*bi], ebits, sizeof(sbit_t) * 104);
	priv->sync_id = sync_id;
	priv->bi_fn[bi] = priv->fn;
	priv->burst_cnt += 1;

	/* Is it time to flush ? */
	if (priv->burst_cnt == 4)
		_rx_tch3_facch_flush(sa);

	return rv;
}

static int
_rx_tch3_speech(struct sample_actor *sa, struct osmo_cxvec *burst, float *toa)
{
	struct tch3_sink_priv *priv = sa->priv;
	sbit_t ebits[212];
	ubit_t sbits[4], ciph[208];
	uint8_t frame0[10], frame1[10];
	int rv, conv[2];

	/* Debug */
	fprintf(stderr, "[.]   TCH3\n");

	/* Demodulate burst */
	rv = gmr1_pi4cxpsk_demod(
		&gmr1_nt3_speech_burst,
		burst, priv->as->sps, -priv->freq_err,
		ebits, NULL, toa, NULL
	);
	if (rv < 0)
		return rv;

	/* Decode it */
	gmr1_a5(priv->ciph, priv->kc, priv->fn, 208, ciph, NULL);

	gmr1_tch3_decode(frame0, frame1, sbits, ebits, ciph, 0, &conv[0], &conv[1]);

	/* More debug */
	fprintf(stderr, "toa=%.1f\n", *toa);
	fprintf(stderr, "conv=%3d,%3d\n", conv[0], conv[1]);
	fprintf(stderr, "frame0=%s\n", osmo_hexdump_nospc(frame0, 10));
	fprintf(stderr, "frame1=%s\n", osmo_hexdump_nospc(frame1, 10));

	/* Save to file */
	{
		if (!priv->data) {
			char fname[256];
			sprintf(fname, "/tmp/gmr1_speech_%d_%d_%d.dat", priv->as->arfcn[priv->chan_id], priv->tn, priv->fn);
			priv->data = fopen(fname, "wb");
		}
		fwrite(frame0, 10, 1, priv->data);
		fwrite(frame1, 10, 1, priv->data);
	}

	return 0;
}

static int
tch3_sink_work(struct sample_actor *sa,
               float complex *data, unsigned int data_len)
{
	static struct gmr1_pi4cxpsk_burst *burst_types[] = {
		&gmr1_nt3_facch_burst,
		&gmr1_nt3_speech_burst,
		NULL
	};

	struct tch3_sink_priv *priv = sa->priv;
	struct osmo_cxvec _burst, *burst = &_burst;
	int sps, base_align, frame_len;
	int e_toa, btid, sid;
	float be, det, toa;
	int rv;

	/* Get quick params */
	sps = priv->as->sps;
	frame_len = sps * 39 * 24;
	base_align = sps * TCH3_MARGIN;

	/* If not aligned ... do that first */
	if (!priv->aligned) {
		/* Basically we want to have :
		 * |#|Frame|#| with the # being margin.
		 */

		uint64_t target = priv->align - sps * TCH3_MARGIN;
		int discard;

		if (target < sa->time) {
			target += frame_len;
			priv->fn += 1;
			priv->align += frame_len;
		}

		discard = target - sa->time;

		if (discard > data_len)
			return data_len;

		priv->aligned = 1;

		return discard;
	}

	/* Check we have enough data (frame_len) */
	if (data_len < (frame_len + 2*TCH3_MARGIN))
		return 0;

	/* Map potential burst (use FACCH3 as reference) */
	e_toa = burst_map(burst, data, data_len, base_align, sps,
	                  &gmr1_nt3_facch_burst, priv->tn, sps + sps/2);
	if (e_toa < 0)
		return e_toa;

	/* Burst energy (and check for DKAB) */
	be = burst_energy(burst);

	det = (priv->energy_dkab + priv->energy_burst) / 4.0f;

	if (be < det) {
		rv = _rx_tch3_dkab(sa, burst, &toa);

		if (rv < 0)
			return rv;
		else if (rv == 1) {
			if (priv->weak_cnt++ > 8) {
				fprintf(stderr, "END @%d\n", priv->fn);
				return -1;
			}
		} else {
			priv->energy_dkab =
				(0.1f * be) +
				(0.9f * priv->energy_dkab);
		}

		goto done;
	} else
		priv->weak_cnt = 0;

	priv->energy_burst =
		(0.1f * be) +
		(0.9f * priv->energy_burst);

	/* Detect burst type */
	rv = gmr1_pi4cxpsk_detect(
		burst_types, (float)e_toa,
		burst, sps, -priv->freq_err,
		&btid, &sid, &toa
	);
	if (rv < 0)
		return rv;

	/* Delegate appropriately */
	if (btid == 0)
		rv = _rx_tch3_facch(sa, burst, &toa);
	else
		rv = _rx_tch3_speech(sa, burst, &toa);

	if (rv < 0)
		return rv;

done:
	/* Accumulate alignement error */
	fprintf(stderr, "toa=%f | %d %d %d\n", toa, e_toa, ((int)roundf(toa)) - e_toa, priv->align_err);
	priv->align_err += ((int)roundf(toa)) - e_toa;

	/* Take align_err into account */
	if (priv->align_err > 4) {
		frame_len++;
		priv->align_err -= 4;
		fprintf(stderr, ">>>> REALIGN +++ %d\n", priv->align_err);
	} else if (priv->align_err < -4) {
		frame_len--;
		priv->align_err += 4;
		fprintf(stderr, ">>>> REALIGN --- %d\n", priv->align_err);
	}

	/* Done, go to next frame */
	priv->fn += 1;

	return frame_len;
}

const struct sample_actor_desc tch3_sink = {
	.init = tch3_sink_init,
	.fini = tch3_sink_fini,
	.work = tch3_sink_work,
	.priv_size = sizeof(struct tch3_sink_priv),
};


/* BCCH / CCCH ------------------------------------------------------------ */

struct bcch_sink_params {
	struct app_state *as;
	int chan_id;
	uint64_t align;
	float freq_err;
};

struct bcch_sink_priv {
	struct app_state *as;
	int chan_id;

	uint64_t align;
	int align_err;
	float freq_err;

	uint32_t fn;
	int sa_sirfn_delay;
	int sa_bcch_stn;

	float bcch_energy;
	int bcch_err;

	int la_arfcn;
	int la_tn;
	int la_dkab_pos;

	int aligned;
};

static int
bcch_sink_init(struct sample_actor *sa, void *params_ptr)
{
	struct bcch_sink_priv *priv = sa->priv;
	struct bcch_sink_params *params = params_ptr;

	priv->as = params->as;
	priv->chan_id = params->chan_id;

	priv->align = params->align;
	priv->freq_err = params->freq_err;

	return 0;
}

static void
bcch_sink_fini(struct sample_actor *sa)
{
	/* struct bcch_sink_priv *priv = sa->priv; */

	/* Nothing to do */
}

static int
_bcch_tdma_align(struct bcch_sink_priv *priv, uint8_t *l2)
{
	int sa_sirfn_delay, sa_bcch_stn;
	int superframe_num, multiframe_num, mffn_high_bit;
	int fn;

	/* Check if it's a SI1 */
	if ((l2[0] & 0xf8) != 0x08)
		return 0;

	/* Check if it contains a Seg 2A bis */
	if ((l2[9] & 0xfc) != 0x80)
		return 0;

	/* Retrieve SA_SIRFN_DELAY, SA_BCCH_STN,
	 * Superframe number, Multiframe number, MFFN high bit */
	sa_sirfn_delay =  (l2[10] >> 3) & 0x0f;
	sa_bcch_stn    = ((l2[10] << 2) & 0x1c) | (l2[11] >> 6);

	superframe_num = ((l2[11] & 0x3f) << 7) | (l2[12] >> 1);
	multiframe_num = ((l2[12] & 0x01) << 1) | (l2[13] >> 7);
	mffn_high_bit  = ((l2[13] & 0x40) >> 6);

	/* Compute frame number */
	fn = (superframe_num << 6) |
	     (multiframe_num << 4) |
	     (mffn_high_bit << 3) |
	     ((2 + sa_sirfn_delay) & 7);

	/* Fix SDR alignement */
	priv->align_err += (priv->sa_bcch_stn - sa_bcch_stn) * 39 * priv->as->sps;

	/* Align TDMA */
	priv->fn = fn;
	priv->sa_sirfn_delay = sa_sirfn_delay;
	priv->sa_bcch_stn = sa_bcch_stn;

	return 0;
}

static int
_rx_bcch(struct sample_actor *sa,
         float complex *data, unsigned int data_len)
{
	struct bcch_sink_priv *priv = sa->priv;
	struct osmo_cxvec _burst, *burst = &_burst;
	sbit_t ebits[424];
	uint8_t l2[24];
	float freq_err, toa;
	int sps, base_align;
	int rv, crc, conv, e_toa;

	/* Get quick params */
	sps = priv->as->sps;
	base_align = sps * BCCH_MARGIN;

	/* Debug */
	fprintf(stderr, "[.]   BCCH\n");

	/* Demodulate burst */
	e_toa = burst_map(burst, data, data_len, base_align, sps,
	                  &gmr1_bcch_burst, priv->sa_bcch_stn, 20 * sps);
	if (e_toa < 0)
		return e_toa;

	rv = gmr1_pi4cxpsk_demod(
		&gmr1_bcch_burst,
		burst, sps, -priv->freq_err,
		ebits, NULL, &toa, &freq_err
	);

	if (rv)
		return rv;

	/* Measure energy as a reference */
	priv->bcch_energy = burst_energy(burst);

	/* Decode burst */
	crc = gmr1_bcch_decode(l2, ebits, &conv);

	fprintf(stderr, "crc=%d, conv=%d\n", crc, conv);

	/* If burst turned out OK, use data to align channel */
	if (!crc) {
		/* SDR alignement */
		priv->align_err += ((int)roundf(toa)) - e_toa;
		priv->freq_err += freq_err;

		/* Acquire TDMA alignement */
		_bcch_tdma_align(priv, l2);
	}

	/* Count errors */
	if (!crc)
		priv->bcch_err = 0;
	else
		priv->bcch_err++;

	/* Send to GSMTap if correct */
	if (!crc)
		gsmtap_sendmsg(priv->as->gti, gmr1_gsmtap_makemsg(
			GSMTAP_GMR1_BCCH,
			priv->as->arfcn[priv->chan_id],
			priv->fn, priv->sa_bcch_stn, l2, 24));

	return 0;
}

static inline int
_ccch_is_imm_ass(const uint8_t *l2)
{
	return (l2[1] == 0x06) && (l2[2] == 0x3f);
}

static void
_ccch_imm_ass_parse(const uint8_t *l2, int *arfcn, int *rx_tn, int *p)
{
	*p = (l2[8] & 0xfc) >> 2;
	*rx_tn = ((l2[8] & 0x03) << 3) | (l2[9] >> 5);
	*arfcn = ((l2[9] & 0x1f) << 6) | (l2[10] >> 2);
}

static int
_rx_ccch(struct sample_actor *sa,
         float complex *data, unsigned int data_len)
{
	struct bcch_sink_priv *priv = sa->priv;
	struct osmo_cxvec _burst, *burst = &_burst;
	sbit_t ebits[432];
	uint8_t l2[24];
	int sps, base_align;
	int rv, crc, conv, e_toa;

	/* Get quick params */
	sps = priv->as->sps;
	base_align = sps * BCCH_MARGIN;

	/* Map potential burst */
	e_toa = burst_map(burst, data, data_len, base_align, sps,
	                  &gmr1_dc6_burst, priv->sa_bcch_stn, 10 * sps);
	if (e_toa < 0)
		return e_toa;

	/* Energy detection */
	if (burst_energy(burst) < (priv->bcch_energy / 2.0f))
		return 0; /* Nothing to do */

	/* Debug */
	fprintf(stderr, "[.]   CCCH\n");

	/* Demodulate burst */
	rv = gmr1_pi4cxpsk_demod(
		&gmr1_dc6_burst,
		burst, sps, -priv->freq_err,
		ebits, NULL, NULL, NULL
	);

	if (rv)
		return rv;

	/* Decode burst */
	crc = gmr1_ccch_decode(l2, ebits, &conv);

	fprintf(stderr, "crc=%d, conv=%d\n", crc, conv);

	/* Check for IMM.ASS */
	if (!crc && _ccch_is_imm_ass(l2)) {
		struct tch3_sink_params p;
		struct sample_actor *nsa;
		int arfcn, tn, dkab_pos;
		int i;

		/* Parse ASS */
		_ccch_imm_ass_parse(l2, &arfcn, &tn, &dkab_pos);

		/* Quick & Dirty check for dupes */
		if ((priv->la_arfcn == arfcn) &&
		    (priv->la_tn == tn) &&
		    (priv->la_dkab_pos == dkab_pos))
			goto nofollow;

		priv->la_arfcn = arfcn;
		priv->la_tn = tn;
		priv->la_dkab_pos = dkab_pos;

		/* Debug print */
		fprintf(stderr, "[+] TCH3 assigned on ARFCN %d TN %d DKAB %d\n",
			arfcn, tn, dkab_pos);

		/* Find matching channel ID */
		for (i=0; i<priv->as->n_chans; i++)
			if (priv->as->arfcn[i] == arfcn)
				break;

		if (i == priv->as->n_chans) {
			fprintf(stderr, "No data stream available for that ARFCN\n");
			goto nofollow;
		}

		/* Start TCH3 task */
		p.as       = priv->as;
		p.chan_id  = i;
		p.fn       = priv->fn;
		p.align    = sa->time + base_align;
		p.freq_err = priv->freq_err;
		p.tn       = tn;
		p.dkab_pos = dkab_pos;
		p.ref_energy = priv->bcch_energy / 2.0f;

		nsa = sbuf_add_consumer(priv->as->buf, p.chan_id, &tch3_sink, &p);
		if (!nsa) {
			fprintf(stderr, "[!] Failed to create TCH3 sink for stream #%d\n", p.chan_id);
			return -ENOMEM;
		}
	}
nofollow:

	/* Send to GSMTap if correct */
	if (!crc)
		gsmtap_sendmsg(priv->as->gti, gmr1_gsmtap_makemsg(
			GSMTAP_GMR1_CCCH,
			priv->as->arfcn[priv->chan_id],
			priv->fn, priv->sa_bcch_stn, l2, 24));

	return 0;
}

static int
bcch_sink_work(struct sample_actor *sa,
               float complex *data, unsigned int data_len)
{
	struct bcch_sink_priv *priv = sa->priv;
	int sps, base_align, frame_len, sirfn;

	/* Get quick params */
	sps = priv->as->sps;
	frame_len = sps * 39 * 24;
	base_align = sps * BCCH_MARGIN;

	/* If not aligned ... do that first */
	if (!priv->aligned) {
		/* Basically we want to have :
		 * |#|Frame0|Frame1|#|  with the # being
		 * margin blocks
		 */

		uint64_t target = priv->align - sps * BCCH_MARGIN;
		int discard;

		if (target < sa->time) {
			target += frame_len;
			priv->align += frame_len;
		}

		discard = target - sa->time;

		if (discard > data_len)
			return data_len;

		priv->aligned = 1;

		return discard;
	}

	/* Check we have enough data (2*BCCH_MARGIN + 2*frame_len) */
	if (data_len < (2*BCCH_MARGIN*sps + 2*frame_len))
		return 0;

	/* Debug print */
	fprintf(stderr, "[-]  FN: %6d (@%d:%llu)\n",
		priv->fn, priv->chan_id, (long long unsigned int)(sa->time + base_align));

	/* SI relative frame number inside an hyperframe */
	sirfn = (priv->fn - priv->sa_sirfn_delay) & 63;

	/* BCCH */
	if (sirfn % 8 == 2)
		_rx_bcch(sa, data, data_len);

	if (priv->bcch_err > 10)
		return -1;

	/* CCCH */
	if ((sirfn % 8 != 0) && (sirfn % 8 != 2))
		_rx_ccch(sa, data, data_len);

	/* Next frame */
	priv->fn += 1;

	frame_len += priv->align_err;
	priv->align_err = 0;

	return frame_len;
}

const struct sample_actor_desc bcch_sink = {
	.init = bcch_sink_init,
	.fini = bcch_sink_fini,
	.work = bcch_sink_work,
	.priv_size = sizeof(struct bcch_sink_priv),
};


/* FCCH ------------------------------------------------------------------- */

#define START_DISCARD	8000


struct fcch_sink_params {
	struct app_state *as;
	int chan_id;
};

struct fcch_sink_priv {
	struct app_state *as;
	int chan_id;

	enum {
		FCCH_STATE_SINGLE = 0,
		FCCH_STATE_MULTI = 1,
	} state;

	float freq_err;
};

static int
fcch_sink_init(struct sample_actor *sa, void *params_ptr)
{
	struct fcch_sink_priv *priv = sa->priv;
	struct fcch_sink_params *params = params_ptr;

	priv->as = params->as;
	priv->chan_id = params->chan_id;

	return 0;
}

static void
fcch_sink_fini(struct sample_actor *sa)
{
	/* struct fcch_sink_priv *priv = sa->priv; */

	/* Nothing to do */
}

static int
_fcch_sink_work_single(struct sample_actor *sa,
                       float complex *data, unsigned int data_len)
{
	struct fcch_sink_priv *priv = sa->priv;
	struct osmo_cxvec _win, *win = &_win;
	int sps, win_len, base_align, toa;
	int rv;

	/* Params */
	sps = priv->as->sps;
	base_align = START_DISCARD;

	/* Get large enough window (330 ms) */
	win_len = ((330 * GMR1_SYM_RATE * sps) / 1000);

	rv = win_map(win, data, data_len, base_align, win_len);
	if (rv < 0)
		return 0; /* Not enough data yet */

	/* FCCH rough retect */
	rv = gmr1_fcch_rough(win, sps, 0.0f, &toa);
	if (rv < 0) {
		fprintf(stderr, "[!] Error during FCCH rough acquisition (%d)\n", rv);
		return rv;
	}

	/* Fine FCCH detection */
	win_map(win, data, data_len, base_align + toa, GMR1_FCCH_SYMS * sps);

	rv = gmr1_fcch_fine(win, sps, 0.0f, &toa, &priv->freq_err);
	if (rv < 0) {
		fprintf(stderr, "[!] Error during FCCH fine acquisition (%d)\n", rv);
		return rv;
	}

	base_align += toa;

	/* Debug print */
	fprintf(stderr, "[+] Primary FCCH found @%d:%d [freq_err = %.1f Hz]\n",
			priv->chan_id, base_align, to_hz(priv->freq_err));

	/* Take a safety margin for next step */
	base_align -= GMR1_FCCH_SYMS * sps;
	if (base_align < 0)
		base_align = 0;

	/* Next step is multi */
	priv->state = FCCH_STATE_MULTI;

	/* Done. We discard what we won't use */
	return base_align;
}

static int
_fcch_sink_work_multi(struct sample_actor *sa,
                      float complex *data, unsigned int len)
{
	struct fcch_sink_priv *priv = sa->priv;
	struct osmo_cxvec _win, *win = &_win;
	int win_len, sps, mtoa[16], n_fcch;
	float ref_snr = 0.0f, ref_freq_err = 0.0f;
	int rv, i, j;

	/* Params */
	sps = priv->as->sps;

	/* Get large enough window */
	win_len = ((650 * GMR1_SYM_RATE * sps) / 1000);

	rv = win_map(win, data, len, 0, win_len);
	if (rv < 0)
		return 0; /* Not enough data yet */

	/* Multi FCCH detection */
	rv = gmr1_fcch_rough_multi(win, sps, -priv->freq_err, mtoa, 16);
	if (rv < 0) {
		fprintf(stderr, "[!] Error during FCCH rough mutli-acquisition (%d)\n", rv);
		return rv;
	}

	n_fcch = rv;

	/* Check each of them for validity */
	for (i=0, j=0; i<n_fcch; i++) {
		float freq_err, e_fcch, e_cich, snr;
		int toa;

		/* Perform fine acquisition */
		win_map(win, data, len,
		        mtoa[i], GMR1_FCCH_SYMS * sps);

		rv = gmr1_fcch_fine(win, sps, -priv->freq_err, &toa, &freq_err);
		if (rv) {
			fprintf(stderr, "[!] Error during FCCH fine acquisition (%d)\n", rv);
			return rv;
		}

		/* Compute SNR (comparing energy with neighboring CICH) */
		win_map(win, data, len,
		        mtoa[i] + toa + 5 * sps,
		        (117 - 10) * sps);

		e_fcch = burst_energy(win);

		win_map(win, data, len,
		        mtoa[i] + toa + (5 + 117) * sps,
		        (117 - 10) * sps);

		e_cich = burst_energy(win);

		snr = e_fcch / e_cich;

                /* Check against strongest */
		if (i==0) {
			/* This _is_ the reference */
			ref_snr = snr;
			ref_freq_err = freq_err;
		} else {
			/* Check if SNR is 'good enough' */
			if (snr < 2.0f)
				continue;

			if (snr < (ref_snr / 6.0f))
				continue;

			/* Check if frequency error is not too "off" */
			if (to_hz(fabs(ref_freq_err - freq_err)) > 500.0f)
				continue;
		}

		/* Debug print */
		fprintf(stderr, "[.]  Potential FCCH @%d:%d [snr = %.1f dB, freq_err = %.1f Hz]\n",
			priv->chan_id,
			(int)(sa->time + mtoa[i] + toa),
			to_db(snr),
			to_hz(freq_err + priv->freq_err)
		);

		/* Save it */
		mtoa[j++] = mtoa[i] + toa;
	}

	n_fcch = j;

	/* Create processing tasks for survivors */
	for (i=0; i<n_fcch; i++) {
		struct bcch_sink_params p = {
			.as = priv->as,
			.chan_id = priv->chan_id,
			.align = sa->time + mtoa[i],
			.freq_err = priv->freq_err,
		};
		sa = sbuf_add_consumer(priv->as->buf, priv->chan_id, &bcch_sink, &p);
		if (!sa) {
			fprintf(stderr, "[!] Failed to create BCCH sink for stream #%d\n", i);
			return -ENOMEM;
		}
	}

	/* All done here */
	return -1;
}

static int
fcch_sink_work(struct sample_actor *sa,
               float complex *data, unsigned int len)
{
	struct fcch_sink_priv *priv = sa->priv;

	if (priv->state == FCCH_STATE_SINGLE)
		return _fcch_sink_work_single(sa, data, len);
	else if (priv->state == FCCH_STATE_MULTI)
		return _fcch_sink_work_multi(sa, data, len);
	else
		return -EINVAL;
}

const struct sample_actor_desc fcch_sink = {
	.init = fcch_sink_init,
	.fini = fcch_sink_fini,
	.work = fcch_sink_work,
	.priv_size = sizeof(struct fcch_sink_priv),
};


/* Main ------------------------------------------------------------------- */

int main(int argc, char *argv[])
{
	struct app_state _as, *as = &_as;
	int rv = 0, i;

	/* Init app state */
	memset(as, 0x00, sizeof(struct app_state));

	/* Args */
	if (argc < 3) {
		fprintf(stderr, "Usage: %s sps arfcn1:file1 [arfcn2:file2] ...\n", argv[0]);
		return -EINVAL;
	}

	as->n_chans = argc - 2;

	as->sps = atoi(argv[1]);
	if (as->sps < 1 || as->sps > 16) {
		fprintf(stderr, "[!] sps must be withing [1,16]\n");
		return -EINVAL;
	}

	/* Init GSMTap */
	as->gti = gsmtap_source_init("127.0.0.1", GSMTAP_UDP_PORT, 0);
	gsmtap_source_add_sink(as->gti);

	/* Buffer */
	as->buf = sbuf_alloc(as->n_chans);
	if (!as->buf) {
		rv = -ENOMEM;
		goto err;
	}

	/* Parse arguments */
	for (i=0; i<as->n_chans; i++)
	{
		char *d;

		d = strchr(argv[i+2], ':');
		if (!d) {
			fprintf(stderr, "[!] Arguments must be of the form arfcn:filename\n");
			rv = -EINVAL;
			goto err;
		}

		*d = '\0';

		as->arfcn[i] = atoi(argv[i+2]);
		as->filename[i] = d+1;
	}

	/* Create all the sources */
	for (i=0; i<as->n_chans; i++) {
		struct sample_actor *sa;
		sa = sbuf_set_producer(as->buf, i, &sa_file_src, as->filename[i]);
		if (!sa) {
			fprintf(stderr, "[!] Failed to create source for stream #%d\n", i);
			rv = -EIO;
			goto err;
		}
	}

	/* Attribute single 'FCCH detect' sink to each channel */
	for (i=0; i<as->n_chans; i++) {
		struct sample_actor *sa;
		struct fcch_sink_params p = { .as = as, .chan_id = i };
		sa = sbuf_add_consumer(as->buf, i, &fcch_sink, &p);
		if (!sa) {
			fprintf(stderr, "[!] Failed to create FCCH sink for stream #%d\n", i);
			rv = -ENOMEM;
			goto err;
		}
	}

	/* Go forth and process ! */
	sbuf_work(as->buf);

	/* Done ! */
	rv = 0;

	/* Clean up */
err:
	sbuf_free(as->buf);

	return rv;
}
