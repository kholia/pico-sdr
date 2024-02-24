/*
 * Copyright (C) Jan Hamal Dvořák <mordae@anilinux.org>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <pico/stdlib.h>
#include <pico/stdio_usb.h>
#include <pico/multicore.h>
#include <pico/util/queue.h>

#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/pll.h>
#include <hardware/vreg.h>
#include <hardware/sync.h>
#include <hardware/pio.h>
#include <hardware/pwm.h>
#include <hardware/interp.h>

#include <hardware/regs/clocks.h>
#include <hardware/structs/bus_ctrl.h>

#include <math.h>
#include <stdio.h>
#include <limits.h>
#include <stdlib.h>

#define CLK_SYS_HZ (250 * MHZ)
#define BANDWIDTH 768000
#define DECIMATION 32
#define LPF_SIZE DECIMATION

#define IQ_BLOCK_LEN 64
#define RX_BLOCK_LEN (IQ_BLOCK_LEN * DECIMATION)

#define XOR_ADDR 0x1000
#define LO_COS_ACCUMULATOR (&pio1->sm[2].pinctrl)
#define LO_SIN_ACCUMULATOR (&pio1->sm[3].pinctrl)

#define LO_BITS_DEPTH 13
#define LO_WORDS (1 << LO_BITS_DEPTH)
static uint32_t lo_cos[LO_WORDS] __attribute__((__aligned__(LO_WORDS * 4)));
static uint32_t lo_sin[LO_WORDS] __attribute__((__aligned__(LO_WORDS * 4)));

#define RX_BITS_DEPTH 12
#define RX_WORDS (1 << RX_BITS_DEPTH)
static uint32_t rx_cos[RX_WORDS] __attribute__((__aligned__(RX_WORDS * 4)));
static uint32_t rx_sin[RX_WORDS] __attribute__((__aligned__(RX_WORDS * 4)));

static_assert(RX_WORDS > RX_BLOCK_LEN, "RX buffers too short for given decimation");

/* rx -> cp -> cos -> sin -> pio_cos -> pio_sin -> rx ... */
static int dma_ch_rx = -1;
static int dma_ch_cp = -1;
static int dma_ch_cos = -1;
static int dma_ch_sin = -1;
static int dma_ch_pio_cos = -1;
static int dma_ch_pio_sin = -1;

static int dma_ch_samp_trig = -1;
static int dma_ch_samp_cos = -1;
static int dma_ch_samp_sin = -1;

static int dma_t_samp = -1;

static int dma_ch_in_cos = -1;
static int dma_ch_in_sin = -1;

static queue_t iq_queue;
static int gap = 0;

static void bias_init(int in_pin, int out_pin)
{
	gpio_disable_pulls(in_pin);
	gpio_disable_pulls(out_pin);

	pio_gpio_init(pio1, out_pin);

	gpio_set_input_hysteresis_enabled(in_pin, false);
	gpio_set_input_hysteresis_enabled(out_pin, false);
	gpio_set_drive_strength(out_pin, GPIO_DRIVE_STRENGTH_2MA);
	gpio_set_slew_rate(out_pin, GPIO_SLEW_RATE_SLOW);

	const uint16_t insn[] = {
		pio_encode_mov_not(pio_pins, pio_pins) | pio_encode_sideset(1, 1),
		pio_encode_nop() | pio_encode_sideset(1, 0) | pio_encode_delay(0),
	};

	pio_program_t prog = {
		.instructions = insn,
		.length = sizeof(insn) / sizeof(*insn),
		.origin = 16,
	};

	pio_sm_set_enabled(pio1, 0, false);
	pio_sm_restart(pio1, 0);
	pio_sm_clear_fifos(pio1, 0);

	if (pio_can_add_program(pio1, &prog))
		pio_add_program(pio1, &prog);

	pio_sm_config pc = pio_get_default_sm_config();
	sm_config_set_sideset(&pc, 1, false, true);
	sm_config_set_sideset_pins(&pc, out_pin);
	sm_config_set_in_pins(&pc, in_pin);
	sm_config_set_out_pins(&pc, out_pin, 1);
	sm_config_set_set_pins(&pc, out_pin, 1);
	sm_config_set_wrap(&pc, prog.origin, prog.origin + prog.length - 1);
	sm_config_set_clkdiv_int_frac(&pc, 1, 0);
	pio_sm_init(pio1, 0, prog.origin, &pc);

	pio_sm_set_consecutive_pindirs(pio1, 0, out_pin, 1, GPIO_OUT);

	pio_sm_set_enabled(pio1, 0, true);
}

static void watch_init(int in_pin)
{
	gpio_disable_pulls(in_pin);

	gpio_set_input_hysteresis_enabled(in_pin, false);

	const uint16_t insn[] = {
		pio_encode_in(pio_pins, 1),
	};

	pio_program_t prog = {
		.instructions = insn,
		.length = 1,
		.origin = 31,
	};

	pio_sm_set_enabled(pio1, 1, false);
	pio_sm_restart(pio1, 1);
	pio_sm_clear_fifos(pio1, 1);

	if (pio_can_add_program(pio1, &prog))
		pio_add_program(pio1, &prog);

	pio_sm_config pc = pio_get_default_sm_config();
	sm_config_set_in_pins(&pc, in_pin);
	sm_config_set_wrap(&pc, prog.origin, prog.origin + prog.length - 1);
	sm_config_set_clkdiv_int_frac(&pc, 1, 0);
	sm_config_set_fifo_join(&pc, PIO_FIFO_JOIN_RX);
	sm_config_set_in_shift(&pc, false, true, 32);
	pio_sm_init(pio1, 1, prog.origin, &pc);

	pio_sm_set_enabled(pio1, 1, true);
}

static void send_init(int out_pin)
{
	gpio_disable_pulls(out_pin);
	pio_gpio_init(pio1, out_pin);
	gpio_set_drive_strength(out_pin, GPIO_DRIVE_STRENGTH_2MA);
	gpio_set_slew_rate(out_pin, GPIO_SLEW_RATE_SLOW);

	const uint16_t insn[] = {
		pio_encode_out(pio_pins, 1) | pio_encode_sideset(1, 1),
	};

	pio_program_t prog = {
		.instructions = insn,
		.length = 1,
		.origin = 30,
	};

	pio_sm_set_enabled(pio1, 1, false);
	pio_sm_restart(pio1, 1);
	pio_sm_clear_fifos(pio1, 1);

	if (pio_can_add_program(pio1, &prog))
		pio_add_program(pio1, &prog);

	pio_sm_config pc = pio_get_default_sm_config();
	sm_config_set_sideset(&pc, 1, false, true);
	sm_config_set_sideset_pins(&pc, out_pin);
	sm_config_set_out_pins(&pc, out_pin, 1);
	sm_config_set_set_pins(&pc, out_pin, 1);
	sm_config_set_wrap(&pc, prog.origin, prog.origin + prog.length - 1);
	sm_config_set_clkdiv_int_frac(&pc, 1, 0);
	sm_config_set_fifo_join(&pc, PIO_FIFO_JOIN_TX);
	sm_config_set_out_shift(&pc, false, true, 32);
	pio_sm_init(pio1, 1, prog.origin, &pc);

	pio_sm_set_consecutive_pindirs(pio1, 1, out_pin, 1, GPIO_OUT);

	pio_sm_set_enabled(pio1, 1, true);
}

static void adder_init()
{
	const uint16_t insn[] = {
		pio_encode_jmp_y_dec(1),
		pio_encode_out(pio_pc, 2),
		pio_encode_out(pio_pc, 2),
		pio_encode_jmp_x_dec(2),

		/* Avoid Y-- on wrap. */
		pio_encode_out(pio_pc, 2),
	};

	pio_program_t prog = {
		.instructions = insn,
		.length = sizeof(insn) / sizeof(*insn),
		.origin = 0,
	};

	pio_sm_set_enabled(pio1, 2, false);
	pio_sm_set_enabled(pio1, 3, false);

	pio_sm_restart(pio1, 2);
	pio_sm_restart(pio1, 3);

	pio_sm_clear_fifos(pio1, 2);
	pio_sm_clear_fifos(pio1, 3);

	if (pio_can_add_program(pio1, &prog))
		pio_add_program(pio1, &prog);

	pio_sm_config pc = pio_get_default_sm_config();
	sm_config_set_wrap(&pc, prog.origin, prog.origin + prog.length - 1);
	sm_config_set_clkdiv_int_frac(&pc, 1, 0);
	sm_config_set_in_shift(&pc, false, true, 32);
	sm_config_set_out_shift(&pc, false, true, 32);
	pio_sm_init(pio1, 2, prog.origin + prog.length - 1, &pc);
	pio_sm_init(pio1, 3, prog.origin + prog.length - 1, &pc);

	pio_sm_set_enabled(pio1, 2, true);
	pio_sm_set_enabled(pio1, 3, true);
}

static float lo_freq_init(double req_freq)
{
	const double step_hz = (double)CLK_SYS_HZ / (LO_WORDS * 32);
	double freq = round(req_freq / step_hz) * step_hz;

	unsigned step = ((double)UINT_MAX + 1.0) / (double)CLK_SYS_HZ * freq;
	unsigned asin = UINT_MAX / 4;
	unsigned acos = 0;

	for (int i = 0; i < LO_WORDS; i++) {
		unsigned bsin = 0, bcos = 0;

		for (int j = 0; j < 32; j++) {
			bsin |= asin >> 31;
			bsin <<= 1;
			asin += step;

			bcos |= acos >> 31;
			bcos <<= 1;
			acos += step;
		}

		lo_sin[i] = bsin;
		lo_cos[i] = bcos;
	}

	return freq;
}

inline static __unused int cheap_atan2(int y, int x)
{
	if (y > 0) {
		if (x > 0) {
			if (y > x)
				return 16 << 24;
			return 0;
		} else {
			if (-x > y)
				return 48 << 24;
			return 32 << 24;
		}
	} else {
		if (x < 0) {
			if (y < x)
				return 80 << 24;
			return 64 << 24;
		} else {
			if (x > -y)
				return 112 << 24;
			return 96 << 24;
		}
	}
}

inline static __unused int cheap_angle_diff(int angle1, int angle2)
{
	int diff = angle2 - angle1;

	if (diff > INT_MAX / 2)
		return diff - INT_MAX;

	if (diff < INT_MIN / 2)
		return diff + INT_MAX;

	return diff;
}

static const uint32_t samp_insn[] __attribute__((__aligned__(16))) = {
	0x4020, /* IN X, 32 */
	0x4040, /* IN Y, 32 */
	0xe020, /* SET X, 0 */
	0xe040, /* SET Y, 0 */
};

static uint32_t null, one = 1;

static float rf_rx_start(int rx_pin, int bias_pin, float freq, int frac_num, int frac_denom)
{
	dma_ch_rx = dma_claim_unused_channel(true);
	dma_ch_cp = dma_claim_unused_channel(true);
	dma_ch_cos = dma_claim_unused_channel(true);
	dma_ch_sin = dma_claim_unused_channel(true);
	dma_ch_pio_cos = dma_claim_unused_channel(true);
	dma_ch_pio_sin = dma_claim_unused_channel(true);

	dma_ch_samp_cos = dma_claim_unused_channel(true);
	dma_ch_samp_sin = dma_claim_unused_channel(true);
	dma_ch_samp_trig = dma_claim_unused_channel(true);

	dma_t_samp = dma_claim_unused_timer(true);

	dma_channel_config dma_conf;

	/* Read received word into accumulator I. */
	dma_conf = dma_channel_get_default_config(dma_ch_rx);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_conf, false);
	channel_config_set_write_increment(&dma_conf, false);
	channel_config_set_dreq(&dma_conf, pio_get_dreq(pio1, 1, false));
	channel_config_set_chain_to(&dma_conf, dma_ch_cp);
	dma_channel_configure(dma_ch_rx, &dma_conf, LO_COS_ACCUMULATOR, &pio1->rxf[1], 1, false);

	/* Copy accumulator I to accumulator Q. */
	dma_conf = dma_channel_get_default_config(dma_ch_cp);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_conf, false);
	channel_config_set_write_increment(&dma_conf, false);
	channel_config_set_chain_to(&dma_conf, dma_ch_cos);
	dma_channel_configure(dma_ch_cp, &dma_conf, LO_SIN_ACCUMULATOR, LO_COS_ACCUMULATOR, 1,
			      false);

	/* Read lo_cos into accumulator I with XOR. */
	dma_conf = dma_channel_get_default_config(dma_ch_cos);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_conf, true);
	channel_config_set_write_increment(&dma_conf, false);
	channel_config_set_ring(&dma_conf, false, LO_BITS_DEPTH + 2);
	channel_config_set_chain_to(&dma_conf, dma_ch_sin);
	dma_channel_configure(dma_ch_cos, &dma_conf, LO_COS_ACCUMULATOR + XOR_ADDR / 4, lo_cos, 1,
			      false);

	/* Read lo_sin into accumulator Q with XOR. */
	dma_conf = dma_channel_get_default_config(dma_ch_sin);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_conf, true);
	channel_config_set_write_increment(&dma_conf, false);
	channel_config_set_ring(&dma_conf, false, LO_BITS_DEPTH + 2);
	channel_config_set_chain_to(&dma_conf, dma_ch_pio_cos);
	dma_channel_configure(dma_ch_sin, &dma_conf, LO_SIN_ACCUMULATOR + XOR_ADDR / 4, lo_sin, 1,
			      false);

	/* Copy mixed I accumulator to PIO adder I. */
	dma_conf = dma_channel_get_default_config(dma_ch_pio_cos);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_conf, false);
	channel_config_set_write_increment(&dma_conf, false);
	channel_config_set_dreq(&dma_conf, pio_get_dreq(pio1, 2, true));
	channel_config_set_chain_to(&dma_conf, dma_ch_pio_sin);
	dma_channel_configure(dma_ch_pio_cos, &dma_conf, &pio1->txf[2], LO_COS_ACCUMULATOR, 1,
			      false);

	/* Copy mixed Q accumulator to PIO adder Q. */
	dma_conf = dma_channel_get_default_config(dma_ch_pio_sin);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_conf, false);
	channel_config_set_write_increment(&dma_conf, false);
	channel_config_set_dreq(&dma_conf, pio_get_dreq(pio1, 3, true));
	channel_config_set_chain_to(&dma_conf, dma_ch_rx);
	dma_channel_configure(dma_ch_pio_sin, &dma_conf, &pio1->txf[3], LO_SIN_ACCUMULATOR, 1,
			      false);

	/* Pacing timer for the sampling script trigger channel. */
	dma_timer_set_fraction(dma_t_samp, frac_num, frac_denom);

	/* Sampling trigger channel. */
	dma_conf = dma_channel_get_default_config(dma_ch_samp_trig);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_conf, false);
	channel_config_set_write_increment(&dma_conf, false);
	channel_config_set_dreq(&dma_conf, dma_get_timer_dreq(dma_t_samp));
	channel_config_set_chain_to(&dma_conf, dma_ch_samp_cos);
	dma_channel_configure(dma_ch_samp_trig, &dma_conf, &null, &one, 1, false);

	/* Trigger I accumulator values push. */
	dma_conf = dma_channel_get_default_config(dma_ch_samp_cos);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_conf, true);
	channel_config_set_write_increment(&dma_conf, false);
	channel_config_set_ring(&dma_conf, false, 4);
	channel_config_set_chain_to(&dma_conf, dma_ch_samp_sin);
	dma_channel_configure(dma_ch_samp_cos, &dma_conf, &pio1->sm[2].instr, samp_insn, 4, false);

	/* Trigger Q accumulator values push. */
	dma_conf = dma_channel_get_default_config(dma_ch_samp_sin);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_conf, true);
	channel_config_set_write_increment(&dma_conf, false);
	channel_config_set_ring(&dma_conf, false, 4);
	channel_config_set_chain_to(&dma_conf, dma_ch_samp_trig);
	dma_channel_configure(dma_ch_samp_sin, &dma_conf, &pio1->sm[3].instr, samp_insn, 4, false);

	bias_init(rx_pin, bias_pin);
	adder_init();

	float actual = lo_freq_init(freq);

	dma_channel_start(dma_ch_rx);
	dma_channel_start(dma_ch_samp_trig);
	watch_init(rx_pin);

	return actual;
}

static void rf_rx_stop(void)
{
	pio_sm_set_enabled(pio1, 0, false);
	pio_sm_set_enabled(pio1, 1, false);
	pio_sm_set_enabled(pio1, 2, false);
	pio_sm_set_enabled(pio1, 3, false);

	pio_sm_restart(pio1, 0);
	pio_sm_restart(pio1, 1);
	pio_sm_restart(pio1, 2);
	pio_sm_restart(pio1, 3);

	pio_sm_clear_fifos(pio1, 0);
	pio_sm_clear_fifos(pio1, 1);
	pio_sm_clear_fifos(pio1, 2);
	pio_sm_clear_fifos(pio1, 3);

	sleep_us(10);

	dma_channel_abort(dma_ch_rx);
	dma_channel_abort(dma_ch_cp);
	dma_channel_abort(dma_ch_cos);
	dma_channel_abort(dma_ch_sin);
	dma_channel_abort(dma_ch_pio_cos);
	dma_channel_abort(dma_ch_pio_sin);
	dma_channel_abort(dma_ch_samp_cos);
	dma_channel_abort(dma_ch_samp_sin);
	dma_channel_abort(dma_ch_samp_trig);

	dma_channel_cleanup(dma_ch_rx);
	dma_channel_cleanup(dma_ch_cp);
	dma_channel_cleanup(dma_ch_cos);
	dma_channel_cleanup(dma_ch_sin);
	dma_channel_cleanup(dma_ch_pio_cos);
	dma_channel_cleanup(dma_ch_pio_sin);
	dma_channel_cleanup(dma_ch_samp_cos);
	dma_channel_cleanup(dma_ch_samp_sin);
	dma_channel_cleanup(dma_ch_samp_trig);

	dma_channel_unclaim(dma_ch_rx);
	dma_channel_unclaim(dma_ch_cp);
	dma_channel_unclaim(dma_ch_cos);
	dma_channel_unclaim(dma_ch_sin);
	dma_channel_unclaim(dma_ch_pio_cos);
	dma_channel_unclaim(dma_ch_pio_sin);
	dma_channel_unclaim(dma_ch_samp_cos);
	dma_channel_unclaim(dma_ch_samp_sin);
	dma_channel_unclaim(dma_ch_samp_trig);

	dma_timer_unclaim(dma_t_samp);

	dma_ch_rx = -1;
	dma_ch_cp = -1;
	dma_ch_cos = -1;
	dma_ch_sin = -1;
	dma_ch_pio_cos = -1;
	dma_ch_pio_sin = -1;
	dma_ch_samp_cos = -1;
	dma_ch_samp_sin = -1;
	dma_ch_samp_trig = -1;

	dma_t_samp = -1;
}

static void rf_tx_start(int tx_pin)
{
	send_init(tx_pin);

	dma_ch_cos = dma_claim_unused_channel(true);

	dma_channel_config dma_conf = dma_channel_get_default_config(dma_ch_cos);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_conf, true);
	channel_config_set_write_increment(&dma_conf, false);
	channel_config_set_ring(&dma_conf, false, LO_BITS_DEPTH + 2);
	channel_config_set_dreq(&dma_conf, pio_get_dreq(pio1, 1, true));
	dma_channel_configure(dma_ch_cos, &dma_conf, &pio1->txf[1], lo_cos, UINT_MAX, true);
}

static void rf_tx_stop()
{
	puts("Stopping TX...");
	pio_sm_set_enabled(pio1, 1, false);
	pio_sm_restart(pio1, 1);
	pio_sm_clear_fifos(pio1, 1);

	puts("Stopping DMA...");
	dma_channel_abort(dma_ch_cos);
	dma_channel_cleanup(dma_ch_cos);
	dma_channel_unclaim(dma_ch_cos);
	dma_ch_cos = -1;
}

static void rf_rx(void)
{
	static int8_t block[IQ_BLOCK_LEN];
	uint32_t prev_transfers = dma_hw->ch[dma_ch_in_cos].transfer_count;
	unsigned pos = 0;

	static int lpIh1[LPF_SIZE];
	static int lpQh1[LPF_SIZE];
	int lpIa1 = 0;
	int lpQa1 = 0;

	static int lpIh2[LPF_SIZE];
	static int lpQh2[LPF_SIZE];
	int lpIa2 = 0;
	int lpQa2 = 0;

	static int lpIh3[LPF_SIZE];
	static int lpQh3[LPF_SIZE];
	int lpIa3 = 0;
	int lpQa3 = 0;

	int64_t dcI = 0, dcQ = 0;

	/*
	 * CLK / BW for oversampling, but 1/2 for the -1/0/0/1 PIO outcomes.
	 * Signal can get outside due to DC offset compensation.
	 */
	const int amp_max = CLK_SYS_HZ / 2 / BANDWIDTH * DECIMATION;

	while (true) {
		if (multicore_fifo_rvalid()) {
			multicore_fifo_pop_blocking();
			multicore_fifo_push_blocking(0);
			return;
		}

		int delta = prev_transfers - dma_hw->ch[dma_ch_in_cos].transfer_count;
		gap = RX_BLOCK_LEN - delta;

		while (delta < RX_BLOCK_LEN) {
			delta = prev_transfers - dma_hw->ch[dma_ch_in_cos].transfer_count;
			sleep_us(100);
		}

		prev_transfers -= RX_BLOCK_LEN;

		uint32_t *cos_ptr = rx_cos + pos;
		uint32_t *sin_ptr = rx_sin + pos;

		pos = (pos + RX_BLOCK_LEN) & (RX_WORDS - 1);

		int8_t *blockptr = block;

		for (int i = 0; i < IQ_BLOCK_LEN / 2; i++) {
			int64_t dI = 0;
			int64_t dQ = 0;

			for (int d = 0; d < DECIMATION; d++) {
				uint32_t cos_pos = *cos_ptr++;
				uint32_t cos_neg = *cos_ptr++;
				uint32_t sin_pos = *sin_ptr++;
				uint32_t sin_neg = *sin_ptr++;

				int I = cos_neg - cos_pos;
				int Q = sin_neg - sin_pos;

				int lpP = d & (LPF_SIZE - 1);

				lpIa1 += I - lpIh1[lpP];
				lpQa1 += Q - lpQh1[lpP];

				lpIh1[lpP] = I;
				lpQh1[lpP] = Q;

				I = lpIa1 / LPF_SIZE;
				Q = lpQa1 / LPF_SIZE;

				lpIa2 += I - lpIh2[lpP];
				lpQa2 += Q - lpQh2[lpP];

				lpIh2[lpP] = I;
				lpQh2[lpP] = Q;

				I = lpIa2 / LPF_SIZE;
				Q = lpQa2 / LPF_SIZE;

				lpIa3 += I - lpIh3[lpP];
				lpQa3 += Q - lpQh3[lpP];

				lpIh3[lpP] = I;
				lpQh3[lpP] = Q;

				I = lpIa3 / LPF_SIZE;
				Q = lpQa3 / LPF_SIZE;

				dI += I;
				dQ += Q;
			}

			dcI = ((dcI << 13) - dcI + (dI << 19)) >> 13;
			dcQ = ((dcQ << 13) - dcQ + (dQ << 19)) >> 13;

			dI -= dcI >> 19;
			dQ -= dcQ >> 19;

			if (dI > amp_max)
				dI = amp_max;

			if (dI < -amp_max)
				dI = -amp_max;

			if (dQ > amp_max)
				dQ = amp_max;

			if (dQ < -amp_max)
				dQ = -amp_max;

			*blockptr++ = 127 * dI / amp_max;
			*blockptr++ = 127 * dQ / amp_max;
		}

		if (!queue_try_add(&iq_queue, block)) {
			puts("queue overflow");
		}
	}
}

inline static int icopysign(int x, int s)
{
	return s >= 0 ? abs(x) : -abs(x);
}

static void __unused plot_IQ(int I, int Q)
{
	int mag = I ? icopysign(32 - __builtin_clz(abs(I)), I) : 0;

	if (mag < 0) {
		for (int l = -mag; l < 16; l++)
			putchar(' ');

		for (int l = 0; l < -mag; l++)
			putchar('#');

		printf("%16s", "");
	} else {
		printf("%16s", "");

		for (int l = 0; l < mag; l++)
			putchar('#');

		for (int l = mag; l < 16; l++)
			putchar(' ');
	}

	mag = Q ? icopysign(32 - __builtin_clz(abs(Q)), Q) : 0;

	if (mag < 0) {
		for (int l = -mag; l < 16; l++)
			putchar(' ');

		for (int l = 0; l < -mag; l++)
			putchar('#');

		printf("%16s", "");
	} else {
		printf("%16s", "");

		for (int l = 0; l < mag; l++)
			putchar('#');

		for (int l = mag; l < 16; l++)
			putchar(' ');
	}
}

__unused inline static uint32_t ring_next(uint32_t **ptr, int dma_ch, int us)
{
	const unsigned ring_bits = (dma_hw->ch[dma_ch].al1_ctrl >> 6) & 15;
	const uint32_t ring_mask = (1 << ring_bits) - 1;
	uint32_t uptr = (uint32_t)*ptr;

	while (uptr == dma_hw->ch[dma_ch].write_addr)
		sleep_us(us);

	uptr = (uptr & ~ring_mask) | ((uptr + sizeof(uint32_t)) & ring_mask);
	*ptr = (uint32_t *)uptr;
	return *(uint32_t *)uptr;
}

static void do_rx(int rx_pin, int bias_pin, float freq, char mode)
{
	float actual = rf_rx_start(rx_pin, bias_pin, freq, 1, CLK_SYS_HZ / BANDWIDTH);
	sleep_us(100);

	dma_ch_in_cos = dma_claim_unused_channel(true);
	dma_ch_in_sin = dma_claim_unused_channel(true);

	dma_channel_config dma_conf;

	dma_conf = dma_channel_get_default_config(dma_ch_in_cos);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_conf, false);
	channel_config_set_write_increment(&dma_conf, true);
	channel_config_set_ring(&dma_conf, true, RX_BITS_DEPTH + 2);
	channel_config_set_dreq(&dma_conf, pio_get_dreq(pio1, 2, false));
	dma_channel_configure(dma_ch_in_cos, &dma_conf, rx_cos, &pio1->rxf[2], UINT_MAX, false);

	dma_conf = dma_channel_get_default_config(dma_ch_in_sin);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_conf, false);
	channel_config_set_write_increment(&dma_conf, true);
	channel_config_set_ring(&dma_conf, true, RX_BITS_DEPTH + 2);
	channel_config_set_dreq(&dma_conf, pio_get_dreq(pio1, 3, false));
	dma_channel_configure(dma_ch_in_sin, &dma_conf, rx_sin, &pio1->rxf[3], UINT_MAX, false);

	dma_start_channel_mask((1 << dma_ch_in_sin) | (1 << dma_ch_in_cos));

	multicore_launch_core1(rf_rx);

	printf("Frequency: %.0f\n", actual);

	static int8_t block[IQ_BLOCK_LEN];

	while (queue_try_remove(&iq_queue, block))
		/* Flush the queue */;

	if ('b' == mode) {
		setvbuf(stdout, NULL, _IONBF, 0);
		putchar('$');
	}

	while (true) {
		int c = getchar_timeout_us(0);
		if ('\r' == c)
			break;

		if (queue_try_remove(&iq_queue, block)) {
			if ('b' == mode) {
				fwrite(block, sizeof block, 1, stdout);
				fflush(stdout);
			} else {
				for (int i = 0; i < IQ_BLOCK_LEN / 2; i += 8) {
					int I = block[i * 2];
					int Q = block[i * 2 + 1];
					printf("%2i %12i %12i ", gap, I, Q);
					plot_IQ(I, Q);
					putchar('\n');
				}
			}
		}
	}

	putchar('\n');
	puts("Stopping core1...");
	multicore_fifo_push_blocking(0);
	multicore_fifo_pop_blocking();
	sleep_us(10);
	multicore_reset_core1();

	puts("Stopping RX...");
	rf_rx_stop();

	puts("Stopping readout DMAs...");
	dma_channel_abort(dma_ch_in_cos);
	dma_channel_abort(dma_ch_in_sin);
	dma_channel_cleanup(dma_ch_in_cos);
	dma_channel_cleanup(dma_ch_in_sin);
	dma_channel_unclaim(dma_ch_in_cos);
	dma_channel_unclaim(dma_ch_in_sin);
	dma_ch_in_cos = -1;
	dma_ch_in_sin = -1;

	puts("Done.");
}

static void command(const char *cmd)
{
	static char tmp[83];
	int n, x;
	float f, g;

	if (1 == sscanf(cmd, " help %[\a]", tmp)) {
		puts("help             - this help");
		puts("drive N X        - set GPIO pin drive strength");
		puts("bias I O         - output negated I to O");
		puts("rx N FREQ        - receive on pin N");
		puts("brx N FREQ       - receive on pin N, binary output");
		puts("tx N FREQ        - transmit on pin N");
		puts("sweep N F G S    - sweep from F to G with given step");
		puts("noise N          - transmit random noise");
		return;
	}

	if (3 == sscanf(cmd, " drive %i %i %[\a]", &n, &x, tmp)) {
		if ((x < 0) || (x > 3)) {
			puts("invalid drive strength, use 0-3 for 2, 4, 8, 12 mA");
			return;
		}

		gpio_set_drive_strength(n, x);
		static int strength[] = { 2, 4, 8, 12 };
		printf("gpio%i: %i mA\n", n, strength[x]);
		return;
	}

	if (3 == sscanf(cmd, " bias %i %i %[\a]", &n, &x, tmp)) {
		bias_init(n, x);
		return;
	}

	if (4 == sscanf(cmd, " rx %i %i %f %[\a]", &n, &x, &f, tmp)) {
		do_rx(n, x, f, 'a');
		return;
	}

	if (4 == sscanf(cmd, " brx %i %i %f %[\a]", &n, &x, &f, tmp)) {
		do_rx(n, x, f, 'b');
		return;
	}

	if (3 == sscanf(cmd, " tx %i %f %[\a]", &n, &f, tmp)) {
		float actual = lo_freq_init(f);
		printf("Frequency: %.0f\n", actual);

		rf_tx_start(n);
		puts("Transmitting, press ENTER to stop.");

		bool phase = false;

		while (true) {
			int c = getchar_timeout_us(1000);

			if ('\r' == c) {
				break;
			} else if (' ' == c) {
				phase = !phase;
				gpio_set_outover(n, phase);
			}
		}

		rf_tx_stop();
		gpio_set_outover(n, 0);
		puts("Done.");
		return;
	}

	if (5 == sscanf(cmd, " sweep %i %f %f %i %[\a]", &n, &f, &g, &x, tmp)) {
		const float step_hz = (float)CLK_SYS_HZ / (LO_WORDS * 32);
		const float start = roundf(f / step_hz) * step_hz;
		const float stop = roundf(g / step_hz) * step_hz;

		int steps = roundf((stop - start) / step_hz);

		for (int i = 0; i < LO_WORDS; i++)
			lo_cos[i] = 0;

		rf_tx_start(n);

		for (int i = 0; i < steps; i += x) {
			int c = getchar_timeout_us(10000);
			if ('\r' == c)
				break;

			float actual = lo_freq_init(start + i * step_hz);
			printf("Frequency: %.0f\n", actual);
		}

		rf_tx_stop();
		puts("Done.");
		return;
	}

	if (2 == sscanf(cmd, " noise %i %[\a]", &n, tmp)) {
		for (int i = 0; i < LO_WORDS; i++)
			lo_cos[i] = rand();

		rf_tx_start(n);

		puts("Transmitting noise, press ENTER to stop.");

		while (true) {
			int c = getchar_timeout_us(100);
			if ('\r' == c)
				break;

			for (int i = 0; i < LO_WORDS; i++)
				lo_cos[i] = rand();
		}

		rf_tx_stop();
		puts("Done.");
		return;
	}

	puts("unknown command");
}

int main()
{
	vreg_set_voltage(VREG_VOLTAGE_MAX);
	set_sys_clock_khz(CLK_SYS_HZ / KHZ, true);
	clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, CLK_SYS_HZ,
			CLK_SYS_HZ);

	bus_ctrl_hw->priority |= BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

	stdio_usb_init();

	for (int i = 0; i < 30; i++) {
		if (stdio_usb_connected())
			break;

		sleep_ms(100);
	}

	printf("\nPuppet Online!\n");
	printf("clk_sys = %10.6f MHz\n", (float)clock_get_hz(clk_sys) / MHZ);

	queue_init(&iq_queue, IQ_BLOCK_LEN * sizeof(int8_t), 256);

	static char cmd[83];
	int cmdlen = 0;

	printf("> ");

	while (true) {
		int c;

		while ((c = getchar_timeout_us(10000)) >= 0) {
			if ('\r' == c) {
				/* Enter */
			} else if ((8 == c) && (cmdlen > 0)) {
				cmd[--cmdlen] = 0;
				printf("\b \b");
			} else if ((' ' == c) && (0 == cmdlen)) {
				/* No leading spaces. */
				continue;
			} else if (c < ' ') {
				continue;
			} else {
				cmd[cmdlen++] = c;
				putchar(c);
			}

			if (('\r' == c) || cmdlen == 80) {
				printf("\n");
				if (cmdlen > 0) {
					cmd[cmdlen] = '\a';
					cmd[cmdlen + 1] = 0;
					command(cmd);
					cmdlen = 0;
				}
				printf("> ");
			}
		}
	}
}
