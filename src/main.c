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

#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/pll.h>
#include <hardware/vreg.h>
#include <hardware/sync.h>
#include <hardware/pio.h>
#include <hardware/interp.h>

#include <hardware/regs/clocks.h>

#include <math.h>
#include <stdio.h>
#include <limits.h>
#include <stdlib.h>

#define CLK_SYS_HZ (250 * MHZ)

#define EXTRA_BITS 8
#define NUM_SAMPLES 128
#define RSSI_ALPHA 1
#define LPF_SAMPLES 4
#define HPF_ALPHA 5
#define SPEED 3

#define SLEEP_US 16666

#define LED_PIN 25

#define LO_BITS_DEPTH 13
#define LO_WORDS (1 << LO_BITS_DEPTH)
static uint32_t lo_cos[LO_WORDS] __attribute__((__aligned__(LO_WORDS * 4)));
static uint32_t lo_sin[LO_WORDS] __attribute__((__aligned__(LO_WORDS * 4)));
static uint32_t rx_buf[LO_WORDS] __attribute__((__aligned__(LO_WORDS * 4)));

#if SPEED < 3
static int8_t mixer[256][128];
#endif

static int tx_dma = -1;
static int rx_dma = -1;

static volatile struct status {
	unsigned mtime;
	float rssi_raw;
	float rssi_max;
	unsigned sample_rate;
	int frequency;
	int angle;
	int I, Q;
} status;

#define PLEASE_DIE 0xd1e

static void bias_init(int in_pin, int out_pin)
{
	gpio_disable_pulls(in_pin);
	gpio_disable_pulls(out_pin);

	pio_gpio_init(pio1, out_pin);

	gpio_set_input_hysteresis_enabled(in_pin, false);
	gpio_set_input_hysteresis_enabled(out_pin, false);
	gpio_set_drive_strength(out_pin, GPIO_DRIVE_STRENGTH_2MA);

	const uint16_t lm_insn[] = {
		pio_encode_mov_not(pio_pins, pio_pins),
	};

	pio_program_t lm_prog = {
		.instructions = lm_insn,
		.length = 1,
		.origin = 0,
	};

	pio_sm_set_enabled(pio1, 0, false);
	pio_sm_restart(pio1, 0);

	if (pio_can_add_program(pio1, &lm_prog))
		pio_add_program(pio1, &lm_prog);

	pio_sm_config pc = pio_get_default_sm_config();
	sm_config_set_in_pins(&pc, in_pin);
	sm_config_set_out_pins(&pc, out_pin, 1);
	sm_config_set_set_pins(&pc, out_pin, 1);
	sm_config_set_wrap(&pc, 0, 0);
	sm_config_set_clkdiv_int_frac(&pc, 1, 0);
	pio_sm_init(pio1, 0, 0, &pc);

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
		.origin = 1,
	};

	pio_sm_set_enabled(pio1, 1, false);
	pio_sm_restart(pio1, 1);

	if (pio_can_add_program(pio1, &prog))
		pio_add_program(pio1, &prog);

	pio_sm_config pc = pio_get_default_sm_config();
	sm_config_set_in_pins(&pc, in_pin);
	sm_config_set_wrap(&pc, 1, 1);
	sm_config_set_clkdiv_int_frac(&pc, 1, 0);
	sm_config_set_fifo_join(&pc, PIO_FIFO_JOIN_RX);
	sm_config_set_in_shift(&pc, false, true, 32);
	pio_sm_init(pio1, 1, 1, &pc);

	pio_sm_set_enabled(pio1, 1, true);
}

static void send_init(int out_pin)
{
	gpio_disable_pulls(out_pin);
	pio_gpio_init(pio1, out_pin);
	gpio_set_drive_strength(out_pin, GPIO_DRIVE_STRENGTH_2MA);
	gpio_set_slew_rate(out_pin, GPIO_SLEW_RATE_SLOW);

	const uint16_t insn[] = {
		pio_encode_out(pio_pins, 1),
	};

	pio_program_t prog = {
		.instructions = insn,
		.length = 1,
		.origin = 2,
	};

	pio_sm_set_enabled(pio1, 2, false);
	pio_sm_restart(pio1, 2);

	if (pio_can_add_program(pio1, &prog))
		pio_add_program(pio1, &prog);

	pio_sm_config pc = pio_get_default_sm_config();
	sm_config_set_out_pins(&pc, out_pin, 1);
	sm_config_set_set_pins(&pc, out_pin, 1);
	sm_config_set_wrap(&pc, 2, 2);
	sm_config_set_clkdiv_int_frac(&pc, 1, 0);
	sm_config_set_fifo_join(&pc, PIO_FIFO_JOIN_TX);
	sm_config_set_out_shift(&pc, false, true, 32);
	pio_sm_init(pio1, 2, 2, &pc);

	pio_sm_set_consecutive_pindirs(pio1, 2, out_pin, 1, GPIO_OUT);

	pio_sm_set_enabled(pio1, 2, true);
}

#if SPEED < 3
inline static int lookup_mixer(uint8_t a, uint8_t b)
{
	if (b & 0x80)
		return -mixer[a][(~b) & 0x7f];

	return mixer[a][b];
}

inline static void lpf(int8_t *xs, int len)
{
	int p1, p2;

	p2 = p1 = xs[0];

	for (int i = 0; i < len; i++) {
		int tmp = xs[i];
		xs[i] = (p1 + p2 + tmp) / 3;
		p2 = p1, p1 = tmp;
	}

	p2 = p1 = xs[len - 1];

	for (int i = len - 1; i >= 0; i--) {
		int tmp = xs[i];
		xs[i] = (p1 + p2 + tmp) / 3;
		p2 = p1, p1 = tmp;
	}
}

static int8_t mix(uint8_t a, uint8_t b)
{
	static int8_t ab[16], bb[16];

	for (int i = 0; i < 8; i++) {
		ab[i * 2 + 0] = (a >> 7) ? 127 : -127;
		ab[i * 2 + 1] = ab[i * 2];
		a <<= 1;

		bb[i * 2 + 0] = (b >> 7) ? 127 : -127;
		bb[i * 2 + 1] = bb[i * 2];
		b <<= 1;
	}

	lpf(ab, 16);
	lpf(bb, 16);

	for (int i = 0; i < 16; i++)
		ab[i] = ((int)ab[i] * (int)bb[i]) / 127;

	lpf(ab, 16);

	int accum = 0;

#if SPEED == 1
	for (int i = 4; i < 12; i++)
		accum += ab[i];

	return accum >> 3;
#else
	for (int i = 0; i < 16; i++)
		accum += ab[i];

	return accum >> 4;
#endif
}

static void generate_mixer(void)
{
	for (int i = 0; i < 256; i++)
		for (int j = 0; j < 128; j++)
			mixer[i][j] = mix(i, j);
}
#endif

static float lo_freq_init(float req_freq)
{
	const float step_hz = (float)CLK_SYS_HZ / (LO_WORDS * 32);
	float freq = roundf(req_freq / step_hz) * step_hz;

	unsigned step = (float)UINT_MAX / (float)CLK_SYS_HZ * freq;

	interp0->ctrl[0] = (31 << SIO_INTERP0_CTRL_LANE0_MASK_MSB_LSB) |
			   (0 << SIO_INTERP0_CTRL_LANE0_MASK_LSB_LSB) |
			   (0 << SIO_INTERP0_CTRL_LANE0_SHIFT_LSB);
	interp0->ctrl[1] = interp0->ctrl[0];

	interp0->base[0] = step;
	interp0->base[1] = step;
	interp0->accum[1] = UINT_MAX / 4;

	for (int i = 0; i < LO_WORDS; i++) {
		unsigned bsin = 0, bcos = 0;

		for (int j = 0; j < 32; j++) {
			int sin_bit = interp0->peek[0] >> 31;
			bsin |= sin_bit;
			bsin <<= 1;

			int cos_bit = interp0->pop[1] >> 31;
			bcos |= cos_bit;
			bcos <<= 1;
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

inline static __unused unsigned popcount(unsigned v)
{
	v = v - ((v >> 1) & 0x55555555);
	v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
	return (((v + (v >> 4)) & 0x0f0f0f0f) * 0x01010101) >> 24;
}

static __unused bool is_prime_or_one(int n)
{
	if (n == 1 || n == 2 || n == 3)
		return true;

	if (n <= 1 || n % 2 == 0 || n % 3 == 0)
		return false;

	for (int i = 5; i * i <= n; i += 6) {
		if (n % i == 0 || n % (i + 2) == 0)
			return false;
	}

	return true;
}

static void rf_rx(void)
{
	uint64_t assi0 = 0, assi1 = 0, assi2 = 0;

	status.rssi_max = 0.5f * powf(127.5f * (1 << EXTRA_BITS), 2.0f);

#if HPF_ALPHA
	int64_t hpI = 0, hpQ = 0;
#endif

#if LPF_SAMPLES
	static int lpIh1[LPF_SAMPLES], lpQh1[LPF_SAMPLES];
	int lpIavg1 = 0, lpQavg1 = 0;

	static int lpIh2[LPF_SAMPLES], lpQh2[LPF_SAMPLES];
	int lpIavg2 = 0, lpQavg2 = 0;

	static int lpIh3[LPF_SAMPLES], lpQh3[LPF_SAMPLES];
	int lpIavg3 = 0, lpQavg3 = 0;

	int lpIidx = 0, lpQidx = 0;

	for (int i = 0; i < LPF_SAMPLES; i++) {
		lpIh1[i] = lpIh2[i] = lpIh3[i] = 0;
		lpQh1[i] = lpQh2[i] = lpQh3[i] = 0;
	}
#endif

#if SPEED == 2
	interp0->ctrl[0] = (31 << SIO_INTERP0_CTRL_LANE0_MASK_MSB_LSB) |
			   (0 << SIO_INTERP0_CTRL_LANE0_MASK_LSB_LSB) |
			   (8 << SIO_INTERP0_CTRL_LANE0_SHIFT_LSB);

	interp0->ctrl[1] = SIO_INTERP0_CTRL_LANE0_CROSS_RESULT_BITS |
			   (7 << SIO_INTERP0_CTRL_LANE1_MASK_MSB_LSB) |
			   (0 << SIO_INTERP0_CTRL_LANE1_MASK_LSB_LSB) |
			   (0 << SIO_INTERP0_CTRL_LANE1_SHIFT_LSB);
#endif

#if SPEED == 1
	interp0->ctrl[0] = (31 << SIO_INTERP0_CTRL_LANE0_MASK_MSB_LSB) |
			   (0 << SIO_INTERP0_CTRL_LANE0_MASK_LSB_LSB) |
			   (4 << SIO_INTERP0_CTRL_LANE0_SHIFT_LSB);

	interp0->ctrl[1] = SIO_INTERP0_CTRL_LANE0_CROSS_RESULT_BITS |
			   (7 << SIO_INTERP0_CTRL_LANE1_MASK_MSB_LSB) |
			   (0 << SIO_INTERP0_CTRL_LANE1_MASK_LSB_LSB) |
			   (0 << SIO_INTERP0_CTRL_LANE1_SHIFT_LSB);
#endif

#if SPEED < 3
	interp1->ctrl[0] = interp0->ctrl[0];
	interp1->ctrl[1] = interp0->ctrl[1];
#endif

	int prevI = 0, prevQ = 0;
	int period = 0;
	int frequency = 0;
	int stride = 0;

	int delta_avg = 1;
	int prev_delta_netto = 0;
	unsigned prev_transfers = 0;

	int prev_angle = 0;
	int last_angle_delta = 0;
	int angle_stride = 0;
	int rotation = 0;

	while (true) {
		uint32_t msg = 0;

		if (multicore_fifo_rvalid()) {
			msg = multicore_fifo_pop_blocking();

			if (PLEASE_DIE == msg) {
				multicore_fifo_push_blocking(0);
				return;
			}
		}

		int I = 0, Q = 0;

		if (!dma_channel_is_busy(rx_dma))
			dma_channel_start(rx_dma);

		int delta = ~dma_hw->ch[rx_dma].transfer_count - prev_transfers;
		delta_avg = (delta_avg * 1023 + delta * 256) / 1024;
		int delta_netto = delta_avg / 256;

		if (delta_netto == (prev_delta_netto - 1)) {
			delta_netto = prev_delta_netto;
		} else {
			prev_delta_netto = delta_netto;
		}

		while (delta < delta_netto) {
			if (!dma_channel_is_busy(rx_dma))
				dma_channel_start(rx_dma);

			delta = ~dma_hw->ch[rx_dma].transfer_count - prev_transfers;
		}

		prev_transfers += delta_netto;
		unsigned pos = (prev_transfers - NUM_SAMPLES - 2) & (LO_WORDS - 1);

#if SPEED == 1
		unsigned prev_rx_word = 0;
		unsigned prev_cos_word = 0;
		unsigned prev_sin_word = 0;
#endif

		for (int k = 0; k < NUM_SAMPLES; k++) {
			unsigned rx_word = rx_buf[pos];
			unsigned cos_word = lo_cos[pos];
			unsigned sin_word = lo_sin[pos];

			pos = (pos + 1) & (LO_WORDS - 1);

#if SPEED == 3
			I += popcount(rx_word ^ cos_word);
			Q += popcount(rx_word ^ sin_word);
#elif SPEED == 2
			interp0->accum[0] = rx_word;
			interp0->accum[1] = rx_word;
			interp1->accum[0] = cos_word;
			interp1->accum[1] = cos_word;

			I += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			I += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			I += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			I += lookup_mixer(interp0->pop[1], interp1->pop[1]);

			interp0->accum[0] = rx_word;
			interp0->accum[1] = rx_word;
			interp1->accum[0] = sin_word;
			interp1->accum[1] = sin_word;

			Q += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			Q += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			Q += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			Q += lookup_mixer(interp0->pop[1], interp1->pop[1]);
#elif SPEED == 1
			I += lookup_mixer(((prev_rx_word << 4) | (rx_word >> 28)),
					  ((prev_cos_word << 4) | (cos_word >> 28)));

			Q += lookup_mixer(((prev_rx_word << 4) | (rx_word >> 28)),
					  ((prev_sin_word << 4) | (sin_word >> 28)));

			interp0->accum[0] = rx_word;
			interp0->accum[1] = rx_word;
			interp1->accum[0] = cos_word;
			interp1->accum[1] = cos_word;

			I += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			I += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			I += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			I += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			I += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			I += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			I += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			prev_cos_word = interp1->pop[1];

			interp0->accum[0] = rx_word;
			interp0->accum[1] = rx_word;
			interp1->accum[0] = sin_word;
			interp1->accum[1] = sin_word;

			Q += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			Q += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			Q += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			Q += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			Q += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			Q += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			Q += lookup_mixer(interp0->pop[1], interp1->pop[1]);
			prev_sin_word = interp1->pop[1];
			prev_rx_word = interp0->pop[1];
#endif
		}

		I -= 16 * NUM_SAMPLES;
		Q -= 16 * NUM_SAMPLES;

#if SPEED == 3
		/* Normalize to given number of bits. */
		I = (I * ((1 << (7 + EXTRA_BITS)) - 1)) / 16;
		Q = (Q * ((1 << (7 + EXTRA_BITS)) - 1)) / 16;
#else
		I <<= EXTRA_BITS;
		Q <<= EXTRA_BITS;
#endif
		I /= NUM_SAMPLES;
		Q /= NUM_SAMPLES;

#if HPF_ALPHA
		int64_t tmpI = (int64_t)I << 16;
		I -= hpI >> 16;
		hpI = (hpI * ((1 << 16) - HPF_ALPHA) + tmpI * HPF_ALPHA) >> 16;

		int64_t tmpQ = (int64_t)Q << 16;
		Q -= hpQ >> 16;
		hpQ = (hpQ * ((1 << 16) - HPF_ALPHA) + tmpQ * HPF_ALPHA) >> 16;
#endif

#if LPF_SAMPLES
		lpIavg1 += I - lpIh1[lpIidx];
		lpIh1[lpIidx] = I;

		lpIavg2 += lpIavg1 - lpIh2[lpIidx];
		lpIh2[lpIidx] = lpIavg1;

		lpIavg3 += lpIavg2 - lpIh3[lpIidx];
		lpIh3[lpIidx++] = lpIavg2;

		if (lpIidx >= LPF_SAMPLES)
			lpIidx = 0;

		I = lpIavg3 / (LPF_SAMPLES * LPF_SAMPLES * LPF_SAMPLES);

		lpQavg1 += Q - lpQh1[lpQidx];
		lpQh1[lpQidx] = Q;

		lpQavg2 += lpQavg1 - lpQh2[lpQidx];
		lpQh2[lpQidx] = lpQavg1;

		lpQavg3 += lpQavg2 - lpQh3[lpQidx];
		lpQh3[lpQidx++] = lpQavg2;

		if (lpQidx >= LPF_SAMPLES)
			lpQidx = 0;

		Q = lpQavg3 / (LPF_SAMPLES * LPF_SAMPLES * LPF_SAMPLES);
#endif

		if ((Q < 0) ^ (prevQ < 0)) {
			stride *= ((I < 0) ^ (Q < 0)) ? 1 : -1;
			period = (63 * period + stride) / 64;
			frequency = CLK_SYS_HZ / ((period >> 3) * delta_netto);
			stride = 0;
		} else if ((I < 0) ^ (prevI < 0)) {
			stride *= ((I < 0) ^ (Q < 0)) ? -1 : 1;
			period = (63 * period + stride) / 64;
			frequency = CLK_SYS_HZ / ((period >> 3) * delta_netto);
			stride = 0;
		} else {
			stride += 1024;
		}

		prevI = I;
		prevQ = Q;

		int angle = cheap_atan2(I, Q);
		int angle_delta = cheap_angle_diff(angle, prev_angle);
		prev_angle = angle;

		if (angle_delta) {
			rotation = (rotation * 31 + ((last_angle_delta / angle_stride) >> 16)) / 32;
			last_angle_delta = angle_delta;
			angle_stride = 1;
		} else {
			angle_stride++;
		}

		uint64_t ssi = (uint64_t)I * (uint64_t)I + (uint64_t)Q * (uint64_t)Q;
		const uint64_t alpha = RSSI_ALPHA;

		assi0 = (assi0 * (256 - alpha) + (ssi << 16) * alpha) / 256;
		assi1 = (assi1 * (256 - alpha) + assi0 * alpha) / 256;
		assi2 = (assi2 * (256 - alpha) + assi1 * alpha) / 256;

		status.rssi_raw = assi2 >> 16;
		status.frequency = frequency;
		status.sample_rate = CLK_SYS_HZ / (delta_netto * 32);
		status.angle = rotation << 16;
		status.I = I;
		status.Q = Q;
		status.mtime = time_us_32();
	}
}

static void __unused plot_IQ(int I, int Q)
{
	int mag = I ? copysign(log2f(abs(I)), I) : 0;

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

	mag = Q ? copysign(log2f(abs(Q)), Q) : 0;

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

	if (3 == sscanf(cmd, " rx %i %f %[\a]", &n, &f, tmp)) {
		watch_init(n);
		float actual = lo_freq_init(f);
		printf("actual frequency = %10.6f Hz\n", actual / MHZ);

		dma_channel_config dma_conf = dma_channel_get_default_config(rx_dma);
		channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
		channel_config_set_read_increment(&dma_conf, false);
		channel_config_set_write_increment(&dma_conf, true);
		channel_config_set_ring(&dma_conf, true, LO_BITS_DEPTH + 2);
		channel_config_set_dreq(&dma_conf, pio_get_dreq(pio1, 1, false));
		dma_channel_configure(rx_dma, &dma_conf, rx_buf, &pio1->rxf[1], UINT_MAX, true);

		multicore_launch_core1(rf_rx);
		sleep_us(100);

		unsigned last = 0;

		while (true) {
			int c = getchar_timeout_us(0);

			if (13 == c) {
				multicore_fifo_push_blocking(PLEASE_DIE);
				multicore_fifo_pop_blocking();
				multicore_reset_core1();
				dma_channel_abort(rx_dma);
				dma_channel_cleanup(rx_dma);
				puts("stopped");
				break;
			}

			static struct status st;
			st = status;

			if (st.mtime == last)
				continue;

			last = st.mtime;

			float rssi_rel = st.rssi_raw / st.rssi_max;

			printf("%5.1f dBm (%5.0f) [%5u %+7i] %+5.1f ", 10.0f * log10f(rssi_rel),
			       sqrtf(st.rssi_raw), st.sample_rate,
			       (abs(st.frequency) > (int)(st.sample_rate / 2)) ? 0 : st.frequency,
			       180.0f * st.angle / (float)INT_MAX);

			plot_IQ(st.I, st.Q);
			puts("");

#if SLEEP_US
			sleep_us(SLEEP_US);
#endif
		}

		return;
	}

	if (3 == sscanf(cmd, " tx %i %f %[\a]", &n, &f, tmp)) {
		dma_channel_abort(tx_dma);

		if (!f) {
			gpio_init(n);
			puts("stopped");
			return;
		}

		send_init(n);
		float actual = lo_freq_init(f);
		printf("actual frequency = %10.6f MHz\n", actual / MHZ);

		dma_channel_config dma_conf = dma_channel_get_default_config(tx_dma);
		channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
		channel_config_set_read_increment(&dma_conf, true);
		channel_config_set_write_increment(&dma_conf, false);
		channel_config_set_ring(&dma_conf, false, LO_BITS_DEPTH + 2);
		channel_config_set_dreq(&dma_conf, pio_get_dreq(pio1, 2, true));
		dma_channel_configure(tx_dma, &dma_conf, &pio1->txf[2], lo_cos, UINT_MAX, true);
		puts("started");
		return;
	}

	if (5 == sscanf(cmd, " sweep %i %f %f %i %[\a]", &n, &f, &g, &x, tmp)) {
		dma_channel_abort(tx_dma);

		if (!f || !g) {
			gpio_init(n);
			puts("stopped");
			return;
		}

		send_init(n);

		const float step_hz = (float)CLK_SYS_HZ / (LO_WORDS * 32);
		const float start = roundf(f / step_hz) * step_hz;
		const float stop = roundf(g / step_hz) * step_hz;

		int steps = roundf((stop - start) / step_hz);

		for (int i = 0; i < LO_WORDS; i++)
			lo_cos[i] = 0;

		dma_channel_config dma_conf = dma_channel_get_default_config(tx_dma);
		channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
		channel_config_set_read_increment(&dma_conf, true);
		channel_config_set_write_increment(&dma_conf, false);
		channel_config_set_ring(&dma_conf, false, LO_BITS_DEPTH + 2);
		channel_config_set_dreq(&dma_conf, pio_get_dreq(pio1, 2, true));
		dma_channel_configure(tx_dma, &dma_conf, &pio1->txf[2], lo_cos, UINT_MAX, true);

		for (int i = 0; i < steps; i += x) {
			int c = getchar_timeout_us(0);
			if ('\n' == c || '\r' == c)
				break;

			float actual = lo_freq_init(start + i * step_hz);
			printf("frequency = %10.6f MHz\n", actual / MHZ);
		}

		dma_channel_abort(tx_dma);
		return;
	}

	if (2 == sscanf(cmd, " noise %i %[\a]", &n, tmp)) {
		send_init(n);

		for (int i = 0; i < LO_WORDS; i++)
			lo_cos[i] = rand();

		dma_channel_config dma_conf = dma_channel_get_default_config(tx_dma);
		channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
		channel_config_set_read_increment(&dma_conf, true);
		channel_config_set_write_increment(&dma_conf, false);
		channel_config_set_ring(&dma_conf, false, LO_BITS_DEPTH + 2);
		channel_config_set_dreq(&dma_conf, pio_get_dreq(pio1, 2, true));
		dma_channel_configure(tx_dma, &dma_conf, &pio1->txf[2], lo_cos, UINT_MAX, true);

		puts("Transmitting noise, press ENTER to stop.");

		while (true) {
			int c = getchar_timeout_us(0);
			if ('\n' == c || '\r' == c)
				break;

			for (int i = 0; i < LO_WORDS; i++)
				lo_cos[i] = rand();
		}

		dma_channel_abort(tx_dma);
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

	stdio_usb_init();

	for (int i = 0; i < 30; i++) {
		if (stdio_usb_connected())
			break;

		sleep_ms(100);
	}

	tx_dma = dma_claim_unused_channel(true);
	rx_dma = dma_claim_unused_channel(true);

	printf("\nPuppet Online!\n");
	printf("clk_sys = %10.6f MHz\n", (float)clock_get_hz(clk_sys) / MHZ);

#if SPEED < 3
	printf("Generating mixer lookup table...\n");
	generate_mixer();
#endif

	static char cmd[83];
	int cmdlen = 0;

	printf("> ");

	while (true) {
		int c;

		while ((c = getchar_timeout_us(10000)) >= 0) {
			if (13 == c) {
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

			if ((13 == c) || cmdlen == 80) {
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
