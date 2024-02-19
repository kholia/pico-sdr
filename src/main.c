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
#include <hardware/interp.h>

#include <hardware/regs/clocks.h>

#include <math.h>
#include <stdio.h>
#include <limits.h>
#include <stdlib.h>

#define CLK_SYS_HZ (250 * MHZ)
#define CLKDIV_RF 2
#define CLK_RF_HZ ((unsigned)(CLK_SYS_HZ / CLKDIV_RF))

#define EXTRA_BITS 8
#define NUM_SAMPLES 128
#define LPF_SAMPLES 0 /* 4 */

#define LO_BITS_DEPTH 13
#define LO_WORDS (1 << LO_BITS_DEPTH)
static uint32_t lo_cos[LO_WORDS] __attribute__((__aligned__(LO_WORDS * 4)));
static uint32_t lo_sin[LO_WORDS] __attribute__((__aligned__(LO_WORDS * 4)));
static uint32_t rx_buf[LO_WORDS] __attribute__((__aligned__(LO_WORDS * 4)));

#define IQ_BLOCK_LEN 60
static queue_t rx_queue;

static int tx_dma = -1;
static int rx_dma = -1;

static volatile struct status {
	unsigned sample_rate;
	int gap;
} status;

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
	sm_config_set_sideset(&pc, 2, false, true);
	sm_config_set_sideset_pins(&pc, out_pin);
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
	sm_config_set_clkdiv_int_frac(&pc, CLKDIV_RF, 0);
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
	sm_config_set_clkdiv_int_frac(&pc, CLKDIV_RF, 0);
	sm_config_set_fifo_join(&pc, PIO_FIFO_JOIN_TX);
	sm_config_set_out_shift(&pc, false, true, 32);
	pio_sm_init(pio1, 2, 2, &pc);

	pio_sm_set_consecutive_pindirs(pio1, 2, out_pin, 1, GPIO_OUT);

	pio_sm_set_enabled(pio1, 2, true);
}

static float lo_freq_init(float req_freq)
{
	const float step_hz = (float)CLK_RF_HZ / (LO_WORDS * 32);
	float freq = roundf(req_freq / step_hz) * step_hz;

	unsigned step = (float)UINT_MAX / (float)CLK_RF_HZ * freq;

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

inline static unsigned popcount(unsigned v)
{
	v = v - ((v >> 1) & 0x55555555);
	v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
	return (((v + (v >> 4)) & 0x0f0f0f0f) * 0x01010101) >> 24;
}

__noinline static int mix(uint32_t *buf, uint32_t *lo, int len)
{
	int x = 0;

	for (int k = 0; k < len; k++)
		x += popcount(buf[k] ^ lo[k]);

	return x;
}

static void rf_rx(void)
{
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

	unsigned prev_transfers = 0;

	static int16_t block[IQ_BLOCK_LEN];
	int block_ptr = 0;

	while (true) {
		int delta = ~dma_hw->ch[rx_dma].transfer_count - prev_transfers;
		int gap = NUM_SAMPLES - delta;

		while (delta < NUM_SAMPLES)
			delta = ~dma_hw->ch[rx_dma].transfer_count - prev_transfers;

		if (gap < 0)
			prev_transfers += NUM_SAMPLES;

		unsigned pos = prev_transfers & (LO_WORDS - 1);
		prev_transfers += NUM_SAMPLES;

		int I = mix(rx_buf + pos, lo_cos + pos, NUM_SAMPLES);
		int Q = mix(rx_buf + pos, lo_sin + pos, NUM_SAMPLES);

		/*
		 * Limit our biasing activity to a short period at a time.
		 * This is sufficient to keep the input biased while limiting
		 * the interference from biasing to minimum.
		 */
		pio_sm_exec(pio1, 0, pio_encode_sideset(2, 3) | pio_encode_nop());

		/* Remove the large DC bias. */
		I -= 16 * NUM_SAMPLES;
		Q -= 16 * NUM_SAMPLES;

		/* Not sure why, but this removes a small DC bias. */
		I -= NUM_SAMPLES / 16;
		Q -= NUM_SAMPLES / 16;

		/* Normalize to given number of bits. */
		I = (I * ((1 << (7 + EXTRA_BITS)) - 1)) / 16;
		Q = (Q * ((1 << (7 + EXTRA_BITS)) - 1)) / 16;

		I /= NUM_SAMPLES;
		Q /= NUM_SAMPLES;

		/* Pause biasing. */
		pio_sm_exec(pio1, 0, pio_encode_sideset(2, 2) | pio_encode_nop());

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

		status.sample_rate = CLK_RF_HZ / (NUM_SAMPLES * 32);
		status.gap = gap;

		block[block_ptr++] = I;
		block[block_ptr++] = Q;

		if (block_ptr >= IQ_BLOCK_LEN) {
			queue_try_add(&rx_queue, block);
			block_ptr = 0;
		}
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

		while (true) {
			int c = getchar_timeout_us(0);

			if (13 == c) {
				multicore_reset_core1();
				dma_channel_abort(rx_dma);
				dma_channel_cleanup(rx_dma);
				puts("stopped");
				break;
			}

			static int16_t block[IQ_BLOCK_LEN];

			while (queue_try_remove(&rx_queue, block)) {
				putchar('!');

				for (int i = 0; i < IQ_BLOCK_LEN; i++)
					printf("%04hx", block[i]);

				putchar('\n');
			}

			printf("gap=%i, sr=%i\n", status.gap, status.sample_rate);
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

		const float step_hz = (float)CLK_RF_HZ / (LO_WORDS * 32);
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

	queue_init(&rx_queue, IQ_BLOCK_LEN * 2, 64);

	printf("\nPuppet Online!\n");
	printf("clk_sys = %10.6f MHz\n", (float)clock_get_hz(clk_sys) / MHZ);

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
