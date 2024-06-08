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

#define VREG_VOLTAGE VREG_VOLTAGE_1_20
#define CLK_SYS_HZ (300 * MHZ)
#define PSU_PIN 23

#define IQ_SAMPLES 32
#define IQ_BLOCK_LEN (2 * IQ_SAMPLES)

#define XOR_ADDR 0x1000
#define LO_COS_ACCUMULATOR (&pio1->sm[2].pinctrl)
#define LO_SIN_ACCUMULATOR (&pio1->sm[3].pinctrl)

#define LO_BITS_DEPTH 15
#define LO_WORDS (1 << (LO_BITS_DEPTH - 2))
static uint32_t lo_cos[LO_WORDS] __attribute__((__aligned__(1 << LO_BITS_DEPTH)));
static uint32_t lo_sin[LO_WORDS] __attribute__((__aligned__(1 << LO_BITS_DEPTH)));

#define DECIMATE 2
#define RX_STRIDE (2 * IQ_SAMPLES * DECIMATE)
#define RX_BITS_DEPTH 12
#define RX_WORDS (1 << (RX_BITS_DEPTH - 2))

static_assert(RX_STRIDE * 4 < RX_WORDS, "RX_STRIDE * 4 < RX_WORDS");

static uint32_t rx_cos[RX_WORDS] __attribute__((__aligned__(1 << RX_BITS_DEPTH)));
static uint32_t rx_sin[RX_WORDS] __attribute__((__aligned__(1 << RX_BITS_DEPTH)));

#define INIT_GAIN 120
#define INIT_SAMPLE_RATE 100000
#define INIT_FREQ 94600000

#define NUM_GAINS 29
static int gains[NUM_GAINS] = { 0,   9,	  14,  27,  37,	 77,  87,  125, 144, 157,
				166, 197, 207, 229, 254, 280, 297, 328, 338, 364,
				372, 386, 402, 421, 434, 439, 445, 480, 496 };
static int gain = INIT_GAIN;
static int sample_rate = INIT_SAMPLE_RATE;

#define SIN_PHASE (UINT_MAX / 4)
#define COS_PHASE (0)

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
		pio_encode_set(pio_x, 31) | pio_encode_sideset(1, 0) | pio_encode_delay(15),
		pio_encode_jmp_x_dec(2) | pio_encode_sideset(1, 0) | pio_encode_delay(15),
	};

	pio_program_t prog = {
		.instructions = insn,
		.length = sizeof(insn) / sizeof(*insn),
		.origin = 10,
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
		.origin = 6,
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

static void lo_generate(uint32_t *buf, size_t len, double freq, unsigned phase)
{
	static const double base = (UINT_MAX + 1.0) / CLK_SYS_HZ;

	unsigned step = base * freq;
	unsigned accum = phase;

	for (size_t i = 0; i < len; i++) {
		unsigned bits = 0;

		for (int j = 0; j < 32; j++) {
			bits |= accum >> 31;
			bits <<= 1;
			accum += step;
		}

		buf[i] = bits;
	}
}

static void rx_lo_init(double req_freq)
{
	const double step_hz = (double)CLK_SYS_HZ / (4 << LO_BITS_DEPTH);
	double freq = round(req_freq / step_hz) * step_hz;

	lo_generate(lo_cos, LO_WORDS, freq, COS_PHASE);
	lo_generate(lo_sin, LO_WORDS, freq, SIN_PHASE);
}

static const uint32_t samp_insn[4] __attribute__((__aligned__(16)));
static const uint32_t samp_insn[4] = {
	0x4040, /* IN Y, 32 */
	0x4020, /* IN X, 32 */
	0xe040, /* SET Y, 0 */
	0xe020, /* SET X, 0 */
};

static uint32_t null, one = 1;

static void rf_rx_start(int rx_pin, int bias_pin)
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
	channel_config_set_ring(&dma_conf, false, LO_BITS_DEPTH);
	channel_config_set_chain_to(&dma_conf, dma_ch_sin);
	dma_channel_configure(dma_ch_cos, &dma_conf, LO_COS_ACCUMULATOR + XOR_ADDR / 4, lo_cos, 1,
			      false);

	/* Read lo_sin into accumulator Q with XOR. */
	dma_conf = dma_channel_get_default_config(dma_ch_sin);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_conf, true);
	channel_config_set_write_increment(&dma_conf, false);
	channel_config_set_ring(&dma_conf, false, LO_BITS_DEPTH);
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
	dma_timer_set_fraction(dma_t_samp, 1, CLK_SYS_HZ / (sample_rate * DECIMATE));

	/* Sampling trigger channel. */
	dma_conf = dma_channel_get_default_config(dma_ch_samp_trig);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_conf, false);
	channel_config_set_write_increment(&dma_conf, false);
	channel_config_set_dreq(&dma_conf, dma_get_timer_dreq(dma_t_samp));
	channel_config_set_high_priority(&dma_conf, true);
	channel_config_set_chain_to(&dma_conf, dma_ch_samp_cos);
	dma_channel_configure(dma_ch_samp_trig, &dma_conf, &null, &one, 1, false);

	/* Trigger I accumulator values push. */
	dma_conf = dma_channel_get_default_config(dma_ch_samp_cos);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_conf, true);
	channel_config_set_write_increment(&dma_conf, false);
	channel_config_set_ring(&dma_conf, false, 4);
	channel_config_set_high_priority(&dma_conf, true);
	channel_config_set_chain_to(&dma_conf, dma_ch_samp_sin);
	dma_channel_configure(dma_ch_samp_cos, &dma_conf, &pio1->sm[2].instr, samp_insn, 4, false);

	/* Trigger Q accumulator values push. */
	dma_conf = dma_channel_get_default_config(dma_ch_samp_sin);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_conf, true);
	channel_config_set_write_increment(&dma_conf, false);
	channel_config_set_ring(&dma_conf, false, 4);
	channel_config_set_high_priority(&dma_conf, true);
	channel_config_set_chain_to(&dma_conf, dma_ch_samp_trig);
	dma_channel_configure(dma_ch_samp_sin, &dma_conf, &pio1->sm[3].instr, samp_insn, 4, false);

	bias_init(rx_pin, bias_pin);
	adder_init();

	dma_channel_start(dma_ch_rx);
	dma_channel_start(dma_ch_samp_trig);
	watch_init(rx_pin);
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

static void rf_rx(void)
{
	static uint8_t block[IQ_BLOCK_LEN];
	uint32_t prev_transfers = dma_hw->ch[dma_ch_in_cos].transfer_count;
	unsigned pos = 0;

	int64_t dcI = 0, dcQ = 0;
	int prevI = 0, prevQ = 0;

	while (true) {
		if (multicore_fifo_rvalid()) {
			multicore_fifo_pop_blocking();
			multicore_fifo_push_blocking(0);
			return;
		}

		int delta = prev_transfers - dma_hw->ch[dma_ch_in_cos].transfer_count;

		while (delta < RX_STRIDE) {
			delta = prev_transfers - dma_hw->ch[dma_ch_in_cos].transfer_count;
			sleep_us(1);
		}

		prev_transfers -= RX_STRIDE;

		uint32_t *cos_ptr = rx_cos + pos;
		uint32_t *sin_ptr = rx_sin + pos;

		pos = (pos + RX_STRIDE) & (RX_WORDS - 1);

		uint8_t *blockptr = block;

		/*
		 * Since every 2 samples add to either +1 or -1,
		 * the maximum amplitude in one direction is 1/2.
		 */
		int64_t max_amplitude = CLK_SYS_HZ / 2;

		/*
		 * Since the waveform is normally half of the time
		 * above zero, we can halve once more.
		 *
		 * This is not perfect, so we do not max out the base
		 * gain but keep it slightly below the maximum to make
		 * sure we do not overshoot often.
		 */
		max_amplitude = max_amplitude / 2;

		/*
		 * We are allowing the counters to only go as high
		 * as sampling rate.
		 */
		max_amplitude /= sample_rate;

		for (int i = 0; i < IQ_SAMPLES; i++) {
			uint32_t cos_neg = *cos_ptr++;
			uint32_t cos_pos = *cos_ptr++;
			int I1 = cos_neg - cos_pos;

			cos_neg = *cos_ptr++;
			cos_pos = *cos_ptr++;
			int I2 = cos_neg - cos_pos;

			int64_t I = (prevI + I1 + I1 + I2) / 2;
			prevI = I2;

			int64_t I16 = I << 16;
			dcI = ((dcI << 16) - dcI + I16) >> 16;
			I = (I16 - dcI) >> 16;

			I *= gain;
			I /= max_amplitude;

			if (I > 127)
				I = 127;
			else if (I < -128)
				I = -128;

			*blockptr++ = I + 128;

			uint32_t sin_neg = *sin_ptr++;
			uint32_t sin_pos = *sin_ptr++;
			int Q1 = sin_neg - sin_pos;

			sin_neg = *sin_ptr++;
			sin_pos = *sin_ptr++;
			int Q2 = sin_neg - sin_pos;

			int64_t Q = (prevQ + Q1 + Q1 + Q2) / 2;
			prevQ = Q2;

			int64_t Q16 = Q << 16;
			dcQ = ((dcQ << 16) - dcQ + Q16) >> 16;
			Q = (Q16 - dcQ) >> 16;

			Q *= gain;
			Q /= max_amplitude;

			if (Q > 127)
				Q = 127;
			else if (Q < -128)
				Q = -128;

			*blockptr++ = (uint8_t)Q + 128;
		}

		(void)queue_try_add(&iq_queue, block);
	}
}

static void run_command(uint8_t cmd, uint32_t arg)
{
	if (0x01 == cmd) {
		/* Tune to a new center frequency */
		rx_lo_init(arg);
	} else if (0x02 == cmd) {
		/* Set the rate at which IQ sample pairs are sent */
		sample_rate = arg;
		dma_timer_set_fraction(dma_t_samp, 1, CLK_SYS_HZ / (sample_rate * DECIMATE));
	} else if (0x04 == cmd) {
		/* Set the tuner gain level */
		gain = INIT_GAIN * powf(10.0f, 0.005f * arg);
	} else if (0x0d == cmd) {
		/* Set tuner gain by the tuner's gain index */
		if (arg < NUM_GAINS) {
			gain = INIT_GAIN * powf(10.0f, 0.005f * gains[arg]);
		}
	}
}

static int check_command(void)
{
	static uint8_t buf[5];
	static int pos = 0;

	int c;

	while ((c = getchar_timeout_us(0)) >= 0) {
		if (0 == pos && 0 == c)
			return 0;

		buf[pos++] = c;

		if (5 == pos) {
			uint32_t arg = (buf[1] << 24) | (buf[2] << 16) | (buf[3] << 8) | buf[4];
			run_command(buf[0], arg);
			pos = 0;
			return buf[0];
		}
	}

	return -1;
}

static void do_rx(int rx_pin, int bias_pin)
{
	rf_rx_start(rx_pin, bias_pin);
	sleep_us(100);

	dma_ch_in_cos = dma_claim_unused_channel(true);
	dma_ch_in_sin = dma_claim_unused_channel(true);

	dma_channel_config dma_conf;

	dma_conf = dma_channel_get_default_config(dma_ch_in_cos);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_conf, false);
	channel_config_set_write_increment(&dma_conf, true);
	channel_config_set_ring(&dma_conf, true, RX_BITS_DEPTH);
	channel_config_set_dreq(&dma_conf, pio_get_dreq(pio1, 2, false));
	dma_channel_configure(dma_ch_in_cos, &dma_conf, rx_cos, &pio1->rxf[2], UINT_MAX, false);

	dma_conf = dma_channel_get_default_config(dma_ch_in_sin);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_conf, false);
	channel_config_set_write_increment(&dma_conf, true);
	channel_config_set_ring(&dma_conf, true, RX_BITS_DEPTH);
	channel_config_set_dreq(&dma_conf, pio_get_dreq(pio1, 3, false));
	dma_channel_configure(dma_ch_in_sin, &dma_conf, rx_sin, &pio1->rxf[3], UINT_MAX, false);

	dma_start_channel_mask((1 << dma_ch_in_sin) | (1 << dma_ch_in_cos));

	multicore_launch_core1(rf_rx);

	static uint8_t block[IQ_BLOCK_LEN];

	while (queue_try_remove(&iq_queue, block))
		/* Flush the queue */;

	while (true) {
		int cmd;

		while ((cmd = check_command()) >= 0)
			if (0 == cmd)
				goto done;

		if (queue_try_remove(&iq_queue, block)) {
			fwrite(block, sizeof block, 1, stdout);
			fflush(stdout);
		}
	}

done:
	multicore_fifo_push_blocking(0);
	multicore_fifo_pop_blocking();
	sleep_us(10);
	multicore_reset_core1();

	rf_rx_stop();

	dma_channel_abort(dma_ch_in_cos);
	dma_channel_abort(dma_ch_in_sin);
	dma_channel_cleanup(dma_ch_in_cos);
	dma_channel_cleanup(dma_ch_in_sin);
	dma_channel_unclaim(dma_ch_in_cos);
	dma_channel_unclaim(dma_ch_in_sin);
	dma_ch_in_cos = -1;
	dma_ch_in_sin = -1;
}

int main()
{
	vreg_set_voltage(VREG_VOLTAGE);
	set_sys_clock_khz(CLK_SYS_HZ / KHZ, true);
	clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, CLK_SYS_HZ,
			CLK_SYS_HZ);

	/* Enable PSU PWM mode. */
	gpio_init(PSU_PIN);
	gpio_set_dir(PSU_PIN, GPIO_OUT);
	gpio_put(PSU_PIN, 1);

	bus_ctrl_hw->priority |= BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

	stdio_usb_init();
	setvbuf(stdout, NULL, _IONBF, 0);

	queue_init(&iq_queue, IQ_BLOCK_LEN, 256);

	rx_lo_init(INIT_FREQ);

	while (true) {
		if (check_command() > 0) {
			static const uint32_t header[3] = { __builtin_bswap32(0x52544c30),
							    __builtin_bswap32(5),
							    __builtin_bswap32(NUM_GAINS) };
			fwrite(header, sizeof header, 1, stdout);
			fflush(stdout);

			do_rx(10, 11);
		}

		sleep_ms(10);
	}
}
