#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "ssd1306.h"
#include "kiss_fftr.h"

#define max(X, Y) (((X) > (Y)) ? (X) : (Y))

// set this to determine sample rate
// 0     = 500,000 Hz
// 960   = 50,000 Hz
// 9600  = 5,000 Hz
#define CLOCK_DIV 960
#define FSAMP 50000

// Channel 0 is GPIO26
#define CAPTURE_CHANNEL 0
#define POT_CHANNEL 1
#define NSAMP 512

// globals
dma_channel_config dma_cfg;
uint dma_chan;
float freqs[NSAMP];

void setup();

void sample(uint8_t *capture_buf);

uint16_t read_pot();

int main() {
    setbuf(stdout, 0);
    uint8_t cap_buf[NSAMP];
    kiss_fft_scalar fft_in[NSAMP];
    kiss_fft_cpx fft_out[NSAMP];
    kiss_fftr_cfg fft_cfg = kiss_fftr_alloc(NSAMP, false, 0, 0);
    setup();

    ssd1306_t disp;
    disp.external_vcc = false;
    ssd1306_init(&disp, 128, 32, 0x3C, i2c1);
    ssd1306_clear(&disp);

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    while (1) {
        // get NSAMP samples at FSAMP
        sample(cap_buf);
        // fill fourier transform input while subtracting DC component
        uint64_t sum = 0;
        for (int i = 0; i < NSAMP; i++) { sum += cap_buf[i]; }
        float avg = (float) sum / NSAMP;
        for (int i = 0; i < NSAMP; i++) { fft_in[i] = (float) cap_buf[i] - avg; }

        // compute fast fourier transform
        kiss_fftr(fft_cfg, fft_in, fft_out);

        ssd1306_clear(&disp);
        uint16_t pot_value = read_pot();
        for (int i = 0; i < NSAMP / 2; i++) {
            int y = (int) ((fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i) / 2) / pot_value;
            int x = i / 2;
            ssd1306_draw_line(&disp, x, max(0, 22 - y), x, 23);
            if (x > 0 && x % 32 == 0) {
                float freq = freqs[i];
                char temp_str[8];
                snprintf(temp_str, 8, "%.0fk", (freq / 1000));
                ssd1306_draw_line(&disp, x, 20, x, 24);
                int offset = (strlen(strtok(temp_str, " ")) * 6 / 2);
                ssd1306_draw_string(&disp, x - offset, 25, 1, temp_str);
            }

        }
        ssd1306_draw_line(&disp, 0, 20, 0, 28);
        ssd1306_draw_line(&disp, 127, 20, 127, 28);
        ssd1306_show(&disp);
    }

#pragma clang diagnostic pop
    kiss_fft_free(fft_cfg);
}

uint16_t read_pot() {
    adc_select_input(POT_CHANNEL);
    return adc_read();
}

void sample(uint8_t *capture_buf) {
    adc_fifo_drain();
    adc_run(false);

    dma_channel_configure(dma_chan, &dma_cfg,
                          capture_buf,    // dst
                          &adc_hw->fifo,  // src
                          NSAMP,          // transfer count
                          true            // start immediately
    );

    adc_run(true);
    dma_channel_wait_for_finish_blocking(dma_chan);
}

void setup() {
    i2c_init(i2c1, 400000);
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);
    stdio_init_all();

    adc_gpio_init(26 + CAPTURE_CHANNEL);

    adc_init();
    adc_gpio_init(27);
    adc_select_input(CAPTURE_CHANNEL);
    adc_fifo_setup(
            true,    // Write each completed conversion to the sample FIFO
            true,    // Enable DMA data request (DREQ)
            1,       // DREQ (and IRQ) asserted when at least 1 sample present
            false,   // We won't see the ERR bit because of 8 bit reads; disable.
            true     // Shift each sample to 8 bits when pushing to FIFO
    );

    // set sample rate
    adc_set_clkdiv(CLOCK_DIV);

    // Set up the DMA to start transferring data as soon as it appears in FIFO
    dma_chan = dma_claim_unused_channel(true);
    dma_cfg = dma_channel_get_default_config(dma_chan);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_cfg, false);
    channel_config_set_write_increment(&dma_cfg, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&dma_cfg, DREQ_ADC);

    // calculate frequencies of each bin
    float f_max = FSAMP;
    float f_res = f_max / NSAMP;
    for (int i = 0; i < NSAMP; i++) { freqs[i] = f_res * (float) i; }
}