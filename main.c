#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "ssd1306.h"
#include "kiss_fftr.h"

#define max(X, Y) (((X) > (Y)) ? (X) : (Y))
#define CLOCK_DIV 960
#define FSAMP 50000
#define CAPTURE_CHANNEL 0
#define NSAMP 1024
#define ZOOMED_IN_MAX_INDEX 128
#define MEDIUM_ZOOM_MAX_INDEX 256
#define ZOOMED_OUT_MAX_INDEX 512

dma_channel_config dma_cfg;
uint dma_chan;
float freqs[NSAMP];
int zoom_levels[] = {ZOOMED_OUT_MAX_INDEX, MEDIUM_ZOOM_MAX_INDEX, ZOOMED_IN_MAX_INDEX};
int zoom_level = 0;

void gpio_callback(uint gpio);

void setup();

void sample(uint8_t *capture_buf);

void draw_vertical_guideline(ssd1306_t disp, int x, int length);

void draw_horizontal_guidelines(ssd1306_t disp);

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
        sample(cap_buf);
        uint64_t sum = 0;
        for (int i = 0; i < NSAMP; i++) { sum += cap_buf[i]; }
        float avg = (float) sum / NSAMP;
        for (int i = 0; i < NSAMP; i++) { fft_in[i] = (float) cap_buf[i] - avg; }

        kiss_fftr(fft_cfg, fft_in, fft_out);

        ssd1306_clear(&disp);
        draw_horizontal_guidelines(disp);
        double factor = 0.00000015;
        for (int x = 0; x < 127; x++) {
            int i = (int) ((float) zoom_levels[zoom_level] / (float) 128 * (float) x);
            int y = (int) ((fft_out[i].r * fft_out[i].r + fft_out[i].i * fft_out[i].i +
                            fft_out[i + 1].r * fft_out[i + 1].r + fft_out[i + 1].i * fft_out[i + 1].i) *
                           factor);
            ssd1306_draw_line(&disp, x, max(0, 23 - y), x, 23);
            if (x > 0 && x % 32 == 0) {
                draw_vertical_guideline(disp, x, 24);
                float freq = freqs[i];
                char temp_str[8];
                snprintf(temp_str, 8, "%.0fk", roundf(freq / 1000));
                ssd1306_draw_string(&disp, x - (strlen(strtok(temp_str, " ")) * 6 / 2), 25, 1, temp_str);
            }
        }
        ssd1306_show(&disp);
    }

#pragma clang diagnostic pop
    kiss_fft_free(fft_cfg);
}

void draw_vertical_guideline(ssd1306_t disp, int x, int length) {
    for (int y = 0; y < length + 1; y += 2) {
        ssd1306_draw_pixel(&disp, x, y);
    }
}

void draw_horizontal_guidelines(ssd1306_t disp) {
    for (int y = 8; y < 24; y += 8) {
        for (int x = 0; x < 127; x += 2) {
            ssd1306_draw_pixel(&disp, x, y);
        }
    }
}

void gpio_callback(uint gpio) {
    if (gpio == 16) {
        zoom_level++;
        if (zoom_level == 3) {
            zoom_level = 0;
        }
    }
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
    gpio_pull_up(16);
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

    gpio_set_irq_enabled_with_callback(16, GPIO_IRQ_EDGE_FALL, true, (gpio_irq_callback_t) &gpio_callback);
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