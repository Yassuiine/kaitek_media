#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/dt-bindings/adc/nrf-saadc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(piezo_adc, LOG_LEVEL_INF);

#define ADC_NODE       DT_NODELABEL(adc)
#define ADC_CHANNEL    0
#define ADC_RESOLUTION 12
#define VREF_MV        600
#define VREF_GAIN      ADC_GAIN_1_4

static const struct device *adc_dev;
static int16_t sample_buffer;

static const struct adc_channel_cfg ch_cfg = {
    .gain             = ADC_GAIN_1_4,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = ADC_CHANNEL,
    .input_positive   = NRF_SAADC_AIN6,   /* P1.04 */
};

static struct adc_sequence sequence = {
    .channels    = BIT(ADC_CHANNEL),
    .buffer      = &sample_buffer,
    .buffer_size = sizeof(sample_buffer),
    .resolution  = ADC_RESOLUTION,
};

int main(void)
{
    int ret;

    adc_dev = DEVICE_DT_GET(ADC_NODE);
    if (!device_is_ready(adc_dev)) {
        printk("ADC device not ready\n");
        return -1;
    }
    printk("ADC device ready\n");

    ret = adc_channel_setup(adc_dev, &ch_cfg);
    if (ret < 0) {
        printk("adc_channel_setup failed: %d\n", ret);
        return ret;
    }
    printk("ADC canal 0 — P1.04 (AIN6) configure\n");

    while (1) {
        ret = adc_read(adc_dev, &sequence);
        if (ret < 0) {
            printk("adc_read failed: %d\n", ret);
            k_sleep(K_MSEC(100));
            continue;
        }

        int32_t mv = sample_buffer;
        adc_raw_to_millivolts(VREF_MV, VREF_GAIN, ADC_RESOLUTION, &mv);

        printk("RAW: %6d | Tension: %4d mV\n", sample_buffer, mv);

        k_sleep(K_MSEC(10));
    }

    return 0;
}