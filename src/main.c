#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <nrfx_spim.h>
//#include <zephyr/dt-bindings/adc/adc.h>
#include "SEGGER_RTT.h"
//#include "/opt/nordic/ncs/v2.5.0/modules/debug/segger/SEGGER/SEGGER_RTT.h"


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

#define SW0_NODE	DT_ALIAS(sw0)
/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,{0});
static struct gpio_callback button_cb_data;

/** @brief Symbol specifying SPIM instance to be used. */
#define SPIM_INST_IDX 1

/** @brief Symbol specifying pin number for MOSI. */
#define MOSI_PIN 30

/** @brief Symbol specifying pin number for MISO. */
#define MISO_PIN 31

/** @brief Symbol specifying pin number for SCK. */
#define SCK_PIN 29

/** @brief Symbol specifying message to be sent via SPIM data transfer. */
#define MSG_TO_SEND "Nordic Semiconductor"

/** @brief Transmit buffer initialized with the specified message ( @ref MSG_TO_SEND ). */
static uint8_t m_tx_buffer[] = MSG_TO_SEND;

/** @brief Receive buffer defined with the size to store specified message ( @ref MSG_TO_SEND ). */
static uint8_t m_rx_buffer[sizeof(MSG_TO_SEND)];


#if 1
	int16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
                .resolution = 8,
	};
 #endif       
void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	int ret;
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
	ret = gpio_pin_toggle_dt(&led);
	if (ret < 0) {
		//return 0;
	}	

}
uint32_t data;
int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}
        adc_is_ready_dt(&adc_channels[0]);
        adc_channel_setup_dt(&adc_channels[0]);

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	ret = gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);

	if (ret < 0) {
		return 0;
	}
	gpio_pin_configure_dt(&button, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

        //sequence.channels = BIT(0); // ADC 채널 설정, 여기서는 0번 채널 사용
        //sequence.buffer = &adc_raw_value;
        //sequence.buffer_size = sizeof(adc_raw_value);
        //sequence.resolution = 12; // ADC 해상도 설정 (예: 12비트)
        //adc_sequence_init(&sequence);
        //adc_sequence_init_dt(adc_dev, &sequence);
    nrfx_spim_t spim_inst ;//= NRFX_SPIM_INSTANCE(SPIM_INST_IDX);
	spim_inst.drv_inst_idx = 1;
	spim_inst.p_reg = 0x40003000;

    nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG(SCK_PIN,
                                                              MOSI_PIN,
                                                              MISO_PIN,
                                                              NRF_SPIM_PIN_NOT_CONNECTED);

	nrfx_spim_init(&spim_inst, &spim_config, NULL, NULL);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TRX(m_tx_buffer, sizeof(m_tx_buffer),
                                                              m_rx_buffer, sizeof(m_rx_buffer));

	

	while (1) {

		k_msleep(SLEEP_TIME_MS);
		printk("test : %d\n",buf);
		//SEGGER_RTT_Write(unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
		SEGGER_RTT_WriteString(0, "test");
                //SEGGER_RTT_printf(0,"daa");
		gpio_pin_toggle_dt(&led2);
                adc_sequence_init_dt(&adc_channels[0], &sequence);
                adc_read(adc_channels[0].dev, &sequence);
                data++;
				nrfx_spim_xfer(&spim_inst, &spim_xfer_desc, 0);
	}
	return 0;
}
