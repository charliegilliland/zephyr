/*
 * Copyright (c) 2024 Charlie Gilliland
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_dfsdm_dmic

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(stm32_dfsdm_dmic, CONFIG_AUDIO_DMIC_LOG_LEVEL);

//#include <string.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_stm32.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/audio/dmic.h>
#include <soc.h>

/*
TODO: i think soc.h will include the dfsdm header if it exists for the target
need a way to check whether the target device is compatible with dfsdm - does Kconfig already do this??

#include <stm32f7xx_hal_dfsdm.h> // TODO: need a macro to include the appropriate version of this header file - is there a better way?
*/

struct dfsdm_stm32_chan_cfg {
	// DMA
	const struct device *dma_dev;
	uint32_t dma_chan;
	struct dma_config dma_cfg;

	// TODO: maybe interrupts

	// TODO: its probably possible to get the Channel_inst defined through the macro magic...
	int channel;
	int filter;
	int dfsdm_inst; // TODO: thie needs to be set during init

	// These parameters could probably be fetched inside functions later
	// That would allow us to use the same bindings as the linux driver
	int sample_edge;
	int alt_channel; // default 0, or 1 in the case of following channel's pin
	uint32_t filter_order;

	// TODO: these values need to be set during clock setup
	int chan_clk_div, filt_oversample;

	/* --- dfsdm-chan stuff --- */
	DFSDM_Channel_HandleTypeDef hdfsdm_channel;
	/* --- dfsdm-filt stuff --- */
	DFSDM_Filter_HandleTypeDef hdfsdm_filter;
};


struct dfsdm_stm32_drv_data {

	// TODO: stuff like buffers, k_msg_q, other runtime data will be stored in here
	struct k_mem_slab *mem_slab;
	uint32_t block_size;

	uint8_t act_num_channels;

	enum dmic_state dmic_state;
};

struct dfsdm_stm32_drv_cfg {
	// Clocks
	const struct stm32_pclken *pclken;
	size_t pclk_len;

	// Pinctrl
	const struct pinctrl_dev_config *pcfg;

	unsigned long dfsdm_base_addr;

	// array of chan_cfg structs
	int num_channels;
	struct dfsdm_stm32_chan_cfg **channels;
};

static DFSDM_Channel_TypeDef *get_hal_channel_instance(int dfsdm_inst, int channel)
{
	/* TODO: make sure this function covers all dfsdm instance / channel count for all chips*/

	if (dfsdm_inst == 1) {
		switch (channel) {
			case 0:
				return DFSDM1_Channel0;
			case 1:
				return DFSDM1_Channel1;
			case 2:
				return DFSDM1_Channel2;
			case 3:
				return DFSDM1_Channel3;
#ifdef DFSDM1_Channel4 // TODO: make sure this macro covers all cases
			case 4:
				return DFSDM1_Channel4;
			case 5:
				return DFSDM1_Channel5;
			case 6:
				return DFSDM1_Channel6;
			case 7:
				return DFSDM1_Channel7;
#endif /* DFSDM1_Channel4 */
		}
	}
#ifdef DFSDM2_BASE
	else if (dfsdm_inst == 2) {
		switch (channel) {
			case 0:
				return DFSDM2_Channel0;
			case 1:
				return DFSDM2_Channel1;
			case 2:
				return DFSDM2_Channel2;
			case 3:
				return DFSDM2_Channel3;
#ifdef DFSDM2_Channel4 // TODO: make sure this macro covers all cases
			case 4:
				return DFSDM2_Channel4;
			case 5:
				return DFSDM2_Channel5;
			case 6:
				return DFSDM2_Channel6;
			case 7:
				return DFSDM2_Channel7;
#endif /* DFSDM2_Channel4 */
		}
	}
#endif /* DFSDM2_BASE */

	// error
	return NULL;
}

static DFSDM_Filter_TypeDef *get_hal_filter_instance(int dfsdm_inst, int filter)
{
	/* TODO: make sure this function covers all dfsdm instance / filter count for all chips*/

	if (dfsdm_inst == 1) {
		switch (filter) {
			case 0:
				return DFSDM1_Filter0;
			case 1:
				return DFSDM1_Filter1;
#ifdef DFSDM1_Filter2 // TODO: make sure this macro covers all cases
			case 2:
				return DFSDM1_Filter2;
			case 3:
				return DFSDM1_Filter3;
#endif /* DFSDM1_Filter2 */
		}
	}
#ifdef DFSDM2_BASE
	else if (dfsdm_inst == 2) {
		switch (filter) {
			case 0:
				return DFSDM2_Filter0;
			case 1:
				return DFSDM2_Filter1;
#ifdef DFSDM2_Filter2 // TODO: make sure this macro covers all cases
			case 2:
				return DFSDM2_Filter2;
			case 3:
				return DFSDM2_Filter3;
#endif /* DFSDM2_Filter2 */
		}
	}
#endif /* DFSDM2_BASE */

	return NULL;
}

static int dfsdm_channel_init(struct dfsdm_stm32_chan_cfg *chan_cfg)
{
	int err = HAL_OK;
	DFSDM_Channel_HandleTypeDef *hdfsdm_channel = &(chan_cfg->hdfsdm_channel);
	DFSDM_Filter_HandleTypeDef *hdfsdm_filter = &(chan_cfg->hdfsdm_filter);

	__HAL_DFSDM_CHANNEL_RESET_HANDLE_STATE(hdfsdm_channel);

	hdfsdm_channel->Instance = get_hal_channel_instance(chan_cfg->dfsdm_inst, chan_cfg->channel);

	// TODO: get this value from devicetree
	hdfsdm_channel->Init.OutputClock.Activation = ENABLE;
	hdfsdm_channel->Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_AUDIO;
	hdfsdm_channel->Init.OutputClock.Divider = chan_cfg->chan_clk_div;
	hdfsdm_channel->Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
	hdfsdm_channel->Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE; /* N.U. */

	if (chan_cfg->alt_channel == 1) {
		hdfsdm_channel->Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
	} else {
		hdfsdm_channel->Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
	}

	// TODO: change this to match linux dt binding (compare string prop SPI_R / SPI_F)
	hdfsdm_channel->Init.SerialInterface.Type = chan_cfg->sample_edge;

	// TODO: some of these values might also be taken from devicetree
	hdfsdm_channel->Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
	hdfsdm_channel->Init.Awd.FilterOrder          = DFSDM_CHANNEL_FASTSINC_ORDER; /* N.U. */
	hdfsdm_channel->Init.Awd.Oversampling         = 10; /* N.U. */
	hdfsdm_channel->Init.Offset                   = 0;
	hdfsdm_channel->Init.RightBitShift            = 0;

	err = HAL_DFSDM_ChannelInit(hdfsdm_channel);
	if (err != HAL_OK) {
		LOG_ERR("HAL_DFSDM_ChannelInit failed. err = %d", err);
		return err;
	}

	__HAL_DFSDM_FILTER_RESET_HANDLE_STATE(hdfsdm_filter);
	
	hdfsdm_filter->Instance = get_hal_filter_instance(chan_cfg->dfsdm_inst, chan_cfg->filter);
	hdfsdm_filter->Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER; // or SYNC_TRIGGER ???
	hdfsdm_filter->Init.RegularParam.FastMode = ENABLE;
	hdfsdm_filter->Init.RegularParam.DmaMode = ENABLE;

	// TODO: do i have to set all these if they are not used ??
	hdfsdm_filter->Init.InjectedParam.Trigger = DFSDM_FILTER_SW_TRIGGER; /* N.U. */
	hdfsdm_filter->Init.InjectedParam.ScanMode = ENABLE; /* N.U. */
	hdfsdm_filter->Init.InjectedParam.DmaMode = DISABLE; /* N.U. */
	hdfsdm_filter->Init.InjectedParam.ExtTrigger = DFSDM_FILTER_EXT_TRIG_TIM1_TRGO; /* N.U. */
	hdfsdm_filter->Init.InjectedParam.ExtTriggerEdge = DFSDM_FILTER_EXT_TRIG_RISING_EDGE; /* N.U. */

	hdfsdm_filter->Init.FilterParam.SincOrder = chan_cfg->filter_order;
	hdfsdm_filter->Init.FilterParam.Oversampling = chan_cfg->filt_oversample;
	hdfsdm_filter->Init.FilterParam.IntOversampling = 1;

	err = HAL_DFSDM_FilterInit(hdfsdm_filter);
	if (err != HAL_OK) {
		LOG_ERR("HAL_DFSDM_FilterInit failed. err = %d", err);
		return err;
	}

	uint32_t dfsdm_channel_x = (chan_cfg->channel << 16) | (1 << chan_cfg->channel);
	err = HAL_DFSDM_FilterConfigRegChannel(hdfsdm_filter, dfsdm_channel_x, DFSDM_CONTINUOUS_CONV_ON);
	if (err != HAL_OK) {
		LOG_ERR("HAL_DFSDM_FilterConfigRegChannel failed. err = %d", err);
		return err;
	}

	return 0;
}


/* Driver Initialization Function */
static int dfsdm_stm32_init(const struct device *dev)
{
	int ret;
	const struct dfsdm_stm32_drv_cfg *cfg = dev->config;
	const struct device *clk;

	clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	if (!device_is_ready(clk)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	// Enable DFSDM Kernel Clock
	ret = clock_control_on(clk, (clock_control_subsys_t)&cfg->pclken[0]);
	if (ret != 0) {
		LOG_ERR("Could not enable DFSDM clock");
		return -EIO;
	}

	// TODO: i should get these clock phandles by name

	// Configure I2S1 clock source
	if (cfg->pclk_len > 1) {
		ret = clock_control_configure(clk,
					      (clock_control_subsys_t)&cfg->pclken[1],
					      NULL);
		if (ret < 0) {
			LOG_ERR("Could not configure I2S domain clock");
			return -EIO;
		}
	}

	// Configure DFSDM Audio Clock source to use I2S1 clock
	if (cfg->pclk_len > 2) {
		ret = clock_control_configure(clk,
					      (clock_control_subsys_t)&cfg->pclken[2],
					      NULL);
		if (ret < 0) {
			LOG_ERR("Could not configure DFSDM audio clock");
			return -EIO;
		}
	}
	// NOTE: we require the user to set up the appropriate mul/div in devicetree
	// and we will report when pcm_rate can't be achieved with the provided clock

	// Configure pins
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("DFSDM pinctrl setup failed (%d)", ret);
		return ret;
	}

	// TODO: any DFSDM channel config stuff that couldn't be done at compile time
	// aka leftover stuff that I couldn't figure out by macros

	if (cfg->dfsdm_base_addr == DFSDM1_BASE) {
		for (int i = 0; i < cfg->num_channels; i++) {
			cfg->channels[i]->dfsdm_inst = 1;
		}
	}
#ifdef DFSDM2_BASE
	else if (cfg->dfsdm_base_addr == DFSDM2_BASE) {
		for (int i = 0; i < cfg->num_channels; i++) {
			cfg->channels[i]->dfsdm_inst = 2;
		}
	}
#endif /* DFSDM2_BASE */

	return 0;
}


/*

TODO: API implementation functions

*/
static int dfsdm_stm32_configure(const struct device *dev, struct dmic_cfg *config)
{
	int ret;
	struct dfsdm_stm32_drv_cfg const *cfg = dev->config;
	struct dfsdm_stm32_drv_data *data = dev->data;
	struct pdm_chan_cfg *pdm_chan = &(config->channel);
	struct pcm_stream_cfg *pdm_streams = config->streams;

	uint32_t min_clk = config->io.min_pdm_clk_freq;
	uint32_t max_clk = config->io.max_pdm_clk_freq;

	if (pdm_chan->req_num_chan > cfg->num_channels) {
		LOG_ERR("Requested %d channels but only %d configured in devicetree", pdm_chan->req_num_chan, cfg->num_channels);
		return -ENOTSUP;
	}


	// Output Clock Divider must be between 2 and 256
	uint32_t aclk_freq = 0;

	// TODO: i should really get this audio clock phandle by name "audio"
	if (clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
					(clock_control_subsys_t)&cfg->pclken[2],
					&aclk_freq) < 0) {
		LOG_ERR("Failed call clock_control_get_rate(pclken[2])");
		return -EIO;
	}

	// I have the current audio clock frequency
	// and the desired PCM output frequency
	// I need to determine the PDM clock divider (thus output frequency)
	// and the oversampling rate (decimation factor)

	uint32_t oclk_div, oclk_freq, osr = 0;
	for (oclk_div = 2; oclk_div <= 256; oclk_div++) {
		oclk_freq = aclk_freq / oclk_div;
		if (oclk_freq <= max_clk && oclk_freq >= min_clk) {
			if (oclk_freq % pdm_streams[0].pcm_rate == 0) {
				osr = oclk_freq / pdm_streams[0].pcm_rate;
				break;
			}
		}
	}

	// error if we didn't find good values
	if (osr == 0) {
		LOG_ERR("Failed to determine clock / oversample settings");
		return -EINVAL;
	}

	// set values
	for (int i = 0; i < cfg->num_channels; i++) {
		cfg->channels[i]->chan_clk_div = oclk_div;
		cfg->channels[i]->filt_oversample = osr;
		// TODO: right shift according to requested bit depth ???
	}

	// TODO: this probably won't actually work like this...
	// TODO: User might request a channel map that doesn't use all available channels
	for (int i = 0; i < cfg->num_channels; i++) {
		ret = dfsdm_channel_init(cfg->channels[i]);
		if (ret != HAL_OK) {
			LOG_ERR("dfsdm_channel_init failed");
			return 0;
		}
	}

	// TODO: channel map setup ???

	data->act_num_channels = pdm_chan->req_num_chan;
	data->mem_slab = pdm_streams->mem_slab;
	data->block_size = pdm_streams->block_size;
	data->dmic_state = DMIC_STATE_CONFIGURED;
    return 0;
}

static int start_dma()
{

	// TODO: allocate buffer(s)

	// TODO: set up DMA handles

	// TODO: link DMA to DFSDM

	// TODO: assign dma callback function ???

	return 0;
}

static int start_dfsdm()
{
	// TODO: call start_dma on the active channels (filters)

	// TODO: then call DFSDM_RegConvStart on each of the active channels (filters)

	return 0;
}

static int stop_dfsdm()
{
	// TODO: implement me
	return 0;
}

static int dfsdm_stm32_trigger(const struct device *dev, enum dmic_trigger cmd)
{
	struct dfsdm_stm32_drv_data *drv_data = dev->data;

	switch (cmd) {
	case DMIC_TRIGGER_PAUSE:
	case DMIC_TRIGGER_STOP:
		return stop_dfsdm();
		break;
	case DMIC_TRIGGER_RELEASE:
	case DMIC_TRIGGER_START:
		return start_dfsdm();
		break;
	case DMIC_TRIGGER_RESET:
	default:
		LOG_ERR("Invalid command: %d", cmd);
		return -EINVAL;
	}

    return 0;
}


static int dfsdm_stm32_read(const struct device *dev, uint8_t stream, void **buffer, size_t *size, int32_t timeout)
{
    // TODO: dummy return value
    return 0;
}

static const struct _dmic_ops dmic_ops = {
	.configure = dfsdm_stm32_configure,
	.trigger = dfsdm_stm32_trigger,
	.read = dfsdm_stm32_read,
};

#define SAMPLE_EDGE_STRING(node_id) \
	UTIL_CAT(DFSDM_CHANNEL_SPI_, DT_STRING_UPPER_TOKEN(node_id, st_sample_edge))

// TODO: filter order
/*
#define DFSDM_FILTER_SINC0_ORDER DFSDM_FILTER_FASTSYNC_ORDER
#define GET_FILTER_ORDER(number) \
	UTIL_CAT(DFSDM_FILTER_SINC##number, _ORDER)
*/
#define DFSDM_CHANNEL_DEFINE(node_id) \
	static struct dfsdm_stm32_chan_cfg dfsdm_chan_cfg##node_id = { \
		.dma_dev = DEVICE_DT_GET(DT_DMAS_CTLR(node_id)), \
		.dma_chan = DT_DMAS_CELL_BY_IDX(node_id, 0, channel), \
		.dma_cfg = { \
			.dma_slot = DT_DMAS_CELL_BY_IDX(node_id, 0, slot), \
		}, \
		.channel = DT_PROP(node_id, st_adc_channels), \
		.filter = DT_REG_ADDR(node_id), \
		.sample_edge = SAMPLE_EDGE_STRING(node_id), \
		.alt_channel = DT_PROP_OR(node_id, st_adc_alt_channel, 0), \
	};

#define DFSDM_CHANNEL_GET(node_id) \
	&dfsdm_chan_cfg##node_id,

#define DFSDM_CHANNELS_DEFINE(idx) \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, DFSDM_CHANNEL_DEFINE);

#define DFSDM_CHANNELS_GET(idx) \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, DFSDM_CHANNEL_GET)

#define STM32_DFSDM_DEVICE(idx) \
	DFSDM_CHANNELS_DEFINE(idx) \
	PINCTRL_DT_INST_DEFINE(idx); \
	static struct dfsdm_stm32_drv_data dfsdm_stm32_drv_data##idx = { \
		.dmic_state = DMIC_STATE_UNINIT, \
	}; \
	static const struct stm32_pclken clk_##idx[] = \
				 STM32_DT_INST_CLOCKS(idx); \
	static struct dfsdm_stm32_chan_cfg \
		*dfsdm##idx##_channels[DT_INST_CHILD_NUM_STATUS_OKAY(idx)] = { \
			DFSDM_CHANNELS_GET(idx) \
	}; \
	static struct dfsdm_stm32_drv_cfg dfsdm_stm32_drv_cfg##idx = { \
		.pclken = clk_##idx, \
		.pclk_len = DT_INST_NUM_CLOCKS(idx), \
		.dfsdm_base_addr = DT_INST_REG_ADDR(idx), \
		.num_channels = DT_INST_CHILD_NUM_STATUS_OKAY(idx), \
		.channels = dfsdm##idx##_channels, \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx), \
	}; \
	DEVICE_DT_INST_DEFINE(idx, dfsdm_stm32_init, NULL, \
			&dfsdm_stm32_drv_data##idx, &dfsdm_stm32_drv_cfg##idx, \
			POST_KERNEL, CONFIG_AUDIO_DMIC_INIT_PRIORITY, \
			&dmic_ops);

DT_INST_FOREACH_STATUS_OKAY(STM32_DFSDM_DEVICE)