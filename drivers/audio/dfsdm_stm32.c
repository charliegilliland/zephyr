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
	
	/* --- dfsdm-chan stuff --- */
	DFSDM_Channel_HandleTypeDef hdfsdm_channel;
	/* --- dfsdm-filt stuff --- */
	DFSDM_Filter_HandleTypeDef hdfsdm_filter;
};


struct dfsdm_stm32_drv_data {

	// TODO: stuff like buffers, k_msg_q, other runtime data will be stored in here

	int dummy;
};

struct dfsdm_stm32_drv_cfg {
	// TODO: clocks ??
	// TODO: Audio clock ??

	// Pinctrl
	const struct pinctrl_dev_config *pcfg;

	// array of chan_cfg structs
	int num_channels;
	struct dfsdm_stm32_chan_cfg **channels;
};

static int dfsdm_stm32_init(const struct device *dev)
{

	// this is all Zephyr API calls to device pointers that we'll get at compile time through macro magic
	// TODO: pinctrl stuff
	// TODO: DMA stuff
	// TODO: clock stuff

	// TODO: any DFSDM instance initialization to do here??? not just channel / filter init ???

	// TODO: link DMA to DFSDM ??

	// TODO: dummy
	return 0;
}


/*

TODO: API implementation functions

*/
static int dfsdm_stm32_configure(const struct device *dev, struct dmic_cfg *config)
{
    // TODO: dummy return value
    return 0;
}


static int dfsdm_stm32_trigger(const struct device *dev, enum dmic_trigger cmd)
{
	struct dfsdm_stm32_drv_data *drv_data = dev->data;

	switch (cmd) {
	case DMIC_TRIGGER_PAUSE:
	case DMIC_TRIGGER_STOP:
		break;
	case DMIC_TRIGGER_RELEASE:
	case DMIC_TRIGGER_START:
		break;
	default:
		LOG_ERR("Invalid command: %d", cmd);
		return -EINVAL;
	}

    // TODO: dummy return value
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

#define CHAN(i) channel##i
#define FILT(i) filter##i

// get node identifier for the channelX node at {node_id}->channel{i}
#define NODE_CHAN(node_id, i) DT_NODELABEL(CHAN(i)) // TODO: i think this works for now, but would conflict when there are more than 1 dfsdm instances...
//#define NODE_CHAN(node_id, i) DT_CHILD(node_id, CHAN(i))

// get node identifier for the filterX node at {node_id}->filter{i}
#define NODE_FILT(node_id, i) DT_NODELABEL(FILT(i))
//#define NODE_FILT(node_id, i) DT_CHILD(node_id, FILT(i))

// get node identifier for the channelX node at {node_id}->dfsdm_configs[{p_idx}]->channelX
#define GET_NODE_CHAN(node_id, prop, p_idx) \
	NODE_CHAN(DT_PHANDLE_BY_IDX(node_id, prop, p_idx), \
		DT_PHA_BY_IDX(node_id, prop, p_idx, chan))

// get node identifier for the filterX node at {node_id}->dfsdm_configs[{p_idx}]->filterX
#define GET_NODE_FILT(node_id, prop, p_idx) \
	NODE_FILT(DT_PHANDLE_BY_IDX(node_id, prop, p_idx), \
		DT_PHA_BY_IDX(node_id, prop, p_idx, filt))

// this gets called on each of the <&dfsdm X Y> in dfsdm-configs
#define DFSDM_CHANNEL_DEFINE(node_id, prop, p_idx, inst_idx) \
	static struct dfsdm_stm32_chan_cfg dfsdm##inst_idx##_chan_cfg##p_idx = { \
		.dma_dev = DEVICE_DT_GET(DT_DMAS_CTLR(GET_NODE_FILT(node_id, prop, p_idx))), \
		.dma_chan = DT_DMAS_CELL_BY_IDX(GET_NODE_FILT(node_id, prop, p_idx), 0, channel), \
		.dma_cfg = { \
			.dma_slot = DT_DMAS_CELL_BY_IDX(GET_NODE_FILT(node_id, prop, p_idx), 0, slot), \
		}, \
		.hdfsdm_channel.Instance = (DFSDM_Channel_TypeDef *) \
			(DT_REG_ADDR(DT_PHANDLE_BY_IDX(node_id, prop, p_idx)) + \
			 DT_REG_ADDR(GET_NODE_CHAN(node_id, prop, p_idx))), \
		.hdfsdm_filter.Instance = (DFSDM_Filter_TypeDef *) \
			(DT_REG_ADDR(DT_PHANDLE_BY_IDX(node_id, prop, p_idx)) + \
			 DT_REG_ADDR(GET_NODE_FILT(node_id, prop, p_idx))), \
	};

#define DFSDM_CHANNEL_GET(node_id, prop, p_idx, inst_idx) \
	&dfsdm##inst_idx##_chan_cfg##p_idx,

#define DFSDM_CHANNELS_DEFINE(idx) \
	DT_FOREACH_PROP_ELEM_VARGS(DT_DRV_INST(idx), dfsdm_configs, \
									DFSDM_CHANNEL_DEFINE, idx);

#define DFSDM_CHANNELS_GET(idx) \
	DT_FOREACH_PROP_ELEM_VARGS(DT_DRV_INST(idx), dfsdm_configs, \
									DFSDM_CHANNEL_GET, idx)

#define STM32_DFSDM_DEVICE(idx) \
	DFSDM_CHANNELS_DEFINE(idx) \
	PINCTRL_DT_INST_DEFINE(idx); \
	static struct dfsdm_stm32_drv_data dfsdm_stm32_drv_data##idx = { \
		.dummy = 0, \
	}; \
	static struct dfsdm_stm32_chan_cfg \
		*dfsdm##idx##_channels[DT_PROP_LEN(DT_DRV_INST(idx), dfsdm_configs)] = { \
			DFSDM_CHANNELS_GET(idx) \
	}; \
	static struct dfsdm_stm32_drv_cfg dfsdm_stm32_drv_cfg##idx = { \
		.num_channels = DT_PROP_LEN(DT_DRV_INST(idx), dfsdm_configs), \
		.channels = dfsdm##idx##_channels, \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx), \
	}; \
	DEVICE_DT_INST_DEFINE(idx, dfsdm_stm32_init, NULL, \
			&dfsdm_stm32_drv_data##idx, &dfsdm_stm32_drv_cfg##idx, \
			POST_KERNEL, CONFIG_AUDIO_DMIC_INIT_PRIORITY, \
			&dmic_ops);

DT_INST_FOREACH_STATUS_OKAY(STM32_DFSDM_DEVICE)