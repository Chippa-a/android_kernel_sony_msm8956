/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
/*
 * NOTE: This file has been modified by Sony Mobile Communications Inc.
 * Modifications are Copyright (c) 2014 Sony Mobile Communications Inc,
 * and licensed under the license of the file.
 */

#include <linux/mfd/wcd9xxx/core.h>
#include <linux/of.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include "msm8952-slimbus.h"
#include "qdsp6v2/msm-pcm-routing-v2.h"
#include "../codecs/wcd9335.h"

static struct snd_soc_card snd_soc_card_msm[MAX_CODECS];
static struct snd_soc_card snd_soc_card_msm_card;

static struct snd_soc_ops msm8952_quat_mi2s_be_ops = {
	.startup = msm_quat_mi2s_snd_startup,
	.hw_params = msm_mi2s_snd_hw_params,
	.shutdown = msm_quat_mi2s_snd_shutdown,
};

static struct snd_soc_ops msm8952_quin_mi2s_be_ops = {
	.startup = msm_quin_mi2s_snd_startup,
	.hw_params = msm_mi2s_snd_hw_params,
	.shutdown = msm_quin_mi2s_snd_shutdown,
};

static struct snd_soc_ops msm_pri_auxpcm_be_ops = {
	.startup = msm_prim_auxpcm_startup,
	.shutdown = msm_prim_auxpcm_shutdown,
};

static struct snd_soc_ops msm8952_slimbus_be_ops = {
	.hw_params = msm_snd_hw_params,
};


static struct snd_soc_ops msm8952_cpe_ops = {
	.hw_params = msm_snd_cpe_hw_params,
};

static struct snd_soc_ops msm8952_slimbus_2_be_ops = {
	.hw_params = msm8952_slimbus_2_hw_params,
};

static struct snd_soc_dai_link msm8952_tasha_fe_dai[] = {
	/* tasha_vifeedback for speaker protection */
	{
		.name = LPASS_BE_SLIMBUS_4_TX,
		.stream_name = "Slimbus4 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16393",
		.platform_name = "msm-pcm-hostless",
		.codec_name = "tasha_codec",
		.codec_dai_name = "tasha_vifeedback",
		.be_id = MSM_BACKEND_DAI_SLIMBUS_4_TX,
		.be_hw_params_fixup = msm_slim_4_tx_be_hw_params_fixup,
		.ops = &msm8952_slimbus_be_ops,
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
	},
	/* Ultrasound RX DAI Link */
	{
		.name = "SLIMBUS_2 Hostless Playback",
		.stream_name = "SLIMBUS_2 Hostless Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16388",
		.platform_name = "msm-pcm-hostless",
		.codec_name = "tasha_codec",
		.codec_dai_name = "tasha_rx2",
		.ignore_suspend = 1,
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ops = &msm8952_slimbus_2_be_ops,
	},
	/* Ultrasound TX DAI Link */
	{
		.name = "SLIMBUS_2 Hostless Capture",
		.stream_name = "SLIMBUS_2 Hostless Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16389",
		.platform_name = "msm-pcm-hostless",
		.codec_name = "tasha_codec",
		.codec_dai_name = "tasha_tx2",
		.ignore_suspend = 1,
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ops = &msm8952_slimbus_2_be_ops,
	},
	/* CPE LSM direct dai-link */
	{
		.name = "CPE Listen service",
		.stream_name = "CPE Listen Audio Service",
		.cpu_dai_name = "msm-dai-slim",
		.platform_name = "msm-cpe-lsm",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.codec_dai_name = "tasha_mad1",
		.codec_name = "tasha_codec",
		.ops = &msm8952_cpe_ops,
	},
	/* FM Capture */
	{
		.name = "QUIN_MI2S Hostless",
		.stream_name = "QUIN_MI2S Hostless",
		.cpu_dai_name = "QUIN_MI2S_TX_HOSTLESS",
		.platform_name = "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
};

static struct snd_soc_dai_link msm8952_tasha_be_dai[] = {
	/* Backend DAI Links */
	{
		.name = LPASS_BE_SLIMBUS_0_RX,
		.stream_name = "Slimbus Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16384",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tasha_codec",
		.codec_dai_name = "tasha_mix_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_0_RX,
		.init = &msm_audrx_init,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		/* this dainlink has playback support */
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.ops = &msm8952_slimbus_be_ops,
	},
	{
		.name = LPASS_BE_SLIMBUS_0_TX,
		.stream_name = "Slimbus Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16385",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tasha_codec",
		.codec_dai_name = "tasha_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_0_TX,
		.be_hw_params_fixup = msm_slim_0_tx_be_hw_params_fixup,
		.ignore_suspend = 1,
		.ops = &msm8952_slimbus_be_ops,
	},
	{
		.name = LPASS_BE_SLIMBUS_1_RX,
		.stream_name = "Slimbus1 Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16386",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tasha_codec",
		.codec_dai_name = "tasha_mix_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_1_RX,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		.ops = &msm8952_slimbus_be_ops,
		/* dai link has playback support */
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_1_TX,
		.stream_name = "Slimbus1 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16387",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tasha_codec",
		.codec_dai_name = "tasha_tx3",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_1_TX,
		.be_hw_params_fixup = msm_slim_1_tx_be_hw_params_fixup,
		.ops = &msm8952_slimbus_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_3_RX,
		.stream_name = "Slimbus3 Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16390",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tasha_codec",
		.codec_dai_name = "tasha_mix_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_3_RX,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		.ops = &msm8952_slimbus_be_ops,
		/* dai link has playback support */
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_3_TX,
		.stream_name = "Slimbus3 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16391",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tasha_codec",
		.codec_dai_name = "tasha_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_3_TX,
		.be_hw_params_fixup = msm_slim_0_tx_be_hw_params_fixup,
		.ops = &msm8952_slimbus_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_4_RX,
		.stream_name = "Slimbus4 Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16392",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tasha_codec",
		.codec_dai_name = "tasha_mix_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_4_RX,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		.ops = &msm8952_slimbus_be_ops,
		/* dai link has playback support */
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_5_RX,
		.stream_name = "Slimbus5 Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16394",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tasha_codec",
		.codec_dai_name = "tasha_rx3",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_5_RX,
		.be_hw_params_fixup = msm_slim_5_rx_be_hw_params_fixup,
		.ops = &msm8952_slimbus_be_ops,
		/* dai link has playback support */
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	/* MAD BE */
	{
		.name = LPASS_BE_SLIMBUS_5_TX,
		.stream_name = "Slimbus5 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16395",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tasha_codec",
		.codec_dai_name = "tasha_mad1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_5_TX,
		.be_hw_params_fixup = msm_slim_5_tx_be_hw_params_fixup,
		.ops = &msm8952_slimbus_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_MI2S_HDMI_RX,
		.stream_name = "MI2S HDMI Playback",
		.cpu_dai_name = "msm-dai-q6-mi2s-hdmi.4118",
		.platform_name = "msm-pcm-routing",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_MI2S_HDMI_RX,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ops = &msm8952_quin_mi2s_be_ops,
		.ignore_pmdown_time = 1, /* dai link has playback support */
		.ignore_suspend = 1,
	},
};

static struct snd_soc_dai_link msm8952_tomtom_fe_dai[] = {
	{ /* hw:x,28 */
		.name = LPASS_BE_SLIMBUS_4_TX,
		.stream_name = "Slimbus4 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16393",
		.platform_name = "msm-pcm-hostless",
		.codec_name = "tomtom_codec",
		.codec_dai_name = "tomtom_vifeedback",
		.be_id = MSM_BACKEND_DAI_SLIMBUS_4_TX,
		.be_hw_params_fixup = msm_slim_4_tx_be_hw_params_fixup,
		.ops = &msm8952_slimbus_be_ops,
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
	},
	/* CPE LSM FE */
	{ /* hw:x,29 */
		.name = "CPE Listen service",
		.stream_name = "CPE Listen Audio Service",
		.cpu_dai_name = "msm-dai-slim",
		.platform_name = "msm-cpe-lsm",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST },
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.codec_dai_name = "tomtom_mad1",
		.codec_name = "tomtom_codec",
		.ops = &msm8952_cpe_ops,
	},
};

static struct snd_soc_dai_link msm8952_tomtom_be_dai[] = {
	/* Backend DAI Links */
	{
		.name = LPASS_BE_SLIMBUS_0_RX,
		.stream_name = "Slimbus Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16384",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tomtom_codec",
		.codec_dai_name	= "tomtom_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_0_RX,
		.init = &msm_audrx_init,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		.ignore_pmdown_time = 1, /* dai link has playback support */
		.ignore_suspend = 1,
		.ops = &msm8952_slimbus_be_ops,
	},
	{
		.name = LPASS_BE_SLIMBUS_0_TX,
		.stream_name = "Slimbus Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16385",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tomtom_codec",
		.codec_dai_name = "tomtom_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_0_TX,
		.be_hw_params_fixup = msm_slim_0_tx_be_hw_params_fixup,
		.ignore_suspend = 1,
		.ops = &msm8952_slimbus_be_ops,
	},
	{
		.name = LPASS_BE_SLIMBUS_1_RX,
		.stream_name = "Slimbus1 Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16386",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tomtom_codec",
		.codec_dai_name	= "tomtom_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_1_RX,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		.ops = &msm8952_slimbus_be_ops,
		/* dai link has playback support */
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_1_TX,
		.stream_name = "Slimbus1 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16387",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tomtom_codec",
		.codec_dai_name	= "tomtom_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_1_TX,
		.be_hw_params_fixup = msm_slim_0_tx_be_hw_params_fixup,
		.ops = &msm8952_slimbus_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_3_RX,
		.stream_name = "Slimbus3 Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16390",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tomtom_codec",
		.codec_dai_name	= "tomtom_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_3_RX,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		.ops = &msm8952_slimbus_be_ops,
		/* dai link has playback support */
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_3_TX,
		.stream_name = "Slimbus3 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16391",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tomtom_codec",
		.codec_dai_name	= "tomtom_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_3_TX,
		.be_hw_params_fixup = msm_slim_0_tx_be_hw_params_fixup,
		.ops = &msm8952_slimbus_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_4_RX,
		.stream_name = "Slimbus4 Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16392",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tomtom_codec",
		.codec_dai_name	= "tomtom_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_4_RX,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		.ops = &msm8952_slimbus_be_ops,
		/* dai link has playback support */
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	/* MAD BE */
	{
		.name = LPASS_BE_SLIMBUS_5_TX,
		.stream_name = "Slimbus5 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16395",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tomtom_codec",
		.codec_dai_name = "tomtom_mad1",
		.no_pcm = 1,
		.async_ops = ASYNC_DPCM_SND_SOC_PREPARE,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_5_TX,
		.be_hw_params_fixup = msm_slim_5_tx_be_hw_params_fixup,
		.ignore_suspend = 1,
	},
};

static struct snd_soc_dai_link msm8952_common_fe_dai[] = {
	/* FrontEnd DAI Links */
	{/* hw:x,0 */
		.name = "MSM8X16 Media1",
		.stream_name = "MultiMedia1",
		.cpu_dai_name	= "MultiMedia1",
		.platform_name  = "msm-pcm-dsp.0",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA1
	},
	{/* hw:x,1 */
		.name = "MSM8X16 Media2",
		.stream_name = "MultiMedia2",
		.cpu_dai_name   = "MultiMedia2",
		.platform_name  = "msm-pcm-dsp.0",
		.dynamic = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA2,
	},
	{/* hw:x,2 */
		.name = "Circuit-Switch Voice",
		.stream_name = "CS-Voice",
		.cpu_dai_name   = "CS-VOICE",
		.platform_name  = "msm-pcm-voice",
		.dynamic = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_CS_VOICE,
	},
	{/* hw:x,3 */
		.name = "MSM VoIP",
		.stream_name = "VoIP",
		.cpu_dai_name	= "VoIP",
		.platform_name  = "msm-voip-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_VOIP,
	},
	{/* hw:x,4 */
		.name = "MSM8X16 ULL",
		.stream_name = "ULL",
		.cpu_dai_name	= "MultiMedia3",
		.platform_name  = "msm-pcm-dsp.2",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA3,
	},
	/* Hostless PCM purpose */
	{/* hw:x,5 */
		.name = "SLIMBUS_0 Hostless",
		.stream_name = "SLIMBUS_0 Hostless",
		.cpu_dai_name = "SLIMBUS0_HOSTLESS",
		.platform_name	= "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		 /* This dai link has MI2S support */
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	{/* hw:x,6 */
		.name = "INT_FM Hostless",
		.stream_name = "INT_FM Hostless",
		.cpu_dai_name	= "INT_FM_HOSTLESS",
		.platform_name  = "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	{/* hw:x,7 */
		.name = "MSM AFE-PCM RX",
		.stream_name = "AFE-PROXY RX",
		.cpu_dai_name = "msm-dai-q6-dev.241",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.platform_name  = "msm-pcm-afe",
		.ignore_suspend = 1,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
	},
	{/* hw:x,8 */
		.name = "MSM AFE-PCM TX",
		.stream_name = "AFE-PROXY TX",
		.cpu_dai_name = "msm-dai-q6-dev.240",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.platform_name  = "msm-pcm-afe",
		.ignore_suspend = 1,
	},
	{/* hw:x,9 */
		.name = "MSM8X16 Compress1",
		.stream_name = "Compress1",
		.cpu_dai_name	= "MultiMedia4",
		.platform_name  = "msm-compress-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			 SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		 /* this dai link has playback support */
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA4,
	},
	{/* hw:x,10 */
		.name = "AUXPCM Hostless",
		.stream_name = "AUXPCM Hostless",
		.cpu_dai_name   = "AUXPCM_HOSTLESS",
		.platform_name  = "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	{/* hw:x,11 */
		.name = "SLIMBUS_1 Hostless",
		.stream_name = "SLIMBUS_1 Hostless",
		.cpu_dai_name = "SLIMBUS1_HOSTLESS",
		.platform_name = "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, /* dai link has playback support */
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	{/* hw:x,12 */
		.name = "MSM8952 LowLatency",
		.stream_name = "MultiMedia5",
		.cpu_dai_name   = "MultiMedia5",
		.platform_name  = "msm-pcm-dsp.1",
		.dynamic = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA5,
	},
	{/* hw:x,13 */
		.name = "Voice2",
		.stream_name = "Voice2",
		.cpu_dai_name   = "Voice2",
		.platform_name  = "msm-pcm-voice",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.be_id = MSM_FRONTEND_DAI_VOICE2,
	},
	{/* hw:x,14 */
		.name = "MSM8952 Media9",
		.stream_name = "MultiMedia9",
		.cpu_dai_name   = "MultiMedia9",
		.platform_name  = "msm-pcm-dsp.0",
		.dynamic = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		/* This dailink has playback support */
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA9,
	},
	{ /* hw:x,15 */
		.name = "VoLTE",
		.stream_name = "VoLTE",
		.cpu_dai_name   = "VoLTE",
		.platform_name  = "msm-pcm-voice",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.be_id = MSM_FRONTEND_DAI_VOLTE,
	},
	{ /* hw:x,16 */
		.name = "VoWLAN",
		.stream_name = "VoWLAN",
		.cpu_dai_name   = "VoWLAN",
		.platform_name  = "msm-pcm-voice",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.be_id = MSM_FRONTEND_DAI_VOWLAN,
	},
	{/* hw:x,17 */
		.name = "INT_HFP_BT Hostless",
		.stream_name = "INT_HFP_BT Hostless",
		.cpu_dai_name = "INT_HFP_BT_HOSTLESS",
		.platform_name  = "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	{/* hw:x,18 */
		.name = "MSM8916 HFP TX",
		.stream_name = "MultiMedia6",
		.cpu_dai_name = "MultiMedia6",
		.platform_name  = "msm-pcm-loopback",
		.dynamic = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA6,
	},
	/* LSM FE */
	{/* hw:x,19 */
		.name = "Listen 1 Audio Service",
		.stream_name = "Listen 1 Audio Service",
		.cpu_dai_name = "LSM1",
		.platform_name = "msm-lsm-client",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST },
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.be_id = MSM_FRONTEND_DAI_LSM1,
	},
	{/* hw:x,20 */
		.name = "Listen 2 Audio Service",
		.stream_name = "Listen 2 Audio Service",
		.cpu_dai_name = "LSM2",
		.platform_name = "msm-lsm-client",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST },
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.be_id = MSM_FRONTEND_DAI_LSM2,
	},
	{/* hw:x,21 */
		.name = "Listen 3 Audio Service",
		.stream_name = "Listen 3 Audio Service",
		.cpu_dai_name = "LSM3",
		.platform_name = "msm-lsm-client",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST },
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.be_id = MSM_FRONTEND_DAI_LSM3,
	},
	{/* hw:x,22 */
		.name = "Listen 4 Audio Service",
		.stream_name = "Listen 4 Audio Service",
		.cpu_dai_name = "LSM4",
		.platform_name = "msm-lsm-client",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST },
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.be_id = MSM_FRONTEND_DAI_LSM4,
	},
	{/* hw:x,23 */
		.name = "Listen 5 Audio Service",
		.stream_name = "Listen 5 Audio Service",
		.cpu_dai_name = "LSM5",
		.platform_name = "msm-lsm-client",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST },
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.be_id = MSM_FRONTEND_DAI_LSM5,
	},
	{/* hw:x,24 */
		.name = "MSM8X16 Compress2",
		.stream_name = "Compress2",
		.cpu_dai_name   = "MultiMedia7",
		.platform_name  = "msm-compress-dsp",
		.dynamic = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA7,
	},
	{/* hw:x,25 */
		.name = "SLIMBUS_3 Hostless",
		.stream_name = "SLIMBUS_3 Hostless",
		.cpu_dai_name = "SLIMBUS3_HOSTLESS",
		.platform_name = "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, /* dai link has playback support */
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	{/* hw:x,26 */
		.name = "SLIMBUS_4 Hostless",
		.stream_name = "SLIMBUS_4 Hostless",
		.cpu_dai_name = "SLIMBUS4_HOSTLESS",
		.platform_name = "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, /* dai link has playback support */
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	{ /* hw:x,27 */
		.name = "QUAT_MI2S Hostless",
		.stream_name = "QUAT_MI2S Hostless",
		.cpu_dai_name = "QUAT_MI2S_RX_HOSTLESS",
		.platform_name = "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	{/* hw:x,28 */
		.name = "MSM8X16 Compress3",
		.stream_name = "Compress3",
		.cpu_dai_name	= "MultiMedia10",
		.platform_name  = "msm-compress-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			 SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		 /* this dai link has playback support */
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA10,
	},
	{/* hw:x,29 */
		.name = "MSM8X16 Compress4",
		.stream_name = "Compress4",
		.cpu_dai_name	= "MultiMedia11",
		.platform_name  = "msm-compress-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			 SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		 /* this dai link has playback support */
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA11,
	},
	{/* hw:x,30 */
		.name = "MSM8X16 Compress5",
		.stream_name = "Compress5",
		.cpu_dai_name	= "MultiMedia12",
		.platform_name  = "msm-compress-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			 SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		 /* this dai link has playback support */
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA12,
	},
	{/* hw:x,31 */
		.name = "MSM8X16 Compress6",
		.stream_name = "Compress6",
		.cpu_dai_name	= "MultiMedia13",
		.platform_name  = "msm-compress-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			 SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		 /* this dai link has playback support */
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA13,
	},
	{/* hw:x,32 */
		.name = "MSM8X16 Compress7",
		.stream_name = "Compress7",
		.cpu_dai_name	= "MultiMedia14",
		.platform_name  = "msm-compress-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			 SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		 /* this dai link has playback support */
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA14,
	},
	{/* hw:x,33 */
		.name = "MSM8X16 Compress8",
		.stream_name = "Compress8",
		.cpu_dai_name	= "MultiMedia15",
		.platform_name  = "msm-compress-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			 SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		 /* this dai link has playback support */
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA15,
	},
	{/* hw:x,34 */
		.name = "MSM8X16 Compress9",
		.stream_name = "Compress9",
		.cpu_dai_name	= "MultiMedia16",
		.platform_name  = "msm-compress-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			 SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		 /* this dai link has playback support */
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA16,
	},
	{/* hw:x,35 */
		.name = "VoiceMMode1",
		.stream_name = "VoiceMMode1",
		.cpu_dai_name = "VoiceMMode1",
		.platform_name = "msm-pcm-voice",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.be_id = MSM_FRONTEND_DAI_VOICEMMODE1,
	},
	{/* hw:x,36 */
		.name = "VoiceMMode2",
		.stream_name = "VoiceMMode2",
		.cpu_dai_name = "VoiceMMode2",
		.platform_name = "msm-pcm-voice",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.be_id = MSM_FRONTEND_DAI_VOICEMMODE2,
	},
};

static struct snd_soc_dai_link msm8952_common_be_dai[] = {
	/* Backend I2S DAI Links */
	{
		.name = LPASS_BE_QUAT_MI2S_RX,
		.stream_name = "Quaternary MI2S Playback",
		.cpu_dai_name = "msm-dai-q6-mi2s.3",
		.platform_name = "msm-pcm-routing",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_QUATERNARY_MI2S_RX,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ops = &msm8952_quat_mi2s_be_ops,
		.ignore_pmdown_time = 1, /* dai link has playback support */
		.ignore_suspend = 1,
	},
	/* Primary AUX PCM Backend DAI Links */
	{
		.name = LPASS_BE_AUXPCM_RX,
		.stream_name = "AUX PCM Playback",
		.cpu_dai_name = "msm-dai-q6-auxpcm.1",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_AUXPCM_RX,
		.be_hw_params_fixup = msm_auxpcm_be_params_fixup,
		.ops = &msm_pri_auxpcm_be_ops,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_AUXPCM_TX,
		.stream_name = "AUX PCM Capture",
		.cpu_dai_name = "msm-dai-q6-auxpcm.1",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_AUXPCM_TX,
		.be_hw_params_fixup = msm_auxpcm_be_params_fixup,
		.ops = &msm_pri_auxpcm_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_QUAT_MI2S_TX,
		.stream_name = "Quaternary MI2S Capture",
		.cpu_dai_name = "msm-dai-q6-mi2s.3",
		.platform_name = "msm-pcm-routing",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_QUATERNARY_MI2S_TX,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ops = &msm8952_quat_mi2s_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_INT_BT_SCO_RX,
		.stream_name = "Internal BT-SCO Playback",
		.cpu_dai_name = "msm-dai-q6-dev.12288",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name	= "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_INT_BT_SCO_RX,
		.be_hw_params_fixup = msm_btsco_be_hw_params_fixup,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_INT_BT_SCO_TX,
		.stream_name = "Internal BT-SCO Capture",
		.cpu_dai_name = "msm-dai-q6-dev.12289",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name	= "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_INT_BT_SCO_TX,
		.be_hw_params_fixup = msm_btsco_be_hw_params_fixup,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_INT_FM_RX,
		.stream_name = "Internal FM Playback",
		.cpu_dai_name = "msm-dai-q6-dev.12292",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_INT_FM_RX,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_INT_FM_TX,
		.stream_name = "Internal FM Capture",
		.cpu_dai_name = "msm-dai-q6-dev.12293",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_INT_FM_TX,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_AFE_PCM_RX,
		.stream_name = "AFE Playback",
		.cpu_dai_name = "msm-dai-q6-dev.224",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_AFE_PCM_RX,
		.be_hw_params_fixup = msm_proxy_rx_be_hw_params_fixup,
		/* this dai link has playback support */
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_AFE_PCM_TX,
		.stream_name = "AFE Capture",
		.cpu_dai_name = "msm-dai-q6-dev.225",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_AFE_PCM_TX,
		.be_hw_params_fixup = msm_proxy_tx_be_hw_params_fixup,
		.ignore_suspend = 1,
	},
	/* Incall Record Uplink BACK END DAI Link */
	{
		.name = LPASS_BE_INCALL_RECORD_TX,
		.stream_name = "Voice Uplink Capture",
		.cpu_dai_name = "msm-dai-q6-dev.32772",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_INCALL_RECORD_TX,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ignore_suspend = 1,
	},
	/* Incall Record Downlink BACK END DAI Link */
	{
		.name = LPASS_BE_INCALL_RECORD_RX,
		.stream_name = "Voice Downlink Capture",
		.cpu_dai_name = "msm-dai-q6-dev.32771",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_INCALL_RECORD_RX,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ignore_suspend = 1,
	},
	/* Incall Music BACK END DAI Link */
	{
		.name = LPASS_BE_VOICE_PLAYBACK_TX,
		.stream_name = "Voice Farend Playback",
		.cpu_dai_name = "msm-dai-q6-dev.32773",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_VOICE_PLAYBACK_TX,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ignore_suspend = 1,
	},
	/* Incall Music 2 BACK END DAI Link */
	{
		.name = LPASS_BE_VOICE2_PLAYBACK_TX,
		.stream_name = "Voice2 Farend Playback",
		.cpu_dai_name = "msm-dai-q6-dev.32770",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_VOICE2_PLAYBACK_TX,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_QUIN_MI2S_RX,
		.stream_name = "Quinary MI2S Playback",
		.cpu_dai_name = "msm-dai-q6-mi2s.5",
		.platform_name = "msm-pcm-routing",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_QUINARY_MI2S_RX,
		.be_hw_params_fixup = msm_quin_be_hw_params_fixup,
		.ops = &msm8952_quin_mi2s_be_ops,
		.ignore_pmdown_time = 1, /* dai link has playback support */
		.ignore_suspend = 1,
	},
	/* Temporary patch for FM Capture link */
	{
		.name = LPASS_BE_QUIN_MI2S_TX,
		.stream_name = "Quinary MI2S Capture",
		.cpu_dai_name = "msm-dai-q6-mi2s.5",
		.platform_name = "msm-pcm-routing",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_QUINARY_MI2S_TX,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ops = &msm8952_quin_mi2s_be_ops,
		.ignore_suspend = 1,
	},
};

static struct snd_soc_aux_dev msm895x_aux_dev[] = {
	{
		.name = "wsa881x.0",
		.codec_name =  NULL,
		.init = msm895x_wsa881x_init,
	},
	{
		.name = "wsa881x.1",
		.codec_name = NULL,
		.init = msm895x_wsa881x_init,
	},
};

static struct snd_soc_codec_conf msm895x_codec_conf[] = {
	{
		.dev_name = NULL,
		.name_prefix = NULL,
	},
	{
		.dev_name = NULL,
		.name_prefix = NULL,
	},
};

static struct snd_soc_dai_link msm8952_tomtom_dai_links[
ARRAY_SIZE(msm8952_common_fe_dai) +
ARRAY_SIZE(msm8952_tomtom_fe_dai) +
ARRAY_SIZE(msm8952_common_be_dai) +
ARRAY_SIZE(msm8952_tomtom_be_dai)];

static struct snd_soc_dai_link msm8952_tasha_dai_links[
ARRAY_SIZE(msm8952_common_fe_dai) +
ARRAY_SIZE(msm8952_tasha_fe_dai) +
ARRAY_SIZE(msm8952_common_be_dai) +
ARRAY_SIZE(msm8952_tasha_be_dai)];

struct snd_soc_card *populate_snd_card_dailinks(struct device *dev)
{
	struct snd_soc_card *card = &snd_soc_card_msm_card;
	struct snd_soc_dai_link *msm8952_dai_links = NULL;
	int num_links, ret, len1, len2, len3, i;
	const char *wsa = "qcom,aux-codec";
	const char *wsa_prefix = "qcom,aux-codec-prefix";
	int num_strings;
	char *temp_str = NULL;
	const char *wsa_str = NULL;
	const char *wsa_prefix_str = NULL;
	enum codec_variant codec_ver = 0;
	const char *tasha_lite = "msm8976-tashalite-snd-card";
	u32 *index = NULL;
	u32 max_aux_dev = 0;
	int found = 0;

	card->dev = dev;
	ret = snd_soc_of_parse_card_name(card, "qcom,model");
	if (ret) {
		dev_err(dev, "%s: parse card name failed, err:%d\n",
				__func__, ret);
		return NULL;
	}

	if (!strcmp(card->name, "msm8952-tomtom-snd-card")) {
		len1 = ARRAY_SIZE(msm8952_common_fe_dai);
		len2 = len1 + ARRAY_SIZE(msm8952_tomtom_fe_dai);
		len3 = len2 + ARRAY_SIZE(msm8952_common_be_dai);
		snd_soc_card_msm[TOMTOM_CODEC].name = card->name;
		card = &snd_soc_card_msm[TOMTOM_CODEC];
		num_links = ARRAY_SIZE(msm8952_tomtom_dai_links);
		memcpy(msm8952_tomtom_dai_links, msm8952_common_fe_dai,
				sizeof(msm8952_common_fe_dai));
		memcpy(msm8952_tomtom_dai_links + len1,
			msm8952_tomtom_fe_dai, sizeof(msm8952_tomtom_fe_dai));
		memcpy(msm8952_tomtom_dai_links + len2,
			msm8952_common_be_dai, sizeof(msm8952_common_be_dai));
		memcpy(msm8952_tomtom_dai_links + len3,
			msm8952_tomtom_be_dai, sizeof(msm8952_tomtom_be_dai));
		msm8952_dai_links = msm8952_tomtom_dai_links;
	} else if (!strcmp(card->name, "msm8976-tasha-snd-card") ||
			!strcmp(card->name, "msm8976-tasha-skun-snd-card")) {
		codec_ver = tasha_codec_ver();
		if (codec_ver == WCD9326)
			card->name = tasha_lite;

		len1 = ARRAY_SIZE(msm8952_common_fe_dai);
		len2 = len1 + ARRAY_SIZE(msm8952_tasha_fe_dai);
		len3 = len2 + ARRAY_SIZE(msm8952_common_be_dai);
		snd_soc_card_msm[TASHA_CODEC].name = card->name;
		card = &snd_soc_card_msm[TASHA_CODEC];
		num_links = ARRAY_SIZE(msm8952_tasha_dai_links);
		memcpy(msm8952_tasha_dai_links, msm8952_common_fe_dai,
				sizeof(msm8952_common_fe_dai));
		memcpy(msm8952_tasha_dai_links + len1,
			msm8952_tasha_fe_dai, sizeof(msm8952_tasha_fe_dai));
		memcpy(msm8952_tasha_dai_links + len2,
			msm8952_common_be_dai, sizeof(msm8952_common_be_dai));
		memcpy(msm8952_tasha_dai_links + len3,
			msm8952_tasha_be_dai, sizeof(msm8952_tasha_be_dai));
		msm8952_dai_links = msm8952_tasha_dai_links;

		ret = of_property_read_u32(dev->of_node,
				"qcom,max-aux-codec", &max_aux_dev);
		if (ret) {
			dev_warn(dev,
				"%s: max-aux-codec property missing in DT %s, ret = %d\n",
				__func__, dev->of_node->full_name, ret);
			goto ret_card;
		}
		if (max_aux_dev == 0) {
			dev_warn(dev,
				"%s: No aux codec defined for this target.\n",
				__func__);
			goto ret_card;
		}
		index = devm_kzalloc(dev, (sizeof(u32) * max_aux_dev),
				GFP_KERNEL);
		if (!index)
			return NULL;

		num_strings = of_property_count_strings(dev->of_node,
				wsa);
		for (i = 0; i < num_strings; i++) {
			ret = of_property_read_string_index(
					dev->of_node, wsa,
					i, &wsa_str);
			if (ret) {
				dev_err(dev,
						"%s:of read string %s i %d error %d\n",
						__func__, wsa, i, ret);
				goto err;
			}
			if (soc_check_aux_dev_byname(card, wsa_str) == 0) {
				if (found >= max_aux_dev) {
					dev_err(dev,
					"%s: found  %d components. total %d\n",
					__func__, found, max_aux_dev);
					goto err;
				}
				index[found] = i;

				temp_str = NULL;
				temp_str = kstrdup(wsa_str, GFP_KERNEL);
				if (!temp_str)
					goto err;
				msm895x_aux_dev[found].codec_name = temp_str;

				temp_str = NULL;
				temp_str = kstrdup(wsa_str, GFP_KERNEL);
				if (!temp_str)
					goto err;
				msm895x_codec_conf[found].dev_name = temp_str;
				temp_str = NULL;
				found++;
			}
		}

		if (found < max_aux_dev) {
			dev_err(dev,
			"%s: failed to find %d components. Found only %d\n",
			__func__, max_aux_dev, found);
			goto err;
		}

		for (i = 0; i < max_aux_dev; i++) {
			ret = of_property_read_string_index(
					dev->of_node, wsa_prefix,
					index[i], &wsa_prefix_str);
			if (ret) {
				dev_err(dev,
					"%s:of read string %s i %d error %d\n",
					__func__, wsa_prefix, i, ret);
				goto err;
			}

			temp_str = kstrdup(wsa_prefix_str, GFP_KERNEL);
			if (!temp_str)
				goto err;
			msm895x_codec_conf[i].name_prefix = temp_str;

			temp_str = NULL;
		}
	}
	card->aux_dev = msm895x_aux_dev;
	card->codec_conf = msm895x_codec_conf;
ret_card:
	card->num_configs = max_aux_dev;
	card->num_aux_devs = max_aux_dev;
	card->dai_link = msm8952_dai_links;
	card->num_links = num_links;
	card->dev = dev;

	return card;
err:
	if (max_aux_dev > 0) {
		for (i = 0; i < max_aux_dev; i++) {
			kfree(msm895x_aux_dev[i].codec_name);
			kfree(msm895x_codec_conf[i].dev_name);
			kfree(msm895x_codec_conf[i].name_prefix);
		}
	}
	return NULL;
}

void msm895x_free_auxdev_mem(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	int i;

	if (card->num_aux_devs > 0) {
		for (i = 0; i < card->num_aux_devs; i++) {
			kfree(msm895x_aux_dev[i].codec_name);
			kfree(msm895x_codec_conf[i].dev_name);
			kfree(msm895x_codec_conf[i].name_prefix);
		}
	}
}
