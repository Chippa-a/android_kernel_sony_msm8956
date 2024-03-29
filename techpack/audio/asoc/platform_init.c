/*
Copyright (c) 2017, 2020 The Linux Foundation. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 and
only version 2 as published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
*
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include "platform_init.h"

static int __init audio_platform_init(void)
{
	msm_compress_dsp_init();
	msm_fe_dai_init();
	msm_dai_q6_hdmi_init();
	msm_dai_q6_init();
	msm_dai_slim_init();
	msm_dai_stub_init();
	msm_lsm_client_init();
	msm_pcm_afe_init();
	msm_pcm_dtmf_init();
	msm_pcm_hostless_init();
	msm_voice_host_init();
	msm_pcm_loopback_init();
	msm_pcm_noirq_init();
	msm_pcm_dsp_init();
	msm_soc_routing_platform_init();
	msm_pcm_voice_init();
	msm_pcm_voip_init();
	msm_transcode_loopback_init();
#if defined(CONFIG_SND_SOC_MSM8909) || \
	defined(CONFIG_SND_SOC_MSM8952)
	voice_svc_init();
#endif

	return 0;
}

static void audio_platform_exit(void)
{
	msm_transcode_loopback_exit();
	msm_pcm_voip_exit();
	msm_pcm_voice_exit();
	msm_soc_routing_platform_exit();
	msm_pcm_dsp_exit();
	msm_pcm_noirq_exit();
	msm_pcm_loopback_exit();
	msm_voice_host_exit();
	msm_pcm_hostless_exit();
	msm_pcm_dtmf_exit();
	msm_pcm_afe_exit();
	msm_lsm_client_exit();
	msm_dai_stub_exit();
	msm_dai_slim_exit();
	msm_dai_q6_exit();
	msm_dai_q6_hdmi_exit();
	msm_fe_dai_exit();
	msm_compress_dsp_exit();
#if defined(CONFIG_SND_SOC_MSM8909) || \
	defined(CONFIG_SND_SOC_MSM8952)
	voice_svc_exit();
#endif
}

module_init(audio_platform_init);
module_exit(audio_platform_exit);

MODULE_DESCRIPTION("Audio Platform driver");
MODULE_LICENSE("GPL v2");
