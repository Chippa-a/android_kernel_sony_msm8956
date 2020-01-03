# MSM Machine Support

# for APQ8009 external codec sound card driver
snd-soc-ext-codec-8909-objs := apq8009-i2s-ext-codec.o
obj-$(CONFIG_SND_SOC_EXT_CODEC_8909) += snd-soc-ext-codec-8909.o

# for MSM8909 sound card driver
snd-soc-msm8909-objs := msm8952.o voice_svc.o
obj-$(CONFIG_SND_SOC_MSM8909) += snd-soc-msm8909.o

snd-soc-bg-8909-objs := msm_bg.o
obj-$(CONFIG_SND_SOC_BG_8909) += snd-soc-bg-8909.o

snd-soc-8909-dig-cdc-objs := msm8909.o
obj-$(CONFIG_SND_SOC_8909_DIG_CDC) += snd-soc-8909-dig-cdc.o

# for MSM8952 sound card driver
snd-soc-msm8952-objs := msm8952.o msm8952-slimbus.o msm8952-dai-links.o voice_svc.o
obj-$(CONFIG_SND_SOC_MSM8952) += snd-soc-msm8952.o

# for SDM450 internal codec sound card driver
snd-soc-sdm450-objs := msm8952.o
obj-$(CONFIG_SND_SOC_SDM450) += snd-soc-sdm450.o

# for SDM450 external codec sound card driver
snd-soc-ext-codec-sdm450-objs := msm8952-slimbus.o msm8952-dai-links.o
obj-$(CONFIG_SND_SOC_EXT_CODEC_SDM450) += snd-soc-ext-codec-sdm450.o

# for SDM6xx sound card driver
snd-soc-sdm670-objs := sdm660-common.o
obj-$(CONFIG_SND_SOC_SDM670) += snd-soc-sdm670.o

snd-soc-int-codec-objs := sdm660-internal.o
obj-$(CONFIG_SND_SOC_INT_CODEC) += snd-soc-int-codec.o

snd-soc-ext-codec-objs := sdm660-external.o sdm660-ext-dai-links.o
obj-$(CONFIG_SND_SOC_EXT_CODEC) += snd-soc-ext-codec.o

# for SDXPOORWILLS sound card driver
snd-soc-sdxpoorwills-objs := sdxpoorwills.o
obj-$(CONFIG_SND_SOC_MACHINE_SDXPOORWILLS) += snd-soc-sdxpoorwills.o

# for SDM845 sound card driver
snd-soc-sdm845-objs := sdm845.o
obj-$(CONFIG_SND_SOC_MACHINE_SDM845) += snd-soc-sdm845.o

# for CPE drivers
snd-soc-cpe-objs := msm-cpe-lsm.o
obj-$(CONFIG_SND_SOC_CPE) += snd-soc-cpe.o

snd-soc-qdsp6v2-objs := msm-audio-effects-q6-v2.o msm-compress-q6-v2.o \
            msm-dai-fe.o msm-dai-q6-hdmi-v2.o msm-dai-q6-v2.o msm-dai-slim.o  \
            msm-dai-stub-v2.o msm-lsm-client.o msm-pcm-afe-v2.o msm-pcm-dtmf-v2.o \
            msm-pcm-host-voice-v2.o msm-pcm-loopback-v2.o msm-pcm-q6-noirq.o \
            msm-pcm-q6-v2.o msm-pcm-routing-v2.o msm-pcm-voice-v2.o msm-pcm-voip-v2.o \
            msm-transcode-loopback-q6-v2.o
obj-$(CONFIG_SND_SOC_QDSP6V2) += snd-soc-qdsp6v2.o

snd-soc-hostless-pcm-objs := msm-pcm-hostless.o
obj-$(CONFIG_SND_SOC_MSM_HOSTLESS_PCM) += snd-soc-hostless-pcm.o

obj-$(CONFIG_SND_HWDEP) += msm-pcm-routing-devdep.o
obj-$(CONFIG_DOLBY_DAP) += msm-dolby-dap-config.o
obj-$(CONFIG_DOLBY_DS2) += msm-ds2-dap-config.o
obj-$(CONFIG_AHC) += msm-ahc-config.o
obj-$(CONFIG_DOLBY_LICENSE) += msm-ds2-dap-config.o
obj-$(CONFIG_DTS_SRS_TM) += msm-dts-srs-tm-config.o
obj-$(CONFIG_QTI_PP) += msm-qti-pp-config.o
obj-y += codecs/
