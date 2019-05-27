snd-soc-wcd9xxx-codec-core-objs := wcd9xxx-rst.o wcd9xxx-core-init.o \
            wcd9xxx-core.o wcd9xxx-irq.o wcd9xxx-slimslave.o \
            wcd9xxx-utils.o wcd9335-regmap.o wcd9335-tables.o \
            wcd934x/wcd934x-regmap.o wcd934x/wcd934x-tables.o
obj-$(CONFIG_WCD9XXX_CODEC_CORE) += snd-soc-wcd9xxx-codec-core.o

snd-soc-wcd9xxx-v2-objs := wcd9xxx-common-v2.o wcd9xxx-resmgr-v2.o \
            wcdcal-hwdep.o wcd9xxx-soc-init.o wcd-dsp-utils.o \
            wcd-dsp-mgr.o
obj-$(CONFIG_SND_SOC_WCD9XXX_V2) += snd-soc-wcd9xxx-v2.o

obj-$(CONFIG_COMMON_CLK_QCOM) += audio-ext-clk-up.o
obj-$(CONFIG_COMMON_CLK_MSM) += audio-ext-clk.o

obj-$(CONFIG_SND_SOC_WCD9335) += wcd9335.o

snd-soc-wsa881x-objs := wsa881x.o wsa881x-tables.o \
            wsa881x-regmap.o wsa881x-temp-sensor.o
obj-$(CONFIG_SND_SOC_WSA881X) += snd-soc-wsa881x.o

snd-soc-wsa881x-analog-objs := wsa881x-analog.o wsa881x-tables-analog.o \
                wsa881x-regmap-analog.o
obj-$(CONFIG_SND_SOC_WSA881X_ANALOG) += snd-soc-wsa881x-analog.o

obj-$(CONFIG_SND_SOC_MSM_STUB) += msm_stub.o

obj-$(CONFIG_SND_SOC_WCD_SPI) += wcd-spi.o
obj-$(CONFIG_SND_SOC_WCD_CPE) += wcd_cpe_core.o wcd_cpe_services.o
obj-$(CONFIG_SND_SOC_WCD_MBHC) += wcd-mbhc-v2.o
obj-$(CONFIG_SND_SOC_WCD_MBHC_ADC) += wcd-mbhc-adc.o
obj-$(CONFIG_SND_SOC_WCD_MBHC_LEGACY) += wcd-mbhc-legacy.o

obj-$(CONFIG_SND_SOC_MSM_HDMI_CODEC_RX) += msm_hdmi_codec_rx.o

obj-$(CONFIG_MSM_CDC_PINCTRL) += msm-cdc-pinctrl.o
obj-$(CONFIG_MSM_CDC_SUPPLY) += msm-cdc-supply.o

obj-$(CONFIG_SND_SOC_BG_CODEC) += msm_bg/
obj-$(CONFIG_SND_SOC_WCD934X) += wcd934x/
obj-$(CONFIG_SND_SOC_SDM660_CDC) += sdm660_cdc/
obj-$(CONFIG_SND_SOC_MSM_SDW) += msm_sdw/
