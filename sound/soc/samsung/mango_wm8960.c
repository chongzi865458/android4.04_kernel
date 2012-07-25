/*
 *  smdk_wm8960.c
 *
 *  Copyright (c) 2009 Samsung Electronics Co. Ltd
 *  Author: Jaswinder Singh <jassi.brar@samsung.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <asm/mach-types.h>

#include "../codecs/wm8960.h"
#include "s3c-dma.h"
#include "s5pc1xx-i2s.h"

static int set_epll_rate(unsigned long rate)
{
        struct clk *fout_epll;

        fout_epll = clk_get(NULL, "fout_epll");
        if (IS_ERR(fout_epll)) {
                printk(KERN_ERR "%s: failed to get fout_epll\n", __func__);
                return -ENOENT;
        }

        if (rate == clk_get_rate(fout_epll))
                goto out;

        clk_set_rate(fout_epll, rate);
out:
        clk_put(fout_epll);

        return 0;
}

static int smdk_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
        struct snd_soc_dai *codec_dai = rtd->codec_dai;
        unsigned int rclk, psr = 1;
        int bfs, rfs, ret;

        switch (params_format(params)) {
        case SNDRV_PCM_FORMAT_U24:
        case SNDRV_PCM_FORMAT_S24:
                bfs = 48;
                break;
        case SNDRV_PCM_FORMAT_U16_LE:
        case SNDRV_PCM_FORMAT_S16_LE:
                bfs = 32;
                break;
        default:
                return -EINVAL;
        }

        /* The Fvco for WM8960 PLLs must fall within [90,100]MHz.
         * This criterion can't be met if we request PLL output
         * as {8000x256, 64000x256, 11025x256}Hz.
         * As a wayout, we rather change rfs to a minimum value that
         * results in (params_rate(params) * rfs), and itself, acceptable
         * to both - the CODEC and the CPU.
         */
        switch (params_rate(params)) {
        case 16000:
        case 22050:
        case 24000:
        case 32000:
        case 44100:
        case 48000:
        case 88200:
        case 96000:
                if (bfs == 48)
                        rfs = 384;
                else
                        rfs = 256;
                break;
        case 64000:
                rfs = 384;
                break;
        case 8000:
        case 11025:
        case 12000:
                if (bfs == 48)
                        rfs = 768;
                else
                        rfs = 512;
                break;
        default:
                return -EINVAL;
        }

        rclk = params_rate(params) * rfs;

        switch (rclk) {
        case 4096000:
        case 5644800:
        case 6144000:
        case 8467200:
        case 9216000:
                psr = 8;
                break;
        case 8192000:
        case 11289600:
        case 12288000:
        case 16934400:
        case 18432000:
                psr = 4;
                break;
        case 22579200:
        case 24576000:
        case 33868800:
        case 36864000:
                psr = 2;
                break;
        case 67737600:
        case 73728000:
                psr = 1;
                break;
        default:
                printk(KERN_ERR "Not yet supported!\n");
                return -EINVAL;
        }

        set_epll_rate(rclk * psr);

        ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
                                SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);

        if (ret < 0)
                return ret;
        ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
                                SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);

        if (ret < 0)

                return ret;

        ret = snd_soc_dai_set_sysclk(cpu_dai, S3C64XX_CLKSRC_CDCLK,
                                        0, SND_SOC_CLOCK_OUT);
        if (ret < 0)
                return ret;

        /* We use MUX for basic ops in SoC-Master mode */
        ret = snd_soc_dai_set_sysclk(cpu_dai, S3C64XX_CLKSRC_MUX,
                                        0, SND_SOC_CLOCK_IN);
        if (ret < 0)
                return ret;

        ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_I2SV2_DIV_PRESCALER, psr-1);
        if (ret < 0)
                return ret;

        ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_I2SV2_DIV_BCLK, bfs);
        if (ret < 0)
                return ret;

        ret = snd_soc_dai_set_clkdiv(cpu_dai, S3C_I2SV2_DIV_RCLK, rfs);
        if (ret < 0)
                return ret;

        return 0;
}

/*
 * SMDK WM8960 DAI operations.
 */
static struct snd_soc_ops smdk_ops = {
	.hw_params = smdk_hw_params,
};

/* SMDK Playback widgets */
static const struct snd_soc_dapm_widget wm8960_dapm_widgets_pbk[] = {
        SND_SOC_DAPM_HP("Speaker-L/R", NULL),
        SND_SOC_DAPM_HP("HP-L/R", NULL),
};

/* SMDK Capture widgets */
static const struct snd_soc_dapm_widget wm8960_dapm_widgets_cpt[] = {
        SND_SOC_DAPM_MIC("MicIn", NULL),
        SND_SOC_DAPM_LINE("LineIn", NULL),
};

/* SMDK-PAIFTX connections */
static const struct snd_soc_dapm_route audio_map_tx[] = {
        /* MicIn feeds LINPUT1/2 */
        {"LINPUT1", NULL, "MicIn"},
        {"LINPUT2", NULL, "MicIn"},
#if 1
        {"LINPUT3", NULL, "LineIn"},
        {"RINPUT3", NULL, "LineIn"},
#endif
};

/* SMDK-PAIFRX connections */
static const struct snd_soc_dapm_route audio_map_rx[] = {
        {"Speaker-L/R", NULL, "SPK_LP"},
        {"Speaker-L/R", NULL, "SPK_LN"},
        {"Speaker-L/R", NULL, "SPK_RP"},
        {"Speaker-L/R", NULL, "SPK_RN"},
        {"HP-L/R", NULL, "HP_L"},
        {"HP-L/R", NULL, "HP_R"},
};

static int smdk_wm8960_init_paiftx(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	/* Add smdk specific Capture widgets */
	snd_soc_dapm_new_controls(dapm, wm8960_dapm_widgets_cpt,
				  ARRAY_SIZE(wm8960_dapm_widgets_cpt));

	/* Set up PAIFTX audio path */
	snd_soc_dapm_add_routes(dapm, audio_map_tx, ARRAY_SIZE(audio_map_tx));

	/* Enabling the microphone requires the fitting of a 0R
	 * resistor to connect the line from the microphone jack.
	 */
	snd_soc_dapm_disable_pin(dapm, "MicIn");

	/* signal a DAPM event */
	snd_soc_dapm_sync(dapm);

	return 0;
}

static int smdk_wm8960_init_paifrx(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	/* Add smdk specific Playback widgets */
	snd_soc_dapm_new_controls(dapm, wm8960_dapm_widgets_pbk,
				  ARRAY_SIZE(wm8960_dapm_widgets_pbk));

	/* Set up PAIFRX audio path */
	snd_soc_dapm_add_routes(dapm, audio_map_rx, ARRAY_SIZE(audio_map_rx));

	/* signal a DAPM event */
	snd_soc_dapm_sync(dapm);

	return 0;
}

enum {
	PRI_PLAYBACK = 0,
	PRI_CAPTURE,
	SEC_PLAYBACK,
};

static struct snd_soc_dai_link smdk_dai[] = {
	[PRI_PLAYBACK] = { /* Primary Playback i/f */
		.name = "WM8960 PAIF RX",
		.stream_name = "Playback",
		.cpu_dai_name = "samsung-i2s.0",
		.codec_dai_name = "wm8960-hifi",
		.platform_name = "samsung-audio",
		.codec_name = "wm8960-codec.0-001a",
		.init = smdk_wm8960_init_paifrx,
		.ops = &smdk_ops,
	},
	[PRI_CAPTURE] = { /* Primary Capture i/f */
		.name = "WM8960 PAIF TX",
		.stream_name = "Capture",
		.cpu_dai_name = "samsung-i2s.0",
		.codec_dai_name = "wm8960-hifi",
		.platform_name = "samsung-audio",
		.codec_name = "wm8960-codec.0-001a",
		.init = smdk_wm8960_init_paiftx,
		.ops = &smdk_ops,
	},
};

static struct snd_soc_card smdk = {
	.name = "SMDK-I2S",
	.dai_link = smdk_dai,
	.num_links = 2,
};

static struct platform_device *smdk_snd_device;

static int __init smdk_audio_init(void)
{
	int ret;
	char *str;
#if 0
	if (machine_is_smdkc100()
			|| machine_is_smdkv210() || machine_is_smdkc110()) {
		smdk.num_links = 3;
		/* Secondary is at offset SAMSUNG_I2S_SECOFF from Primary */
		str = (char *)smdk_dai[SEC_PLAYBACK].cpu_dai_name;
		str[strlen(str) - 1] = '0' + SAMSUNG_I2S_SECOFF;
	} else if (machine_is_smdk6410()) {
		str = (char *)smdk_dai[PRI_PLAYBACK].cpu_dai_name;
		str[strlen(str) - 1] = '2';
		str = (char *)smdk_dai[PRI_CAPTURE].cpu_dai_name;
		str[strlen(str) - 1] = '2';
	}
#endif
	smdk_snd_device = platform_device_alloc("soc-audio", -1);
	if (!smdk_snd_device)
		return -ENOMEM;

	platform_set_drvdata(smdk_snd_device, &smdk);
	ret = platform_device_add(smdk_snd_device);

	if (ret)
		platform_device_put(smdk_snd_device);

	return ret;
}
module_init(smdk_audio_init);

static void __exit smdk_audio_exit(void)
{
	platform_device_unregister(smdk_snd_device);
}
module_exit(smdk_audio_exit);

MODULE_AUTHOR("Jaswinder Singh, jassi.brar@samsung.com");
MODULE_DESCRIPTION("ALSA SoC SMDK WM8960");
MODULE_LICENSE("GPL");
