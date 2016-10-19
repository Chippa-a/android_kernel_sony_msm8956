/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#ifndef _DT_BINDINGS_CLK_MSM_GCC_SKUNK_H
#define _DT_BINDINGS_CLK_MSM_GCC_SKUNK_H

/* GCC clocks */
#define GCC_AGGRE_NOC_PCIE_TBU_CLK				0
#define GCC_AGGRE_UFS_CARD_AXI_CLK				1
#define GCC_AGGRE_UFS_PHY_AXI_CLK				2
#define GCC_AGGRE_USB3_PRIM_AXI_CLK				3
#define GCC_AGGRE_USB3_SEC_AXI_CLK				4
#define GCC_BOOT_ROM_AHB_CLK					5
#define GCC_CAMERA_AHB_CLK					6
#define GCC_CAMERA_AXI_CLK					7
#define GCC_CAMERA_XO_CLK					8
#define GCC_CFG_NOC_USB3_PRIM_AXI_CLK				9
#define GCC_CFG_NOC_USB3_SEC_AXI_CLK				10
#define GCC_CPUSS_AHB_CLK					11
#define GCC_CPUSS_AHB_CLK_SRC					12
#define GCC_CPUSS_DVM_BUS_CLK					13
#define GCC_CPUSS_GNOC_CLK					14
#define GCC_CPUSS_RBCPR_CLK					15
#define GCC_CPUSS_RBCPR_CLK_SRC					16
#define GCC_CXO_TX1_CLKREF_CLK					17
#define GCC_DDRSS_GPU_AXI_CLK					18
#define GCC_DISP_AHB_CLK					19
#define GCC_DISP_AXI_CLK					20
#define GCC_DISP_GPLL0_CLK_SRC					21
#define GCC_DISP_GPLL0_DIV_CLK_SRC				22
#define GCC_DISP_XO_CLK						23
#define GCC_GP1_CLK						24
#define GCC_GP1_CLK_SRC						25
#define GCC_GP2_CLK						26
#define GCC_GP2_CLK_SRC						27
#define GCC_GP3_CLK						28
#define GCC_GP3_CLK_SRC						29
#define GCC_GPU_CFG_AHB_CLK					30
#define GCC_GPU_GPLL0_CLK_SRC					31
#define GCC_GPU_GPLL0_DIV_CLK_SRC				32
#define GCC_GPU_MEMNOC_GFX_CLK					33
#define GCC_GPU_SNOC_DVM_GFX_CLK				34
#define GCC_MMSS_QM_AHB_CLK					35
#define GCC_MMSS_QM_CORE_CLK					36
#define GCC_MMSS_QM_CORE_CLK_SRC				37
#define GCC_MSS_AXIS2_CLK					38
#define GCC_MSS_CFG_AHB_CLK					39
#define GCC_MSS_GPLL0_DIV_CLK_SRC				40
#define GCC_MSS_MFAB_AXIS_CLK					41
#define GCC_MSS_Q6_MEMNOC_AXI_CLK				42
#define GCC_MSS_SNOC_AXI_CLK					43
#define GCC_PCIE_0_AUX_CLK					44
#define GCC_PCIE_0_AUX_CLK_SRC					45
#define GCC_PCIE_0_CFG_AHB_CLK					46
#define GCC_PCIE_0_CLKREF_CLK					47
#define GCC_PCIE_0_MSTR_AXI_CLK					48
#define GCC_PCIE_0_PIPE_CLK					49
#define GCC_PCIE_0_SLV_AXI_CLK					50
#define GCC_PCIE_0_SLV_Q2A_AXI_CLK				51
#define GCC_PCIE_1_AUX_CLK					52
#define GCC_PCIE_1_AUX_CLK_SRC					53
#define GCC_PCIE_1_CFG_AHB_CLK					54
#define GCC_PCIE_1_CLKREF_CLK					55
#define GCC_PCIE_1_MSTR_AXI_CLK					56
#define GCC_PCIE_1_PIPE_CLK					57
#define GCC_PCIE_1_SLV_AXI_CLK					58
#define GCC_PCIE_1_SLV_Q2A_AXI_CLK				59
#define GCC_PCIE_PHY_AUX_CLK					60
#define GCC_PCIE_PHY_REFGEN_CLK					61
#define GCC_PCIE_PHY_REFGEN_CLK_SRC				62
#define GCC_PDM2_CLK						63
#define GCC_PDM2_CLK_SRC					64
#define GCC_PDM_AHB_CLK						65
#define GCC_PDM_XO4_CLK						66
#define GCC_PRNG_AHB_CLK					67
#define GCC_QMIP_CAMERA_AHB_CLK					68
#define GCC_QMIP_DISP_AHB_CLK					69
#define GCC_QMIP_VIDEO_AHB_CLK					70
#define GCC_QUPV3_WRAP0_CORE_2X_CLK				71
#define GCC_QUPV3_WRAP0_CORE_2X_CLK_SRC				72
#define GCC_QUPV3_WRAP0_CORE_CLK				73
#define GCC_QUPV3_WRAP0_S0_CLK					74
#define GCC_QUPV3_WRAP0_S0_CLK_SRC				75
#define GCC_QUPV3_WRAP0_S1_CLK					76
#define GCC_QUPV3_WRAP0_S1_CLK_SRC				77
#define GCC_QUPV3_WRAP0_S2_CLK					78
#define GCC_QUPV3_WRAP0_S2_CLK_SRC				79
#define GCC_QUPV3_WRAP0_S3_CLK					80
#define GCC_QUPV3_WRAP0_S3_CLK_SRC				81
#define GCC_QUPV3_WRAP0_S4_CLK					82
#define GCC_QUPV3_WRAP0_S4_CLK_SRC				83
#define GCC_QUPV3_WRAP0_S5_CLK					84
#define GCC_QUPV3_WRAP0_S5_CLK_SRC				85
#define GCC_QUPV3_WRAP0_S6_CLK					86
#define GCC_QUPV3_WRAP0_S6_CLK_SRC				87
#define GCC_QUPV3_WRAP0_S7_CLK					88
#define GCC_QUPV3_WRAP0_S7_CLK_SRC				89
#define GCC_QUPV3_WRAP1_CORE_2X_CLK				90
#define GCC_QUPV3_WRAP1_CORE_CLK				91
#define GCC_QUPV3_WRAP1_S0_CLK					92
#define GCC_QUPV3_WRAP1_S0_CLK_SRC				93
#define GCC_QUPV3_WRAP1_S1_CLK					94
#define GCC_QUPV3_WRAP1_S1_CLK_SRC				95
#define GCC_QUPV3_WRAP1_S2_CLK					96
#define GCC_QUPV3_WRAP1_S2_CLK_SRC				97
#define GCC_QUPV3_WRAP1_S3_CLK					98
#define GCC_QUPV3_WRAP1_S3_CLK_SRC				99
#define GCC_QUPV3_WRAP1_S4_CLK					100
#define GCC_QUPV3_WRAP1_S4_CLK_SRC				101
#define GCC_QUPV3_WRAP1_S5_CLK					102
#define GCC_QUPV3_WRAP1_S5_CLK_SRC				103
#define GCC_QUPV3_WRAP1_S6_CLK					104
#define GCC_QUPV3_WRAP1_S6_CLK_SRC				105
#define GCC_QUPV3_WRAP1_S7_CLK					106
#define GCC_QUPV3_WRAP1_S7_CLK_SRC				107
#define GCC_QUPV3_WRAP_0_M_AHB_CLK				108
#define GCC_QUPV3_WRAP_0_S_AHB_CLK				109
#define GCC_QUPV3_WRAP_1_M_AHB_CLK				110
#define GCC_QUPV3_WRAP_1_S_AHB_CLK				111
#define GCC_RX1_USB2_CLKREF_CLK					112
#define GCC_RX2_QLINK_CLKREF_CLK				113
#define GCC_RX3_MODEM_CLKREF_CLK				114
#define GCC_SDCC2_AHB_CLK					115
#define GCC_SDCC2_APPS_CLK					116
#define GCC_SDCC2_APPS_CLK_SRC					117
#define GCC_SDCC4_AHB_CLK					118
#define GCC_SDCC4_APPS_CLK					119
#define GCC_SDCC4_APPS_CLK_SRC					120
#define GCC_SYS_NOC_CPUSS_AHB_CLK				121
#define GCC_TSIF_AHB_CLK					122
#define GCC_TSIF_INACTIVITY_TIMERS_CLK				123
#define GCC_TSIF_REF_CLK					124
#define GCC_TSIF_REF_CLK_SRC					125
#define GCC_UFS_CARD_AHB_CLK					126
#define GCC_UFS_CARD_AXI_CLK					127
#define GCC_UFS_CARD_AXI_CLK_SRC				128
#define GCC_UFS_CARD_CLKREF_CLK					129
#define GCC_UFS_CARD_ICE_CORE_CLK				130
#define GCC_UFS_CARD_ICE_CORE_CLK_SRC				131
#define GCC_UFS_CARD_PHY_AUX_CLK				132
#define GCC_UFS_CARD_PHY_AUX_CLK_SRC				133
#define GCC_UFS_CARD_RX_SYMBOL_0_CLK				134
#define GCC_UFS_CARD_RX_SYMBOL_1_CLK				135
#define GCC_UFS_CARD_TX_SYMBOL_0_CLK				136
#define GCC_UFS_CARD_UNIPRO_CORE_CLK				137
#define GCC_UFS_CARD_UNIPRO_CORE_CLK_SRC			138
#define GCC_UFS_MEM_CLKREF_CLK					139
#define GCC_UFS_PHY_AHB_CLK					140
#define GCC_UFS_PHY_AXI_CLK					141
#define GCC_UFS_PHY_AXI_CLK_SRC					142
#define GCC_UFS_PHY_ICE_CORE_CLK				143
#define GCC_UFS_PHY_ICE_CORE_CLK_SRC				144
#define GCC_UFS_PHY_PHY_AUX_CLK					145
#define GCC_UFS_PHY_PHY_AUX_CLK_SRC				146
#define GCC_UFS_PHY_RX_SYMBOL_0_CLK				147
#define GCC_UFS_PHY_RX_SYMBOL_1_CLK				148
#define GCC_UFS_PHY_TX_SYMBOL_0_CLK				149
#define GCC_UFS_PHY_UNIPRO_CORE_CLK				150
#define GCC_UFS_PHY_UNIPRO_CORE_CLK_SRC				151
#define GCC_USB30_PRIM_MASTER_CLK				152
#define GCC_USB30_PRIM_MASTER_CLK_SRC				153
#define GCC_USB30_PRIM_MOCK_UTMI_CLK				154
#define GCC_USB30_PRIM_MOCK_UTMI_CLK_SRC			155
#define GCC_USB30_PRIM_SLEEP_CLK				156
#define GCC_USB30_SEC_MASTER_CLK				157
#define GCC_USB30_SEC_MASTER_CLK_SRC				158
#define GCC_USB30_SEC_MOCK_UTMI_CLK				159
#define GCC_USB30_SEC_MOCK_UTMI_CLK_SRC				160
#define GCC_USB30_SEC_SLEEP_CLK					161
#define GCC_USB3_PRIM_CLKREF_CLK				162
#define GCC_USB3_PRIM_PHY_AUX_CLK				163
#define GCC_USB3_PRIM_PHY_AUX_CLK_SRC				164
#define GCC_USB3_PRIM_PHY_COM_AUX_CLK				165
#define GCC_USB3_PRIM_PHY_PIPE_CLK				166
#define GCC_USB3_SEC_CLKREF_CLK					167
#define GCC_USB3_SEC_PHY_AUX_CLK				168
#define GCC_USB3_SEC_PHY_AUX_CLK_SRC				169
#define GCC_USB3_SEC_PHY_COM_AUX_CLK				170
#define GCC_USB3_SEC_PHY_PIPE_CLK				171
#define GCC_USB_PHY_CFG_AHB2PHY_CLK				172
#define GCC_VIDEO_AHB_CLK					173
#define GCC_VIDEO_AXI_CLK					174
#define GCC_VIDEO_XO_CLK					175
#define GPLL0							176
#define GPLL0_OUT_EVEN						177
#define GPLL0_OUT_MAIN						178
#define GPLL1							179
#define GPLL1_OUT_MAIN						180

/* RPMh controlled clocks */
#define RPMH_CXO_CLK						0
#define RPMH_CXO_A_CLK						1
#define RPMH_QDSS_CLK						2
#define RPMH_QDSS_A_CLK						3

/* GCC reset clocks */
#define GCC_GPU_BCR						0
#define GCC_MMSS_BCR						1
#define GCC_PCIE_0_BCR						2
#define GCC_PCIE_1_BCR						3
#define GCC_PCIE_PHY_BCR					4
#define GCC_PDM_BCR						5
#define GCC_PRNG_BCR						6
#define GCC_QUPV3_WRAPPER_0_BCR					7
#define GCC_QUPV3_WRAPPER_1_BCR					8
#define GCC_SDCC2_BCR						9
#define GCC_SDCC4_BCR						10
#define GCC_TSIF_BCR						11
#define GCC_UFS_CARD_BCR					12
#define GCC_UFS_PHY_BCR						13
#define GCC_USB30_PRIM_BCR					14
#define GCC_USB30_SEC_BCR					15
#define GCC_USB_PHY_CFG_AHB2PHY_BCR				16

#endif
