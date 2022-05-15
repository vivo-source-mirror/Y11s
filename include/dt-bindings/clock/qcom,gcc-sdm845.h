/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 */

#ifndef _DT_BINDINGS_CLK_SDM_GCC_SDM845_H
#define _DT_BINDINGS_CLK_SDM_GCC_SDM845_H

/* GCC clock registers */
#define GCC_AGGRE_NOC_PCIE_TBU_CLK				0
#define GCC_AGGRE_UFS_CARD_AXI_CLK				1
#define GCC_AGGRE_UFS_PHY_AXI_CLK				2
#define GCC_AGGRE_USB3_PRIM_AXI_CLK				3
#define GCC_AGGRE_USB3_SEC_AXI_CLK				4
#define GCC_BOOT_ROM_AHB_CLK					5
#define GCC_CAMERA_AHB_CLK					6
#define GCC_CAMERA_AXI_CLK					7
#define GCC_CAMERA_XO_CLK					8
#define GCC_CE1_AHB_CLK						9
#define GCC_CE1_AXI_CLK						10
#define GCC_CE1_CLK						11
#define GCC_CFG_NOC_USB3_PRIM_AXI_CLK				12
#define GCC_CFG_NOC_USB3_SEC_AXI_CLK				13
#define GCC_CPUSS_AHB_CLK					14
#define GCC_CPUSS_AHB_CLK_SRC					15
#define GCC_CPUSS_RBCPR_CLK					16
#define GCC_CPUSS_RBCPR_CLK_SRC					17
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
#define GCC_MSS_AXIS2_CLK					35
#define GCC_MSS_CFG_AHB_CLK					36
#define GCC_MSS_GPLL0_DIV_CLK_SRC				37
#define GCC_MSS_MFAB_AXIS_CLK					38
#define GCC_MSS_Q6_MEMNOC_AXI_CLK				39
#define GCC_MSS_SNOC_AXI_CLK					40
#define GCC_PCIE_0_AUX_CLK					41
#define GCC_PCIE_0_AUX_CLK_SRC					42
#define GCC_PCIE_0_CFG_AHB_CLK					43
#define GCC_PCIE_0_CLKREF_CLK					44
#define GCC_PCIE_0_MSTR_AXI_CLK					45
#define GCC_PCIE_0_PIPE_CLK					46
#define GCC_PCIE_0_SLV_AXI_CLK					47
#define GCC_PCIE_0_SLV_Q2A_AXI_CLK				48
#define GCC_PCIE_1_AUX_CLK					49
#define GCC_PCIE_1_AUX_CLK_SRC					50
#define GCC_PCIE_1_CFG_AHB_CLK					51
#define GCC_PCIE_1_CLKREF_CLK					52
#define GCC_PCIE_1_MSTR_AXI_CLK					53
#define GCC_PCIE_1_PIPE_CLK					54
#define GCC_PCIE_1_SLV_AXI_CLK					55
#define GCC_PCIE_1_SLV_Q2A_AXI_CLK				56
#define GCC_PCIE_PHY_AUX_CLK					57
#define GCC_PCIE_PHY_REFGEN_CLK					58
#define GCC_PCIE_PHY_REFGEN_CLK_SRC				59
#define GCC_PDM2_CLK						60
#define GCC_PDM2_CLK_SRC					61
#define GCC_PDM_AHB_CLK						62
#define GCC_PDM_XO4_CLK						63
#define GCC_PRNG_AHB_CLK					64
#define GCC_QMIP_CAMERA_AHB_CLK					65
#define GCC_QMIP_DISP_AHB_CLK					66
#define GCC_QMIP_VIDEO_AHB_CLK					67
#define GCC_QUPV3_WRAP0_S0_CLK					68
#define GCC_QUPV3_WRAP0_S0_CLK_SRC				69
#define GCC_QUPV3_WRAP0_S1_CLK					70
#define GCC_QUPV3_WRAP0_S1_CLK_SRC				71
#define GCC_QUPV3_WRAP0_S2_CLK					72
#define GCC_QUPV3_WRAP0_S2_CLK_SRC				73
#define GCC_QUPV3_WRAP0_S3_CLK					74
#define GCC_QUPV3_WRAP0_S3_CLK_SRC				75
#define GCC_QUPV3_WRAP0_S4_CLK					76
#define GCC_QUPV3_WRAP0_S4_CLK_SRC				77
#define GCC_QUPV3_WRAP0_S5_CLK					78
#define GCC_QUPV3_WRAP0_S5_CLK_SRC				79
#define GCC_QUPV3_WRAP0_S6_CLK					80
#define GCC_QUPV3_WRAP0_S6_CLK_SRC				81
#define GCC_QUPV3_WRAP0_S7_CLK					82
#define GCC_QUPV3_WRAP0_S7_CLK_SRC				83
#define GCC_QUPV3_WRAP1_S0_CLK					84
#define GCC_QUPV3_WRAP1_S0_CLK_SRC				85
#define GCC_QUPV3_WRAP1_S1_CLK					86
#define GCC_QUPV3_WRAP1_S1_CLK_SRC				87
#define GCC_QUPV3_WRAP1_S2_CLK					88
#define GCC_QUPV3_WRAP1_S2_CLK_SRC				89
#define GCC_QUPV3_WRAP1_S3_CLK					90
#define GCC_QUPV3_WRAP1_S3_CLK_SRC				91
#define GCC_QUPV3_WRAP1_S4_CLK					92
#define GCC_QUPV3_WRAP1_S4_CLK_SRC				93
#define GCC_QUPV3_WRAP1_S5_CLK					94
#define GCC_QUPV3_WRAP1_S5_CLK_SRC				95
#define GCC_QUPV3_WRAP1_S6_CLK					96
#define GCC_QUPV3_WRAP1_S6_CLK_SRC				97
#define GCC_QUPV3_WRAP1_S7_CLK					98
#define GCC_QUPV3_WRAP1_S7_CLK_SRC				99
#define GCC_QUPV3_WRAP_0_M_AHB_CLK				100
#define GCC_QUPV3_WRAP_0_S_AHB_CLK				101
#define GCC_QUPV3_WRAP_1_M_AHB_CLK				102
#define GCC_QUPV3_WRAP_1_S_AHB_CLK				103
#define GCC_SDCC2_AHB_CLK					104
#define GCC_SDCC2_APPS_CLK					105
#define GCC_SDCC2_APPS_CLK_SRC					106
#define GCC_SDCC4_AHB_CLK					107
#define GCC_SDCC4_APPS_CLK					108
#define GCC_SDCC4_APPS_CLK_SRC					109
#define GCC_SYS_NOC_CPUSS_AHB_CLK				110
#define GCC_TSIF_AHB_CLK					111
#define GCC_TSIF_INACTIVITY_TIMERS_CLK				112
#define GCC_TSIF_REF_CLK					113
#define GCC_TSIF_REF_CLK_SRC					114
#define GCC_UFS_CARD_AHB_CLK					115
#define GCC_UFS_CARD_AXI_CLK					116
#define GCC_UFS_CARD_AXI_CLK_SRC				117
#define GCC_UFS_CARD_CLKREF_CLK					118
#define GCC_UFS_CARD_ICE_CORE_CLK				119
#define GCC_UFS_CARD_ICE_CORE_CLK_SRC				120
#define GCC_UFS_CARD_PHY_AUX_CLK				121
#define GCC_UFS_CARD_PHY_AUX_CLK_SRC				122
#define GCC_UFS_CARD_RX_SYMBOL_0_CLK				123
#define GCC_UFS_CARD_RX_SYMBOL_1_CLK				124
#define GCC_UFS_CARD_TX_SYMBOL_0_CLK				125
#define GCC_UFS_CARD_UNIPRO_CORE_CLK				126
#define GCC_UFS_CARD_UNIPRO_CORE_CLK_SRC			127
#define GCC_UFS_MEM_CLKREF_CLK					128
#define GCC_UFS_PHY_AHB_CLK					129
#define GCC_UFS_PHY_AXI_CLK					130
#define GCC_UFS_PHY_AXI_CLK_SRC					131
#define GCC_UFS_PHY_ICE_CORE_CLK				132
#define GCC_UFS_PHY_ICE_CORE_CLK_SRC				133
#define GCC_UFS_PHY_PHY_AUX_CLK					134
#define GCC_UFS_PHY_PHY_AUX_CLK_SRC				135
#define GCC_UFS_PHY_RX_SYMBOL_0_CLK				136
#define GCC_UFS_PHY_RX_SYMBOL_1_CLK				137
#define GCC_UFS_PHY_TX_SYMBOL_0_CLK				138
#define GCC_UFS_PHY_UNIPRO_CORE_CLK				139
#define GCC_UFS_PHY_UNIPRO_CORE_CLK_SRC				140
#define GCC_USB30_PRIM_MASTER_CLK				141
#define GCC_USB30_PRIM_MASTER_CLK_SRC				142
#define GCC_USB30_PRIM_MOCK_UTMI_CLK				143
#define GCC_USB30_PRIM_MOCK_UTMI_CLK_SRC			144
#define GCC_USB30_PRIM_SLEEP_CLK				145
#define GCC_USB30_SEC_MASTER_CLK				146
#define GCC_USB30_SEC_MASTER_CLK_SRC				147
#define GCC_USB30_SEC_MOCK_UTMI_CLK				148
#define GCC_USB30_SEC_MOCK_UTMI_CLK_SRC				149
#define GCC_USB30_SEC_SLEEP_CLK					150
#define GCC_USB3_PRIM_CLKREF_CLK				151
#define GCC_USB3_PRIM_PHY_AUX_CLK				152
#define GCC_USB3_PRIM_PHY_AUX_CLK_SRC				153
#define GCC_USB3_PRIM_PHY_COM_AUX_CLK				154
#define GCC_USB3_PRIM_PHY_PIPE_CLK				155
#define GCC_USB3_SEC_CLKREF_CLK					156
#define GCC_USB3_SEC_PHY_AUX_CLK				157
#define GCC_USB3_SEC_PHY_AUX_CLK_SRC				158
#define GCC_USB3_SEC_PHY_PIPE_CLK				159
#define GCC_USB3_SEC_PHY_COM_AUX_CLK				160
#define GCC_USB_PHY_CFG_AHB2PHY_CLK				161
#define GCC_VIDEO_AHB_CLK					162
#define GCC_VIDEO_AXI_CLK					163
#define GCC_VIDEO_XO_CLK					164
#define GPLL0							165
#define GPLL0_OUT_EVEN						166
#define GPLL0_OUT_MAIN						167
#define GCC_GPU_IREF_CLK					168
#define GCC_SDCC1_AHB_CLK					169
#define GCC_SDCC1_APPS_CLK					170
#define GCC_SDCC1_ICE_CORE_CLK					171
#define GCC_SDCC1_APPS_CLK_SRC					172
#define GCC_SDCC1_ICE_CORE_CLK_SRC				173
#define GCC_APC_VS_CLK						174
#define GCC_GPU_VS_CLK						175
#define GCC_MSS_VS_CLK						176
#define GCC_VDDA_VS_CLK						177
#define GCC_VDDCX_VS_CLK					178
#define GCC_VDDMX_VS_CLK					179
#define GCC_VS_CTRL_AHB_CLK					180
#define GCC_VS_CTRL_CLK						181
#define GCC_VS_CTRL_CLK_SRC					182
#define GCC_VSENSOR_CLK_SRC					183
#define GPLL4							184
#define GCC_CPUSS_DVM_BUS_CLK					185
#define GCC_CPUSS_GNOC_CLK					186

/* GCC Resets */
#define GCC_MMSS_BCR						0
#define GCC_PCIE_0_BCR						1
#define GCC_PCIE_1_BCR						2
#define GCC_PCIE_PHY_BCR					3
#define GCC_PDM_BCR						4
#define GCC_PRNG_BCR						5
#define GCC_QUPV3_WRAPPER_0_BCR					6
#define GCC_QUPV3_WRAPPER_1_BCR					7
#define GCC_QUSB2PHY_PRIM_BCR					8
#define GCC_QUSB2PHY_SEC_BCR					9
#define GCC_SDCC2_BCR						10
#define GCC_SDCC4_BCR						11
#define GCC_TSIF_BCR						12
#define GCC_UFS_CARD_BCR					13
#define GCC_UFS_PHY_BCR						14
#define GCC_USB30_PRIM_BCR					15
#define GCC_USB30_SEC_BCR					16
#define GCC_USB3_PHY_PRIM_BCR					17
#define GCC_USB3PHY_PHY_PRIM_BCR				18
#define GCC_USB3_DP_PHY_PRIM_BCR				19
#define GCC_USB3_PHY_SEC_BCR					20
#define GCC_USB3PHY_PHY_SEC_BCR					21
#define GCC_USB3_DP_PHY_SEC_BCR					22
#define GCC_USB_PHY_CFG_AHB2PHY_BCR				23
#define GCC_PCIE_0_PHY_BCR					24
#define GCC_PCIE_1_PHY_BCR					25

/* GCC GDSCRs */
#define PCIE_0_GDSC						0
#define PCIE_1_GDSC						1
#define UFS_CARD_GDSC						2
#define UFS_PHY_GDSC						3
#define USB30_PRIM_GDSC						4
#define USB30_SEC_GDSC						5
#define HLOS1_VOTE_AGGRE_NOC_MMU_AUDIO_TBU_GDSC			6
#define HLOS1_VOTE_AGGRE_NOC_MMU_PCIE_TBU_GDSC			7
#define HLOS1_VOTE_AGGRE_NOC_MMU_TBU1_GDSC			8
#define HLOS1_VOTE_AGGRE_NOC_MMU_TBU2_GDSC			9
#define HLOS1_VOTE_MMNOC_MMU_TBU_HF0_GDSC			10
#define HLOS1_VOTE_MMNOC_MMU_TBU_HF1_GDSC			11
#define HLOS1_VOTE_MMNOC_MMU_TBU_SF_GDSC			12

#endif
