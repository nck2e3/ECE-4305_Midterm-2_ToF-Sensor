transcript off
onbreak {quit -force}
onerror {quit -force}
transcript on

vlib work
vlib activehdl/xpm
vlib activehdl/microblaze_v11_0_13
vlib activehdl/xil_defaultlib
vlib activehdl/lib_cdc_v1_0_3
vlib activehdl/proc_sys_reset_v5_0_15
vlib activehdl/lmb_v10_v3_0_14
vlib activehdl/lmb_bram_if_cntlr_v4_0_24
vlib activehdl/blk_mem_gen_v8_4_8
vlib activehdl/axi_lite_ipif_v3_0_4
vlib activehdl/mdm_v3_2_26
vlib activehdl/xlconcat_v2_1_6
vlib activehdl/iomodule_v3_1_10

vmap xpm activehdl/xpm
vmap microblaze_v11_0_13 activehdl/microblaze_v11_0_13
vmap xil_defaultlib activehdl/xil_defaultlib
vmap lib_cdc_v1_0_3 activehdl/lib_cdc_v1_0_3
vmap proc_sys_reset_v5_0_15 activehdl/proc_sys_reset_v5_0_15
vmap lmb_v10_v3_0_14 activehdl/lmb_v10_v3_0_14
vmap lmb_bram_if_cntlr_v4_0_24 activehdl/lmb_bram_if_cntlr_v4_0_24
vmap blk_mem_gen_v8_4_8 activehdl/blk_mem_gen_v8_4_8
vmap axi_lite_ipif_v3_0_4 activehdl/axi_lite_ipif_v3_0_4
vmap mdm_v3_2_26 activehdl/mdm_v3_2_26
vmap xlconcat_v2_1_6 activehdl/xlconcat_v2_1_6
vmap iomodule_v3_1_10 activehdl/iomodule_v3_1_10

vlog -work xpm  -sv2k12 -l xpm -l microblaze_v11_0_13 -l xil_defaultlib -l lib_cdc_v1_0_3 -l proc_sys_reset_v5_0_15 -l lmb_v10_v3_0_14 -l lmb_bram_if_cntlr_v4_0_24 -l blk_mem_gen_v8_4_8 -l axi_lite_ipif_v3_0_4 -l mdm_v3_2_26 -l xlconcat_v2_1_6 -l iomodule_v3_1_10 \
"/tools/Xilinx/Vivado/2024.1/data/ip/xpm/xpm_memory/hdl/xpm_memory.sv" \

vcom -work xpm -  \
"/tools/Xilinx/Vivado/2024.1/data/ip/xpm/xpm_VCOMP.vhd" \

vcom -work microblaze_v11_0_13 -  \
"../../../ipstatic/hdl/microblaze_v11_0_vh_rfs.vhd" \

vcom -work xil_defaultlib -  \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_0/sim/bd_3914_microblaze_I_0.vhd" \

vcom -work lib_cdc_v1_0_3 -  \
"../../../ipstatic/hdl/lib_cdc_v1_0_rfs.vhd" \

vcom -work proc_sys_reset_v5_0_15 -  \
"../../../ipstatic/hdl/proc_sys_reset_v5_0_vh_rfs.vhd" \

vcom -work xil_defaultlib -  \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_1/sim/bd_3914_rst_0_0.vhd" \

vcom -work lmb_v10_v3_0_14 -  \
"../../../ipstatic/hdl/lmb_v10_v3_0_vh_rfs.vhd" \

vcom -work xil_defaultlib -  \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_2/sim/bd_3914_ilmb_0.vhd" \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_3/sim/bd_3914_dlmb_0.vhd" \

vcom -work lmb_bram_if_cntlr_v4_0_24 -  \
"../../../ipstatic/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd" \

vcom -work xil_defaultlib -  \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_4/sim/bd_3914_dlmb_cntlr_0.vhd" \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_5/sim/bd_3914_ilmb_cntlr_0.vhd" \

vlog -work blk_mem_gen_v8_4_8  -v2k5 -l xpm -l microblaze_v11_0_13 -l xil_defaultlib -l lib_cdc_v1_0_3 -l proc_sys_reset_v5_0_15 -l lmb_v10_v3_0_14 -l lmb_bram_if_cntlr_v4_0_24 -l blk_mem_gen_v8_4_8 -l axi_lite_ipif_v3_0_4 -l mdm_v3_2_26 -l xlconcat_v2_1_6 -l iomodule_v3_1_10 \
"../../../ipstatic/simulation/blk_mem_gen_v8_4.v" \

vlog -work xil_defaultlib  -v2k5 -l xpm -l microblaze_v11_0_13 -l xil_defaultlib -l lib_cdc_v1_0_3 -l proc_sys_reset_v5_0_15 -l lmb_v10_v3_0_14 -l lmb_bram_if_cntlr_v4_0_24 -l blk_mem_gen_v8_4_8 -l axi_lite_ipif_v3_0_4 -l mdm_v3_2_26 -l xlconcat_v2_1_6 -l iomodule_v3_1_10 \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_6/sim/bd_3914_lmb_bram_I_0.v" \

vcom -work axi_lite_ipif_v3_0_4 -  \
"../../../ipstatic/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd" \

vcom -work mdm_v3_2_26 -  \
"../../../ipstatic/hdl/mdm_v3_2_vh_rfs.vhd" \

vcom -work xil_defaultlib -  \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_7/sim/bd_3914_mdm_0_0.vhd" \

vlog -work xlconcat_v2_1_6  -v2k5 -l xpm -l microblaze_v11_0_13 -l xil_defaultlib -l lib_cdc_v1_0_3 -l proc_sys_reset_v5_0_15 -l lmb_v10_v3_0_14 -l lmb_bram_if_cntlr_v4_0_24 -l blk_mem_gen_v8_4_8 -l axi_lite_ipif_v3_0_4 -l mdm_v3_2_26 -l xlconcat_v2_1_6 -l iomodule_v3_1_10 \
"../../../ipstatic/hdl/xlconcat_v2_1_vl_rfs.v" \

vlog -work xil_defaultlib  -v2k5 -l xpm -l microblaze_v11_0_13 -l xil_defaultlib -l lib_cdc_v1_0_3 -l proc_sys_reset_v5_0_15 -l lmb_v10_v3_0_14 -l lmb_bram_if_cntlr_v4_0_24 -l blk_mem_gen_v8_4_8 -l axi_lite_ipif_v3_0_4 -l mdm_v3_2_26 -l xlconcat_v2_1_6 -l iomodule_v3_1_10 \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_8/sim/bd_3914_xlconcat_0_0.v" \

vcom -work iomodule_v3_1_10 -  \
"../../../ipstatic/hdl/iomodule_v3_1_vh_rfs.vhd" \

vcom -work xil_defaultlib -  \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_9/sim/bd_3914_iomodule_0_0.vhd" \

vlog -work xil_defaultlib  -v2k5 -l xpm -l microblaze_v11_0_13 -l xil_defaultlib -l lib_cdc_v1_0_3 -l proc_sys_reset_v5_0_15 -l lmb_v10_v3_0_14 -l lmb_bram_if_cntlr_v4_0_24 -l blk_mem_gen_v8_4_8 -l axi_lite_ipif_v3_0_4 -l mdm_v3_2_26 -l xlconcat_v2_1_6 -l iomodule_v3_1_10 \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/sim/bd_3914.v" \
"../../../../MidtermV1.gen/sources_1/ip/cpu/sim/cpu.v" \

vlog -work xil_defaultlib \
"glbl.v"

