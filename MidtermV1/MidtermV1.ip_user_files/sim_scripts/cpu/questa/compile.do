vlib questa_lib/work
vlib questa_lib/msim

vlib questa_lib/msim/xpm
vlib questa_lib/msim/microblaze_v11_0_13
vlib questa_lib/msim/xil_defaultlib
vlib questa_lib/msim/lib_cdc_v1_0_3
vlib questa_lib/msim/proc_sys_reset_v5_0_15
vlib questa_lib/msim/lmb_v10_v3_0_14
vlib questa_lib/msim/lmb_bram_if_cntlr_v4_0_24
vlib questa_lib/msim/blk_mem_gen_v8_4_8
vlib questa_lib/msim/axi_lite_ipif_v3_0_4
vlib questa_lib/msim/mdm_v3_2_26
vlib questa_lib/msim/xlconcat_v2_1_6
vlib questa_lib/msim/iomodule_v3_1_10

vmap xpm questa_lib/msim/xpm
vmap microblaze_v11_0_13 questa_lib/msim/microblaze_v11_0_13
vmap xil_defaultlib questa_lib/msim/xil_defaultlib
vmap lib_cdc_v1_0_3 questa_lib/msim/lib_cdc_v1_0_3
vmap proc_sys_reset_v5_0_15 questa_lib/msim/proc_sys_reset_v5_0_15
vmap lmb_v10_v3_0_14 questa_lib/msim/lmb_v10_v3_0_14
vmap lmb_bram_if_cntlr_v4_0_24 questa_lib/msim/lmb_bram_if_cntlr_v4_0_24
vmap blk_mem_gen_v8_4_8 questa_lib/msim/blk_mem_gen_v8_4_8
vmap axi_lite_ipif_v3_0_4 questa_lib/msim/axi_lite_ipif_v3_0_4
vmap mdm_v3_2_26 questa_lib/msim/mdm_v3_2_26
vmap xlconcat_v2_1_6 questa_lib/msim/xlconcat_v2_1_6
vmap iomodule_v3_1_10 questa_lib/msim/iomodule_v3_1_10

vlog -work xpm -64 -incr -mfcu  -sv \
"/tools/Xilinx/Vivado/2024.1/data/ip/xpm/xpm_memory/hdl/xpm_memory.sv" \

vcom -work xpm -64 -93  \
"/tools/Xilinx/Vivado/2024.1/data/ip/xpm/xpm_VCOMP.vhd" \

vcom -work microblaze_v11_0_13 -64 -93  \
"../../../ipstatic/hdl/microblaze_v11_0_vh_rfs.vhd" \

vcom -work xil_defaultlib -64 -93  \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_0/sim/bd_3914_microblaze_I_0.vhd" \

vcom -work lib_cdc_v1_0_3 -64 -93  \
"../../../ipstatic/hdl/lib_cdc_v1_0_rfs.vhd" \

vcom -work proc_sys_reset_v5_0_15 -64 -93  \
"../../../ipstatic/hdl/proc_sys_reset_v5_0_vh_rfs.vhd" \

vcom -work xil_defaultlib -64 -93  \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_1/sim/bd_3914_rst_0_0.vhd" \

vcom -work lmb_v10_v3_0_14 -64 -93  \
"../../../ipstatic/hdl/lmb_v10_v3_0_vh_rfs.vhd" \

vcom -work xil_defaultlib -64 -93  \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_2/sim/bd_3914_ilmb_0.vhd" \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_3/sim/bd_3914_dlmb_0.vhd" \

vcom -work lmb_bram_if_cntlr_v4_0_24 -64 -93  \
"../../../ipstatic/hdl/lmb_bram_if_cntlr_v4_0_vh_rfs.vhd" \

vcom -work xil_defaultlib -64 -93  \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_4/sim/bd_3914_dlmb_cntlr_0.vhd" \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_5/sim/bd_3914_ilmb_cntlr_0.vhd" \

vlog -work blk_mem_gen_v8_4_8 -64 -incr -mfcu  \
"../../../ipstatic/simulation/blk_mem_gen_v8_4.v" \

vlog -work xil_defaultlib -64 -incr -mfcu  \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_6/sim/bd_3914_lmb_bram_I_0.v" \

vcom -work axi_lite_ipif_v3_0_4 -64 -93  \
"../../../ipstatic/hdl/axi_lite_ipif_v3_0_vh_rfs.vhd" \

vcom -work mdm_v3_2_26 -64 -93  \
"../../../ipstatic/hdl/mdm_v3_2_vh_rfs.vhd" \

vcom -work xil_defaultlib -64 -93  \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_7/sim/bd_3914_mdm_0_0.vhd" \

vlog -work xlconcat_v2_1_6 -64 -incr -mfcu  \
"../../../ipstatic/hdl/xlconcat_v2_1_vl_rfs.v" \

vlog -work xil_defaultlib -64 -incr -mfcu  \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_8/sim/bd_3914_xlconcat_0_0.v" \

vcom -work iomodule_v3_1_10 -64 -93  \
"../../../ipstatic/hdl/iomodule_v3_1_vh_rfs.vhd" \

vcom -work xil_defaultlib -64 -93  \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/ip/ip_9/sim/bd_3914_iomodule_0_0.vhd" \

vlog -work xil_defaultlib -64 -incr -mfcu  \
"../../../../MidtermV1.gen/sources_1/ip/cpu/bd_0/sim/bd_3914.v" \
"../../../../MidtermV1.gen/sources_1/ip/cpu/sim/cpu.v" \

vlog -work xil_defaultlib \
"glbl.v"

