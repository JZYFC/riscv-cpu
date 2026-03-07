# Recreate the Vivado project metadata from the checked-in HDL sources.
#
# Usage:
#   source /home/jzyfc/Xilinx/2025.1/Vivado/settings64.sh
#   vivado -mode batch -source scripts/create_project.tcl

set script_dir [file dirname [file normalize [info script]]]
set repo_root [file normalize [file join $script_dir ".."]]

set project_name cpu
set project_dir  $repo_root
set part_name    xc7z020clg400-2
set source_tree  [file join $repo_root cpu.srcs sources_1 new]
set backup_tree  [file join /tmp "${project_name}_sources_backup_[pid]_[clock milliseconds]"]

set design_files [list \
    [file join $repo_root cpu.srcs sources_1 new riscv_define.v] \
    [file join $repo_root cpu.srcs sources_1 new BranchPredictor.v] \
    [file join $repo_root cpu.srcs sources_1 new FU.v] \
    [file join $repo_root cpu.srcs sources_1 new IF.v] \
    [file join $repo_root cpu.srcs sources_1 new InstructionBuffer.v] \
    [file join $repo_root cpu.srcs sources_1 new IssueBuffer.v] \
    [file join $repo_root cpu.srcs sources_1 new PhysicalRegFileShared.v] \
    [file join $repo_root cpu.srcs sources_1 new PostDecode.v] \
    [file join $repo_root cpu.srcs sources_1 new PreDecode.v] \
    [file join $repo_root cpu.srcs sources_1 new RegRename.v] \
    [file join $repo_root cpu.srcs sources_1 new ibex_fpu.v] \
    [file join $repo_root cpu.srcs sources_1 new Top.v] \
    [file join $repo_root cpu.srcs sources_1 new Cache.v] \
    [file join $repo_root cpu.srcs sources_1 new ICache.v] \
    [file join $repo_root cpu.srcs sources_1 new LSU.v] \
    [file join $repo_root cpu.srcs sources_1 new MemArbiter.v] \
    [file join $repo_root cpu.srcs sources_1 new MainMemory.v] \
    [file join $repo_root cpu.srcs sources_1 new TLB.v] \
]

set sim_1_files [list \
    [file join $repo_root cpu.srcs sources_1 new Top_tb.v] \
]

set sim_bin_files [list \
    [file join $repo_root cpu.srcs sources_1 new Top_tb_bin.v] \
]

proc require_files {paths} {
    foreach path $paths {
        if {![file exists $path]} {
            error "Missing required source file: $path"
        }
    }
}

proc snapshot_tree {source_dir backup_dir} {
    if {![file isdirectory $source_dir]} {
        error "Missing required source directory: $source_dir"
    }

    file mkdir $backup_dir
    foreach entry [glob -nocomplain -directory $source_dir *] {
        file copy -force -- $entry [file join $backup_dir [file tail $entry]]
    }
}

proc restore_tree {backup_dir target_dir} {
    if {![file isdirectory $backup_dir]} {
        error "Backup directory does not exist: $backup_dir"
    }

    file mkdir $target_dir
    foreach entry [glob -nocomplain -directory $backup_dir *] {
        file copy -force -- $entry [file join $target_dir [file tail $entry]]
    }
}

proc configure_simset {simset_name top_module runtime args} {
    set simset [get_filesets $simset_name]
    set_property top $top_module $simset
    set_property top_lib xil_defaultlib $simset
    set_property source_set sources_1 $simset
    set_property -name {xsim.simulate.runtime} -value $runtime -objects $simset

    if {[llength $args] > 0} {
        set_property -name {xsim.simulate.xsim.more_options} \
            -value [lindex $args 0] \
            -objects $simset
    }
}

require_files $design_files
require_files $sim_1_files
require_files $sim_bin_files

close_project -quiet
snapshot_tree $source_tree $backup_tree

create_project -force $project_name $project_dir -part $part_name
restore_tree $backup_tree $source_tree

set project [current_project]
set_property source_mgmt_mode None $project
set_property target_language Verilog $project
set_property simulator_language Verilog $project
set_property default_lib xil_defaultlib $project
set_property enable_core_container false $project

add_files -fileset sources_1 $design_files
set_property top Top [get_filesets sources_1]

add_files -fileset sim_1 $sim_1_files
configure_simset sim_1 Top_tb 100us

create_fileset -simset sim_bin
add_files -fileset sim_bin $sim_bin_files
configure_simset \
    sim_bin \
    Top_tb_bin \
    2ms \
    "-testplusarg BIN=[file join $repo_root sw test.bin] -testplusarg TIMEOUT=200000"
current_fileset -simset [get_filesets sim_bin]

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1
update_compile_order -fileset sim_bin

close_project

puts "INFO: Recreated Vivado project at [file join $project_dir ${project_name}.xpr]"
puts "INFO: Source snapshot stored at $backup_tree"
