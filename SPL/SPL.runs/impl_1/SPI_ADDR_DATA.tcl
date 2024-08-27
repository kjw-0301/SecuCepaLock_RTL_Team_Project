# 
# Report generation script generated by Vivado
# 

proc create_report { reportName command } {
  set status "."
  append status $reportName ".fail"
  if { [file exists $status] } {
    eval file delete [glob $status]
  }
  send_msg_id runtcl-4 info "Executing : $command"
  set retval [eval catch { $command } msg]
  if { $retval != 0 } {
    set fp [open $status w]
    close $fp
    send_msg_id runtcl-5 warning "$msg"
  }
}
proc start_step { step } {
  set stopFile ".stop.rst"
  if {[file isfile .stop.rst]} {
    puts ""
    puts "*** Halting run - EA reset detected ***"
    puts ""
    puts ""
    return -code error
  }
  set beginFile ".$step.begin.rst"
  set platform "$::tcl_platform(platform)"
  set user "$::tcl_platform(user)"
  set pid [pid]
  set host ""
  if { [string equal $platform unix] } {
    if { [info exist ::env(HOSTNAME)] } {
      set host $::env(HOSTNAME)
    }
  } else {
    if { [info exist ::env(COMPUTERNAME)] } {
      set host $::env(COMPUTERNAME)
    }
  }
  set ch [open $beginFile w]
  puts $ch "<?xml version=\"1.0\"?>"
  puts $ch "<ProcessHandle Version=\"1\" Minor=\"0\">"
  puts $ch "    <Process Command=\".planAhead.\" Owner=\"$user\" Host=\"$host\" Pid=\"$pid\">"
  puts $ch "    </Process>"
  puts $ch "</ProcessHandle>"
  close $ch
}

proc end_step { step } {
  set endFile ".$step.end.rst"
  set ch [open $endFile w]
  close $ch
}

proc step_failed { step } {
  set endFile ".$step.error.rst"
  set ch [open $endFile w]
  close $ch
}


start_step place_design
set ACTIVE_STEP place_design
set rc [catch {
  create_msg_db place_design.pb
  set_param chipscope.maxJobs 3
  open_checkpoint SPI_ADDR_DATA_opt.dcp
  set_property webtalk.parent_dir C:/Users/kimhk/Documents/GitHub/SecuCepaLock_RTL_Team_Project/SPL/SPL.cache/wt [current_project]
  if { [llength [get_debug_cores -quiet] ] > 0 }  { 
    implement_debug_core 
  } 
  place_design 
  write_checkpoint -force SPI_ADDR_DATA_placed.dcp
  create_report "impl_1_place_report_io_0" "report_io -file SPI_ADDR_DATA_io_placed.rpt"
  create_report "impl_1_place_report_utilization_0" "report_utilization -file SPI_ADDR_DATA_utilization_placed.rpt -pb SPI_ADDR_DATA_utilization_placed.pb"
  create_report "impl_1_place_report_control_sets_0" "report_control_sets -verbose -file SPI_ADDR_DATA_control_sets_placed.rpt"
  close_msg_db -file place_design.pb
} RESULT]
if {$rc} {
  step_failed place_design
  return -code error $RESULT
} else {
  end_step place_design
  unset ACTIVE_STEP 
}

start_step phys_opt_design
set ACTIVE_STEP phys_opt_design
set rc [catch {
  create_msg_db phys_opt_design.pb
  phys_opt_design 
  write_checkpoint -force SPI_ADDR_DATA_physopt.dcp
  close_msg_db -file phys_opt_design.pb
} RESULT]
if {$rc} {
  step_failed phys_opt_design
  return -code error $RESULT
} else {
  end_step phys_opt_design
  unset ACTIVE_STEP 
}

start_step route_design
set ACTIVE_STEP route_design
set rc [catch {
  create_msg_db route_design.pb
  route_design 
  write_checkpoint -force SPI_ADDR_DATA_routed.dcp
  create_report "impl_1_route_report_drc_0" "report_drc -file SPI_ADDR_DATA_drc_routed.rpt -pb SPI_ADDR_DATA_drc_routed.pb -rpx SPI_ADDR_DATA_drc_routed.rpx"
  create_report "impl_1_route_report_methodology_0" "report_methodology -file SPI_ADDR_DATA_methodology_drc_routed.rpt -pb SPI_ADDR_DATA_methodology_drc_routed.pb -rpx SPI_ADDR_DATA_methodology_drc_routed.rpx"
  create_report "impl_1_route_report_power_0" "report_power -file SPI_ADDR_DATA_power_routed.rpt -pb SPI_ADDR_DATA_power_summary_routed.pb -rpx SPI_ADDR_DATA_power_routed.rpx"
  create_report "impl_1_route_report_route_status_0" "report_route_status -file SPI_ADDR_DATA_route_status.rpt -pb SPI_ADDR_DATA_route_status.pb"
  create_report "impl_1_route_report_timing_summary_0" "report_timing_summary -max_paths 10 -file SPI_ADDR_DATA_timing_summary_routed.rpt -pb SPI_ADDR_DATA_timing_summary_routed.pb -rpx SPI_ADDR_DATA_timing_summary_routed.rpx -warn_on_violation "
  create_report "impl_1_route_report_incremental_reuse_0" "report_incremental_reuse -file SPI_ADDR_DATA_incremental_reuse_routed.rpt"
  create_report "impl_1_route_report_clock_utilization_0" "report_clock_utilization -file SPI_ADDR_DATA_clock_utilization_routed.rpt"
  create_report "impl_1_route_report_bus_skew_0" "report_bus_skew -warn_on_violation -file SPI_ADDR_DATA_bus_skew_routed.rpt -pb SPI_ADDR_DATA_bus_skew_routed.pb -rpx SPI_ADDR_DATA_bus_skew_routed.rpx"
  close_msg_db -file route_design.pb
} RESULT]
if {$rc} {
  write_checkpoint -force SPI_ADDR_DATA_routed_error.dcp
  step_failed route_design
  return -code error $RESULT
} else {
  end_step route_design
  unset ACTIVE_STEP 
}

start_step write_bitstream
set ACTIVE_STEP write_bitstream
set rc [catch {
  create_msg_db write_bitstream.pb
  catch { write_mem_info -force SPI_ADDR_DATA.mmi }
  write_bitstream -force SPI_ADDR_DATA.bit 
  catch {write_debug_probes -quiet -force SPI_ADDR_DATA}
  catch {file copy -force SPI_ADDR_DATA.ltx debug_nets.ltx}
  close_msg_db -file write_bitstream.pb
} RESULT]
if {$rc} {
  step_failed write_bitstream
  return -code error $RESULT
} else {
  end_step write_bitstream
  unset ACTIVE_STEP 
}

