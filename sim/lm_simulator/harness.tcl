# harness.tcl -- run Virtual AGC's LM Simulator (Stephan Hotto,
# virtualagc/Contributed/LM_Simulator) unmodified, with a control port.
#
# Usage:  wish   sim/lm_simulator/harness.tcl LMSIM_DIR CTRL_PORT ?--port=N? ?--cfg=F?
#         tclsh  sim/lm_simulator/harness.tcl LMSIM_DIR CTRL_PORT ?--port=N? ?--cfg=F?
#
# The simulator's own lm_simulator.tcl is sourced as is, from LMSIM_DIR
# (it loads ./modules and ./doc relative to the working directory); the
# remaining arguments are its command line (--port= / --cfg=).
#
# CTRL_PORT (127.0.0.1) takes one Tcl script per line, evaluated at global
# level inside the simulator's interpreter; the reply is one line,
# "OK result" or "ERR message".  sim/lm_simulator/lm_sim.py uses it to press the
# simulator's own buttons (`.dsky.b16 invoke`, `.crewinp.cb6b invoke`), to
# move its RHC the way its scale does (`write_rhc PITCH 20`), and to read
# its model state (IMUX_ANGLE, Omega_Pitch, ...).  Nothing of the
# simulator is replaced: this is the crew's hand on its controls.
#
# Under wish the simulator's windows appear as usual.  Under tclsh (no Tk)
# the Tk commands are inert stand-ins that remember each widget's
# -command / -variable / -value / -onvalue / -offvalue, so `invoke` runs
# the simulator's own button script exactly as a click would; `winfo
# exists` answers 0, which skips the simulator's drawing-only code (FDAI
# canvas, DSKY Lite segments, output lamps); its log window lines go to
# stdout prefixed "SIMLOG ".

set lmdir    [lindex $argv 0]
set ctrlport [lindex $argv 1]
set argv     [lrange $argv 2 end]
set argc     [llength $argv]
set harness_headless [expr {[info commands winfo] eq ""}]

if {$harness_headless} {
  namespace eval stub {
    variable opt
    variable kind
  }
  proc stub::create {cls path args} {
    variable opt
    variable kind
    set kind($path) $cls
    set opt($path) [dict create]
    foreach {k v} $args {dict set opt($path) $k $v}
    return $path
  }
  foreach cls {frame label entry button checkbutton radiobutton scale canvas text scrollbar menu menubutton toplevel} {
    proc $cls {path args} "stub::create $cls \$path {*}\$args"
  }
  proc stub::get {path key default} {
    variable opt
    if {[info exists opt($path)] && [dict exists $opt($path) $key]} {return [dict get $opt($path) $key]}
    return $default
  }
  proc stub::widget {path op args} {
    variable kind
    if {![info exists kind($path)]} {return ""}
    switch -- $op {
      invoke {
        set var [stub::get $path -variable ""]
        if {$kind($path) eq "checkbutton" && $var ne ""} {
          upvar #0 $var v
          set on [stub::get $path -onvalue 1]
          set off [stub::get $path -offvalue 0]
          if {[info exists v] && $v eq $on} {set v $off} else {set v $on}
        }
        if {$kind($path) eq "radiobutton" && $var ne ""} {
          upvar #0 $var v
          set v [stub::get $path -value ""]
        }
        return [uplevel #0 [stub::get $path -command ""]]
      }
      get {
        set var [stub::get $path -variable ""]
        if {$var ne ""} {upvar #0 $var v; if {[info exists v]} {return $v}}
        return [stub::get $path -from 0]
      }
      insert {
        if {$path eq ".text"} {puts -nonewline "SIMLOG [lindex $args 1]"; flush stdout}
        return ""
      }
      cget {return [stub::get $path [lindex $args 0] ""]}
      default {return ""}
    }
  }
  rename unknown stub::tcl_unknown
  proc unknown {cmd args} {
    if {[string index $cmd 0] eq "."} {return [stub::widget $cmd {*}$args]}
    return [uplevel 1 [list stub::tcl_unknown $cmd {*}$args]]
  }
  foreach cmd {wm pack grid place focus bind destroy image font bell raise lower grab option} {
    proc $cmd {args} {return ""}
  }
  proc winfo {op args} {return 0}
  proc tkwait {kind name} {vwait $name}
  proc tk_messageBox {args} {
    puts stderr "lm_sim_harness: message box: [dict get $args -message]"
    return ok
  }
}

proc ctrl_accept {ch addr port} {
  fconfigure $ch -buffering line -translation lf -blocking 0
  fileevent $ch readable [list ctrl_read $ch]
}
proc ctrl_read {ch} {
  if {[gets $ch line] < 0} {
    if {[eof $ch]} {close $ch}
    return
  }
  if {[catch {uplevel #0 $line} r]} {
    puts $ch "ERR [string map {"\n" " "} $r]"
  } else {
    puts $ch "OK [string map {"\n" " "} $r]"
  }
}
socket -server ctrl_accept -myaddr 127.0.0.1 $ctrlport

cd $lmdir
source lm_simulator.tcl
