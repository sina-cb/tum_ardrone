#!/bin/sh
#tmux "gdb --args $*"
gnome-terminal -x sh -c "gdb --args $*; bash"
