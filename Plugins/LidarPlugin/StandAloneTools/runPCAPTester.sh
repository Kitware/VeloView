#! /bin/bash
PCAPTESTER="$1"
CALIBFILE="$2"
TOPDIR="$3"

$PCAPTESTER --showCSVHeader
find "$TOPDIR" -iname "*.pcap" -exec sh -c "echo \"processing:{}\" && \"$PCAPTESTER\" --pcapFile \"{}\" --calibrationFile \"$CALIBFILE\" --showPCAPPath --notPretty 2>&1 | tail -n 1 2>&1" \;
