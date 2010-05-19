#!/bin/sh

if [ -z "$1" ]; then
	../Debug/SimpleTiming.exe | sed "/reg/ d"
else
	../Debug/SimpleTiming.exe | sed "/reg/ d" > $1
	unix2dos $1
fi
