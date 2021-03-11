#!/bin/bash

# This script uncomments (removes "//") the specified test in
# conf/mods.conf and conf/system_start.inc.

TEST=`echo $1 | sed 's/-/_/g'`

case $TEST in
	pjsip_test)
		# fallthrough
		;&
	pjlib_test)
		sed -i -E "s/\/\/(.*)$TEST/\1$TEST/g" conf/mods.conf
		sed -i -E "s/\/\/(.*)\"$TEST\"(.*)/\1\"$TEST\"\2/g" conf/system_start.inc
		;;
	*)
		echo "Availbale tests:"
		echo "    pjlib-test pjsip-test"
		exit 1
		;;
esac
