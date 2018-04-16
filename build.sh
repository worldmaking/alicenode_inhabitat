#!/usr/bin/env bash
if [ $# -ge 1 ]
then
    echo path
	ALICEPATH=$1
else
	ALICEPATH="../alicenode"
    echo path to alicenode not specified, assuming: $ALICEPATH
fi

clang++ -shared -O3 -Wall -std=c++11 -stdlib=libc++ -fexceptions -I$ALICEPATH/include -L$ALICEPATH/lib/osx -undefined dynamic_lookup project.cpp -o project.dylib