#!/usr/bin


zip logs/$1.zip -q logs/*$1*; 
rm -f logs/*$1.o*;
rm -f logs/*$1-*;
