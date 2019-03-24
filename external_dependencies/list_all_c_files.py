#!/usr/bin/env python
import os

if __name__ == "__main__":

    out = []

    for dirname, dirnames, filenames in os.walk( os.getcwd() +'/sapi'):
        out.extend( [ os.path.relpath( os.path.join(dirname, x) ) for x in filenames if x[-2:] == '.c'] ) # list all files but only if it's a .c file

    out.sort()

    for x in out:
        print('sapi_srcs += [\'' + x + '\']')
