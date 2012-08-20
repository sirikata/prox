#!/usr/bin/env python

'''
Builds all configurations of libprox + proxsim. This is just a sanity
check to make sure they call continue to function.
'''

import collections
import subprocess
import os.path

BuildOption = collections.namedtuple('BuildOption', ['name', 'values'])

# All the options. Note that we leave the common configuration as the
# last value so we end up with that version at the end and can
# continue working.
build_options = [
    BuildOption(name='CMAKE_BUILD_TYPE', values=['Debug', 'Release']),
    BuildOption(name='LIBPROX_BUILD_PROXSIM', values=['FALSE', 'TRUE']),
    BuildOption(name='LIBPROX_RTREE_DATA', values=['BOUNDS', 'MAXSIZE', 'SIMILARMAXSIZE']),
    BuildOption(name='LIBPROX_DEBUG_VALIDATE', values=['ON', 'OFF']),
    BuildOption(name='LIBPROX_RTREE_LIFT_CUTS', values=['OFF', 'ON'])
]

def call_success(returncode):
    return (returncode == 0)

def build_config(flags):
    print "LOG: Building configuration " + ' '.join(flags)
    wd = os.path.join(os.path.dirname(__file__), '..', 'build')
    success = \
        call_success(subprocess.call(['rm', '-f', 'CMakeCache.txt'], cwd=wd)) and \
        call_success(subprocess.call(['cmake', '.'] + flags, cwd=wd)) and \
        call_success(subprocess.call(['make', '-j4'], cwd=wd))
    if not success:
        raise Exception("Build failed: " + ' '.join(flags))
    if not call_success(subprocess.call(['./libprox_tests'])):
        raise Exception("Tests failed: " + ' '.join(flags))

def select_option(flags, opt_idx):
    # If we are done selecting options
    if opt_idx >= len(build_options):
        build_config(flags)
        return

    bo = build_options[opt_idx]
    for bo_val in bo.values:
        opt_val = '-D' + bo.name + '=' + bo_val
        flags.append(opt_val)
        select_option(flags, opt_idx+1)
        flags.pop()


# start the process by telling it to start with no settings and start with the first
select_option([], 0)
