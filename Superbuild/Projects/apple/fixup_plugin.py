#!/usr/bin/env python
# This is simple script that can be used to fixup a library (dylib or so)
# Usage:
#   ./fixup_plugin.py <full path to lib to fix or dir with libraries to fix> "key=val" ["key=val" ...]
# This is simply replaces any referece to 'key' with 'val' in the libraries referred to
# by the plugin lib to fix. 'key' can be a Python regular expression.
# The order of the key=value pairs is significant. The expressions are
# tested in the order specified.

import commands
import sys
import os.path
import re
import shutil
from fixup_bundle import *

plugin_dir = sys.argv[1]
prefix_map = {}
prefix_keys_in_priority_order = []
for arg in sys.argv[2:]:
  key, value = arg.split("=")
  prefix_map[key] = value
  prefix_keys_in_priority_order.append(key)

libs_to_fix = commands.getoutput('find %s -type f | xargs file --separator ":--:" | grep -i ":--:.*Mach-O" | sed "s/:.*//" | sort | uniq ' % plugin_dir).split()
print "Found", len(libs_to_fix), "libraries to fix."
print "\n".join(libs_to_fix)
print ""

for plugin_lib in libs_to_fix:
  commands.getoutput('chmod u+w "%s"' % plugin_lib)
  # find all libraries the plugin depends on.
  external_libraries = commands.getoutput(
    'find %s | xargs file | grep "Mach-O" | sed "s/:.*//" | xargs otool -l | grep " name" | sort | uniq | sed "s/name\ //" | grep -v "@" | sed "s/ (offset.*)//"' % plugin_lib).split()
  for elib in external_libraries:
    if not isexcluded(elib):
      # for each lib that the plugin depends on, we check if the prefix for the name
      # matches one of the arguments passed to the script. If so, we can fix
      # the reference.
      for key in prefix_keys_in_priority_order:
        m = re.match(r'^%s(.*)$' % key, elib)
        if m:
          oldid = elib
          newid = "%s%s" % (prefix_map[key], m.group(1))
          #print "%s ==> %s" % (elib, newid)
          commands.getoutput("install_name_tool -change %s %s %s" % (oldid, newid, plugin_lib))
          break
  commands.getoutput('chmod a-w "%s"' % plugin_lib)
