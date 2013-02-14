#!/usr/bin/python

import sys
import commands
import re

print "Fixing bundle for Mountain Lion for Application: ", sys.argv[1]

App = sys.argv[1]
references_to_fix = commands.getoutput("""find %s | xargs file | grep "Mach-O" | sed "s/:.*//"  | xargs otool -l | grep " name" | sort | uniq | sed "s/name\ //" | grep "@executable_path" | awk '{print $1}'"""  % App)
#print "references_to_fix: ", references_to_fix

install_name_tool_command = ""
for ref in references_to_fix.split():
  match = re.match("^@executable_path/../Libraries/(.*)$", ref)
  if match:
    print " -> %s" % ref
    install_name_tool_command += " -change \"%s\" \"@executable_path/%s\"" % (ref, match.group(1))

if len(install_name_tool_command) == 0:
  print "No library path need to be adjust"
else:
  binaries_to_fix = commands.getoutput('find %s -type f | xargs file --separator ":--:" | grep -i ":--:.*Mach-O"| grep -v Framework | sed "s/:.*//" | sort | uniq ' % App).split()
  for dep in binaries_to_fix:
    commands.getoutput('chmod u+w "%s"' % dep)
    #print "Fixing '%s'" % dep
    commands.getoutput('install_name_tool %s "%s"' % (install_name_tool_command, dep))
    commands.getoutput('install_name_tool -id "@executable_path/%s" "%s"' % ( dep.split("/")[-1] , dep))
    commands.getoutput('chmod a-w "%s"' % dep)

  commands.getoutput("cd %s/Contents/MacOS && ln -s ../Libraries/*.dylib ." % App)
  commands.getoutput("cd %s/Contents/bin && ln -s ../Libraries/*.dylib ." % App)
