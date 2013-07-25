#!/usr/bin/env python
#---------------------------------------------------------------------------------
# This script expect a path to an application bundle as argument and will
# shorten the libraries names in order to reduce the dylib loader path
# so the Mountain Lion bug could be overcome.
#
# This script has been written to fix the ParaView application bundle.
#---------------------------------------------------------------------------------

import commands
import sys
import os
import os.path
from os import listdir
from os.path import isfile, join
import re
import shutil

#---------------------------------------------------------------------------------

path_bundle_to_fix = sys.argv[1]
library_name_mapping = {}
hash_table = "abcdefghijklmnopqrstuvwxyz0123456789"
hash_table_size = len(hash_table)

#---------------------------------------------------------------------------------

def writeMapping(dir):
    fo = open(dir + "/LibraryMapping.txt", "w")
    for key in library_name_mapping.iterkeys():
        fo.write(key + ":" + library_name_mapping[key] + "\n")
    fo.close()

#---------------------------------------------------------------------------------

def readMapping(dir):
    global library_name_mapping
    try:
        f = open(dir + "/LibraryMapping.txt", "r")
        for line in f:
            oldNew = line.split(":")
            if len(oldNew) == 2:
                library_name_mapping[oldNew[0]] = oldNew[1][0:-1]
        f.close()
    except:
        pass

#---------------------------------------------------------------------------------

def removeLinks(dir):
    commands.getoutput("cd %s && find . -type l -maxdepth 1 -exec rm {} \;" % dir)

#---------------------------------------------------------------------------------

def numberToString(number):
    result = ""
    while(number > hash_table_size):
        result = hash_table[number%hash_table_size] + result
        number = number / hash_table_size
    result = hash_table[number%hash_table_size] + result
    return result

#---------------------------------------------------------------------------------

def getExtension(fileName):
    return fileName[fileName.rfind("."):]

#---------------------------------------------------------------------------------

def getNewFileName(fileName):
    global library_name_mapping
    if fileName.find(".dylib") != -1:
        id = len(library_name_mapping)
        new_name = numberToString(id)# + getExtension(fileName)
        library_name_mapping[fileName] = new_name
    else:
        # No renaming
        library_name_mapping[fileName] = fileName

#---------------------------------------------------------------------------------

def renameLibraries(dir):
    for f in listdir(dir):
        sys.stdout.write('.')
        getNewFileName(f)
    print

#---------------------------------------------------------------------------------

def fixInternalLibraryPath(dir):
    print "Fixing libraries in", dir
    for f in listdir(dir):
        fullPath = dir + "/" + f
        libs = commands.getoutput("otool -L %s | grep executable_path | awk '{print $1}'" % fullPath).split()
        changeName = " -id @executable_path/" + f
        for lib in libs:
            libname = lib[lib.rfind('/')+1:]
            if library_name_mapping.has_key(libname):
                sys.stdout.write('.')
                changeName += " -change " + lib + " @executable_path/" + library_name_mapping[libname]

        commands.getoutput('chmod u+w "%s"' % fullPath)
        commands.getoutput('install_name_tool %s "%s"' % (changeName, fullPath))
        commands.getoutput('chmod a-w "%s"' % fullPath)
        if not library_name_mapping.has_key(f):
            continue
        newFullPath = dir + "/" + library_name_mapping[f]
        os.rename(fullPath, newFullPath)
    print "\n"

#---------------------------------------------------------------------------------

def updateFrameworkPath(dir, dest):
    global library_name_mapping
    print "Moving frameworks from", dir, "to", dest
    frameworks = commands.getoutput('find %s -type f | xargs file --separator ":--:" | grep -i ":--:.*Mach-O" | sed "s/:.*//" | sort | uniq' % dir).split()
    for f in frameworks:
        fname = f[f.rfind('/')+1:]
        library_name_mapping[fname] = fname
        fullPath = f
        newFullPath = dest + "/" + fname
        os.rename(fullPath, newFullPath)
        print " -", fname


#---------------------------------------------------------------------------------

def fixExecutables(dir, copyLibs=True):
    for f in listdir(dir):
        print "Fixing executable", f
        fullPath = dir + "/" + f
        libs = commands.getoutput("otool -L %s | grep executable_path | awk '{print $1}'" % fullPath).split()
        changeName = ""
        for lib in libs:
            libname = lib[lib.rfind('/')+1:]
            if library_name_mapping.has_key(libname):
                sys.stdout.write('.')
                changeName += " -change " + lib + " @executable_path/" + library_name_mapping[libname]

        commands.getoutput('chmod u+w "%s"' % fullPath)
        commands.getoutput('install_name_tool %s "%s"' % (changeName, fullPath))
        commands.getoutput('chmod a-w "%s"' % fullPath)
        print
    if copyLibs:
        commands.getoutput("cd %s && ln -s ../Libraries/* ." % dir)

#---------------------------------------------------------------------------------

print "Patch bundle", path_bundle_to_fix

lib_dir = path_bundle_to_fix + "/Contents/Libraries"
frameworks_dir = path_bundle_to_fix + "/Contents/Frameworks"
bin_dir = path_bundle_to_fix + "/Contents/bin"
pv_dir = path_bundle_to_fix + "/Contents/MacOS"
plugins_dir = path_bundle_to_fix + "/Contents/Plugins"

# Clean up previous bundle fix
print "Remove links."
removeLinks(lib_dir)
removeLinks(bin_dir)
removeLinks(pv_dir)

# Move frameworks to libraries
#updateFrameworkPath(frameworks_dir, lib_dir)

# Rename libraries to shorten their path names
readMapping(lib_dir)
if len(library_name_mapping) == 0:
    sys.stdout.write("Rename libraries")
    renameLibraries(lib_dir)
    writeMapping(lib_dir)
else:
    print "Read previous mapping definition of", len(library_name_mapping), "libraries."

# Update libraries to point to the new lib names
fixInternalLibraryPath(lib_dir)

# Update framework refs
frameworks = commands.getoutput('find %s -type f | xargs file --separator ":--:" | grep -i ":--:.*Mach-O" | sed "s/:.*//" | sort | uniq' % frameworks_dir).split()
for f in frameworks:
    fixExecutables(f[0:f.rfind("/")], False)

# Update plugins refs
fixExecutables(plugins_dir, False)

# Update executable to point to the new lib names
fixExecutables(bin_dir)
fixExecutables(pv_dir)
