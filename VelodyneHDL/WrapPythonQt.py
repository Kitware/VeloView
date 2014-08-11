#!/usr/bin/env python
# Copyright 2013 Velodyne Acoustics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import re
import sys

if len(sys.argv) != 4:
    print "Usage: %s <method_infile> <decorator_outfile> <export_symbol>" % sys.argv[0]
    sys.exit(1)

inFileName = sys.argv[1]
outFileName = sys.argv[2]
exportSymbol = sys.argv[3]

if not outFileName.endswith(".h"):
    print "Error: out file must end with .h"
    sys.exit(1)

inFile = open(inFileName, 'r')
lines = inFile.read().splitlines()
inFile.close()

classNamePrefixes = ['vtk', 'pq', 'vv']
classNameRegexes = [re.compile('%s[a-zA-Z0-9]*' % prefix) for prefix in classNamePrefixes]

generatedCode = str()
rePattern = re.compile('(?:(.+)\s+)?(\S+)::(\S+)\((.*)\).*')

includeClasses = set()
includeLines = list()

for line in lines:


    if line.startswith("//"):
        generatedCode += "  " + line + "\n"
        continue
    elif line.startswith("#include"):
        includeLines.append(line)
        continue
    elif not line.strip():
        generatedCode += "\n"
        continue

    matchList = rePattern.findall(line)
    if not matchList or len(matchList[0]) != 4:
        print "Failed to match: '%s'" % line
        sys.exit(1)

    matchList = list(matchList[0])
    return_type = matchList[0].strip()
    class_name = matchList[1]
    method_name = matchList[2]
    args = matchList[3]
    arg_list = args.split(",") if args.strip() else []

    is_static = return_type.startswith("static ")
    is_destructor = '~' == method_name[0]
    is_constructor = not is_destructor and return_type == ''
    if is_static:
        return_type = return_type[7:]
        decorator_method_name = "static_%s_%s" % (class_name, method_name)
    elif is_destructor:
        return_type = 'void'
        decorator_method_name = "delete_%s" % class_name
    elif is_constructor:
        return_type = '%s*' % class_name
        decorator_method_name = "new_%s" % class_name
    else:
        decorator_method_name = method_name

    includeClasses.add(class_name)

    for regex in classNameRegexes:
        classname_matches = regex.findall(return_type)
        for classname in classname_matches:
            includeClasses.add(classname)

    wrap_args = []
    if not is_static and not is_constructor:
        wrap_args.append("%s* inst" % class_name)

    wrap_args_call = []
    for i, arg_type in enumerate(arg_list):
        arg_name = "arg%d" % i
        wrap_args.append("%s %s" % (arg_type.strip(), arg_name))
        wrap_args_call.append(arg_name)

    callStatement = "%s(%s)" % (method_name, ", ".join(wrap_args_call))
    if is_static:
        callStatement = "%s::%s" % (class_name, callStatement)
    elif is_destructor:
        callStatement = 'delete inst'
    elif is_constructor:
        callStatement = 'new %s' % callStatement
    else:
        callStatement = "inst->%s" % callStatement

    if return_type == 'void':
        returnStatement = "%s;" % callStatement
    else:
        returnStatement = "return %s;" % callStatement


    outStr = \
"""
  %s %s(%s)
    {
    %s
    }
"""
    outStr = outStr % (return_type,
                       decorator_method_name,
                       ", ".join(wrap_args),
                       returnStatement)

    generatedCode += outStr


sortedClasses = list(includeClasses)
sortedClasses.sort()
classIncludes = "\n".join(['#include "%s.h"' % className for className in sortedClasses] + includeLines)

classRegisters = "\n".join(['    this->registerClassForPythonQt(&%s::staticMetaObject);' % className
                            for className in sortedClasses if className.startswith(('pq', 'vv'))])

decoratorClassName = os.path.basename(outFileName).replace(".h", "")

outFile = open(outFileName, 'w')
outFile.write("""
#ifndef __%s_h
#define __%s_h

#include <QObject>
#include "PythonQt.h"

%s

class %s %s : public QObject
{
  Q_OBJECT

public:

  %s(QObject* parent=0) : QObject(parent)
    {
%s
    }

  inline void registerClassForPythonQt(const QMetaObject* metaobject)
    {
    PythonQt::self()->registerClass(metaobject, "paraview");
    }

public slots:

%s

};

#endif
""" % (

  decoratorClassName,
  decoratorClassName,
  classIncludes,
  exportSymbol,
  decoratorClassName,
  decoratorClassName,
  classRegisters,
  generatedCode))

outFile.close()
