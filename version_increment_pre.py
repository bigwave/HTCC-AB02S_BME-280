""" Create version header and tracker file if missing """
import datetime
import os

Import("env")

## DO NOT EDIT THIS FILE, edit version file if you want to start from a different version
#
#  version_increment_pre.py - Simple versioning script for Platformio
#
#  Copyright (C) 2020  Davide Perini
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy of
#  this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in
#  all copies or substantial portions of the Software.
#
#  You should have received a copy of the MIT License along with this program.
#  If not, see <https://opensource.org/licenses/MIT/>.
#

VERSION_FILE = 'version'
VERSION_HEADER = 'Version.h'
VERSION_PREFIX = '0.1.'
VERSION_PATCH_NUMBER = 0

if not os.path.exists(".version_no_increment"):
    try:
        with open(VERSION_FILE) as FILE:
            VERSION_PATCH_NUMBER = FILE.readline()
            VERSION_PREFIX = VERSION_PATCH_NUMBER[0:VERSION_PATCH_NUMBER.rindex('.')+1]
            VERSION_PATCH_NUMBER = int(VERSION_PATCH_NUMBER[VERSION_PATCH_NUMBER.rindex('.')+1:]) + 1
    except:
        print('No version file found or incorrect data in it. Starting from 0.1.0')
        VERSION_PATCH_NUMBER = 0
    with open(VERSION_FILE, 'w+') as FILE:
        FILE.write(VERSION_PREFIX + str(VERSION_PATCH_NUMBER))
        print('Build number: {}'.format(VERSION_PREFIX + str(VERSION_PATCH_NUMBER)))

    HEADER_FILE = """
    // AUTO GENERATED FILE, DO NOT EDIT
    #ifndef VERSION
        #define VERSION "{}"
    #endif
    #ifndef BUILD_TIMESTAMP
        #define BUILD_TIMESTAMP "{}"
    #endif
    """.format(VERSION_PREFIX + str(VERSION_PATCH_NUMBER), datetime.datetime.now())

    if os.environ.get('PLATFORMIO_INCLUDE_DIR') is not None:
        VERSION_HEADER = os.environ.get('PLATFORMIO_INCLUDE_DIR') + os.sep + VERSION_HEADER
    elif os.path.exists("include"):
        VERSION_HEADER = "include" + os.sep + VERSION_HEADER
    else:
        PROJECT_DIR = env.subst("$PROJECT_DIR")
        os.mkdir(PROJECT_DIR + os.sep + "include")
        VERSION_HEADER = "include" + os.sep + VERSION_HEADER

    with open(VERSION_HEADER, 'w+') as FILE:
        FILE.write(HEADER_FILE)

    open('.version_no_increment', 'a').close()
else:
    if os.path.exists("version"):
        FILE = open(VERSION_FILE)
        VERSION_NUMBER = FILE.readline()
        print('Build number: {} (waiting for upload before next increment)'.format(str(VERSION_NUMBER)))
    else:
        print('No version file found or incorrect data in it!!')