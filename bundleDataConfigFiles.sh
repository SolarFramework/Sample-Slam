#!/bin/bash

#
# Take a list of archive files and add configuration files
# that have been changed to look for libraries in the modules/
# directory instead of looking in the user remaken cache.
#


function usage()
{
    echo "Usage:"
    echo "    `basename "$0"` [windows|linux] <zip file>+"
}



# Check args
if [ "$#" -eq 0 ]; then
   echo "Error: not argument provided"
   usage
   exit 1
fi

platform=$1

if [ "$platform" != "windows" ] && [ "$platform" != "linux" ]; then
   echo "Unknown platform '$1', only 'windows' and 'linux' are supported"
   usage
   exit 1
fi

echo "Platform: $platform"

shift

# Build zip files list
zipFiles=""
while [ "$1" != "" ]; do
   if [ "$zipFiles" == "" ]; then
      zipFiles="$1"
   else
      zipFiles="$zipFiles $1"
   fi
   shift
done

# Copy configuration file, update path, add to zip and
# delete afterwards to keep git status clean
for configFile in `find data -name "*_conf.xml"`;
do
   cp $configFile $configFile.copy
   sed -i 's/path=".*"/path="modules"/g' $configFile
   for zipFile in $zipFiles;
   do
      if [ "$platform" == "linux" ]; then
         zip $zipFile $configFile
      else
         7z a -tzip $zipFile $configFile
      fi
   done
   mv $configFile.copy $configFile
done
