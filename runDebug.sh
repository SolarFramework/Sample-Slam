#!/bin/bash

export REMAKEN_PKG_ROOT=~/.remaken
echo "REMAKEN_PKG_ROOT=$REMAKEN_PKG_ROOT"

# Update configuration files by replacing win-cl-1.1 by linux in module paths
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/$1_conf.xml

for dataConfFile in "../../data/*_conf.xml" ; do
   sed -i 's/win-cl-14.1/linux-gcc/' $dataConfFile
done

ld_library_path="./"
for modulePath in $(grep -o "\$REMAKEN_PKG_ROOT.*lib" $1_conf.xml)
do
   modulePath=${modulePath/"\$REMAKEN_PKG_ROOT"/${REMAKEN_PKG_ROOT}}   
   if ! [[ $ld_library_path =~ "$modulePath/x86_64/shared/debug" ]]
   then
      ld_library_path=$ld_library_path:$modulePath/x86_64/shared/debug
   fi 
done

echo "LD_LIBRARY_PATH=$ld_library_path $1 $2"
LD_LIBRARY_PATH=$ld_library_path $1 $2


