#! /bin/bash

# Assumes petalinux tools are in the path

if [ -z "$2" ]; then
	echo "USAGE: build_petalinux.sh VIVADO_EXPORT_DIR PROJECT_DIR"
  exit
fi

VIVADO_EXPORT_DIR=`realpath $1`
PROJECT_DIR=`realpath $2`

if [ -e $PROJECT_DIR ]; then
	echo "Project directory $PROJECT_DIR already exists.  Please choose another location."
  exit
fi

# Copy the files to the project directory
cp -r project_skeleton $PROJECT_DIR
cp bootgen_static.bif $PROJECT_DIR
cp regs.init $PROJECT_DIR

pushd $PROJECT_DIR

# Fix the config TMPDIR
sed -i "s,<PETALINUX_PROJECT_ROOT>,$PROJECT_DIR," project-spec/configs/config

# Make an empty directory to Petalinux doesn't choke on the import
mkdir project-spec/hw-description

petalinux-config --get-hw-description=$VIVADO_EXPORT_DIR --oldconfig
petalinux-build
petalinux-package --boot --force --bif bootgen_static.bif

echo "Done!"
echo "BOOT.BIN and images/linux/image.ub can now be copied to the boot partition of your SD card."

popd
