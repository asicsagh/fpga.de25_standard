#!/bin/bash -e

export ROOTDIR=$(pwd)

export QUARTUS_ROOTDIR=/eda/Intel/altera_pro/25.3/quartus
export PATH=${ROOTDIR}/tools:${QUARTUS_ROOTDIR}/bin:${PATH}

export LM_LICENSE_FILE="/eda/Intel/licenses/evaluation_lic.dat"
