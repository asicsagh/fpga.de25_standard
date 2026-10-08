#!/bin/bash -e

git clean -fXd .

quartus_sh --flow compile de25_standard
