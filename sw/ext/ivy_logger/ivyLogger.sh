#!/bin/bash
cd sw/tools/ivy_logger
make
exec ./ivyLogger $*
