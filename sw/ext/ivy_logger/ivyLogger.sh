#!/bin/bash
cd sw/ext/ivy_logger
make
exec ./ivyLogger $*
