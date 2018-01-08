#!/bin/bash
cd sw/tools/rustlink-forwarder
exec cargo run --release -- $*
