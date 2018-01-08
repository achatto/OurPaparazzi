#!/bin/bash
cd sw/ext/rustlink-forwarder
exec cargo run --release -- $*
