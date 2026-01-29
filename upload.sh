#!/usr/bin/env bash

DRIVER=$(gum choose "ty" "trent")
ROBOT=$(gum choose "purple" "black")

# RUSTFLAGS="--cfg driver=\"$DRIVER\" --cfg robot=\"$ROBOT\"" cargo-v5 v5 upload --name "${ROBOT}_${DRIVER}_control"
RUSTFLAGS="--cfg driver=\"$DRIVER\" --cfg robot=\"$ROBOT\"" cargo build
