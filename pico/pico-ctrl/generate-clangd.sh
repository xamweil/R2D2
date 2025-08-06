#!/bin/bash

PLATFORMIO_DIR="$HOME/.platformio/packages"
TOOLCHAIN_DIR="$PLATFORMIO_DIR/toolchain-rp2040-earlephilhower"

# TODO: make sure it picks the latest version or provide a way to speciy the version
GCC_VERSION=$(ls "$TOOLCHAIN_DIR/arm-none-eabi/include/c++/" | head -1)

cat >.clangd <<EOL
CompileFlags:
  Add: 
    - --sysroot=$TOOLCHAIN_DIR/arm-none-eabi
    - -nostdinc++
    - -isystem$TOOLCHAIN_DIR/arm-none-eabi/include/c++/$GCC_VERSION
    - -isystem$TOOLCHAIN_DIR/arm-none-eabi/include/c++/$GCC_VERSION/arm-none-eabi
    - -isystem$TOOLCHAIN_DIR/arm-none-eabi/include
EOL

echo "Generated .clangd with GCC version $GCC_VERSION"
