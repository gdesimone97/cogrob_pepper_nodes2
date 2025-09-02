#!/bin/bash

ARCH=$(uname -m)

if [[ "$ARCH" == "x86_64" ]]; then
    echo "Installing qi for x86_64"
    pip install qi
elif [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
    echo "Installing qi for aarch64"
    QI_WHL_URL="https://github.com/gdesimone97/cogrob_pepper_nodes/releases/download/qi-3.1.5/qi-3.1.5-cp310-cp310-linux_aarch64.whl"
    wget "$QI_WHL_URL"
    pip install ./qi-3.1.5-cp310-cp310-linux_aarch64.whl
    rm qi-3.1.5-cp310-cp310-linux_aarch64.whl
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi
