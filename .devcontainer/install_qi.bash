if [[ $(uname -m) == "x86_64" ]];
then
    echo "Installing qi for x86_64"
    pip install qi
else
    echo "Installing qi for aarch64"
    wget https://github.com/gdesimone97/cogrob_pepper_nodes/releases/download/qi-3.1.5/qi-3.1.5-cp310-cp310-linux_aarch64.whl
    pip install ./qi-3.1.5-cp310-cp310-linux_aarch64.whl
fi