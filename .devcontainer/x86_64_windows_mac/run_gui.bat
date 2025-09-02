docker build .. -f ../Dockerfile.base -t cogrob
docker run --rm -p 33890:3389 cogrob