docker build .. -f ../Dockerfile -t cogrob
docker run --rm -p 33890:3389 -v ${PWD}\..\..:/workspace cogrob