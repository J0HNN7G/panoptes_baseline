docker run --rm -it --runtime nvidia --network host --hostname $(cat /etc/hostname) -v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw' -v .:/opt/jonathan2 jonathan:latest /bin/bash
#docker run --rm --runtime nvidia --network host --hostname $(cat /etc/hostname) -v .:/opt/benchmark benchmark:latest /bin/bash -c 'python3 benchmark.py'
