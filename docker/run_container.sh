#!/bin/bash

rocker --nvidia --x11 --user --net host --privileged --volume aichallenge:/aichallenge --volume /dev/input/:/dev/input/ --volume /var/run/dbus/:/var/run/dbus/ -- aichallenge-train
