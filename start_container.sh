#!/bin/bash

xhost local:root

docker start $1

docker attach $1
