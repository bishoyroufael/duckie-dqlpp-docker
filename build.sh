#!/bin/bash

docker -H bishoybot.local  build --network="host" -t dqlpp-ros .
