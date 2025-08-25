#!/bin/bash

cd "$(dirname "$0")"

astyle --indent=spaces=4 ./Core/Src/*.c
astyle --indent=spaces=4 ./Core/Inc/*.h
rm ./Core/Src/*.orig
rm ./Core/Inc/*.orig
