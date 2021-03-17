#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd $DIR/../ttyd/build && ./ttyd -p 5000 bash
