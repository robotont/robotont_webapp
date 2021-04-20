#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd $DIR/../backend && npm install
cd $DIR/../frontend && npm install

cd $DIR/../frontend && npm run build
mv $DIR/../frontend/dist/ $DIR/../backend/

cd $DIR/../ttyd && mkdir build && cd build
cmake ..
make && sudo make install
catkin build
