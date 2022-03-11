#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd $DIR/../backend && npm install
cd $DIR/../frontend && npm install
cd $DIR/../frontend && npm run build

rm -fr $DIR/../backend/dist
mv $DIR/../frontend/dist $DIR/../backend/
