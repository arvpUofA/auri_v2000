#!/usr/bin/env sh

# check if ros master is running
{
  rostopic list > /dev/null
}||{
  exit 1
}

DIR="$(dirname $0)"
python $DIR/dotcode_generator.py > $1.dot
dot -Tpng $1.dot -o $1.png
rm $1.dot
