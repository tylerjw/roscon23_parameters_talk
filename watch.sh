#!/usr/bin/env bash

inotifywait -q -m -e close_write parameters_talk.py |
while read -r filename event; do
  ./parameters_talk.py
done
