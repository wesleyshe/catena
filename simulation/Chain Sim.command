#!/bin/bash
# Double-click in Finder to open the chain sim control window (and the MuJoCo viewer).
cd "$(dirname "$0")" && exec .venv/bin/python panel.py
