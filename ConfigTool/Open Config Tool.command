#!/bin/bash
# Opens the CAN isolator config tool in Google Chrome (Safari can't talk to USB serial devices).
DIR="$(cd "$(dirname "$0")" && pwd)"
if [ -d "/Applications/Google Chrome.app" ]; then
  open -a "Google Chrome" "$DIR/index.html"
elif [ -d "/Applications/Microsoft Edge.app" ]; then
  open -a "Microsoft Edge" "$DIR/index.html"
else
  osascript -e 'display alert "Chrome or Edge needed" message "The config tool needs Google Chrome or Microsoft Edge (Safari cannot talk to USB serial devices)."'
fi
