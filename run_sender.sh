#!/bin/bash
CONFIG_PATH="./config.json"

if [ ! -f "$CONFIG_PATH" ]; then
	echo "Config file missing"
	exit 1
fi

echo "Sender-node ..."

export LOG_LEVEL=0

./build/sender_node "$CONFIG_PATH"
