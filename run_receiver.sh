#!/bin/bash
CONFIG_PATH="./config.json"

if [ ! -f "$CONFIG_PATH" ]; then
	echo "Config file missing"
	exit 1
fi

echo "receiving_node ..."

export LOG_LEVEL=0

./build/receiver_node "$CONFIG_PATH"
