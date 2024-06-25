#!/bin/bash
if which udevadm >/dev/null; then
    echo "Restarting UDEV Service to enable hotpluging"
    set +e # Disable exit on error
    service udev restart
    set -e # Re-enable exit on error
    echo "UDEV Service restarted"
fi
value="yarn start:docker"
echo "Starting MyContainer Service"
exec $value