#!/bin/bash

# Check if the argument is provided
if [ -z "$1" ]
then
    echo "No argument supplied. Please provide either 'AP' or 'Wifi'."
    exit 1
fi

# Check if the argument is either 'AP' or 'Wifi'
if [ "$1" != "AP" ] && [ "$1" != "Wifi" ]
then
    echo "Invalid argument. Please provide either 'AP' or 'Wifi'."
    exit 1
fi

# Modify the configuration file
if [ "$1" == "AP" ]
then
    sed -i '/nohook wpa_supplicant/s/^#//g' /etc/dhcpcd.conf
    sed -i '/interface wlan0/s/^#//g' /etc/dhcpcd.conf
    sed -i '/static ip_address=192.168.42.10\/24/s/^#//g' /etc/dhcpcd.conf
    sed -i '/static routers=192.168.42.1/s/^#//g' /etc/dhcpcd.conf
elif [ "$1" == "Wifi" ]
then
    sed -i '/nohook wpa_supplicant/s/^/#/g' /etc/dhcpcd.conf
    sed -i '/interface wlan0/s/^/#/g' /etc/dhcpcd.conf
    sed -i '/static ip_address=192.168.42.10\/24/s/^/#/g' /etc/dhcpcd.conf
    sed -i '/static routers=192.168.42.1/s/^/#/g' /etc/dhcpcd.conf
fi

echo "Configuration updated successfully. Please reboot the system."

