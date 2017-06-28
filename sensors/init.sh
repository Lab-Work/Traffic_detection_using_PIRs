#!/bin/bash

hostname=$1
ssid="raspberry"
psk="raspberry"

if [$# = 0];
then
  echo "Please enter a hostname"
else
  echo "Installing Pi as $hostname"
  sudo sed -i "1s/.*/$hostname/" /etc/hostname
  sudo sed -i "/127.0.1.1/ c\127.0.1.1 $hostname" /etc/hosts

  echo "Installing WiFi:"
  echo "SSID: $ssid"
  echo "PSK: $psk"
  

  if sudo grep -Fxq "$ssid" /etc/wpa_supplicant/wpa_supplicant.conf
  then
    echo "WiFi network already installed. Skipping..."
  else
    sudo sed -i "/country/ c\country=US" /etc/wpa_supplicant/wpa_supplicant.conf
    sudo bash -c "echo -e \"\n\" >> /etc/wpa_supplicant/wpa_supplicant.conf"      
    sudo bash -c "echo \"network={\" >> /etc/wpa_supplicant/wpa_supplicant.conf"
    sudo bash -c "echo -e \"\tssid=\\\"$ssid\\\"\" >> /etc/wpa_supplicant/wpa_supplicant.conf"
    sudo bash -c "echo -e \"\tpsk=\\\"$psk\\\"\" >> /etc/wpa_supplicant/wpa_supplicant.conf"
    sudo bash -c "echo -e \"\tscan_ssid=1\" >> /etc/wpa_supplicant/wpa_supplicant.conf"    
    sudo bash -c "echo \"}\" >> /etc/wpa_supplicant/wpa_supplicant.conf"
  fi
fi
