#!/bin/bash

list_wlan_interfaces() {
  for dir in /sys/class/net/*/wireless; do
    if [ -d "$dir" ]; then
      IFACE="$(basename "$(dirname "$dir")")"
      echo $IFACE
    fi
  done
}
IFACE="$(list_wlan_interfaces | head -n 1)"

nmcli con add type wifi ifname $IFACE con-name $1 #autoconnect yes ssid $2
nmcli con modify $1 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared
nmcli con up $1
