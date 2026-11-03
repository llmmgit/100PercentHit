#!/bin/sh
# 以root执行一次。先使用nmcli创建名为home-wifi的家庭Wi-Fi连接。
set -eu
AP_NAME=percenthit-ap
SSID=100PercentHit
PASS='CHANGE_ME_123'
AP_ADDR=192.168.86.1/24
nmcli con delete "$AP_NAME" 2>/dev/null || true
nmcli con add type wifi ifname wlan0 con-name "$AP_NAME" autoconnect no ssid "$SSID"
nmcli con modify "$AP_NAME" 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared ipv4.addresses "$AP_ADDR" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PASS"
echo "热点已创建: $SSID, 地址: $AP_ADDR"
