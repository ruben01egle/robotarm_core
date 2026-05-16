sudo nmcli device set eno1 managed no

sudo ip addr flush dev eno1
sudo ip addr add 192.168.0.10/24 dev eno1
sudo ip link set eno1 up