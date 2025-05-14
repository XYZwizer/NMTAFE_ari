bluetooth_mac="$1"

timeout 1 bluetoothctl scan on

echo "waiting"
while [[ "$(bluetoothctl devices)" != *"$bluetooth_mac"* ]]; do
	echo "" > /dev/null
        timeout 1 bluetoothctl scan on
done
echo "trying to pair"
echo "pair $bluetooth_mac" | bluetoothctl

while [[ "$(bluetoothctl paired-devices)" != *"$bluetooth_mac"* ]]; do
	timeout 2 bluetoothctl pair $bluetooth_mac
	echo "" > /dev/null
done

timeout 2 bluetoothctl connect $bluetooth_mac
timeout 2 bluetoothctl trust $bluetooth_mac

echo "paird"
#echo "connect $bluetooth_mac" | bluetoothctl

#if [[ "$(bluetoothctl devices Paired)" != *"$bluetooth_mac"* ]]; then
#	echo "device is not paired"
#	echo "pair $bluetooth_mac" | bluetoothctl
#fi


