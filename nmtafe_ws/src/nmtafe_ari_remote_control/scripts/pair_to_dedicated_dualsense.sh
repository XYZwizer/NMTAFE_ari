bluetooth_mac="$1"

echo "\n ==removing $bluetooth_mac==\n"
timeout 2 bluetoothctl remove $bluetooth_mac

echo "\n ==waiting for $bluetooth_mac==\n"
while [[ "$(bluetoothctl devices)" != *"$bluetooth_mac"* ]]; do
	echo "" > /dev/null
        timeout 1 bluetoothctl scan on
done

echo "\n ==trying to pair to $bluetooth_mac==\n"
while [[ "$(bluetoothctl paired-devices)" != *"$bluetooth_mac"* ]]; do
	timeout 2 bluetoothctl pair $bluetooth_mac
	echo "" > /dev/null
done

timeout 2 bluetoothctl connect $bluetooth_mac
timeout 2 bluetoothctl trust $bluetooth_mac

echo "paird"
