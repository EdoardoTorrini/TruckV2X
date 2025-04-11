## Listener for CAM msg

```python
from scapy.all import sniff, Raw
import asn1tools

# Load the ASN.1 schema
cam = asn1tools.compile_files('ITS-Container.asn', 'uper')

# Callback function to process each packet
def packet_callback(packet):
    if packet.haslayer(Raw):  # Check if there's an application layer payload
        raw_data = packet[Raw].load
        try:
            # Decode the UPER-encoded CAM message
            decoded_data = cam.decode('CAM', raw_data)
            print(f"Decoded CAM Message: {decoded_data}")
        except Exception as e:
            print(f"Error decoding packet: {e}")

# Start sniffing packets
sniff(iface="wlan1", prn=packet_callback, count=10)
```

## Message CAM Example
```json
{
    'header': {
        'protocolVersion': 17, 
        'messageID': 0, 
        'stationID': 4043382864
    }, 
    'cam': {
        'generationDeltaTime': 512, 
        'camParameters': {
            'basicContainer': {
                'stationType': 3, 
                'referencePosition': {
                    'latitude': -631039776, 
                    'longitude': -1705256010, 
                    'positionConfidenceEllipse': {
                        'semiMajorConfidence': 520, 
                        'semiMinorConfidence': 2632, 
                        'semiMajorOrientation': 0
                    }, 
                    'altitude': {
                        'altitudeValue': 149208, 
                        'altitudeConfidence': 'alt-200-00'
                    }
                }
            }, 
            'highFrequencyContainer': ('rsuContainerHighFrequency', {})
        }
    }
}
```
For checking the frequency where the interface operates:
```bash
$ iwlist wlan1 channel
```

Il file in cui è incluso il di configurazione dell'iP statico è: `/etc/NetworkManager/system-connections/Connessione\ via\ cavo\ 1.nmconnection`

Access on rpi `riccio:4316`

On raspberry to enable the bridge on pc:
```bash
$ sudo ip route add default via 10.0.1.4 dev eth0
```
## Gps

```bash
$ sudo apt-get install gpsd gpsd-clients
$ sudo systemctl enable gpsd.socket
$ sudo gpsd /dev/ttyACM0 -F /var/run/gpsd.sock
$ sudo cgps -s
```

```bash
$ docker run -it --rm --device=/dev/ttyACM0 --net=host ros:humble-ros-core
```