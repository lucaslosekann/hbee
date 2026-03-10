import struct

def decode_payload(payload_hex):
    try:
        value = bytes.fromhex(payload_hex)
        if value[0] != 2:
            return None
        # Unpack fields
        commandId = value[0]
        layer = value[1]
        ENId = int.from_bytes(value[2:4], 'big')
        parent_id = int.from_bytes(value[4:6], 'big')
        data_len = int.from_bytes(value[6:8], 'big')
        voltage = struct.unpack('<f', value[8:12])[0]
        current = struct.unpack('<f', value[12:16])[0] * 1000
        originalNode = int.from_bytes(value[16:18], 'big')
        originalLayer = value[18]
        pkt_number = value[19]
        size = len(value)
        #How many time 8 bytes repeat after the first 20 bytes
        repeat_count = (size - 20) // 8
        path = []
        for i in range(repeat_count):
            segment = value[20 + i * 8:28 + i * 8]
            node = int.from_bytes(segment[0:2], 'little')
            rssi = int.from_bytes(segment[2:4], 'little', signed=True)
            snr = struct.unpack('<f', segment[4:8])[0]
            path.append({
                'node': node,
                'rssi': rssi,
                'snr': snr
            })
        return {
            'commandId': commandId,
            'layer': layer,
            'ENId': ENId,
            'parent_id': parent_id,
            'data_len': data_len,
            'voltage': voltage,
            'current': current,
            'originalNode': originalNode,
            'originalLayer': originalLayer,
            'pkt_number': pkt_number,
            'path': path
        }
    except Exception as e:
        print(f"Error decoding payload: {e}")
        return None