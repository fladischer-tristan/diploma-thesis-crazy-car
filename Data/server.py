import socket
import csv
import os

import struct

fmt = "<B I f f f f f f f f f f f I I B"
packet_size = struct.calcsize(fmt)

FIELDS = [
    "startByte", "packetNumber", "velocity", "batteryVoltage",
    "leftDistance", "middleDistance", "rightDistance",
    "ax", "ay", "az", "gx", "gy", "gz",
    "servoPulse", "escPulse", "stopByte"
]

csv_filename = "home_straight_path_V1.csv"
file_exists = os.path.isfile(csv_filename)

with open(csv_filename, "a", newline="") as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=FIELDS, delimiter=';')

    # Wenn Datei neu: Header schreiben
    if not file_exists:
        writer.writeheader()

    s = socket.socket()
    s.bind(("0.0.0.0", 5000))
    s.listen(1)
    print("Warte auf Verbindung ...")
    conn, addr = s.accept()
    print("Verbunden mit", addr)

    buffer = b""
    while True:
        data = conn.recv(1024)
        if not data:
            break
        buffer += data

        while len(buffer) >= packet_size:
            if buffer[0] != 0xAA:
                buffer = buffer[1:]
                continue

            packet_bytes = buffer[:packet_size]
            buffer = buffer[packet_size:]

            values = struct.unpack(fmt, packet_bytes)

            sensor_data = dict(zip(FIELDS, values))

            # Zeile in CSV schreiben
            writer.writerow(sensor_data)
            csvfile.flush()  # sofort speichern

            print(f"Paket {sensor_data['packetNumber']} gespeichert")
