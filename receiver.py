import socket
import rtmidi
import time
import threading

'''
FOR MACOS:
✅ 1. Enable IAC Driver in macOS
	1.	Open Audio MIDI Setup
	2.	Press Cmd + 2 to show the MIDI Studio
	3.	Double-click “IAC Driver”
	4.	Enable “Device is online”
	5.	Add a new port, name it e.g., "DrumBridge"
'''

recent_notes = {}

UDP_IP = "0.0.0.0"
UDP_PORT = 6666 # Port to listen for UDP packets (It is also specified in the ESP32 code)

def midi_keep_alive():
    while True:
        midiout.send_message([0xFE])  # MIDI Active Sensing message
        time.sleep(1.0)  # every 1 second

# Create virtual MIDI output port
midiout = rtmidi.MidiOut()
available_ports = midiout.get_ports()

# Create a virtual port
midiout.open_port(available_ports.index("IAC Driver DrumBridge")) # Adjust the index based on your system's MIDI setup

print(f"Listening for MIDI UDP packets on port {UDP_PORT}...")

# Start keep-alive thread
threading.Thread(target=midi_keep_alive, daemon=True).start()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(1024)

    if len(data) < 8:
        continue  # Skip if packet is too short

    now = time.time() * 1000  # current time in milliseconds

    # Extract raw MIDI message
    status = data[5]
    note = data[6]
    velocity = data[7]

    key = (status, note)
    last_time = recent_notes.get(key, 0)
    ghost_block_ms = 70
    if note in [42, 46, 44, 51]:  # Example: Ride and Hi-Hat notes
        ghost_block_ms = 200 # Adjust ms according to your needs

    if now - last_time < ghost_block_ms:
        print(f"🛑 Ghost note filtered: {[status, note, velocity]}")
        continue

    recent_notes[key] = now

    midi_msg = [status, note, velocity]
    print(f"From {addr}: MIDI {midi_msg}")
    midiout.send_message(midi_msg)

    # Schedule a virtual Note Off after 100 ms
    def send_note_off(status, note):
        off_status = 0x80 | (status & 0x0F)
        midiout.send_message([off_status, note, 0])
        print(f"Virtual Note Off: {[off_status, note, 0]}")

    threading.Timer(0.1, send_note_off, args=(status, note)).start()