import socket
import threading
import queue
import time
import struct

from more_itertools.more import difference

INACTIVITY_TIMEOUT = 3600 # nach einer stunde bitte raus aus verteilerliste
RECEIVE_PORT = 6969
SEND_PORT = 420

messages = queue.Queue()    # Queue ist race condition safe (eingebautes mutex lock)
clients = {}    # dictionary nach dem muster (address: last_seen)
clients_lock = threading.Lock()     # lock, für das dict

print(socket.gethostbyname(socket.gethostname()))

server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server.bind(("192.168.178.103", RECEIVE_PORT))

def receive():
    seqs = {}
    anz_packets = 0
    anz_verloren = 0
    anz_recovered = 0
    while True:
        try:
            message, address = server.recvfrom(8)  # ein packet der maximalen länge 8 byte empfangen, blockiert den thread, bis was kommt
            messages.put((message, address))    # nachricht und herkunft speichern (thread safe)

            session, seq, data = struct.unpack("BBB", message[:3])
            print(f"Recieved Session={session}, Seq={seq}, Data={data:08b} from {address}, Time={time.time()}")
            anz_packets = anz_packets + 1

            if session in seqs:
                difference = seq - seqs[session]
                if difference > 1:
                    anz_verloren = anz_verloren + difference - 1
                    print(f"difference: {seq - seqs[session]}, anteil verloren: {anz_verloren}/{anz_packets} = {anz_verloren / anz_packets}")
                if difference < 1:
                    anz_recovered = anz_recovered + 1
                    print(f"difference: {seq - seqs[session]}, anteil recovered: {anz_recovered}/{anz_packets} = {anz_recovered / anz_packets}")

            if anz_packets >= 1000:
                anz_packets = 0
                anz_verloren = 0
                anz_recovered = 0

            seqs.update({session: seq})


            with clients_lock:  # clients nicht thread sicher
                clients.update({address: time.time()}) # client dictionary mit zuletzt gesehener zeit
        except Exception as e:
            print(f"Error in receive {e}")

def broadcast():
    while True:
        try:
            message, from_address = messages.get()   # wartet solange bis ein element drin ist. blockiert die queue nicht!
            session, seq, data = struct.unpack("BBB", message[:3])
            to_remove = []  # muss zwischen gespeichert werden, weil dictionary kann nicht bearbeitet werdem während iteration
            with clients_lock: # lock nicht thread sicher
                if session == 0:
                    server.sendto(message, from_address)
                else:
                    for client, last_seen in clients.items():
                        #print(f"from_address:{from_address} client:{client}")
                        if time.time() - last_seen > INACTIVITY_TIMEOUT:  # eine stunde offline?
                            to_remove.append(client)
                        else:
                            if client != from_address:   # nicht an sich selber senden bitte
                                server.sendto(message, client)
                                print(f"Sent Session={session}, Seq={seq}, Data={data:08b} to {client}, Time={time.time()}")
                    for client in to_remove:    # alle clients aus dictionary entfernen, die inaktiv waren
                        del clients[client]
        except Exception as e:
            print(f"Error in broadcast {e}")

t1 = threading.Thread(target=receive) # daemon wird mit geschlossen, wenn hauptprogramm beendet wird
t2 = threading.Thread(target=broadcast)

t1.start()
t2.start()
