import _thread
import time
import network
import socket
import uselect
import ujson
import random
import errno
import utime

MAX_CLIENTS = 10


stop_flag = False
stop_lock = _thread.allocate_lock()

sensor_state = {
    "active": False,
    "bpm": 0,
    "spo2": 0,
    "finger_on": False,
}

def get_mime_type(filename):
    if filename.endswith('.html'):
        return 'text/html'
    if filename.endswith('.css'):
        return 'text/css'
    if filename.endswith('.js'):
        return 'application/javascript'
    if filename.endswith('.png'):
        return 'image/png'
    if filename.endswith('.jpg') or filename.endswith('.jpeg'):
        return 'image/jpeg'
    return 'application/octet-stream'

def open_ap():
    wlan = network.WLAN(network.AP_IF)
    wlan.active(True)
    wlan.config(
        ssid='walrus',
        key='12345678',
        security=network.WLAN.SEC_WPA3
    )
    print("Access point started: walrus")
    return wlan

def serve_static_file(sock, file_path):
    try:
        with open(file_path, 'rb') as f:
            mime = get_mime_type(file_path)
            header = (
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: {}\r\n"
                "Connection: close\r\n"
                "\r\n"
            ).format(mime)
            sock.sendall(header.encode())
            while True:
                chunk = f.read(512)
                if not chunk:
                    break
                sent = 0
                while sent < len(chunk):
                    try:
                        sent_now = sock.send(chunk[sent:])
                        if sent_now is None:
                            sent_now = 0
                        sent += sent_now
                    except OSError as e:
                        if hasattr(e, 'errno') and e.errno == errno.EAGAIN:
                            print("Socket send EAGAIN, sleeping 100ms...")
                            utime.sleep_ms(100)
                            continue
                        else:
                            raise
    except Exception as e:
        print("File not found:", file_path, e)
        sock.sendall(b"HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\n404 Not Found")

def process_http_request(sock, req_str, sensor_state, probe_paths, redir):
    lines = req_str.split('\r\n')
    if lines:
        parts = lines[0].split()
        if len(parts) >= 2 and parts[0] == 'GET':
            path = parts[1]
            # captive portal probe paths
            if path in probe_paths:
                if path == "/generate_204":  # Android check
                    sock.sendall(b"HTTP/1.1 204 No Content\r\nConnection: close\r\n\r\n")
                else:
                    sock.sendall(redir.encode())
            elif path == '/start':
                sensor_state["active"] = True
                response = ujson.dumps({"status": "started"})
                sock.sendall(b"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n" + response.encode())
            elif path == '/stop':
                sensor_state["active"] = False
                response = ujson.dumps({"status": "stopped"})
                sock.sendall(b"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n" + response.encode())
            elif path == '/data':
                response = ujson.dumps(sensor_state)
                sock.sendall(b"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n" + response.encode())
            else:  # Serve static files
                if path == '/':
                    path = '/index.html'
                file_path = '/public' + path
                serve_static_file(sock, file_path)
        else:
            sock.sendall(b"HTTP/1.1 400 Bad Request\r\nConnection: close\r\n\r\n")

def run_web_server(wlan_ap):
    dns_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dns_sock.bind(('0.0.0.0', 53))

    http_sock = socket.socket()
    http_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    http_sock.bind(('0.0.0.0', 80))
    http_sock.listen(MAX_CLIENTS - 1)

    poll = uselect.poll()
    poll.register(dns_sock, uselect.POLLIN)
    poll.register(http_sock, uselect.POLLIN)
    client_socks = set()

    print("Captive portal running (DNS + HTTP)...")
    
    # This was the original HTML response, now replaced with a file read
    # html = (
    # "HTTP/1.1 200 OK\r\n"
    # "Content-Type: text/html\r\n"
    # "Connection: close\r\n"
    # "\r\n"
    # "<html><head><title>Captive portal</title></head>"
    # "<body><h1>Welcome to the Walrus network</h1>"
    # "<p>Please authenticate to access the internet.</p>"
    # "</body></html>"
    # )
    
    probe_paths = [
        "/redirect", "/connecttest.txt", "/ncsi.txt", "/hotspot-detect.html",
        "/generate_204", "/library/test/success.html"
    ]
    
    redir = (
    "HTTP/1.1 302 Found\r\n"
    "Location: http://192.168.4.1/\r\n"
    "Connection: close\r\n"
    "\r\n"
    )
    
    try:
        while True:
            with stop_lock:
                if stop_flag:
                    break
            # print("thread_task working!")

            events = poll.poll(100)
            for sock, event in events:
                if sock == dns_sock:
                    try:
                        data, addr = dns_sock.recvfrom(512)
                        if data:
                            txid = data[:2]
                            flags = b'\x81\x80'
                            qdcount = b'\x00\x01'
                            ancount = b'\x00\x01'
                            response = txid + flags + qdcount + ancount + b'\x00\x00\x00\x00'
                            query = data[12:]
                            response += query
                            response += b'\xC0\x0C\x00\x01\x00\x01\x00\x00\x00\x3C\x00\x04'
                            response += bytes([192, 168, 4, 1])
                            dns_sock.sendto(response, addr)
                    except Exception as e:
                        print("DNS loop error:", e)
                elif sock == http_sock:
                    try:
                        conn, addr = http_sock.accept()
                        if len(client_socks) >= MAX_CLIENTS:
                            print("Too many clients, refusing connection from", addr)
                            try:
                                conn.sendall(b"HTTP/1.1 503 Service Unavailable\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nServer busy, try again later.")
                            except Exception:
                                pass
                            conn.close()
                            continue
                        conn.setblocking(False)
                        # conn.settimeout(2) # this was before the non-blocking change
                        print("New HTTP connection from", addr)
                        poll.register(conn, uselect.POLLIN)
                        client_socks.add(conn)
                        # MOVED to elif below
                        # this was the original blocking code, now replaced with non-blocking
                        # try:
                        #     req = conn.recv(1024)
                        #     if not req:
                        #         conn.close()
                        #         continue
                        #     req_str = req.decode()
                        #     if any(x in req_str for x in ["connectivitycheck.android.com", "captive.apple.com", "msftncsi.com"]):
                        #         conn.sendall(redir.encode())
                        #     else:
                        #         conn.sendall(html.encode())
                        # finally:
                        #     conn.close()
                    except Exception as e:
                        print("HTTP loop error:", e)
                elif sock in client_socks:
                    try:
                        req = sock.recv(1024)
                        if not req:
                            poll.unregister(sock)
                            client_socks.remove(sock)
                            sock.close()
                            continue
                        req_str = req.decode()
                        # captive portal magic
                        if any(x in req_str for x in [
                            "connectivitycheck.android.com",
                            "captive.apple.com",
                            "msftncsi.com"
                        ]):
                            sock.sendall(redir.encode())
                        else:
                            process_http_request(sock, req_str, sensor_state, probe_paths, redir)
                        poll.unregister(sock)
                        client_socks.remove(sock)
                        sock.close()
                    except Exception as e:
                        print("HTTP client error:", e)
                        try:
                            poll.unregister(sock)
                        except Exception:
                            pass
                        client_socks.discard(sock)
                        sock.close()
    finally:
        dns_sock.close()
        print("DNS server stopped")
        http_sock.close()
        print("Captive portal HTTP server stopped")
        wlan_ap.active(False)
        print("Access point stopped")
        print("thread_task stopping!")

def read_pulseoximeter():
    # placeholder function to simulate reading from a pulse oximeter
    # todo: replace with real (functional) code
    bpm = random.randint(60, 100)
    spo2 = random.randint(95, 100)
    finger_on = random.choice([True, True, True, True,
                               True, True, True, False]) # type: ignore because issues with Subscriptable
    time.sleep(0.5) # simulate sensor delay
    print(f"Read pulse oximeter: BPM={bpm}, SpO2={spo2}, Finger on={finger_on}")
    return bpm, spo2, finger_on

def main_core():
    global stop_flag
    while True:
        with stop_lock:
            if stop_flag:
                break
        if sensor_state["active"]:
            bpm, spo2, finger_on = read_pulseoximeter()
            sensor_state["bpm"] = bpm
            sensor_state["spo2"] = spo2
            sensor_state["finger_on"] = finger_on
        time.sleep(1) # wait before resending

def second_core():
    global stop_flag
    wlan_ap = open_ap()
    time.sleep(2)
    run_web_server(wlan_ap)

_thread.start_new_thread(second_core, ())

try:
    while True:
        with stop_lock:
            if stop_flag:
                break
        main_core()
except KeyboardInterrupt:
    with stop_lock:
        stop_flag = True
    print("Main thread stopping!")

time.sleep(1)
print("Finished.")