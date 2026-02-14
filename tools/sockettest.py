#!/usr/bin/env python3
import socket
import time

HOST = "0.0.0.0"   # listen on all interfaces
PORT = 6000        # pick a port (use 6000 to match your pilot setup)

def main():
    print(f"[server] binding {HOST}:{PORT} ...")
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)

    print("[server] listening. waiting for client...")
    conn, addr = srv.accept()
    print(f"[server] client connected from {addr}")

    conn.settimeout(2.0)

    total_bytes = 0
    last_print = time.time()

    try:
        while True:
            try:
                data = conn.recv(4096)
            except socket.timeout:
                # no data recently; keep waiting
                continue

            if not data:
                print("[server] client closed connection.")
                break

            total_bytes += len(data)

            # Print content (safe-ish)
            now = time.time()
            if now - last_print > 0.25:
                preview = data[:120]
                try:
                    preview_txt = preview.decode("utf-8", errors="replace")
                except Exception:
                    preview_txt = repr(preview)
                print(f"[server] rx {len(data)} bytes (total={total_bytes}) preview={preview_txt!r}")
                last_print = now

    except KeyboardInterrupt:
        print("\n[server] ctrl+c, quitting.")
    finally:
        try:
            conn.close()
        except Exception:
            pass
        srv.close()

if __name__ == "__main__":
    main()
