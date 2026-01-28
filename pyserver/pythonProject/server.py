import socket
import json
import os
import time
import threading

# =========================================================
# CONFIGURATION
# =========================================================
BOARD_HOST = "192.168.137.1"   # Hotspot IP for STM32 board
BOARD_PORT = 2028
WEB_HOST = "0.0.0.0"           # Allow both local & external (mobile) access
WEB_PORT = 3030
USER_DB = "users.json"

# ---------- USER DATABASE ----------
if not os.path.exists(USER_DB):
    with open(USER_DB, "w") as f:
        json.dump({}, f)

def load_users():
    with open(USER_DB, "r") as f:
        return json.load(f)

def save_users(users):
    with open(USER_DB, "w") as f:
        json.dump(users, f, indent=2)

# =========================================================
# WEB COMMAND HANDLER (for Flask)
# =========================================================
def web_listener(stm32_conn_ref):
    """Runs in background: handles REGISTER / LOGIN / GAME_OVER commands from Flask"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as ws:
        ws.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        ws.bind((WEB_HOST, WEB_PORT))
        ws.listen(5)
        print(f"[WEB] Listening on {WEB_HOST}:{WEB_PORT} for web commands")

        while True:
            wconn, waddr = ws.accept()
            try:
                data = wconn.recv(1024).decode(errors="ignore").strip()
                if not data:
                    wconn.close()
                    continue
                print(f"[WEB] {waddr} -> {data}")

                users = load_users()

                if data.startswith("REGISTER"):
                    _, username, password = data.split(maxsplit=2)
                    if username in users:
                        wconn.sendall(b"Register FAIL\n")
                        print("[WEB] Register FAIL (exists)")
                    else:
                        users[username] = password
                        save_users(users)
                        wconn.sendall(b"Register OK\n")
                        print(f"[WEB] Register OK ({username})")

                elif data.startswith("LOGIN"):
                    _, username, password = data.split(maxsplit=2)
                    if username in users and users[username] == password:
                        wconn.sendall(f"Login OK {username}\n".encode())
                        print(f"[WEB] Login OK ({username})")
                        # Forward to STM32 if connected
                        if stm32_conn_ref[0]:
                            stm32_conn_ref[0].sendall(f"Login OK {username}\n".encode())
                    else:
                        wconn.sendall(b"Login FAIL\n")
                        print(f"[WEB] Login FAIL ({username})")

                elif "GAME_OVER" in data or "Game over" in data:
                    wconn.sendall(b"Game over\n")
                    if stm32_conn_ref[0]:
                        stm32_conn_ref[0].sendall(b"Game over\n")
                    print("[WEB] Game Over relayed to STM32")

                else:
                    wconn.sendall(b"Unknown command\n")

            except Exception as e:
                print(f"[WEB ERROR] {e}")
            finally:
                wconn.close()


# =========================================================
# MAIN STM32 SERVER
# =========================================================
def main():
    print(f"[SERVER] Starting...")

    stm32_conn_ref = [None]  # reference shared with web thread
    threading.Thread(target=web_listener, args=(stm32_conn_ref,), daemon=True).start()

    # Accept STM32 connection
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((BOARD_HOST, BOARD_PORT))
        s.listen(1)
        print(f"[SERVER] Waiting for STM32 on {BOARD_HOST}:{BOARD_PORT} ...")

        conn, addr = s.accept()
        stm32_conn_ref[0] = conn
        print(f"[CONNECTED] STM32 connected from {addr}")

        try:
            while True:
                # Server console control
                print("\n========== USER SYSTEM ==========")
                print("1️⃣ Register new user")
                print("2️⃣ Login user")
                print("3️⃣ Send 'Game Over' to STM32")
                print("4️⃣ Exit")
                choice = input("Select option (1/2/3/4): ").strip()

                if choice == "1":
                    users = load_users()
                    username = input("Username: ").strip()
                    if username in users:
                        print("❌ User exists.")
                        continue
                    password = input("Password: ").strip()
                    users[username] = password
                    save_users(users)
                    print("✅ User registered.")

                elif choice == "2":
                    username = input("Username: ").strip()
                    password = input("Password: ").strip()
                    users = load_users()
                    if username in users and users[username] == password:
                        conn.sendall(f"Login OK {username}\n".encode())
                        print(f"🎮 Sent Login OK to STM32 ({username})")
                    else:
                        conn.sendall(b"Login FAIL\n")
                        print("❌ Login failed.")

                elif choice == "3":
                    conn.sendall(b"Game over\n")
                    print("💀 Game Over sent to STM32")

                elif choice == "4":
                    conn.close()
                    break

        except KeyboardInterrupt:
            print("\n[SERVER] Terminated manually.")
        finally:
            conn.close()
            s.close()


if __name__ == "__main__":
    main()
