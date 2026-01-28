from flask import Flask, render_template, request, jsonify
import socket

app = Flask(__name__)

# --- Connect to the local web listener of server.py ---
SERVER_HOST = "127.0.0.1"  # same machine as server.py
SERVER_PORT = 3030

# =========================================================
# Socket communication helper
# =========================================================
def send_to_server(cmd):
    """Send command to server.py (port 3030) and return response."""
    print(f"[DEBUG] send_to_server -> {cmd}")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((SERVER_HOST, SERVER_PORT))
            s.sendall(cmd.encode())
            data = s.recv(1024).decode(errors="ignore")
            print(f"[RECV] {data.strip()}")
            return data.strip()
    except Exception as e:
        print(f"[ERROR] {e}")
        return f"[ERROR] {e}"

# =========================================================
# Web routes
# =========================================================
@app.route("/")
def home():
    return render_template("index.html")

@app.route("/send", methods=["POST"])
def send():
    cmd = request.form.get("cmd")
    result = send_to_server(cmd)
    return jsonify({"response": result})

# =========================================================
# Run Flask
# =========================================================
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False, threaded=True)
