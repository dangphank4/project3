import socket
import cv2
import signal
import sys

# === Settings ===
HOST = '0.0.0.0'
PORT = 8080

# === MJPEG HTTP header ===
HEADER = (
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
    "\r\n"
)

BOUNDARY = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"

# Global resources to clean up on exit
server_socket = None
cap = None

def signal_handler(sig, frame):
    print("\n🛑 Caught Ctrl+C! Shutting down...")
    if cap:
        cap.release()
        print("📷 Camera released.")
    if server_socket:
        server_socket.close()
        print("🔌 Server socket closed.")
    sys.exit(0)

def start_server():
    global server_socket, cap

    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)

    # Create server socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(5)
    print(f"🌐 Listening on http://{HOST}:{PORT}")

    # Open UVC camera
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)
    ret, frame = cap.read()

    if not cap.isOpened():
        print("❌ Cannot open camera")
        return

    while True:
        client_socket, addr = server_socket.accept()
        print(f"🔌 Client connected from {addr}")

        try:
            client_socket.sendall(HEADER.encode('utf-8'))

            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                #flipped_frame = cv2.flip(frame, 0)
                #_, buffer = cv2.imencode('.jpg', flipped_frame)
                _, buffer = cv2.imencode('.jpg', frame)
                jpeg_bytes = buffer.tobytes()

                client_socket.sendall(BOUNDARY)
                client_socket.sendall(jpeg_bytes)
                client_socket.sendall(b"\r\n")

        except (ConnectionResetError, BrokenPipeError):
            print(f"❗ Client {addr} disconnected")
        finally:
            client_socket.close()

if __name__ == "__main__":
    start_server()