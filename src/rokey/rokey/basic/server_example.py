import socket
import random

def server_socket_open(port):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind(('', port))
        sock.listen(1)
        print(f"[INFO] 포트 {port}에서 서버 대기 중")
        conn, addr = sock.accept()
        print(f"[INFO] 클라이언트 접속: {addr}")
        return conn
    except Exception as e:
        print(f"[ERROR] 서버 소켓 오류: {e}")
        return None

def server_socket_read(sock, timeout=3):
    try:
        sock.settimeout(timeout)
        data = sock.recv(1024)
        return 1, data
    except socket.timeout:
        return 0, b''
    except Exception as e:
        print(f"[ERROR] 수신 실패: {e}")
        return -1, b''

def server_socket_write(sock, data: bytes):
    try:
        sock.sendall(data)
    except Exception as e:
        print(f"[ERROR] 응답 전송 실패: {e}")

def server_socket_close(sock):
    try:
        sock.close()
    except:
        pass

def main():
    sock = server_socket_open(1024)
    if sock:
        while True:
            res, data = server_socket_read(sock)
            if res > 0:
                msg = data.decode().strip()
                print(f"[RECV] 사용자: {msg}")

                if msg.isdigit() and 0 <= int(msg) <= 8:
                    robot_move = random.randint(0, 8)
                    response = str(robot_move)
                    server_socket_write(sock, response.encode())
                    print(f"[SEND] 로봇: {response}")
                else:
                    server_socket_write(sock, b"Invalid input")
            elif res == -1:
                print("[INFO] 연결 종료")
                break
        server_socket_close(sock)

if __name__ == "__main__":
    main()
