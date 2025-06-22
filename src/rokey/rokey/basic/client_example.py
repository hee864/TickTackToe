import socket

def client_socket_open(ip, port):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ip, port))
        return sock
    except Exception as e:
        print(f"[ERROR] 연결 실패: {e}")
        return None

def client_socket_write(sock, data: bytes):
    try:
        sock.sendall(data)
    except Exception as e:
        print(f"[ERROR] 데이터 전송 실패: {e}")

def client_socket_read(sock, timeout=3):
    try:
        sock.settimeout(timeout)
        data = sock.recv(1024)
        return 1, data
    except socket.timeout:
        return 0, b''
    except Exception as e:
        print(f"[ERROR] 데이터 수신 실패: {e}")
        return -1, b''

def client_socket_close(sock):
    try:
        sock.close()
    except:
        pass


def main():
    sock = client_socket_open("127.0.0.1", 1024)
    if not sock:
        return

    try:
        while True:
            user_input = input("0~8 숫자 입력 (q 입력 시 종료): ")
            if user_input == "q":
                break

            if not user_input.isdigit() or not (0 <= int(user_input) <= 8):
                print("0~8 사이의 숫자를 입력하세요.")
                continue

            client_socket_write(sock, (user_input + '\n').encode())
            # client_socket_write(sock, user_input.encode())
            print(f'sent:{user_input}')

            res, data = client_socket_read(sock)

            if res > 0:
                print(f"[RECV] 서버 응답: {data.decode().strip()}")
            else:
                print("[ERROR] 응답 없음")

    finally:
        client_socket_close(sock)

if __name__ == "__main__":
    main()