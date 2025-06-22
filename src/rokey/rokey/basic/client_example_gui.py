import sys
import socket
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QGridLayout, QLabel,
    QLineEdit, QPushButton, QMessageBox
)

# 클라이언트 소켓
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

# PyQt5 GUI 화면창
class GameWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tic Tac Toe - Client")
        self.sock = client_socket_open("127.0.0.1", 1024)
        if not self.sock:
            QMessageBox.critical(self, "연결 실패", "서버에 연결할 수 없습니다.")
            sys.exit()
        self.init_ui()

    def init_ui(self):
        self.grid = QGridLayout()
        self.cells = []

        
        for i in range(9):
            label = QLabel(" ", self)
            label.setStyleSheet("QLabel { border: 1px solid black; font-size: 30px; min-width: 60px; min-height: 60px; qproperty-alignment: AlignCenter; }")
            self.grid.addWidget(label, i // 3, i % 3)
            self.cells.append(label)

        self.input_box = QLineEdit()
        self.input_box.setPlaceholderText("0~8 위치 선택")

        self.confirm_btn = QPushButton("확인")
        self.cancel_btn = QPushButton("취소")

        self.confirm_btn.clicked.connect(self.confirm_clicked)
        self.cancel_btn.clicked.connect(self.cancel_clicked)

        layout = QVBoxLayout()
        layout.addLayout(self.grid)
        layout.addWidget(QLabel("내가 놓고 싶은 위치 (0~8) 입력"))
        layout.addWidget(self.input_box)
        layout.addWidget(self.confirm_btn)
        layout.addWidget(self.cancel_btn)

        self.setLayout(layout)

    def confirm_clicked(self):
        pos = self.input_box.text()
        if not pos.isdigit() or not (0 <= int(pos) <= 8):
            QMessageBox.warning(self, "입력 오류", "0~8 숫자만 입력하세요.")
            return

        reply = QMessageBox.question(self, "확인", "해당 위치에 놓으시겠습니까?",
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.send_position(int(pos))

    # def send_position(self, pos):
    #     client_socket_write(self.sock, (str(pos) + "\n").encode())
    #     res, data = client_socket_read(self.sock)
    #     if res > 0:
    #         robot_pos = int(data.decode().strip())
    #         self.update_board(pos, robot_pos)
    #     else:
    #         QMessageBox.warning(self, "오류", "서버 응답이 없습니다.")

    def send_position(self, pos):
        client_socket_write(self.sock, (str(pos) + "\n").encode())
        res, data = client_socket_read(self.sock)
        if res > 0:
            decoded = data.decode().strip()
            print(f"[DEBUG] Received from server: {decoded}")
            parts = decoded.split(":")
            try:
                robot_pos = int(parts[0])
                self.update_board(pos, robot_pos)
                if len(parts) > 1:
                    QMessageBox.information(self, "게임 결과", parts[1])
            except ValueError:
                QMessageBox.warning(self, "오류", "서버 응답 오류")
        else:
            QMessageBox.warning(self, "오류", "서버 응답 없음")


    def update_board(self, my_pos, robot_pos):
        self.cells[my_pos].setText("X")
        self.cells[robot_pos].setText("O")

    def cancel_clicked(self):
        self.input_box.clear()

    def closeEvent(self, event):
        client_socket_close(self.sock)
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GameWindow()
    window.show()
    sys.exit(app.exec_())
