import socket
import tkinter as tk
from tkinter import messagebox

HOST = '127.0.0.1'
PORT = 1024

class TicTacToeGUI:
    def __init__(self, master):
        self.master = master
        master.title("틱택토 - 사용자: X / 로봇: O")

        self.board = [0] * 9
        self.buttons = []
        
        

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((HOST, PORT))
        except Exception as e:
            messagebox.showerror("연결 실패", f"서버에 연결할 수 없습니다: {e}")
            master.destroy()
            return
        self.master.protocol("WM_DELETE_WINDOW", self.on_close)

        self.select_mode()
        self.create_board()
        self.create_reset_button()

    def select_mode(self):
        mode = messagebox.askquestion("난이도 선택", "하드모드로 시작할까요?\n(예: 하드 / 아니오: 이지)")
        if mode == 'yes':
            self.sock.sendall(b"-1\n")
        else:
            self.sock.sendall(b"-2\n")
        response = self.sock.recv(1024).decode().strip()
        if response == "-99":
            messagebox.showerror("오류", "서버가 난이도 설정을 거부했습니다.")
            self.master.destroy()

    def create_board(self):
        for i in range(9):
            button = tk.Button(self.master, text=" ", width=10, height=4,
                               command=lambda i=i: self.make_move(i), font=("Helvetica", 18))
            button.grid(row=i // 3, column=i % 3)
            self.buttons.append(button)

    def create_reset_button(self):
        reset_button = tk.Button(self.master, text="게임 리셋", width=32, height=2,
                                 command=self.reset_game, font=("Helvetica", 14))
        reset_button.grid(row=3, column=0, columnspan=3, pady=10)

    def make_move(self, index):
        if self.board[index] != 0 :
            return

        try:
            self.sock.sendall(f"{index}\n".encode())
        except Exception as e:
            messagebox.showerror("오류", f"서버 전송 실패: {e}")
            self.master.destroy()
            return

        self.board[index] = 1
        self.buttons[index].config(text="X", state=tk.DISABLED)

        try:
            response = self.sock.recv(1024).decode().strip().splitlines()
            print(f"[DEBUG] 응답: {response}")
            for response in response:
                if response.isdigit() and int(response) in [10,11,12]:
                    print(f"[DEBUG] 응답: {response}")
                    self.show_result(int(response))
                    self.sock.settimeout(None) 
                    return  # 게임 끝났으면 더 이상 진행하지 않음

                elif response.isdigit() and int(response) in range(9):
                    robot_move = int(response)
                    self.board[robot_move] = 2
                    self.buttons[robot_move].config(text="O", state=tk.DISABLED)

                    # 두 번째 응답: 로봇 승/무 여부 (optional)
                    self.sock.settimeout(0.5)
                    try:
                        followup = self.sock.recv(1024).decode().strip()
                        foll=int(followup)
                        print(f"[DEBUG] 추가 응답: {followup}")
                        if foll in [10,11]:
                            self.show_result(foll)
                    except socket.timeout:
                        pass
                    finally:
                        self.sock.settimeout(None)
                else:
                    messagebox.showerror("오류", f"알 수 없는 응답: {response}")
        except Exception as e:
            messagebox.showerror("오류", f"서버 응답 수신 실패: {e}")
            self.master.destroy()
    def reset_game(self):
        try:
            self.sock.sendall(b"9\n")
            # ack = self.sock.recv(1024).decode().strip()
            # if ack == "reset":
            self.board = [0] * 9
            
            for btn in self.buttons:
                btn.config(text=" ", state=tk.NORMAL)
            messagebox.showinfo("리셋", "게임이 초기화되었습니다.")
            # else:
            #     messagebox.showwarning("리셋 실패", f"서버 응답: {ack}")
        except Exception as e:
            messagebox.showerror("오류", f"리셋 중 오류 발생: {e}")

    def show_result(self, code):
        self.board = [0] * 9
        for btn in self.buttons:
                btn.config(text=" ", state=tk.NORMAL)

        
        print(code)
        if code == 10:
            messagebox.showinfo("게임 종료", "로봇이 이겼습니다!")
            self.sock.sendall(b"9\n")
            
        elif code == 11:
            messagebox.showinfo("게임 종료", "무승부입니다.")
            self.sock.sendall(b"9\n")
           
        elif code == 12:
            messagebox.showinfo("게임 종료", "당신이 이겼습니다!")
            self.sock.sendall(b"9\n")
        
        messagebox.showinfo("리셋", "게임이 초기화되었습니다.")
    def on_close(self): #종료 추가 
        try:
            self.sock.sendall(b"exit\n")
            self.sock.close()
        except:
            pass
        self.master.destroy()
      
        
    
def main():
    root = tk.Tk()
    gui = TicTacToeGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
