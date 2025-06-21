import rclpy
import DR_init
from std_msgs.msg import Int32
from rclpy.node import Node

# 기본 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 100

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

class TicTacToeNode(Node):
    def __init__(self):
        super().__init__('tictactoe_node')
        self.board_state = [0] * 9  # 0: empty, 1: user, 2: robot
        self.subscription = self.create_subscription(Int32, '/tictactoe_input', self.topic_callback, 10)
        self.get_logger().info("Subscribed to /tictactoe_input")

    def topic_callback(self, msg):
        rx_index = msg.data
        print(f"[ROS2] Received index: {rx_index}")
        self.ticktacktoe(rx_index)

    def print_board_state(self):
        symbols = {0: ' ', 1: 'X', 2: 'O'}
        print("\nCurrent Board:")
        for i in range(3):
            row = [symbols[self.board_state[3 * i + j]] for j in range(3)]
            print(' ' + ' | '.join(row))
            if i < 2:
                print("---+---+---")
        print()

    def draw_line(self):
        print('draw_line')

    def draw_circle(self, robot_move):
        print(f'draw_circle at {robot_move}')

    def is_winner(self, player):
        win_cases = [
            [0, 1, 2], [3, 4, 5], [6, 7, 8],
            [0, 3, 6], [1, 4, 7], [2, 5, 8],
            [0, 4, 8], [2, 4, 6]
        ]
        return any(all(self.board_state[i] == player for i in line) for line in win_cases)

    def minimax(self, state, is_maximizing):
        if self.check_winner(state, 2): return 1
        if self.check_winner(state, 1): return -1
        if all(cell != 0 for cell in state): return 0

        scores = []
        for i in range(9):
            if state[i] == 0:
                state[i] = 2 if is_maximizing else 1
                score = self.minimax(state, not is_maximizing)
                scores.append((score, i))
                state[i] = 0
        return max(scores)[0] if is_maximizing else min(scores)[0]

    def check_winner(self, state, player):
        win_cases = [
            [0, 1, 2], [3, 4, 5], [6, 7, 8],
            [0, 3, 6], [1, 4, 7], [2, 5, 8],
            [0, 4, 8], [2, 4, 6]
        ]
        return any(all(state[i] == player for i in line) for line in win_cases)

    def get_best_move(self):
        best_score = -float('inf')
        best_move = -1
        for i in range(9):
            if self.board_state[i] == 0:
                self.board_state[i] = 2
                score = self.minimax(self.board_state, False)
                self.board_state[i] = 0
                if score > best_score:
                    best_score = score
                    best_move = i
        return best_move

    def ticktacktoe(self, rx_data):
        if rx_data < 0 or rx_data > 8 or self.board_state[rx_data] != 0:
            print(f"[Invalid Input] {rx_data}")
            return

        self.board_state[rx_data] = 1
        print(f"User placed X at {rx_data}")
        self.print_board_state()

        if self.is_winner(1):
            print("User wins. Game over.")
            return

        if all(cell != 0 for cell in self.board_state):
            print("Draw. Board full.")
            return

        robot_move = self.get_best_move()
        if robot_move == -1:
            print("No move possible.")
            return

        self.board_state[robot_move] = 2
        print(f"Robot places O at {robot_move}")
        self.print_board_state()
        self.draw_circle(robot_move)

        if self.is_winner(2):
            print("Robot wins!")
        elif all(cell != 0 for cell in self.board_state):
            print("Draw. Game over.")

def main(args=None):
    rclpy.init(args=args)
    node = TicTacToeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
