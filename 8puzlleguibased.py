import tkinter as tk
from tkinter import messagebox
import heapq
import time
import math # Import math for consistency, although not strictly needed for manhattan distance if done with abs()

# ----------------- 8-Puzzle Logic -----------------

def get_neighbors(state):
    """Generates all possible next states from the current state."""
    neighbors = []
    state = list(state)
    idx = state.index(0)
    x, y = divmod(idx, 3)
    # Possible moves: Up, Down, Left, Right
    moves = [(-1,0), (1,0), (0,-1), (0,1)]
    for dx, dy in moves:
        nx, ny = x + dx, y + dy
        if 0 <= nx < 3 and 0 <= ny < 3:
            new_state = state[:]
            swap_idx = nx*3 + ny
            # Swap the blank tile (0) with the neighbor
            new_state[idx], new_state[swap_idx] = new_state[swap_idx], new_state[idx]
            neighbors.append(tuple(new_state))
    return neighbors

def manhattan_distance(state, goal):
    """Heuristic: Calculates the sum of Manhattan distances of each tile from its goal position."""
    dist = 0
    for i, val in enumerate(state):
        if val != 0:
            goal_index = goal.index(val)
            # Current (x1, y1) and Goal (x2, y2) coordinates
            x1, y1 = divmod(i,3)
            x2, y2 = divmod(goal_index,3)
            dist += abs(x1-x2) + abs(y1-y2)
    return dist

def a_star_8_puzzle(start, goal):
    """A* search algorithm to find the shortest path from start to goal."""
    pq = []
    # Priority Queue stores (f_cost, state)
    heapq.heappush(pq, (0 + manhattan_distance(start, goal), start))
    g_cost = {start: 0}
    parent = {start: None}

    while pq:
        f, current = heapq.heappop(pq)
        if current == goal:
            break
        
        # Check if the current f_cost from the heap is still the best known f_cost (optimization)
        if f > g_cost.get(current, float('inf')) + manhattan_distance(current, goal):
            continue

        for neighbor in get_neighbors(current):
            temp_g = g_cost[current] + 1
            if neighbor not in g_cost or temp_g < g_cost[neighbor]:
                g_cost[neighbor] = temp_g
                f_cost = temp_g + manhattan_distance(neighbor, goal)
                parent[neighbor] = current
                heapq.heappush(pq, (f_cost, neighbor))

    return parent, g_cost

def reconstruct_path(parent, start, goal):
    """Reconstructs the solution path from the parent map."""
    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = parent.get(current)
    return path[::-1]

# ----------------- GUI -----------------

class PuzzleGUI:
    def __init__(self, master):
        self.master = master
        master.title("8-Puzzle Game (A* Solver)")

        # --- State variables ---
        self.state = [2,8,3,1,6,4,7,0,5] # Default start state
        self.goal = [1,2,3,8,0,4,7,6,5]  # Default goal state
        self.tiles = []
        self.solving = False
        self.delay = 0.5 # Animation delay

        # Input Frame
        self.input_frame = tk.Frame(master)
        self.input_frame.pack(pady=10)

        tk.Label(self.input_frame, text="Start State (0-8, space-separated):", font=("Arial", 10)).grid(row=0, column=0, sticky='w', padx=5, pady=2)
        self.start_entry = tk.Entry(self.input_frame, width=20, font=("Arial", 12))
        self.start_entry.grid(row=0, column=1, padx=5, pady=2)
        self.start_entry.insert(0, "2 8 3 1 6 4 7 0 5")

        tk.Label(self.input_frame, text="Goal State (0-8, space-separated):", font=("Arial", 10)).grid(row=1, column=0, sticky='w', padx=5, pady=2)
        self.goal_entry = tk.Entry(self.input_frame, width=20, font=("Arial", 12))
        self.goal_entry.grid(row=1, column=1, padx=5, pady=2)
        self.goal_entry.insert(0, "1 2 3 8 0 4 7 6 5")

        self.set_button = tk.Button(self.input_frame, text="Set Puzzle", command=self.set_puzzle, font=("Arial", 12, "bold"), bg="#4CAF50", fg="white", activebackground="#388E3C")
        self.set_button.grid(row=2, column=0, columnspan=2, pady=10)

        # Puzzle Frame (Grid)
        self.frame = tk.Frame(master, borderwidth=3, relief="raised", bg="#cccccc")
        self.frame.pack()

        for i in range(9):
            btn = tk.Button(self.frame, text="",
                            font=("Arial", 28, "bold"), width=4, height=2,
                            bg="#f0f0f0", fg="#333333", activebackground="#dddddd",
                            command=lambda i=i: self.move_tile(i))
            btn.grid(row=i//3, column=i%3, padx=2, pady=2)
            self.tiles.append(btn)

        self.solve_btn = tk.Button(master, text="Solve Puzzle (A*)", command=self.solve, font=("Arial", 14, "bold"), bg="#007bff", fg="white", activebackground="#0056b3")
        self.solve_btn.pack(pady=15)
        
        # Message/Status Label
        self.status_label = tk.Label(master, text="Ready. Click 'Set Puzzle' or a tile to play.", font=("Arial", 10), fg="#333")
        self.status_label.pack(pady=5)

        # Initial draw
        self.update_tiles()

    def set_puzzle(self):
        """Parses user input for start/goal states and updates the grid."""
        if self.solving:
            messagebox.showinfo("Wait", "Solver is currently running. Please wait or restart.")
            return
            
        start_str = self.start_entry.get().split()
        goal_str = self.goal_entry.get().split()
        
        try:
            start_state = [int(x) for x in start_str]
            goal_state = [int(x) for x in goal_str]
            
            # Validation checks
            if len(start_state) != 9 or len(goal_state) != 9:
                raise ValueError("Both states must have exactly 9 numbers.")
            if set(start_state) != set(range(9)) or set(goal_state) != set(range(9)):
                raise ValueError("States must contain numbers 0 through 8 exactly once.")
            
        except ValueError as e:
            messagebox.showerror("Invalid Input", f"Error: {e}\nPlease enter numbers 0-8 exactly once in each state, separated by spaces.")
            return

        self.state = start_state
        self.goal = goal_state
        self.update_tiles()
        self.status_label.config(text="New puzzle set. Play or Solve.")

    def move_tile(self, index):
        """Handles manual movement of tiles by clicking."""
        if self.solving:
            return
            
        zero_idx = self.state.index(0)
        if self.is_adjacent(index, zero_idx):
            # Swap tile with the blank space (0)
            self.state[zero_idx], self.state[index] = self.state[index], self.state[zero_idx]
            self.update_tiles()
            
            if self.state == self.goal:
                messagebox.showinfo("Congrats!", "Puzzle Solved Manually!")
                self.status_label.config(text="Puzzle Solved Manually!")

    def is_adjacent(self, idx1, idx2):
        """Checks if two tile indices are horizontally or vertically adjacent."""
        x1, y1 = divmod(idx1,3)
        x2, y2 = divmod(idx2,3)
        return (abs(x1-x2) + abs(y1-y2)) == 1

    def update_tiles(self):
        """Redraws the puzzle tiles based on the current state."""
        for i, btn in enumerate(self.tiles):
            value = self.state[i]
            if value != 0:
                btn.config(text=str(value), bg="#e0e0e0", fg="#333333")
            else:
                btn.config(text="", bg="#888888") # Blank space has a different background color

    def solve(self):
        """Initiates the A* solver."""
        if self.solving:
            messagebox.showinfo("Wait", "Solver is already working...")
            return
            
        self.solving = True
        self.solve_btn.config(state=tk.DISABLED, text="Solving...")
        self.status_label.config(text="Calculating path with A* (This may take a moment)...")
        self.master.update()

        # Run A* in a non-blocking way is complex in tkinter. We run it directly
        # and rely on it finishing fast enough for a 3x3 grid.
        start_time = time.time()
        parent, g_cost = a_star_8_puzzle(tuple(self.state), tuple(self.goal))
        elapsed = time.time() - start_time
        
        self.solve_btn.config(state=tk.NORMAL, text="Solve Puzzle (A*)")
        self.solving = False
        
        if tuple(self.goal) not in g_cost:
            messagebox.showinfo("Result", "No solution found (The puzzle might be unsolvable).")
            self.status_label.config(text=f"No solution found in {elapsed:.2f}s.")
            return
            
        path = reconstruct_path(parent, tuple(self.state), tuple(self.goal))
        self.status_label.config(text=f"Solution found in {elapsed:.2f}s. Path length: {len(path)-1}. Animating...")
        self.animate_solution(path[1:])

    def animate_solution(self, path):
        """Animates the solution step by step."""
        if not path:
            self.status_label.config(text="Animation complete. Puzzle Solved!")
            return

        next_state = path.pop(0)
        self.state = list(next_state)
        self.update_tiles()
        
        # Use master.after for smooth, non-blocking animation
        self.master.after(int(self.delay * 1000), lambda: self.animate_solution(path))

# ----------------- Run GUI -----------------

if __name__ == "__main__":
    root = tk.Tk()
    app = PuzzleGUI(root)
    root.mainloop()
