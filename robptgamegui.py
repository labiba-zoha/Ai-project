import tkinter as tk
import heapq
import math

# ----------------- Grid & A* Logic (Refactored to accept state) -----------------

ROWS, COLS = 5, 5

# Removed global startnode, goalnode, obstacles - they are now instance variables

def is_valid(x, y, obstacles_list):
    """Checks if coordinates are within bounds and not an obstacle."""
    return 0 <= x < ROWS and 0 <= y < COLS and (x, y) not in obstacles_list

def heuristic(a, b):
    """Calculates the Euclidean distance heuristic."""
    (x1, y1), (x2, y2) = a, b
    # Manhattan distance is often sufficient for grid, but keeping Euclidean for now
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def get_neighbors(node, obstacles_list):
    """Returns valid neighbors (up, down, left, right), considering the current obstacles."""
    x, y = node
    directions = [(-1,0),(1,0),(0,-1),(0,1)]
    neighbors = []
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if is_valid(nx, ny, obstacles_list):
            neighbors.append((nx, ny))
    return neighbors

def a_star(startnode, goalnode, obstacles_list):
    """A* algorithm implementation, accepting dynamic state."""
    pq = []
    # Priority Queue stores (f_cost, node)
    heapq.heappush(pq, (heuristic(startnode, goalnode), startnode))
    dist = {startnode: 0} # g_cost
    parent = {startnode: None}
    visited_order = []

    while pq:
        _, current = heapq.heappop(pq)
        visited_order.append(current)

        if current == goalnode:
            break

        # Pass obstacles_list to get_neighbors
        for adjnode in get_neighbors(current, obstacles_list):
            temp_d = dist[current] + 1
            if adjnode not in dist or temp_d < dist[adjnode]:
                dist[adjnode] = temp_d
                f_cost = temp_d + heuristic(adjnode, goalnode)
                parent[adjnode] = current
                heapq.heappush(pq, (f_cost, adjnode))
    return dist, parent, visited_order

def shortestpath(parent, startnode, goalnode):
    """Reconstructs the shortest path from the parent map."""
    path = []
    current = goalnode
    while current is not None:
        path.append(current)
        current = parent.get(current)
    path.reverse()
    return path

# ----------------- GUI -----------------

class RobotGridGUI:
    def __init__(self, master):
        self.master = master
        master.title("Interactive Robot Pathfinding (A* Animation)")

        self.cell_size = 80  # Increased size for better visibility
        self.animation_delay = 300 # milliseconds delay between steps

        # --- State variables for Pathfinding ---
        self.start_node = (0, 0)
        self.goal_node = (4, 4)
        self.obstacles = [(1,2), (2,2), (3,1)] # Default obstacles
        
        # --- GUI State ---
        self.path = []
        self.path_index = 0
        self.robot_radius = 25
        self.robot_id = None
        self.current_mode = tk.StringVar(value="START") # For radio buttons

        # Canvas setup
        self.canvas = tk.Canvas(master, width=COLS*self.cell_size, height=ROWS*self.cell_size, bg="#f8f9fa", highlightthickness=0)
        self.canvas.pack(pady=10, padx=10)
        
        # Bind mouse click event to canvas
        self.canvas.bind("<Button-1>", self.handle_click)

        # --- Control Frame for Mode Selection and Actions ---
        control_frame = tk.Frame(master)
        control_frame.pack(pady=10)

        # Mode Selection Radio Buttons
        mode_frame = tk.LabelFrame(control_frame, text="Setup Mode", padx=10, pady=5, font=("Arial", 10, "bold"))
        mode_frame.pack(side=tk.LEFT, padx=10)

        modes = [
            ("Set Start (Green)", "START"),
            ("Set Goal (Blue)", "GOAL"),
            ("Toggle Obstacle (Black)", "OBSTACLE")
        ]
        for text, mode in modes:
            tk.Radiobutton(mode_frame, text=text, variable=self.current_mode, value=mode, command=self.update_message, font=("Arial", 10)).pack(anchor=tk.W)
        
        # Action Buttons
        action_frame = tk.Frame(control_frame)
        action_frame.pack(side=tk.RIGHT, padx=10)

        self.solve_btn = tk.Button(action_frame, text="Solve Path (A*)", command=self.solve_path, font=("Arial",12,"bold"), bg="#4CAF50", fg="white", activebackground="#388E3C", relief=tk.RAISED)
        self.solve_btn.pack(pady=5)

        self.reset_btn = tk.Button(action_frame, text="Reset Grid", command=self.reset_state, font=("Arial",12), bg="#007bff", fg="white", activebackground="#0056b3", relief=tk.RAISED)
        self.reset_btn.pack(pady=5)

        # Message Label
        self.message_label = tk.Label(master, text="", font=("Arial", 10, "bold"), fg="#333")
        self.message_label.pack(pady=5)

        # Initial drawing
        self.draw_grid()
        self.draw_robot(self.start_node)
        self.update_message() # Display initial message


    def update_message(self):
        """Updates the status message based on the current mode."""
        mode = self.current_mode.get()
        if mode == "START":
            msg = "Current Mode: Setting Start Node (Green). Click a cell to place the robot."
        elif mode == "GOAL":
            msg = "Current Mode: Setting Goal Node (Blue). Click a cell to set the destination."
        elif mode == "OBSTACLE":
            msg = "Current Mode: Toggling Obstacle (Black). Click a cell to add/remove blockades."
        self.message_label.config(text=msg)

    def handle_click(self, event):
        """Processes mouse clicks on the canvas to set start, goal, or obstacles."""
        col = event.x // self.cell_size
        row = event.y // self.cell_size

        if not (0 <= row < ROWS and 0 <= col < COLS):
            return

        node = (row, col)
        mode = self.current_mode.get()

        self.path = [] # Clear previous path whenever grid state changes
        
        # Prevent setting Start/Goal on the same node
        if node == self.start_node and mode != "START": return
        if node == self.goal_node and mode != "GOAL": return

        # 1. Handle the click based on mode
        if mode == "START":
            # Prevent placing start on goal or an obstacle
            if node != self.goal_node and node not in self.obstacles:
                self.start_node = node
                self.draw_robot(self.start_node)
        elif mode == "GOAL":
            # Prevent placing goal on start or an obstacle
            if node != self.start_node and node not in self.obstacles:
                self.goal_node = node
        elif mode == "OBSTACLE":
            if node in self.obstacles:
                self.obstacles.remove(node)
            else:
                # Prevent placing obstacle on Start or Goal
                if node != self.start_node and node != self.goal_node:
                    self.obstacles.append(node)

        # 2. Redraw the grid to reflect changes
        self.draw_grid()
        self.draw_robot(self.start_node)
        
        # Ensure Solve button is ready
        self.solve_btn.config(state=tk.NORMAL, text="Solve Path (A*)", bg="#4CAF50")


    def draw_grid(self, path_to_highlight=None):
        """Draws the grid using instance variables for state."""
        self.canvas.delete("grid_elements")

        if path_to_highlight is None:
            path_to_highlight = []

        for i in range(ROWS):
            for j in range(COLS):
                x0, y0 = j*self.cell_size, i*self.cell_size
                x1, y1 = x0+self.cell_size, y0+self.cell_size

                color = "#fefefe" # Default background
                text = ""

                if (i,j) in self.obstacles:
                    color = "#343a40" # Obstacle (Dark Gray)
                    text = "X"
                elif (i,j) == self.start_node:
                    color = "#d4edda" # Start (Light Green)
                    text = "START"
                elif (i,j) == self.goal_node:
                    color = "#cfe2ff" # Goal (Light Blue)
                    text = "GOAL"

                # Highlight the path segment being animated (excluding Start/Goal)
                if (i, j) in path_to_highlight and (i,j) != self.start_node and (i,j) != self.goal_node:
                    color = "#ffebcc" # Path (Light Orange/Yellow)

                # Draw the cell rectangle
                self.canvas.create_rectangle(x0, y0, x1, y1, fill=color, outline="#cccccc", tags="grid_elements")

                # Draw text label
                if text:
                     self.canvas.create_text(x0 + self.cell_size//2, y0 + self.cell_size//2, text=text, font=("Arial", 10), fill="#333", tags="grid_elements")


    def draw_robot(self, node):
        """Creates or updates the robot's visual representation."""
        r, c = node
        cell_center_x = c * self.cell_size + self.cell_size // 2
        cell_center_y = r * self.cell_size + self.cell_size // 2

        if self.robot_id:
            # If robot exists, just move it to the new position
            self.canvas.coords(
                self.robot_id,
                cell_center_x - self.robot_radius,
                cell_center_y - self.robot_radius,
                cell_center_x + self.robot_radius,
                cell_center_y + self.robot_radius
            )
        else:
            # Create the robot (Red Circle)
            self.robot_id = self.canvas.create_oval(
                cell_center_x - self.robot_radius,
                cell_center_y - self.robot_radius,
                cell_center_x + self.robot_radius,
                cell_center_y + self.robot_radius,
                fill="#d9534f",      # Red
                outline="#c9302c",
                width=2,
                tags="robot_marker"
            )

        # Ensure the robot is always on top of the grid
        self.canvas.tag_raise("robot_marker")


    def reset_state(self):
        """Resets the state, grid, and robot position to default values."""
        self.start_node = (0, 0)
        self.goal_node = (4, 4)
        self.obstacles = [(1,2), (2,2), (3,1)]
        self.path = []
        self.path_index = 0
        self.draw_grid()
        self.draw_robot(self.start_node)
        self.solve_btn.config(state=tk.NORMAL, text="Solve Path (A*)", bg="#4CAF50")
        self.current_mode.set("START")
        self.update_message()


    def solve_path(self):
        """Calculates the path using current state and starts the animation."""
        self.solve_btn.config(state=tk.DISABLED, text="Calculating...")
        
        # Check if Start or Goal are now obstructed
        if self.start_node in self.obstacles or self.goal_node in self.obstacles:
            self.solve_btn.config(text="Start/Goal is blocked!", bg="#dc3545", state=tk.NORMAL)
            return

        # Recalculate A* using current state
        _, parent, _ = a_star(self.start_node, self.goal_node, self.obstacles)
        self.path = shortestpath(parent, self.start_node, self.goal_node)
        self.path_index = 0

        if self.path and self.path[-1] == self.goal_node:
            self.solve_btn.config(text="Path Found! Animating...", bg="#ffc107", fg="#333")
            self.animate_path()
        else:
            # Path not possible
            self.solve_btn.config(text="No Path Found!", bg="#f0ad4e", state=tk.NORMAL)
            print("No path possible.")


    def animate_path(self):
        """Moves the robot and highlights the path step-by-step."""
        if self.path_index < len(self.path):
            current_cell = self.path[self.path_index]

            # 1. Update the grid highlight: highlight all cells visited so far
            self.draw_grid(self.path[:self.path_index + 1])

            # 2. Move the robot to the current cell
            self.draw_robot(current_cell)

            self.path_index += 1

            # Schedule the next step
            self.master.after(self.animation_delay, self.animate_path)
        else:
            # Animation complete
            self.solve_btn.config(state=tk.NORMAL, text="Solved!", bg="#5cb85c", fg="white")
            # Final highlight of the whole path
            self.draw_grid(self.path)
            self.canvas.tag_raise("robot_marker")
            print(f"Shortest path found: {self.path}")

if __name__ == "__main__":
    root = tk.Tk()
    gui = RobotGridGUI(root)
    root.mainloop()
