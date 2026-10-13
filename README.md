
# warehouse-robot
import tkinter as tk
from tkinter import messagebox
import heapq
import time


# ============================================================
# WAREHOUSE ROBOT PATH PLANNING
# A* SEARCH ALGORITHM
# ============================================================

ROWS = 10
COLS = 15
CELL_SIZE = 50

START = (0, 0)
GOAL = (9, 14)


# 0 = free
# 1 = obstacle

warehouse = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0],
    [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0],
    [0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0],
    [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
]


# ============================================================
# A* HEURISTIC
# ============================================================

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# ============================================================
# CHECK VALID CELL
# ============================================================

def is_valid(row, col):

    return (
        0 <= row < ROWS
        and 0 <= col < COLS
        and warehouse[row][col] == 0
    )


# ============================================================
# A* ALGORITHM
# ============================================================

def a_star(start, goal):

    open_list = []

    heapq.heappush(
        open_list,
        (0, start)
    )

    g_cost = {
        start: 0
    }

    parent = {
        start: None
    }

    explored = set()

    exploration_order = []

    directions = [
        (-1, 0),   # Up
        (1, 0),    # Down
        (0, -1),   # Left
        (0, 1)     # Right
    ]

    while open_list:

        f_cost, current = heapq.heappop(open_list)

        if current in explored:
            continue

        explored.add(current)
        exploration_order.append(current)

        # Goal reached
        if current == goal:

            path = []

            while current is not None:

                path.append(current)

                current = parent[current]

            path.reverse()

            return path, exploration_order

        row, col = current

        # Explore neighbours
        for dr, dc in directions:

            new_row = row + dr
            new_col = col + dc

            if not is_valid(new_row, new_col):
                continue

            neighbor = (new_row, new_col)

            new_g = g_cost[current] + 1

            if (
                neighbor not in g_cost
                or new_g < g_cost[neighbor]
            ):

                g_cost[neighbor] = new_g

                h_cost = heuristic(
                    neighbor,
                    goal
                )

                f_cost = new_g + h_cost

                heapq.heappush(
                    open_list,
                    (f_cost, neighbor)
                )

                parent[neighbor] = current

    return None, exploration_order


# ============================================================
# GUI WINDOW
# ============================================================

root = tk.Tk()

root.title(
    "Warehouse Robot Path Planning - A*"
)

root.configure(
    bg="#eeeeee"
)

root.resizable(False, False)


# ============================================================
# TITLE
# ============================================================

title_label = tk.Label(
    root,
    text="WAREHOUSE ROBOT PATH PLANNER",
    font=("Arial", 24, "bold"),
    bg="#eeeeee"
)

title_label.pack(pady=(15, 5))


subtitle_label = tk.Label(
    root,
    text="Shortest Path Planning using A* Algorithm",
    font=("Arial", 12),
    bg="#eeeeee"
)

subtitle_label.pack(
    pady=(0, 12)
)


# ============================================================
# CANVAS
# ============================================================

canvas = tk.Canvas(
    root,
    width=COLS * CELL_SIZE,
    height=ROWS * CELL_SIZE,
    bg="white",
    highlightthickness=2,
    highlightbackground="gray"
)

canvas.pack(
    padx=15
)


# ============================================================
# DRAW GRID
# ============================================================

def draw_grid():

    canvas.delete("all")

    for row in range(ROWS):

        for col in range(COLS):

            x1 = col * CELL_SIZE
            y1 = row * CELL_SIZE

            x2 = x1 + CELL_SIZE
            y2 = y1 + CELL_SIZE

            # Obstacle
            if warehouse[row][col] == 1:

                canvas.create_rectangle(
                    x1,
                    y1,
                    x2,
                    y2,
                    fill="black",
                    outline="gray"
                )

            else:

                canvas.create_rectangle(
                    x1,
                    y1,
                    x2,
                    y2,
                    fill="white",
                    outline="#aaaaaa"
                )

    draw_start_goal()


# ============================================================
# DRAW START AND DESTINATION
# ============================================================

def draw_start_goal():

    # Start
    sr, sc = START

    canvas.create_rectangle(
        sc * CELL_SIZE,
        sr * CELL_SIZE,
        (sc + 1) * CELL_SIZE,
        (sr + 1) * CELL_SIZE,
        fill="#2ecc71",
        outline="gray"
    )

    canvas.create_text(
        sc * CELL_SIZE + CELL_SIZE / 2,
        sr * CELL_SIZE + CELL_SIZE / 2,
        text="🤖",
        font=("Arial", 24)
    )

    # Destination
    gr, gc = GOAL

    canvas.create_rectangle(
        gc * CELL_SIZE,
        gr * CELL_SIZE,
        (gc + 1) * CELL_SIZE,
        (gr + 1) * CELL_SIZE,
        fill="#f39c12",
        outline="gray"
    )

    canvas.create_text(
        gc * CELL_SIZE + CELL_SIZE / 2,
        gr * CELL_SIZE + CELL_SIZE / 2,
        text="📦",
        font=("Arial", 22)
    )


# ============================================================
# CLICK TO ADD OBSTACLE
# ============================================================

def add_obstacle(event):

    col = event.x // CELL_SIZE
    row = event.y // CELL_SIZE

    if not (
        0 <= row < ROWS
        and 0 <= col < COLS
    ):
        return

    position = (row, col)

    # Don't allow start or destination to become obstacles
    if position == START or position == GOAL:
        return

    warehouse[row][col] = 1

    draw_grid()

    result_label.config(
        text="Obstacle added. Click FIND SHORTEST PATH."
    )


# ============================================================
# RIGHT CLICK TO REMOVE OBSTACLE
# ============================================================

def remove_obstacle(event):

    col = event.x // CELL_SIZE
    row = event.y // CELL_SIZE

    if not (
        0 <= row < ROWS
        and 0 <= col < COLS
    ):
        return

    warehouse[row][col] = 0

    draw_grid()

    result_label.config(
        text="Obstacle removed."
    )


# ============================================================
# DRAW EXPLORED CELL
# ============================================================

def draw_explored(position):

    row, col = position

    if position == START or position == GOAL:
        return

    canvas.create_rectangle(
        col * CELL_SIZE,
        row * CELL_SIZE,
        (col + 1) * CELL_SIZE,
        (row + 1) * CELL_SIZE,
        fill="#87CEEB",
        outline="#aaaaaa"
    )


# ============================================================
# DRAW PATH CELL
# ============================================================

def draw_path(position):

    row, col = position

    if position == START or position == GOAL:
        return

    canvas.create_rectangle(
        col * CELL_SIZE,
        row * CELL_SIZE,
        (col + 1) * CELL_SIZE,
        (row + 1) * CELL_SIZE,
        fill="#FFD700",
        outline="#aaaaaa"
    )


# ============================================================
# DRAW ROBOT
# ============================================================

def draw_robot(position):

    row, col = position

    canvas.create_oval(
        col * CELL_SIZE + 10,
        row * CELL_SIZE + 10,
        col * CELL_SIZE + CELL_SIZE - 10,
        row * CELL_SIZE + CELL_SIZE - 10,
        fill="#3498db",
        outline="black",
        width=2
    )

    canvas.create_text(
        col * CELL_SIZE + CELL_SIZE / 2,
        row * CELL_SIZE + CELL_SIZE / 2,
        text="🤖",
        font=("Arial", 20)
    )


# ============================================================
# FIND SHORTEST PATH
# ============================================================

def find_path():

    # Clear previous result
    draw_grid()

    result_label.config(
        text="A* is searching..."
    )

    root.update()

    start_time = time.perf_counter()

    path, exploration_order = a_star(
        START,
        GOAL
    )

    end_time = time.perf_counter()

    execution_time = (
        end_time - start_time
    ) * 1000

    if path is None:

        messagebox.showerror(
            "No Path",
            "No feasible path exists!"
        )

        result_label.config(
            text="No feasible path found."
        )

        return

    distance = len(path) - 1
    nodes = len(exploration_order)

    # Save statistics
    result_label.config(
        text=(
            f"Algorithm: A*   |   "
            f"Distance: {distance} units   |   "
            f"Nodes Explored: {nodes}   |   "
            f"Time: {execution_time:.3f} ms"
        )
    )

    # Animate exploration first
    animate_exploration(
        exploration_order,
        path,
        distance,
        nodes,
        execution_time
    )


# ============================================================
# ANIMATE A* EXPLORATION
# ============================================================

def animate_exploration(
    exploration_order,
    path,
    distance,
    nodes,
    execution_time
):

    if not exploration_order:

        animate_path(
            path,
            0,
            distance,
            nodes,
            execution_time
        )

        return

    position = exploration_order[0]

    draw_explored(position)

    remaining = exploration_order[1:]

    root.after(
        18,
        lambda: animate_exploration(
            remaining,
            path,
            distance,
            nodes,
            execution_time
        )
    )


# ============================================================
# ANIMATE SHORTEST PATH
# ============================================================

def animate_path(
    path,
    index,
    distance,
    nodes,
    execution_time
):

    if index >= len(path):

        draw_start_goal()

        result_label.config(
            text=(
                f"Algorithm: A*   |   "
                f"Distance: {distance} units   |   "
                f"Nodes Explored: {nodes}   |   "
                f"Time: {execution_time:.3f} ms   |   "
                f"PATH FOUND!"
            )
        )

        return

    position = path[index]

    draw_path(position)

    draw_start_goal()

    draw_robot(position)

    root.after(
        100,
        lambda: animate_path(
            path,
            index + 1,
            distance,
            nodes,
            execution_time
        )
    )


# ============================================================
# RESET
# ============================================================

def reset():

    # Restore original obstacles
    original = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0],
        [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0],
        [0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    ]

    for r in range(ROWS):

        for c in range(COLS):

            warehouse[r][c] = original[r][c]

    draw_grid()

    result_label.config(
        text="Grid reset. Ready for a new path."
    )


# ============================================================
# BUTTONS
# ============================================================

button_frame = tk.Frame(
    root,
    bg="#eeeeee"
)

button_frame.pack(
    pady=12
)


find_button = tk.Button(
    button_frame,
    text="FIND SHORTEST PATH",
    command=find_path,
    font=("Arial", 12, "bold"),
    bg="#3498db",
    fg="white",
    padx=20,
    pady=10
)

find_button.pack(
    side=tk.LEFT,
    padx=8
)


reset_button = tk.Button(
    button_frame,
    text="RESET",
    command=reset,
    font=("Arial", 12, "bold"),
    padx=25,
    pady=10
)

reset_button.pack(
    side=tk.LEFT,
    padx=8
)


# ============================================================
# RESULT
# ============================================================

result_label = tk.Label(
    root,
    text="Click FIND SHORTEST PATH to start",
    font=("Arial", 12, "bold"),
    bg="#eeeeee"
)

result_label.pack(
    pady=(0, 8)
)


# ============================================================
# LEGEND
# ============================================================

legend = tk.Label(
    root,
    text="🟩 Start   🟧 Destination   ⬛ Obstacle   🟦 Explored   🟨 Shortest Path",
    font=("Arial", 10),
    bg="#eeeeee"
)

legend.pack(
    pady=(0, 12)
)


# ============================================================
# MOUSE CONTROLS
# ============================================================

canvas.bind(
    "<Button-1>",
    add_obstacle
)

canvas.bind(
    "<Button-3>",
    remove_obstacle
)


# ============================================================
# START APPLICATION
# ============================================================

draw_grid()

root.mainloop()