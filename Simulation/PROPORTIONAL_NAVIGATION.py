#!/usr/bin/env python3
"""
A Simple Proportional Navigation Simulation

Setup:
  1. Click on the canvas:
      - First click sets the rocket’s starting position (red circle)
      - Second click sets the target’s starting position (blue circle)
  2. Enter the following parameters:
      - Rocket Speed (px/s) and Rocket Angle (deg)
      - Target Speed (px/s) and Target Angle (deg)
      - PN Navigation Constant (dimensionless)
      - Maximum allowed Vertical Acceleration (px/s²)
  3. When "Start Simulation" is pressed:
      - A dashed line is drawn between the rocket and target showing their initial separation.
      - The target moves in a constant straight-line path while the rocket uses a PN law (with saturation) to intercept.
      - The trajectories are drawn (rocket: red, target: blue).
      - The current and maximum vertical acceleration commands are displayed.
      - When the rocket comes within 10 pixels of the target, a collision point is marked in green.
  4. Press "Reset Simulation" to clear the canvas and start over.
  
Note: The main window is expanded (maximized) when the app starts.
"""

import tkinter as tk
import math

class SimulationApp:
    def __init__(self, master):
        self.master = master
        master.title("Simple Proportional Navigation Simulation")

        # # Set up an expanded window (maximized)
        # try:
        #     master.state('zoomed')   # Works on Windows
        # except Exception:
        #     master.geometry("1200x800")  # Fallback for other systems

        # Canvas for drawing trajectories.
        self.canvas_width = 800
        self.canvas_height = 600
        self.canvas = tk.Canvas(master, width=self.canvas_width, height=self.canvas_height, bg="white")
        self.canvas.grid(row=0, column=0, columnspan=4, padx=5, pady=5)

        # Instruction label.
        self.info_label = tk.Label(master, text="Click: first sets Rocket position, second sets Target position.")
        self.info_label.grid(row=1, column=0, columnspan=4, sticky="w", padx=5)

        # Input fields for rocket and target parameters.
        # Row 2: Rocket parameters.
        tk.Label(master, text="Rocket Speed (px/s):").grid(row=2, column=0, sticky=tk.E)
        self.rocket_speed_entry = tk.Entry(master, width=10)
        self.rocket_speed_entry.insert(0, "25")
        self.rocket_speed_entry.grid(row=2, column=1, sticky=tk.W)

        tk.Label(master, text="Rocket Angle (deg):").grid(row=2, column=2, sticky=tk.E)
        self.rocket_angle_entry = tk.Entry(master, width=10)
        self.rocket_angle_entry.insert(0, "270")
        self.rocket_angle_entry.grid(row=2, column=3, sticky=tk.W)

        # Row 3: Target parameters.
        tk.Label(master, text="Target Speed (px/s):").grid(row=3, column=0, sticky=tk.E)
        self.target_speed_entry = tk.Entry(master, width=10)
        self.target_speed_entry.insert(0, "15")
        self.target_speed_entry.grid(row=3, column=1, sticky=tk.W)

        tk.Label(master, text="Target Angle (deg):").grid(row=3, column=2, sticky=tk.E)
        self.target_angle_entry = tk.Entry(master, width=10)
        self.target_angle_entry.insert(0, "0")
        self.target_angle_entry.grid(row=3, column=3, sticky=tk.W)

        # Row 4: PN Navigation constant and Maximum allowed vertical acceleration.
        tk.Label(master, text="PN Navigation Constant:").grid(row=4, column=0, sticky=tk.E)
        self.pn_constant_entry = tk.Entry(master, width=10)
        self.pn_constant_entry.insert(0, "3.0")
        self.pn_constant_entry.grid(row=4, column=1, sticky=tk.W)

        tk.Label(master, text="Max Vertical Acc (px/s²):").grid(row=4, column=2, sticky=tk.E)
        self.max_allowed_acc_entry = tk.Entry(master, width=10)
        self.max_allowed_acc_entry.insert(0, "4")
        self.max_allowed_acc_entry.grid(row=4, column=3, sticky=tk.W)

        # Row 5: Start and Reset buttons.
        self.start_button = tk.Button(master, text="Start Simulation", command=self.start_simulation)
        self.start_button.grid(row=5, column=0, columnspan=2, pady=5)
        self.reset_button = tk.Button(master, text="Reset Simulation", command=self.reset_simulation)
        self.reset_button.grid(row=5, column=2, columnspan=2, pady=5)

        # Row 6: Labels for displaying acceleration data.
        self.curr_acc_label = tk.Label(master, text="Current Vertical Acc: 0.0 px/s²")
        self.curr_acc_label.grid(row=6, column=0, columnspan=2, sticky="w", padx=5)
        self.max_acc_label = tk.Label(master, text="Max Vertical Acc: 0.0 px/s²")
        self.max_acc_label.grid(row=6, column=2, columnspan=2, sticky="w", padx=5)

        # Bind mouse click on canvas for positions.
        self.canvas.bind("<Button-1>", self.on_canvas_click)

        # Initialize state variables.
        self.reset_simulation_variables()

        # Simulation time step.
        self.dt = 0.02  # seconds

    def reset_simulation_variables(self):
        # Reset flags and simulation variables.
        self.rocket_set = False
        self.target_set = False
        self.rocket_start = None
        self.target_start = None
        self.simulation_running = False
        self.simulation_time = 0.0
        self.rocket_traj = []
        self.target_traj = []
        self.max_vertical_acc = 0.0  # maximum encountered vertical (lateral) acceleration
        self.initial_distance_line = None
        self.initial_distance_text = None

    def on_canvas_click(self, event):
        # First click sets rocket starting point.
        if not self.rocket_set:
            self.rocket_start = (event.x, event.y)
            self.rocket_set = True
            r = 4
            self.canvas.create_oval(event.x - r, event.y - r, event.x + r, event.y + r, fill="red")
            self.info_label.config(text="Rocket position set. Click to set Target position.")
        # Second click sets target starting point.
        elif not self.target_set:
            self.target_start = (event.x, event.y)
            self.target_set = True
            r = 4
            self.canvas.create_oval(event.x - r, event.y - r, event.x + r, event.y + r, fill="blue")
            self.info_label.config(text="Both positions set. Press 'Start Simulation' to begin.")
        else:
            # Both positions already set.
            pass

    def start_simulation(self):
        # Ensure both positions are set.
        if not (self.rocket_set and self.target_set):
            self.info_label.config(text="Please click to set both Rocket and Target positions!")
            return

        # Retrieve parameters from the entry fields.
        try:
            self.rocket_speed = float(self.rocket_speed_entry.get())
            self.rocket_angle = math.radians(float(self.rocket_angle_entry.get()))
            self.target_speed = float(self.target_speed_entry.get())
            self.target_angle = math.radians(float(self.target_angle_entry.get()))
            self.pn_constant = float(self.pn_constant_entry.get())
            self.max_allowed_acc = float(self.max_allowed_acc_entry.get())
        except ValueError:
            self.info_label.config(text="Invalid parameters! Please enter numeric values.")
            return

        # Initialize positions.
        self.rocket_pos = [float(self.rocket_start[0]), float(self.rocket_start[1])]
        self.target_pos = [float(self.target_start[0]), float(self.target_start[1])]

        # Represent initial distance: draw dashed line and display text.
        dx = self.target_pos[0] - self.rocket_pos[0]
        dy = self.target_pos[1] - self.rocket_pos[1]
        initial_distance = math.hypot(dx, dy)
        self.initial_distance_line = self.canvas.create_line(
            self.rocket_pos[0], self.rocket_pos[1],
            self.target_pos[0], self.target_pos[1],
            dash=(4, 2), fill="gray"
        )
        mid_x = (self.rocket_pos[0] + self.target_pos[0]) / 2
        mid_y = (self.rocket_pos[1] + self.target_pos[1]) / 2
        self.initial_distance_text = self.canvas.create_text(
            mid_x, mid_y - 10,
            text=f"Initial Distance: {initial_distance:.1f} px",
            fill="gray", font=("Arial", 10, "italic")
        )

        # Initialize velocity vectors.
        self.rocket_vel = [self.rocket_speed * math.cos(self.rocket_angle),
                           self.rocket_speed * math.sin(self.rocket_angle)]
        self.target_vel = [self.target_speed * math.cos(self.target_angle),
                           self.target_speed * math.sin(self.target_angle)]

        # Calculate initial Line-Of-Sight (LOS) angle.
        self.prev_LOS = math.atan2(dy, dx)

        # Initialize trajectory lists.
        self.rocket_traj = [tuple(self.rocket_pos)]
        self.target_traj = [tuple(self.target_pos)]

        self.simulation_running = True
        self.simulation_time = 0.0
        self.info_label.config(text="Simulation running...")

        # Disable Start button during simulation.
        self.start_button.config(state=tk.DISABLED)

        # Begin simulation loop.
        self.update_simulation()

    def update_simulation(self):
        if not self.simulation_running:
            return

        dt = self.dt
        self.simulation_time += dt

        # Update target position (straight-line motion).
        self.target_pos[0] += self.target_vel[0] * dt
        self.target_pos[1] += self.target_vel[1] * dt

        # Calculate current LOS angle.
        dx = self.target_pos[0] - self.rocket_pos[0]
        dy = self.target_pos[1] - self.rocket_pos[1]
        current_LOS = math.atan2(dy, dx)

        # Helper: compute difference between two angles.
        def angle_diff(a, b):
            diff = a - b
            while diff > math.pi:
                diff -= 2 * math.pi
            while diff < -math.pi:
                diff += 2 * math.pi
            return diff

        LOS_rate = angle_diff(current_LOS, self.prev_LOS) / dt

        # Compute relative velocity vector.
        rel_vel = [self.target_vel[0] - self.rocket_vel[0],
                   self.target_vel[1] - self.rocket_vel[1]]
        r = math.hypot(dx, dy)
        if r == 0:
            rhat = [0, 0]
        else:
            rhat = [dx / r, dy / r]

        # Define the closing velocity (positive when closing).
        closing_velocity = - (rel_vel[0] * rhat[0] + rel_vel[1] * rhat[1])
        
        # Compute the lateral (vertical) acceleration command using the PN law.
        a_command = self.pn_constant * closing_velocity * LOS_rate  # (px/s²)

        # Saturate the command to the maximum allowed vertical acceleration.
        if abs(a_command) > self.max_allowed_acc:
            a_command = self.max_allowed_acc * (1 if a_command > 0 else -1)

        current_vertical_acc = abs(a_command)
        # Update maximum encountered vertical acceleration display.
        # (For simplicity, we simply use the current value if it is higher.)
        try:
            # Extract current max value from label.
            current_max = float(self.max_acc_label.cget("text").split()[-2])
        except:
            current_max = 0.0
        max_encountered = max(current_vertical_acc, current_max)
        
        # Update display labels.
        self.curr_acc_label.config(text=f"Current Vertical Acc: {current_vertical_acc:.2f} px/s²")
        self.max_acc_label.config(text=f"Max Vertical Acc: {max_encountered:.2f} px/s²")

        # Determine turn rate (since speed is constant).
        turn_rate = a_command / self.rocket_speed

        # Update rocket’s heading angle.
        self.rocket_angle += turn_rate * dt

        # Update rocket velocity.
        self.rocket_vel[0] = self.rocket_speed * math.cos(self.rocket_angle)
        self.rocket_vel[1] = self.rocket_speed * math.sin(self.rocket_angle)

        # Update rocket position.
        self.rocket_pos[0] += self.rocket_vel[0] * dt
        self.rocket_pos[1] += self.rocket_vel[1] * dt

        # Append new positions to trajectories.
        self.rocket_traj.append((self.rocket_pos[0], self.rocket_pos[1]))
        self.target_traj.append((self.target_pos[0], self.target_pos[1]))

        # Draw line segments for trajectories.
        if len(self.rocket_traj) >= 2:
            self.canvas.create_line(self.rocket_traj[-2][0], self.rocket_traj[-2][1],
                                    self.rocket_traj[-1][0], self.rocket_traj[-1][1],
                                    fill="red")
        if len(self.target_traj) >= 2:
            self.canvas.create_line(self.target_traj[-2][0], self.target_traj[-2][1],
                                    self.target_traj[-1][0], self.target_traj[-1][1],
                                    fill="blue")

        # Update previous LOS.
        self.prev_LOS = current_LOS

        # Check termination conditions:
        # (1) Collision if distance r is less than 10 pixels.
        if r < 2:
            self.info_label.config(text="Impact! Rocket hit the target.")
            self.simulation_running = False
            # Mark the collision point with a green circle.
            r_collide = 6
            self.canvas.create_oval(self.rocket_pos[0] - r_collide, self.rocket_pos[1] - r_collide,
                                    self.rocket_pos[0] + r_collide, self.rocket_pos[1] + r_collide,
                                    fill="green")
            self.start_button.config(state=tk.NORMAL)
            return
        # (2) Stop simulation after 30 seconds.
        if self.simulation_time > 30:
            self.info_label.config(text="Simulation time exceeded. Stopping simulation.")
            self.simulation_running = False
            self.start_button.config(state=tk.NORMAL)
            return

        # Schedule next simulation update.
        self.master.after(int(dt * 1000), self.update_simulation)

    def reset_simulation(self):
        # Stop simulation and clear canvas and state variables.
        self.simulation_running = False
        self.canvas.delete("all")
        self.reset_simulation_variables()
        self.info_label.config(text="Click: first sets Rocket position, second sets Target position.")
        self.curr_acc_label.config(text="Current Vertical Acc: 0.0 px/s²")
        self.max_acc_label.config(text="Max Vertical Acc: 0.0 px/s²")
        self.start_button.config(state=tk.NORMAL)

if __name__ == "__main__":
    root = tk.Tk()
    app = SimulationApp(root)
    root.mainloop()