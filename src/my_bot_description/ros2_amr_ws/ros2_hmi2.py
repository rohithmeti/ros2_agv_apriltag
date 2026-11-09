#!/usr/bin/env python3
import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import json

# --- COLORS ---
DARK_BG = (10, 10, 15)
PANEL_BG = (30, 30, 40)
GREEN = (0, 255, 0)
DARK_GREEN = (0, 100, 0)
BRIGHT_GREEN = (50, 255, 50)
RED = (255, 0, 0)
DARK_RED = (100, 0, 0)
BRIGHT_RED = (255, 50, 50)
BLUE = (0, 150, 255)
CYAN = (0, 255, 255)
WHITE = (255, 255, 255)
GREY = (100, 100, 100)
WARNING_ORANGE = (255, 140, 0)

class UnifiedPLCNode(Node):
    def __init__(self):
        super().__init__('unified_plc_scada')
        
        # --- ROS PUBLISHERS ---
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- ROS SUBSCRIBERS ---
        self.create_subscription(Image, '/overhead_camera/image_raw', self.cb_overhead, 10)
        self.create_subscription(Image, '/camera/image_raw', self.cb_front_cam, 10)
        self.create_subscription(LaserScan, '/scan', self.cb_lidar, 10)
        
        self.bridge = CvBridge()

        # --- PLC MEMORY ---
        self.sys_on = False
        self.speed_val = 0.5    
        
        # --- PID MEMORY (The Snake Fix) ---
        self.pid_error = 0.0
        self.prev_error = 0.0  # Remember last error to calculate D
        
        self.line_detected = False
        self.obstacle_detected = False
        self.min_lidar_dist = 99.0
        self.active_rung = 0    
        
        # --- VISUAL BUFFERS ---
        self.surf_overhead = None
        self.surf_front = None
        self.lidar_ranges = []
        self.tracking_enabled = False

        self.get_logger().info("Unified PLC SCADA System Started (PD Control Active).")

    # ================= ROS CALLBACKS =================

    def cb_overhead(self, msg):
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            if self.tracking_enabled:
                mask = (img[:,:,2] > 100) & (img[:,:,0] < 100) & (img[:,:,1] < 150)
                mask_u8 = mask.astype(np.uint8) * 255
                contours, _ = cv2.findContours(mask_u8, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    c = max(contours, key=cv2.contourArea)
                    if cv2.contourArea(c) > 20:
                        x, y, w, h = cv2.boundingRect(c)
                        pad = 20
                        cv2.rectangle(img, (max(0, x-pad), max(0, y-pad)), 
                                      (min(msg.width, x+w+pad), min(msg.height, y+h+pad)), 
                                      (0, 255, 0), 4)
                        cv2.putText(img, "LOCKED", (x, y-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            surf = pygame.image.frombuffer(img.tobytes(), (msg.width, msg.height), 'RGB')
            self.surf_overhead = pygame.transform.scale(surf, (400, 300))
        except: pass

    def cb_front_cam(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, _ = cv_img.shape
            roi = cv_img[int(h*0.8):h, 0:w]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, mask = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                self.line_detected = True
                self.pid_error = cx - (w / 2)
                cv2.circle(cv_img, (cx, int(h*0.9)), 10, (0, 255, 0), -1)
                cv2.line(cv_img, (int(w/2), int(h*0.8)), (int(w/2), h), (255, 0, 0), 2)
            else:
                self.line_detected = False
            img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            surf = pygame.image.frombuffer(img_rgb.tobytes(), (w, h), 'RGB')
            self.surf_front = pygame.transform.scale(surf, (300, 225))
        except: pass

    def cb_lidar(self, msg):
        self.lidar_ranges = msg.ranges
        n_ranges = len(msg.ranges)
        mid_idx = n_ranges // 2
        cone_width = 40 
        start_idx = max(0, mid_idx - cone_width)
        end_idx = min(n_ranges, mid_idx + cone_width)
        front_cone = msg.ranges[start_idx : end_idx]
        valid_ranges = [r for r in front_cone if 0.05 < r < 10.0]
        
        if valid_ranges:
            self.min_lidar_dist = min(valid_ranges)
        else:
            self.min_lidar_dist = 99.0 
            
        if self.min_lidar_dist < 0.65:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    # ================= PLC LOGIC LOOP =================
    
    def run_plc_cycle(self):
        twist = Twist()
        
        # --- SAFETY CHECKS ---
        if not self.sys_on:
            self.active_rung = 0 
            self.prev_error = 0.0 # Reset memory
            self.pub_vel.publish(twist)
            return

        if self.obstacle_detected:
            self.active_rung = 1
            self.prev_error = 0.0 # Reset memory
            self.pub_vel.publish(twist) 
            return

        if not self.line_detected:
            self.active_rung = 2
            self.prev_error = 0.0 # Reset memory
            self.pub_vel.publish(twist) 
            return

        # --- PD CONTROLLER (STABILIZED) ---
        # 1. Base Speed
        base_speed = 0.4 * self.speed_val 
        
        # 2. Gains
        # Kp: How hard to turn based on current error
        # Kd: How hard to resist the turn (Dampening)
        kp = 0.0035 
        kd = 0.0025  # The Anti-Snake Factor
        
        # 3. Calculate Derivative (Rate of change)
        error_rate = self.pid_error - self.prev_error
        
        # 4. Calculate Turn Command (P + D)
        turn_cmd = (-float(self.pid_error) * kp) + (-float(error_rate) * kd)
        
        # 5. Save history
        self.prev_error = self.pid_error
        
        # 6. Dynamic Speed & Anti-Windup
        if abs(self.pid_error) > 50:
            base_speed *= 0.6 # Slow down in turns
            
        # --- RUNG SELECTION ---
        if self.pid_error < -80: 
            self.active_rung = 4   # Left
        elif self.pid_error > 80: 
            self.active_rung = 3   # Right
        else: 
            self.active_rung = 5   # Forward
            
        twist.linear.x = base_speed
        twist.angular.z = turn_cmd
        self.pub_vel.publish(twist)


# ================= VISUALIZATION HELPERS =================

def draw_improved_lidar(screen, node, x, y, size):
    """Draws a Hi-Tech Radar Scope with Rays"""
    center_x = x + size // 2
    center_y = y + size - 20
    radius = size // 2 - 10
    
    # Background Scope
    pygame.draw.circle(screen, (0, 10, 0), (center_x, center_y), radius)
    pygame.draw.circle(screen, GREEN, (center_x, center_y), radius, 2)
    
    # Range Rings
    pygame.draw.circle(screen, (0, 50, 0), (center_x, center_y), int(radius * 0.33), 1)
    pygame.draw.circle(screen, (0, 50, 0), (center_x, center_y), int(radius * 0.66), 1)
    
    # Safety Zone Ring
    safe_px = int(0.65 * (radius / 3.0)) 
    pygame.draw.circle(screen, (100, 0, 0), (center_x, center_y), safe_px, 1)

    if not node.lidar_ranges: 
        screen.blit(font_sys.render("NO SENSOR DATA", True, GREY), (x+50, y+100))
        return
        
    angle_increment = 2 * math.pi / len(node.lidar_ranges)
    
    # Draw Rays
    for i, r in enumerate(node.lidar_ranges):
        if i % 2 != 0: continue
        if r < 0.1 or r > 5.0: continue 
        
        angle = (i * angle_increment) + (math.pi / 2)
        scale = radius / 3.0 
        dist_px = int(r * scale)
        if dist_px > radius: dist_px = radius
        
        end_x = center_x + int(math.cos(angle) * dist_px)
        end_y = center_y + int(math.sin(angle) * dist_px)
        
        if r < 0.65: # Obstacle Color
            color = RED 
            pygame.draw.circle(screen, RED, (end_x, end_y), 3)
        else:
            color = DARK_GREEN
            
        pygame.draw.line(screen, color, (center_x, center_y), (end_x, end_y), 1)

    # Text Overlay
    font = pygame.font.SysFont("Arial", 16)
    dist_txt = f"FRONT: {node.min_lidar_dist:.2f}m"
    screen.blit(font.render(dist_txt, True, WHITE), (x + 10, y + 10))
    
    if node.obstacle_detected:
        if (pygame.time.get_ticks() // 500) % 2 == 0:
            warn_surf = font.render("!! OBSTACLE DETECTED !!", True, WHITE)
            pygame.draw.rect(screen, RED, (x + size//2 - 90, y + size//2 - 10, 180, 25))
            screen.blit(warn_surf, (x + size//2 - 80, y + size//2 - 5))


def draw_ladder(screen, x, y, active_rung, speed_val):
    pygame.draw.line(screen, GREY, (x, y), (x, y+270), 6)
    pygame.draw.line(screen, GREY, (x+300, y), (x+300, y+270), 6)
    
    font = pygame.font.SysFont("Consolas", 18, bold=True)
    
    states = [
        (1, "SAFETY STOP (LIDAR)", BRIGHT_RED),
        (2, "LOST LINE STOP", WARNING_ORANGE),
        (3, "TURN RIGHT", BRIGHT_GREEN), 
        (4, "TURN LEFT", BRIGHT_GREEN),  
        (5, "PID FORWARD", BRIGHT_GREEN)
    ]
    
    curr_y = y + 40
    for r_id, txt, col in states:
        is_active = (active_rung == r_id)
        line_col = col if is_active else (60, 60, 70)
        text_col = col if is_active else (80, 80, 90)
        
        pygame.draw.line(screen, line_col, (x, curr_y), (x+300, curr_y), 4)
        symbol = "[X]" if is_active else "[ ]"
        screen.blit(font.render(f"{symbol} {txt}", True, text_col), (x+20, curr_y-25))
        pygame.draw.circle(screen, line_col, (x+280, curr_y), 8, 2)
        if is_active: pygame.draw.circle(screen, line_col, (x+280, curr_y), 4)
        curr_y += 50

    is_moving = (active_rung >= 3)
    spd_col = CYAN if is_moving else (60, 60, 70)
    pygame.draw.line(screen, spd_col, (x, curr_y), (x+300, curr_y), 4)
    screen.blit(font.render(f"SPEED PWM: {int(speed_val*100)}%", True, spd_col), (x+20, curr_y-25))
    pygame.draw.circle(screen, spd_col, (x+280, curr_y), 8, 2)
    if is_moving: pygame.draw.circle(screen, spd_col, (x+280, curr_y), 4)


# ================= MAIN APP =================

font_sys = None 

def main():
    global font_sys
    rclpy.init()
    node = UnifiedPLCNode()
    
    pygame.init()
    WIDTH, HEIGHT = 1300, 850
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("UNIFIED PLC SCADA SYSTEM [LIDAR + VISION]")
    clock = pygame.time.Clock()
    
    font_head = pygame.font.SysFont("Arial", 32, bold=True)
    font_sub = pygame.font.SysFont("Arial", 20, bold=True)
    font_sys = font_sub
    
    slider_rect = pygame.Rect(50, 700, 300, 20)
    btn_start = pygame.Rect(50, 750, 140, 50)
    btn_stop = pygame.Rect(210, 750, 140, 50)
    btn_track = pygame.Rect(950, 750, 200, 50)

    running = True
    while running:
        rclpy.spin_once(node, timeout_sec=0)
        node.run_plc_cycle() 
        
        screen.fill(DARK_BG)
        
        # --- HEADER ---
        pygame.draw.rect(screen, (10, 10, 15), (0,0,WIDTH,80))
        title = "PLC ROBOT SUPERVISOR: "
        if not node.sys_on: title += "OFFLINE"
        elif node.obstacle_detected: title += "EMERGENCY STOP (OBSTACLE)"
        else: title += "ONLINE (RUNNING)"
        
        head_col = RED if node.obstacle_detected else (GREEN if node.sys_on else GREY)
        screen.blit(font_head.render(title, True, head_col), (50, 25))

        # === COLUMN 1: CONTROLS ===
        screen.blit(font_sub.render(f"PID Vision Error: {node.pid_error:.1f}", True, BLUE), (50, 100))
        
        draw_ladder(screen, 50, 160, node.active_rung, node.speed_val)
        
        screen.blit(font_sub.render("MOTOR SPEED SETPOINT", True, WHITE), (50, 670))
        pygame.draw.rect(screen, GREY, slider_rect, border_radius=5)
        fill_rect = pygame.Rect(slider_rect.x, slider_rect.y, int(slider_rect.width * node.speed_val), slider_rect.height)
        pygame.draw.rect(screen, BLUE, fill_rect, border_radius=5)
        knob_x = slider_rect.x + int(slider_rect.width * node.speed_val)
        pygame.draw.circle(screen, WHITE, (knob_x, slider_rect.centery), 10)
        
        pygame.draw.rect(screen, GREEN, btn_start, border_radius=8)
        screen.blit(font_sub.render("START", True, WHITE), (btn_start.x+40, btn_start.y+15))
        pygame.draw.rect(screen, RED, btn_stop, border_radius=8)
        screen.blit(font_sub.render("STOP", True, WHITE), (btn_stop.x+45, btn_stop.y+15))

        # === COLUMN 2: SENSORS ===
        pygame.draw.rect(screen, GREY, (400, 100, 304, 229), 2)
        if node.surf_front: screen.blit(node.surf_front, (402, 102))
        else: screen.blit(font_sub.render("NO FRONT CAM", True, GREY), (450, 200))
        screen.blit(font_sub.render("ROBOT EYE", True, WHITE), (400, 80))

        pygame.draw.rect(screen, GREY, (400, 380, 300, 300), 2)
        draw_improved_lidar(screen, node, 400, 380, 300)
        screen.blit(font_sub.render("LIDAR RADAR", True, WHITE), (400, 360))

        # === COLUMN 3: MAP ===
        map_rect = (750, 100, 400, 300)
        pygame.draw.rect(screen, GREY, (map_rect[0]-2, map_rect[1]-2, 404, 304), 2)
        if node.surf_overhead:
            screen.blit(node.surf_overhead, (map_rect[0], map_rect[1]))
            if (pygame.time.get_ticks() // 500) % 2 == 0:
                pygame.draw.circle(screen, RED, (map_rect[0]+20, map_rect[1]+20), 6)
            screen.blit(font_sub.render("DRONE VIEW", True, WHITE), (map_rect[0]+35, map_rect[1]+10))
        else:
             screen.blit(font_sub.render("CONNECTING SAT...", True, GREY), (850, 250))
             
        track_col = BLUE if node.tracking_enabled else GREY
        pygame.draw.rect(screen, track_col, btn_track, border_radius=8)
        txt = "TRACK: ON" if node.tracking_enabled else "TRACK: OFF"
        screen.blit(font_sub.render(txt, True, WHITE), (btn_track.x+45, btn_track.y+15))

        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos
                if event.button == 1:
                    if btn_start.collidepoint(mx, my): node.sys_on = True
                    if btn_stop.collidepoint(mx, my): node.sys_on = False
                    if btn_track.collidepoint(mx, my): node.tracking_enabled = not node.tracking_enabled
                    if slider_rect.collidepoint(mx, my):
                        val = (mx - slider_rect.x) / slider_rect.width
                        node.speed_val = max(0.0, min(1.0, val))
            if event.type == pygame.MOUSEMOTION and event.buttons[0]:
                mx, my = event.pos
                if slider_rect.collidepoint(mx, my):
                    val = (mx - slider_rect.x) / slider_rect.width
                    node.speed_val = max(0.0, min(1.0, val))

        pygame.display.flip()
        clock.tick(30)

    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == "__main__":
    main()
