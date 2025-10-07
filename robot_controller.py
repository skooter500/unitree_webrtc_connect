import asyncio
import logging
import json
import sys
import tkinter as tk
from tkinter import ttk, messagebox, StringVar
import threading
import time
from functools import partial
import cv2
import numpy as np
from PIL import Image, ImageTk
from queue import Queue
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD
from aiortc import MediaStreamTrack

# Enable logging for debugging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Default camera frame size
DEFAULT_CAMERA_WIDTH = 640
DEFAULT_CAMERA_HEIGHT = 360

# Default lidar plot size
DEFAULT_LIDAR_WIDTH = 640
DEFAULT_LIDAR_HEIGHT = 480

class RobotController:
    def __init__(self, root):
        self.root = root
        self.root.title("Go2 Robot Controller")
        self.root.geometry("800x600")
        self.root.resizable(True, True)
        
        # Connection variables
        self.connection = None
        self.connection_method = tk.IntVar(value=2)  # Default to LocalSTA
        self.ip_address = tk.StringVar(value="192.168.8.181")
        self.serial_number = tk.StringVar(value="B42D1000P6IAA88B")
        self.username = tk.StringVar()
        self.password = tk.StringVar()
        self.is_connected = False
        self.connection_thread = None
        self.loop = None
        self.heartbeat_task = None
        self.connection_monitor_task = None
        self.shutdown_event = None
        
        # Movement variables
        self.speed = tk.DoubleVar(value=0.5)
        
        # Camera variables
        self.frame_queue = Queue()
        self.camera_enabled = tk.BooleanVar(value=False)
        self.camera_update_id = None
        self.last_frame = None
        
        # Lidar variables
        self.lidar_enabled = tk.BooleanVar(value=False)
        self.lidar_figure = None
        self.lidar_canvas = None
        self.lidar_ax = None
        self.lidar_points = None
        self.lidar_update_id = None
        self.lidar_scatter = None
        
        # Create UI
        self.create_ui()
        
    def create_ui(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Create notebook for tabbed interface
        notebook = ttk.Notebook(main_frame)
        notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Connection tab
        connection_frame = ttk.Frame(notebook, padding="10")
        notebook.add(connection_frame, text="Connection")
        self.create_connection_panel(connection_frame)
        
        # Movement tab
        movement_frame = ttk.Frame(notebook, padding="10")
        notebook.add(movement_frame, text="Movement")
        self.create_movement_panel(movement_frame)
        
        # Actions tab
        actions_frame = ttk.Frame(notebook, padding="10")
        notebook.add(actions_frame, text="Actions")
        self.create_actions_panel(actions_frame)
        
        # Camera tab
        camera_frame = ttk.Frame(notebook, padding="10")
        notebook.add(camera_frame, text="Camera")
        self.create_camera_panel(camera_frame)
        
        # Lidar tab
        lidar_frame = ttk.Frame(notebook, padding="10")
        notebook.add(lidar_frame, text="Lidar")
        self.create_lidar_panel(lidar_frame)
        
        # Status bar at the bottom
        self.status_var = tk.StringVar(value="Not connected")
        status_bar = ttk.Label(main_frame, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
    def create_connection_panel(self, parent):
        # Connection method frame
        method_frame = ttk.LabelFrame(parent, text="Connection Method", padding="10")
        method_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Radiobutton(method_frame, text="Local AP", variable=self.connection_method, 
                        value=1).grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        ttk.Radiobutton(method_frame, text="Local STA", variable=self.connection_method, 
                        value=2).grid(row=1, column=0, sticky=tk.W, padx=5, pady=2)
        ttk.Radiobutton(method_frame, text="Remote", variable=self.connection_method, 
                        value=3).grid(row=2, column=0, sticky=tk.W, padx=5, pady=2)
        
        # Connection details frame
        details_frame = ttk.LabelFrame(parent, text="Connection Details", padding="10")
        details_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # IP Address (for LocalSTA)
        ttk.Label(details_frame, text="IP Address:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        ttk.Entry(details_frame, textvariable=self.ip_address).grid(row=0, column=1, sticky=tk.EW, padx=5, pady=2)
        
        # Serial Number (for LocalSTA and Remote)
        ttk.Label(details_frame, text="Serial Number:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=2)
        ttk.Entry(details_frame, textvariable=self.serial_number).grid(row=1, column=1, sticky=tk.EW, padx=5, pady=2)
        
        # Username and Password (for Remote)
        ttk.Label(details_frame, text="Username:").grid(row=2, column=0, sticky=tk.W, padx=5, pady=2)
        ttk.Entry(details_frame, textvariable=self.username).grid(row=2, column=1, sticky=tk.EW, padx=5, pady=2)
        
        ttk.Label(details_frame, text="Password:").grid(row=3, column=0, sticky=tk.W, padx=5, pady=2)
        ttk.Entry(details_frame, textvariable=self.password, show="*").grid(row=3, column=1, sticky=tk.EW, padx=5, pady=2)
        
        # Connect button
        button_frame = ttk.Frame(parent)
        button_frame.pack(fill=tk.X, padx=5, pady=10)
        
        self.connect_button = ttk.Button(button_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(side=tk.RIGHT)
        
    def create_movement_panel(self, parent):
        # Speed control
        speed_frame = ttk.LabelFrame(parent, text="Speed Control", padding="10")
        speed_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(speed_frame, text="Speed:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        speed_scale = ttk.Scale(speed_frame, from_=0.1, to=1.0, orient=tk.HORIZONTAL, 
                               variable=self.speed, length=300)
        speed_scale.grid(row=0, column=1, sticky=tk.EW, padx=5, pady=2)
        ttk.Label(speed_frame, textvariable=self.speed).grid(row=0, column=2, sticky=tk.W, padx=5, pady=2)
        
        # Movement buttons
        movement_frame = ttk.LabelFrame(parent, text="Movement Controls", padding="10")
        movement_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Create a grid for directional buttons
        btn_frame = ttk.Frame(movement_frame)
        btn_frame.pack(pady=20)
        
        # Forward button
        forward_btn = ttk.Button(btn_frame, text="Forward", 
                                command=lambda: self.send_command("move_forward"))
        forward_btn.grid(row=0, column=1, padx=5, pady=5)
        
        # Left button
        left_btn = ttk.Button(btn_frame, text="Left", 
                             command=lambda: self.send_command("move_left"))
        left_btn.grid(row=1, column=0, padx=5, pady=5)
        
        # Stop button
        stop_btn = ttk.Button(btn_frame, text="Stop", 
                             command=lambda: self.send_command("stop_move"))
        stop_btn.grid(row=1, column=1, padx=5, pady=5)
        
        # Right button
        right_btn = ttk.Button(btn_frame, text="Right", 
                              command=lambda: self.send_command("move_right"))
        right_btn.grid(row=1, column=2, padx=5, pady=5)
        
        # Backward button
        backward_btn = ttk.Button(btn_frame, text="Backward", 
                                 command=lambda: self.send_command("move_backward"))
        backward_btn.grid(row=2, column=1, padx=5, pady=5)
        
        # Stand controls
        stand_frame = ttk.Frame(movement_frame)
        stand_frame.pack(pady=10)
        
        stand_up_btn = ttk.Button(stand_frame, text="Stand Up", 
                                 command=lambda: self.send_command("stand_up"))
        stand_up_btn.grid(row=0, column=0, padx=5, pady=5)
        
        stand_down_btn = ttk.Button(stand_frame, text="Stand Down", 
                                   command=lambda: self.send_command("stand_down"))
        stand_down_btn.grid(row=0, column=1, padx=5, pady=5)
        
    def create_actions_panel(self, parent):
        # Basic actions
        basic_frame = ttk.LabelFrame(parent, text="Basic Actions", padding="10")
        basic_frame.pack(fill=tk.X, padx=5, pady=5)
        
        hello_btn = ttk.Button(basic_frame, text="Hello", 
                              command=lambda: self.send_command("hello"))
        hello_btn.grid(row=0, column=0, padx=5, pady=5)
        
        stretch_btn = ttk.Button(basic_frame, text="Stretch", 
                                command=lambda: self.send_command("stretch"))
        stretch_btn.grid(row=0, column=1, padx=5, pady=5)
        
        sit_btn = ttk.Button(basic_frame, text="Sit", 
                            command=lambda: self.send_command("sit"))
        sit_btn.grid(row=0, column=2, padx=5, pady=5)
        
        rise_sit_btn = ttk.Button(basic_frame, text="Rise Sit", 
                                 command=lambda: self.send_command("rise_sit"))
        rise_sit_btn.grid(row=0, column=3, padx=5, pady=5)
        
        # Dance actions
        dance_frame = ttk.LabelFrame(parent, text="Dance Actions", padding="10")
        dance_frame.pack(fill=tk.X, padx=5, pady=5)
        
        dance1_btn = ttk.Button(dance_frame, text="Dance 1", 
                               command=lambda: self.send_command("dance1"))
        dance1_btn.grid(row=0, column=0, padx=5, pady=5)
        
        dance2_btn = ttk.Button(dance_frame, text="Dance 2", 
                               command=lambda: self.send_command("dance2"))
        dance2_btn.grid(row=0, column=1, padx=5, pady=5)
        
        wiggle_hips_btn = ttk.Button(dance_frame, text="Wiggle Hips", 
                                    command=lambda: self.send_command("wiggle_hips"))
        wiggle_hips_btn.grid(row=0, column=2, padx=5, pady=5)
        
        # Acrobatic actions
        acro_frame = ttk.LabelFrame(parent, text="Acrobatic Actions", padding="10")
        acro_frame.pack(fill=tk.X, padx=5, pady=5)
        
        front_flip_btn = ttk.Button(acro_frame, text="Front Flip", 
                                   command=lambda: self.send_command("front_flip"))
        front_flip_btn.grid(row=0, column=0, padx=5, pady=5)
        
        back_flip_btn = ttk.Button(acro_frame, text="Back Flip", 
                                  command=lambda: self.send_command("back_flip"))
        back_flip_btn.grid(row=0, column=1, padx=5, pady=5)
        
        left_flip_btn = ttk.Button(acro_frame, text="Left Flip", 
                                  command=lambda: self.send_command("left_flip"))
        left_flip_btn.grid(row=0, column=2, padx=5, pady=5)
        
        right_flip_btn = ttk.Button(acro_frame, text="Right Flip", 
                                   command=lambda: self.send_command("right_flip"))
        right_flip_btn.grid(row=0, column=3, padx=5, pady=5)
        
        # Special actions
        special_frame = ttk.LabelFrame(parent, text="Special Actions", padding="10")
        special_frame.pack(fill=tk.X, padx=5, pady=5)
        
        handstand_btn = ttk.Button(special_frame, text="Handstand", 
                                  command=lambda: self.send_command("handstand"))
        handstand_btn.grid(row=0, column=0, padx=5, pady=5)
        
        moon_walk_btn = ttk.Button(special_frame, text="Moon Walk", 
                                  command=lambda: self.send_command("moon_walk"))
        moon_walk_btn.grid(row=0, column=1, padx=5, pady=5)
        
        bound_btn = ttk.Button(special_frame, text="Bound", 
                              command=lambda: self.send_command("bound"))
        bound_btn.grid(row=0, column=2, padx=5, pady=5)
        
    def create_camera_panel(self, parent):
        # Camera control frame
        control_frame = ttk.Frame(parent)
        control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Camera toggle button
        self.camera_toggle_btn = ttk.Button(
            control_frame, 
            text="Enable Camera", 
            command=self.toggle_camera
        )
        self.camera_toggle_btn.pack(side=tk.LEFT, padx=5, pady=5)
        
        # Camera display frame
        display_frame = ttk.LabelFrame(parent, text="Camera Feed", padding="10")
        display_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Canvas for displaying the camera feed
        self.camera_canvas = tk.Canvas(
            display_frame, 
            width=DEFAULT_CAMERA_WIDTH, 
            height=DEFAULT_CAMERA_HEIGHT,
            bg="black"
        )
        self.camera_canvas.pack(fill=tk.BOTH, expand=True)
        
        # Add a label to show when camera is not enabled
        self.camera_placeholder = ttk.Label(
            self.camera_canvas, 
            text="Camera feed will appear here when enabled",
            foreground="white",
            background="black"
        )
        self.camera_canvas.create_window(
            DEFAULT_CAMERA_WIDTH // 2, 
            DEFAULT_CAMERA_HEIGHT // 2, 
            window=self.camera_placeholder
        )
    
    def create_lidar_panel(self, parent):
        # Lidar control frame
        control_frame = ttk.Frame(parent)
        control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Lidar toggle button
        self.lidar_toggle_btn = ttk.Button(
            control_frame, 
            text="Enable Lidar", 
            command=self.toggle_lidar
        )
        self.lidar_toggle_btn.pack(side=tk.LEFT, padx=5, pady=5)
        
        # Lidar display frame
        display_frame = ttk.LabelFrame(parent, text="Lidar Visualization", padding="10")
        display_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Create a matplotlib figure for the lidar visualization
        self.lidar_figure = Figure(figsize=(6, 4), dpi=100)
        self.lidar_ax = self.lidar_figure.add_subplot(111, projection='3d')
        self.lidar_ax.set_xlabel('X')
        self.lidar_ax.set_ylabel('Y')
        self.lidar_ax.set_zlabel('Z')
        self.lidar_ax.set_title('Lidar Point Cloud')
        
        # Create a canvas to display the figure
        self.lidar_canvas = FigureCanvasTkAgg(self.lidar_figure, display_frame)
        self.lidar_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Add a placeholder label
        self.lidar_placeholder = ttk.Label(
            display_frame,
            text="Lidar visualization will appear here when enabled",
            foreground="black",
            background="white"
        )
        self.lidar_placeholder.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
    
    def toggle_camera(self):
        if not self.is_connected:
            messagebox.showwarning("Not Connected", "Please connect to the robot first.")
            return
            
        if self.camera_enabled.get():
            # Disable camera
            self.camera_enabled.set(False)
            self.camera_toggle_btn.config(text="Enable Camera")
            
            # Stop the camera update
            if self.camera_update_id:
                self.root.after_cancel(self.camera_update_id)
                self.camera_update_id = None
                
            # Show placeholder
            self.camera_placeholder.place(
                x=DEFAULT_CAMERA_WIDTH // 2, 
                y=DEFAULT_CAMERA_HEIGHT // 2, 
                anchor="center"
            )
            
            # Disable video channel
            if self.connection and hasattr(self.connection, 'video'):
                asyncio.run_coroutine_threadsafe(self.disable_video_channel(), self.loop)
        else:
            # Enable camera
            self.camera_enabled.set(True)
            self.camera_toggle_btn.config(text="Disable Camera")
            
            # Hide placeholder
            self.camera_placeholder.place_forget()
            
            # Enable video channel
            if self.connection and hasattr(self.connection, 'video'):
                asyncio.run_coroutine_threadsafe(self.enable_video_channel(), self.loop)
                
                # Start the camera update
                self.update_camera_feed()
    
    def toggle_lidar(self):
        if not self.is_connected:
            messagebox.showwarning("Not Connected", "Please connect to the robot first.")
            return
            
        if self.lidar_enabled.get():
            # Disable lidar
            self.lidar_enabled.set(False)
            self.lidar_toggle_btn.config(text="Enable Lidar")
            
            # Stop the lidar update
            if self.lidar_update_id:
                self.root.after_cancel(self.lidar_update_id)
                self.lidar_update_id = None
                
            # Show placeholder
            self.lidar_placeholder.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
            
            # Disable lidar stream
            if self.connection and hasattr(self.connection, 'datachannel'):
                asyncio.run_coroutine_threadsafe(self.disable_lidar_stream(), self.loop)
        else:
            # Enable lidar
            self.lidar_enabled.set(True)
            self.lidar_toggle_btn.config(text="Disable Lidar")
            
            # Hide placeholder
            self.lidar_placeholder.place_forget()
            
            # Clear the plot
            self.lidar_ax.clear()
            self.lidar_ax.set_xlabel('X')
            self.lidar_ax.set_ylabel('Y')
            self.lidar_ax.set_zlabel('Z')
            self.lidar_ax.set_title('Lidar Point Cloud')
            self.lidar_canvas.draw()
            
            # Enable lidar stream
            if self.connection and hasattr(self.connection, 'datachannel'):
                asyncio.run_coroutine_threadsafe(self.enable_lidar_stream(), self.loop)
                
                # Start the lidar update
                self.update_lidar_plot()
    
    async def enable_video_channel(self):
        try:
            # Enable video channel
            self.connection.video.switchVideoChannel(True)
            
            # Add callback to handle received video frames
            self.connection.video.add_track_callback(self.recv_camera_stream)
            
            logger.info("Video channel enabled")
        except Exception as e:
            logger.error(f"Error enabling video channel: {e}")
            self.root.after(0, lambda: messagebox.showerror("Camera Error", f"Failed to enable camera: {e}"))
    
    async def disable_video_channel(self):
        try:
            # Disable video channel
            self.connection.video.switchVideoChannel(False)
            logger.info("Video channel disabled")
        except Exception as e:
            logger.error(f"Error disabling video channel: {e}")
    
    async def enable_lidar_stream(self):
        try:
            if not self.connection:
                raise ValueError("Connection is not established")
                
            if not hasattr(self.connection, 'datachannel') or self.connection.datachannel is None:
                raise ValueError("Data channel is not available")
                
            # Disable traffic saving mode
            try:
                if hasattr(self.connection.datachannel, 'disableTrafficSaving'):
                    # Check if the method is callable
                    disable_traffic_saving = getattr(self.connection.datachannel, 'disableTrafficSaving', None)
                    if callable(disable_traffic_saving):
                        await disable_traffic_saving(True)
                    else:
                        logger.warning("disableTrafficSaving is not callable")
                else:
                    logger.warning("disableTrafficSaving method not available")
            except Exception as e:
                logger.warning(f"Error disabling traffic saving: {e}")
                # Continue even if this fails
            
            # Set the decoder type
            try:
                if hasattr(self.connection.datachannel, 'set_decoder'):
                    # Check if the method is callable
                    set_decoder = getattr(self.connection.datachannel, 'set_decoder', None)
                    if callable(set_decoder):
                        set_decoder(decoder_type='libvoxel')
                        logger.info("Set decoder to libvoxel")
                    else:
                        logger.warning("set_decoder is not callable")
                else:
                    logger.warning("set_decoder method not available")
            except Exception as e:
                logger.warning(f"Error setting decoder: {e}")
                # Continue even if this fails
            
            # Turn on the LIDAR sensor
            if hasattr(self.connection.datachannel, 'pub_sub') and self.connection.datachannel.pub_sub is not None:
                try:
                    # Try different methods to turn on the LIDAR sensor
                    lidar_turned_on = False
                    
                    # Method 1: Try publish_request_new
                    if not lidar_turned_on and hasattr(self.connection.datachannel.pub_sub, 'publish_request_new'):
                        publish_request_new = getattr(self.connection.datachannel.pub_sub, 'publish_request_new', None)
                        if callable(publish_request_new):
                            try:
                                await publish_request_new("rt/utlidar/switch", "on")
                                logger.info("Turned on LIDAR sensor using publish_request_new")
                                lidar_turned_on = True
                            except Exception as e:
                                logger.warning(f"Error using publish_request_new: {e}")
                    
                    # Method 2: Try publish
                    if not lidar_turned_on and hasattr(self.connection.datachannel.pub_sub, 'publish'):
                        publish = getattr(self.connection.datachannel.pub_sub, 'publish', None)
                        if callable(publish):
                            try:
                                publish("rt/utlidar/switch", "on")
                                logger.info("Turned on LIDAR sensor using publish")
                                lidar_turned_on = True
                            except Exception as e:
                                logger.warning(f"Error using publish: {e}")
                    
                    # Method 3: Try publish_without_callback as last resort
                    if not lidar_turned_on and hasattr(self.connection.datachannel.pub_sub, 'publish_without_callback'):
                        publish_without_callback = getattr(self.connection.datachannel.pub_sub, 'publish_without_callback', None)
                        if callable(publish_without_callback):
                            try:
                                await publish_without_callback("rt/utlidar/switch", "on")
                                logger.info("Turned on LIDAR sensor using publish_without_callback")
                                lidar_turned_on = True
                            except Exception as e:
                                logger.warning(f"Error using publish_without_callback: {e}")
                    
                    if not lidar_turned_on:
                        logger.warning("Could not turn on LIDAR sensor using any available method")
                except Exception as e:
                    logger.warning(f"Error turning on LIDAR sensor: {e}")
                    # Continue even if this fails
                
                # Subscribe to the lidar data channel
                try:
                    if hasattr(self.connection.datachannel.pub_sub, 'subscribe'):
                        # Check if the method is callable
                        subscribe = getattr(self.connection.datachannel.pub_sub, 'subscribe', None)
                        if callable(subscribe):
                            # First unsubscribe to avoid duplicate callbacks
                            if hasattr(self.connection.datachannel.pub_sub, 'unsubscribe'):
                                unsubscribe = getattr(self.connection.datachannel.pub_sub, 'unsubscribe', None)
                                if callable(unsubscribe):
                                    # The unsubscribe method only takes the topic name, not the callback
                                    unsubscribe("rt/utlidar/voxel_map_compressed")
                                    logger.info("Unsubscribed from previous lidar data channel")
                            
                            # Now subscribe
                            subscribe("rt/utlidar/voxel_map_compressed", self.lidar_callback)
                            logger.info("Subscribed to lidar data channel")
                        else:
                            logger.warning("subscribe is not callable")
                    else:
                        logger.warning("subscribe method not available")
                except Exception as e:
                    logger.warning(f"Error subscribing to lidar data channel: {e}")
                    # Continue even if this fails
            else:
                logger.warning("pub_sub is not available")
            
            logger.info("Lidar stream enabled")
        except Exception as e:
            logger.error(f"Error enabling lidar stream: {e}")
            error_msg = str(e)
            self.root.after(0, lambda msg=error_msg: messagebox.showerror("Lidar Error", f"Failed to enable lidar: {msg}"))
    
    async def disable_lidar_stream(self):
        try:
            if not self.connection:
                return
                
            if not hasattr(self.connection, 'datachannel') or self.connection.datachannel is None:
                return
                
            # Turn off the LIDAR sensor
            if hasattr(self.connection.datachannel, 'pub_sub') and self.connection.datachannel.pub_sub is not None:
                try:
                    # Try different methods to turn off the LIDAR sensor
                    lidar_turned_off = False
                    
                    # Method 1: Try publish_request_new
                    if not lidar_turned_off and hasattr(self.connection.datachannel.pub_sub, 'publish_request_new'):
                        publish_request_new = getattr(self.connection.datachannel.pub_sub, 'publish_request_new', None)
                        if callable(publish_request_new):
                            try:
                                await publish_request_new("rt/utlidar/switch", "off")
                                logger.info("Turned off LIDAR sensor using publish_request_new")
                                lidar_turned_off = True
                            except Exception as e:
                                logger.warning(f"Error using publish_request_new: {e}")
                    
                    # Method 2: Try publish
                    if not lidar_turned_off and hasattr(self.connection.datachannel.pub_sub, 'publish'):
                        publish = getattr(self.connection.datachannel.pub_sub, 'publish', None)
                        if callable(publish):
                            try:
                                publish("rt/utlidar/switch", "off")
                                logger.info("Turned off LIDAR sensor using publish")
                                lidar_turned_off = True
                            except Exception as e:
                                logger.warning(f"Error using publish: {e}")
                    
                    # Method 3: Try publish_without_callback as last resort
                    if not lidar_turned_off and hasattr(self.connection.datachannel.pub_sub, 'publish_without_callback'):
                        publish_without_callback = getattr(self.connection.datachannel.pub_sub, 'publish_without_callback', None)
                        if callable(publish_without_callback):
                            try:
                                await publish_without_callback("rt/utlidar/switch", "off")
                                logger.info("Turned off LIDAR sensor using publish_without_callback")
                                lidar_turned_off = True
                            except Exception as e:
                                logger.warning(f"Error using publish_without_callback: {e}")
                    
                    if not lidar_turned_off:
                        logger.warning("Could not turn off LIDAR sensor using any available method")
                except Exception as e:
                    logger.warning(f"Error turning off LIDAR sensor: {e}")
                    # Continue even if this fails
                
                # Unsubscribe from the lidar data channel
                try:
                    if hasattr(self.connection.datachannel.pub_sub, 'unsubscribe'):
                        # Check if the method is callable
                        unsubscribe = getattr(self.connection.datachannel.pub_sub, 'unsubscribe', None)
                        if callable(unsubscribe):
                            # The unsubscribe method only takes the topic name, not the callback
                            unsubscribe("rt/utlidar/voxel_map_compressed")
                            logger.info("Unsubscribed from lidar data channel")
                        else:
                            logger.warning("unsubscribe is not callable")
                    else:
                        logger.warning("unsubscribe method not available")
                except Exception as e:
                    logger.warning(f"Error unsubscribing from lidar data channel: {e}")
                    # Continue even if this fails
            
            logger.info("Lidar stream disabled")
        except Exception as e:
            logger.error(f"Error disabling lidar stream: {e}")
    
    def lidar_callback(self, data):
        try:
            if not self.lidar_enabled.get():
                return
                
            # Log the data type for debugging
            logger.info(f"Received lidar data of type: {type(data)}")
            
            # Check if data is a dictionary (decoded JSON)
            if isinstance(data, dict):
                # Extract point cloud data from the lidar data
                points = []
                if 'points' in data and isinstance(data['points'], list):
                    for point in data['points']:
                        if isinstance(point, dict) and 'x' in point and 'y' in point and 'z' in point:
                            x, y, z = point['x'], point['y'], point['z']
                            points.append([x, y, z])
                    
                    # Store the points for visualization if we have any
                    if points:
                        self.lidar_points = np.array(points)
                        logger.info(f"Processed lidar data with {len(points)} points")
                    else:
                        logger.warning("Received lidar data with no valid points")
                else:
                    logger.warning("Received lidar data with invalid format (no points list)")
            # Check if data is binary (raw data)
            elif isinstance(data, (bytes, bytearray)):
                logger.info(f"Received binary lidar data of length: {len(data)}")
                # For binary data, we need to wait for the decoder to process it
                # The decoder will convert it to a format with points that we can visualize
                logger.info("Binary data will be processed by the decoder")
            else:
                logger.warning(f"Received lidar data with unexpected type: {type(data)}")
                
        except Exception as e:
            logger.error(f"Error in lidar callback: {e}")
            import traceback
            logger.error(traceback.format_exc())
    
    def update_lidar_plot(self):
        if not self.lidar_enabled.get():
            return
            
        try:
            if self.lidar_points is not None and len(self.lidar_points) > 0:
                # Clear the previous plot
                self.lidar_ax.clear()
                
                # Set the axis labels and title
                self.lidar_ax.set_xlabel('X')
                self.lidar_ax.set_ylabel('Y')
                self.lidar_ax.set_zlabel('Z')
                self.lidar_ax.set_title(f'Lidar Point Cloud ({len(self.lidar_points)} points)')
                
                # Plot the points
                x = self.lidar_points[:, 0]
                y = self.lidar_points[:, 1]
                z = self.lidar_points[:, 2]
                
                # Calculate distance for coloring
                distances = np.sqrt(x**2 + y**2 + z**2)
                
                # Create a scatter plot
                self.lidar_scatter = self.lidar_ax.scatter(x, y, z, c=distances, cmap='viridis', s=1)
                
                # Set axis limits
                max_range = np.max([np.max(np.abs(x)), np.max(np.abs(y)), np.max(np.abs(z))])
                self.lidar_ax.set_xlim([-max_range, max_range])
                self.lidar_ax.set_ylim([-max_range, max_range])
                self.lidar_ax.set_zlim([-max_range, max_range])
                
                # Add a colorbar
                if not hasattr(self, 'colorbar') or self.colorbar is None:
                    self.colorbar = self.lidar_figure.colorbar(self.lidar_scatter, ax=self.lidar_ax)
                    self.colorbar.set_label('Distance (m)')
                else:
                    self.colorbar.update_normal(self.lidar_scatter)
                
                # Set the view angle for better visualization
                self.lidar_ax.view_init(elev=30, azim=45)
                
                # Redraw the canvas
                self.lidar_canvas.draw()
                
                # Log success
                logger.info(f"Updated lidar plot with {len(self.lidar_points)} points")
            else:
                # Log that we're waiting for data
                logger.debug("Waiting for lidar data to visualize...")
        except Exception as e:
            logger.error(f"Error updating lidar plot: {e}")
            import traceback
            logger.error(traceback.format_exc())
        
        # Schedule the next update
        self.lidar_update_id = self.root.after(100, self.update_lidar_plot)  # Update at 10 Hz
    
    async def recv_camera_stream(self, track: MediaStreamTrack):
        try:
            while self.camera_enabled.get():
                frame = await track.recv()
                # Convert the frame to a NumPy array
                img = frame.to_ndarray(format="bgr24")
                self.frame_queue.put(img)
        except asyncio.CancelledError:
            logger.info("Camera stream task cancelled")
        except Exception as e:
            logger.error(f"Error in camera stream: {e}")
            if "not connected" in str(e).lower() or "connection" in str(e).lower():
                self.root.after(0, self.handle_connection_lost)
    
    def update_camera_feed(self):
        if not self.camera_enabled.get():
            return
            
        if not self.frame_queue.empty():
            # Get the latest frame
            frame = self.frame_queue.get()
            
            # Convert the OpenCV frame to a format Tkinter can display
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Resize the frame to fit the canvas if needed
            canvas_width = self.camera_canvas.winfo_width()
            canvas_height = self.camera_canvas.winfo_height()
            
            if canvas_width > 1 and canvas_height > 1:  # Ensure canvas has valid dimensions
                frame_rgb = cv2.resize(frame_rgb, (canvas_width, canvas_height))
            
            # Convert to PIL Image
            pil_img = Image.fromarray(frame_rgb)
            
            # Convert to PhotoImage
            tk_img = ImageTk.PhotoImage(image=pil_img)
            
            # Keep a reference to prevent garbage collection
            self.last_frame = tk_img
            
            # Update canvas
            self.camera_canvas.create_image(0, 0, anchor=tk.NW, image=tk_img)
        
        # Schedule the next update
        self.camera_update_id = self.root.after(33, self.update_camera_feed)  # ~30 FPS
        
    def toggle_connection(self):
        if not self.is_connected:
            self.connect()
        else:
            self.disconnect()
            
    def connect(self):
        # Disable the connect button while connecting
        self.connect_button.config(state=tk.DISABLED)
        self.status_var.set("Connecting...")
        self.root.update()
        
        # Start connection in a separate thread
        self.connection_thread = threading.Thread(target=self.connect_async)
        self.connection_thread.daemon = True
        self.connection_thread.start()
        
    def connect_async(self):
        # Create a new event loop for this thread
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        # Create a shutdown event for clean termination
        self.shutdown_event = asyncio.Event()
        
        try:
            # Run the connection coroutine
            self.loop.run_until_complete(self.do_connect())
            
            # Start the event loop to handle async tasks
            self.connection_monitor_task = self.loop.create_task(self.monitor_connection())
            self.heartbeat_task = self.loop.create_task(self.check_heartbeat())
            
            # Run the event loop until shutdown is requested
            self.loop.run_until_complete(self.shutdown_event.wait())
            
            # Clean up tasks
            if self.connection_monitor_task and not self.connection_monitor_task.done():
                self.connection_monitor_task.cancel()
            if self.heartbeat_task and not self.heartbeat_task.done():
                self.heartbeat_task.cancel()
                
            # Wait for tasks to complete
            pending = asyncio.all_tasks(self.loop)
            self.loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            
        except Exception as e:
            logger.error(f"Connection thread error: {e}")
            # If connection fails, update UI with error
            error_msg = str(e)
            self.root.after(0, lambda msg=error_msg: self.update_ui_after_error(msg))
        finally:
            # Close the loop
            self.loop.close()
            self.loop = None
            
    async def do_connect(self):
        try:
            # Create connection based on selected method
            method = WebRTCConnectionMethod(self.connection_method.get())
            
            if method == WebRTCConnectionMethod.LocalAP:
                self.connection = Go2WebRTCConnection(method)
            elif method == WebRTCConnectionMethod.LocalSTA:
                if self.ip_address.get():
                    self.connection = Go2WebRTCConnection(method, ip=self.ip_address.get())
                else:
                    self.connection = Go2WebRTCConnection(method, serialNumber=self.serial_number.get())
            elif method == WebRTCConnectionMethod.Remote:
                self.connection = Go2WebRTCConnection(
                    method, 
                    serialNumber=self.serial_number.get(),
                    username=self.username.get(),
                    password=self.password.get()
                )
            
            # Connect to the WebRTC service
            await self.connection.connect()
            
            # Set up connection state change handlers after connection is established
            if hasattr(self.connection, 'pc') and self.connection.pc is not None:
                self.connection.pc.on("connectionstatechange", self.on_connection_state_change)
                self.connection.pc.on("signalingstatechange", self.on_signaling_state_change)
                self.connection.pc.on("icestatechange", self.on_ice_state_change)
            
            # Update UI after successful connection
            self.root.after(0, self.update_ui_after_connect)
            
            # Check current motion mode
            try:
                response = await self.connection.datachannel.pub_sub.publish_request_new(
                    RTC_TOPIC["MOTION_SWITCHER"], 
                    {"api_id": 1001}
                )
                
                if response['data']['header']['status']['code'] == 0:
                    data = json.loads(response['data']['data'])
                    current_motion_switcher_mode = data['name']
                    logger.info(f"Current motion mode: {current_motion_switcher_mode}")
                    
                    # Switch to "normal" mode if not already
                    if current_motion_switcher_mode != "normal":
                        logger.info(f"Switching motion mode from {current_motion_switcher_mode} to 'normal'...")
                        await self.connection.datachannel.pub_sub.publish_request_new(
                            RTC_TOPIC["MOTION_SWITCHER"], 
                            {
                                "api_id": 1002,
                                "parameter": {"name": "normal"}
                            }
                        )
                        await asyncio.sleep(5)  # Wait while it stands up
            except Exception as e:
                logger.error(f"Error checking motion mode: {e}")
                # Continue even if motion mode check fails
            
            return True
            
        except Exception as e:
            logger.error(f"Connection error: {e}")
            raise
            
    def on_connection_state_change(self):
        state = self.connection.pc.connectionState if self.connection else "closed"
        logger.info(f"Connection state changed to: {state}")
        
        if state == "failed" or state == "closed":
            self.root.after(0, self.handle_connection_lost)
            
    def on_signaling_state_change(self):
        state = self.connection.pc.signalingState if self.connection else "closed"
        logger.info(f"Signaling state changed to: {state}")
        
    def on_ice_state_change(self):
        state = self.connection.pc.iceConnectionState if self.connection else "closed"
        logger.info(f"ICE state changed to: {state}")
        
    def handle_connection_lost(self):
        if self.is_connected:
            logger.info("Connection lost, updating UI")
            self.is_connected = False
            self.connect_button.config(text="Connect", state=tk.NORMAL)
            self.status_var.set("Connection lost. Please reconnect.")
            messagebox.showwarning("Connection Lost", "The connection to the robot has been lost. Please reconnect.")
            
            # Disable camera if it was enabled
            if self.camera_enabled.get():
                self.camera_enabled.set(False)
                self.camera_toggle_btn.config(text="Enable Camera")
                
                # Stop the camera update
                if self.camera_update_id:
                    self.root.after_cancel(self.camera_update_id)
                    self.camera_update_id = None
                
                # Show placeholder
                self.camera_placeholder.place(
                    x=DEFAULT_CAMERA_WIDTH // 2, 
                    y=DEFAULT_CAMERA_HEIGHT // 2, 
                    anchor="center"
                )
            
            # Disable lidar if it was enabled
            if self.lidar_enabled.get():
                self.lidar_enabled.set(False)
                self.lidar_toggle_btn.config(text="Enable Lidar")
                
                # Stop the lidar update
                if self.lidar_update_id:
                    self.root.after_cancel(self.lidar_update_id)
                    self.lidar_update_id = None
                
                # Show placeholder
                self.lidar_placeholder.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
            
            # Signal the event loop to shut down
            if self.shutdown_event and not self.shutdown_event.is_set():
                self.loop.call_soon_threadsafe(self.shutdown_event.set)
                
    async def monitor_connection(self):
        """Monitor connection state and attempt reconnection if needed"""
        try:
            while self.is_connected and self.connection:
                if (self.connection.pc.connectionState == "failed" or 
                    self.connection.pc.connectionState == "closed"):
                    logger.info("Connection monitor detected closed connection")
                    self.root.after(0, self.handle_connection_lost)
                    break
                await asyncio.sleep(5)
        except asyncio.CancelledError:
            logger.info("Connection monitor task cancelled")
        except Exception as e:
            logger.error(f"Error in connection monitor: {e}")
            
    async def check_heartbeat(self):
        """Periodically check if heartbeat is working"""
        try:
            heartbeat_count = 0
            while self.is_connected and self.connection:
                try:
                    # Just sleep and let the built-in heartbeat mechanism work
                    await asyncio.sleep(10)
                    heartbeat_count += 1
                    
                    # Log heartbeat status every minute
                    if heartbeat_count % 6 == 0:
                        logger.info("Heartbeat check: Connection still active")
                        
                except Exception as e:
                    logger.error(f"Heartbeat check error: {e}")
                    
        except asyncio.CancelledError:
            logger.info("Heartbeat check task cancelled")
        except Exception as e:
            logger.error(f"Error in heartbeat check: {e}")
            
    def update_ui_after_connect(self):
        self.is_connected = True
        self.connect_button.config(text="Disconnect", state=tk.NORMAL)
        self.status_var.set("Connected")
        
    def update_ui_after_error(self, error_msg):
        self.is_connected = False
        self.connect_button.config(text="Connect", state=tk.NORMAL)
        self.status_var.set(f"Connection failed: {error_msg}")
        messagebox.showerror("Connection Error", f"Failed to connect: {error_msg}")
        
    def disconnect(self):
        if self.connection:
            # Disable the button while disconnecting
            self.connect_button.config(state=tk.DISABLED)
            self.status_var.set("Disconnecting...")
            self.root.update()
            
            # Disable camera if it was enabled
            if self.camera_enabled.get():
                self.camera_enabled.set(False)
                self.camera_toggle_btn.config(text="Enable Camera")
                
                # Stop the camera update
                if self.camera_update_id:
                    self.root.after_cancel(self.camera_update_id)
                    self.camera_update_id = None
                
                # Show placeholder
                self.camera_placeholder.place(
                    x=DEFAULT_CAMERA_WIDTH // 2, 
                    y=DEFAULT_CAMERA_HEIGHT // 2, 
                    anchor="center"
                )
            
            # Disable lidar if it was enabled
            if self.lidar_enabled.get():
                self.lidar_enabled.set(False)
                self.lidar_toggle_btn.config(text="Enable Lidar")
                
                # Stop the lidar update
                if self.lidar_update_id:
                    self.root.after_cancel(self.lidar_update_id)
                    self.lidar_update_id = None
                
                # Show placeholder
                self.lidar_placeholder.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
            
            # Signal the event loop to shut down
            if self.shutdown_event and not self.shutdown_event.is_set():
                self.loop.call_soon_threadsafe(self.shutdown_event.set)
            
            # Start disconnection in a separate thread
            threading.Thread(target=self.disconnect_async).start()
            
    def disconnect_async(self):
        try:
            # Close the connection
            if self.connection:
                # Create a temporary event loop if needed
                temp_loop = None
                if not self.loop or self.loop.is_closed():
                    temp_loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(temp_loop)
                    temp_loop.run_until_complete(self.connection.close())
                    temp_loop.close()
                
            # Update UI
            self.root.after(0, self.update_ui_after_disconnect)
            
        except Exception as e:
            logger.error(f"Disconnection error: {e}")
            error_msg = str(e)
            self.root.after(0, lambda msg=error_msg: messagebox.showerror("Disconnection Error", f"Failed to disconnect: {error_msg}"))
            
    def update_ui_after_disconnect(self):
        self.is_connected = False
        self.connect_button.config(text="Connect", state=tk.NORMAL)
        self.status_var.set("Disconnected")
        
        # Disable camera if it was enabled
        if self.camera_enabled.get():
            self.camera_enabled.set(False)
            self.camera_toggle_btn.config(text="Enable Camera")
            
            # Stop the camera update
            if self.camera_update_id:
                self.root.after_cancel(self.camera_update_id)
                self.camera_update_id = None
            
            # Show placeholder
            self.camera_placeholder.place(
                x=DEFAULT_CAMERA_WIDTH // 2, 
                y=DEFAULT_CAMERA_HEIGHT // 2, 
                anchor="center"
            )
        
        # Disable lidar if it was enabled
        if self.lidar_enabled.get():
            self.lidar_enabled.set(False)
            self.lidar_toggle_btn.config(text="Enable Lidar")
            
            # Stop the lidar update
            if self.lidar_update_id:
                self.root.after_cancel(self.lidar_update_id)
                self.lidar_update_id = None
            
            # Show placeholder
            self.lidar_placeholder.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
        
        self.connection = None
        
    def send_command(self, command):
        if not self.is_connected or not self.connection:
            messagebox.showwarning("Not Connected", "Please connect to the robot first.")
            return
            
        if not self.loop or self.loop.is_closed():
            messagebox.showerror("Connection Error", "Connection loop is not running. Please reconnect.")
            return
            
        # Schedule the command on the event loop
        asyncio.run_coroutine_threadsafe(self.do_send_command(command), self.loop)
            
    async def do_send_command(self, command):
        try:
            if not self.connection or not self.is_connected:
                logger.error("Cannot send command: Not connected")
                self.root.after(0, lambda: messagebox.showerror("Command Error", "Cannot send command: Not connected"))
                return
                
            speed = self.speed.get()
            
            # Update status
            self.root.after(0, lambda: self.status_var.set(f"Executing: {command}"))
            
            try:
                # Execute the command based on the command name
                if command == "move_forward":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {
                            "api_id": SPORT_CMD["Move"],
                            "parameter": {"x": speed, "y": 0, "z": 0}
                        }
                    )
                elif command == "move_backward":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {
                            "api_id": SPORT_CMD["Move"],
                            "parameter": {"x": -speed, "y": 0, "z": 0}
                        }
                    )
                elif command == "move_left":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {
                            "api_id": SPORT_CMD["Move"],
                            "parameter": {"x": 0, "y": speed, "z": 0}
                        }
                    )
                elif command == "move_right":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {
                            "api_id": SPORT_CMD["Move"],
                            "parameter": {"x": 0, "y": -speed, "z": 0}
                        }
                    )
                elif command == "stop_move":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {"api_id": SPORT_CMD["StopMove"]}
                    )
                elif command == "stand_up":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {"api_id": SPORT_CMD["StandUp"]}
                    )
                elif command == "stand_down":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {"api_id": SPORT_CMD["StandDown"]}
                    )
                elif command == "hello":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {"api_id": SPORT_CMD["Hello"]}
                    )
                elif command == "stretch":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {"api_id": SPORT_CMD["Stretch"]}
                    )
                elif command == "sit":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {"api_id": SPORT_CMD["Sit"]}
                    )
                elif command == "rise_sit":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {"api_id": SPORT_CMD["RiseSit"]}
                    )
                elif command == "dance1":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {"api_id": SPORT_CMD["Dance1"]}
                    )
                elif command == "dance2":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {"api_id": SPORT_CMD["Dance2"]}
                    )
                elif command == "wiggle_hips":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {"api_id": SPORT_CMD["WiggleHips"]}
                    )
                elif command == "front_flip":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {"api_id": SPORT_CMD["FrontFlip"]}
                    )
                elif command == "back_flip":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {"api_id": SPORT_CMD["BackFlip"]}
                    )
                elif command == "left_flip":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {"api_id": SPORT_CMD["LeftFlip"]}
                    )
                elif command == "right_flip":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {"api_id": SPORT_CMD["RightFlip"]}
                    )
                elif command == "handstand":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {"api_id": SPORT_CMD["Handstand"]}
                    )
                elif command == "moon_walk":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {"api_id": SPORT_CMD["MoonWalk"]}
                    )
                elif command == "bound":
                    await self.connection.datachannel.pub_sub.publish_request_new(
                        RTC_TOPIC["SPORT_MOD"], 
                        {"api_id": SPORT_CMD["Bound"]}
                    )
                
                # Update status after command execution
                self.root.after(0, lambda: self.status_var.set(f"Executed: {command}"))
                
            except Exception as cmd_error:
                error_msg = str(cmd_error)
                logger.error(f"Command execution error: {error_msg}")
                self.root.after(0, lambda msg=error_msg: messagebox.showerror("Command Error", f"Failed to execute command: {msg}"))
                
                # Check if this is a connection error and handle accordingly
                if "not connected" in error_msg.lower() or "connection" in error_msg.lower():
                    self.root.after(0, self.handle_connection_lost)
            
        except Exception as e:
            error_msg = str(e)
            logger.error(f"Command execution outer error: {error_msg}")
            self.root.after(0, lambda msg=error_msg: messagebox.showerror("Command Error", f"Failed to process command: {msg}"))

def main():
    root = tk.Tk()
    app = RobotController(root)
    root.mainloop()

if __name__ == "__main__":
    main()