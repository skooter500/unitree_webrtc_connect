import asyncio
import logging
import json
import sys
import tkinter as tk
from tkinter import ttk, messagebox, StringVar
import threading
import time
from functools import partial

from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD

# Enable logging for debugging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

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
            self.root.after(0, lambda: messagebox.showerror("Disconnection Error", f"Failed to disconnect: {error_msg}"))
            
    def update_ui_after_disconnect(self):
        self.is_connected = False
        self.connect_button.config(text="Connect", state=tk.NORMAL)
        self.status_var.set("Disconnected")
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