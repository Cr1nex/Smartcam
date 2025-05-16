import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import cv2
import numpy as np
import requests 
import threading
import time
import json 

#Configuration
ESP32_CAM_STREAM_URL = "http://192.168.127.140:81/stream" 
ESP32_CAM_COMMAND_URL = "http://192.168.127.140:8081/command" 


YOLO_CONFIG_PATH = "yolov3-tiny.cfg"  
YOLO_WEIGHTS_PATH = "yolov3-tiny.weights" 
YOLO_CLASSES_PATH = "coco.names"      

#Global Variables
video_capture = None
yolo_net = None
output_layers = None
classes = []
current_pan = 90
current_tilt = 90
auto_tracking_enabled = False
stop_threads = False #Flag to stop background threads

# --- YOLOv3-tiny Initialization ---
def load_yolo():
    """Loads YOLOv3-tiny model and class names."""
    global yolo_net, output_layers, classes
    try:
        yolo_net = cv2.dnn.readNet(YOLO_WEIGHTS_PATH, YOLO_CONFIG_PATH)
        # Check for CUDA availability for GPU acceleration
        if cv2.cuda.getCudaEnabledDeviceCount() > 0:
            yolo_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            yolo_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
            print("YOLO: Using CUDA backend.")
        else:
            print("YOLO: Using CPU backend.")

        layer_names = yolo_net.getLayerNames()
        # Correctly get output layer names for OpenCV 4.x+
        try:
            output_layers = [layer_names[i - 1] for i in yolo_net.getUnconnectedOutLayers().flatten()]
        except AttributeError: # For older OpenCV versions
             output_layers = [layer_names[i[0] - 1] for i in yolo_net.getUnconnectedOutLayers()]


        with open(YOLO_CLASSES_PATH, "r") as f:
            classes = [line.strip() for line in f.readlines()]
        print("YOLO model loaded successfully.")
        return True
    except Exception as e:
        messagebox.showerror("YOLO Load Error", f"Failed to load YOLO model: {e}\n"
                                                "Please ensure yolov3-tiny.cfg, yolov3-tiny.weights, "
                                                "and coco.names are in the correct path.")
        return False

#Servo Control
def send_pan_tilt_command(pan, tilt):
    #Sends pan and tilt angles
    global current_pan, current_tilt
    pan = int(np.clip(pan, 0, 180)) 
    tilt = int(np.clip(tilt, 30, 150)) 
    payload_str = f"S{pan}:{tilt}" 
    try:      
        response = requests.post(ESP32_CAM_COMMAND_URL, data=payload_str, timeout=2)
        response.raise_for_status() #Raise an exception for HTTP errors
        print(f"Sent command: {payload_str}, Response: {response.status_code}")
        current_pan = pan
        current_tilt = tilt
        update_angle_labels()
        return True
    except requests.exceptions.RequestException as e:
        print(f"Error sending command: {e}")
        return False

# --- Video Processing and Object Detection ---
def process_frame(frame):
    """Processes a single frame for object detection and tracking."""
    global current_pan, current_tilt, auto_tracking_enabled

    if frame is None or yolo_net is None:
        return frame

    height, width, _ = frame.shape
    frame_center_x, frame_center_y = width // 2, height // 2

    # --- YOLO Detection ---
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (320, 320), (0, 0, 0), True, crop=False) # 320 or 416
    yolo_net.setInput(blob)
    outs = yolo_net.forward(output_layers)

    class_ids = []
    confidences = []
    boxes = []
    detected_targets = [] # Store (center_x, center_y, w, h) of detected objects

    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:  # Confidence threshold
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # Rectangle coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)
                if classes[class_id] == "person": # Example: track only "person"
                    detected_targets.append((center_x, center_y, w, h))


    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4) # Non-maxima suppression

    # --- Drawing and Tracking Logic ---
    target_acquired = False
    if len(indexes) > 0:
        for i in indexes.flatten():
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            confidence = confidences[i]
            color = (0, 255, 0) # Green for detected objects
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, f"{label} {confidence:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # For auto-tracking, pick the first "person" or largest detected object
            if auto_tracking_enabled and not target_acquired and classes[class_ids[i]] == "person": # Track first person
                target_center_x, target_center_y = x + w // 2, y + h // 2
                target_acquired = True

                # --- Simple Proportional Control for Tracking ---
                error_x = target_center_x - frame_center_x
                error_y = target_center_y - frame_center_y

                # Proportional gain (tune these values)
                gain_pan = 0.05  # Degrees per pixel error
                gain_tilt = -0.05 # Degrees per pixel error (tilt often inverted)

                # Deadzone to prevent jitter when centered
                dead_zone = 20 # Pixels

                delta_pan = 0
                delta_tilt = 0

                if abs(error_x) > dead_zone:
                    delta_pan = gain_pan * error_x
                if abs(error_y) > dead_zone:
                    delta_tilt = gain_tilt * error_y
                
                # Limit change per frame to avoid jerky movements
                max_delta_pan_per_frame = 3 # degrees
                max_delta_tilt_per_frame = 3 # degrees
                delta_pan = np.clip(delta_pan, -max_delta_pan_per_frame, max_delta_pan_per_frame)
                delta_tilt = np.clip(delta_tilt, -max_delta_tilt_per_frame, max_delta_tilt_per_frame)


                if abs(delta_pan) > 0.5 or abs(delta_tilt) > 0.5: # Only send if significant change
                    new_pan = current_pan + delta_pan
                    new_tilt = current_tilt + delta_tilt
                    send_pan_tilt_command(new_pan, new_tilt)
                break # Track only one target for now

    # Draw frame center for reference
    cv2.circle(frame, (frame_center_x, frame_center_y), 5, (0, 0, 255), -1)
    return frame

#Video stream
def video_stream_thread():
    #Handles capturing and displaying video frames.
    global video_capture, stop_threads
    try:
        video_capture = cv2.VideoCapture(ESP32_CAM_STREAM_URL)
        if not video_capture.isOpened():
            messagebox.showerror("Video Error", f"Cannot open stream: {ESP32_CAM_STREAM_URL}", parent=app)
            return

        while not stop_threads:
            ret, frame = video_capture.read()
            if not ret or frame is None:
                print("Failed to grab frame or stream ended.")
                time.sleep(0.5) # Wait before retrying
                #Attempts to reconnect
                video_capture.release()
                video_capture = cv2.VideoCapture(ESP32_CAM_STREAM_URL)
                if not video_capture.isOpened():
                    print(f"Failed to reconnect to stream: {ESP32_CAM_STREAM_URL}")
                    time.sleep(2) #Wait longer before next reconnect attempt
                    continue
                else:
                    print("Reconnected to stream.")
                continue


            # Process frame (YOLO, tracking)
            processed_frame = process_frame(frame.copy()) # Process a copy

            # Convert frame for Tkinter
            img = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
            img_pil = Image.fromarray(img)
            img_tk = ImageTk.PhotoImage(image=img_pil)

            # Update GUI (must be done in main thread or use queue)
            video_label.imgtk = img_tk
            video_label.configure(image=img_tk)
            # app.update_idletasks() # Force update, can cause lag
            time.sleep(0.01) # Adjust for desired frame rate, reduce CPU usage

    except Exception as e:
        if not stop_threads: # Don't show error if we are intentionally stopping
            messagebox.showerror("Video Thread Error", f"Error in video stream: {e}", parent=app)
            print(f"Video thread error: {e}")
    finally:
        if video_capture:
            video_capture.release()
        print("Video thread stopped.")

# --- GUI Functions ---
def toggle_auto_tracking():
    global auto_tracking_enabled
    auto_tracking_enabled = not auto_tracking_enabled
    tracking_status_label.config(text=f"Auto Tracking: {'ON' if auto_tracking_enabled else 'OFF'}")
    print(f"Auto tracking: {auto_tracking_enabled}")

def manual_pan_left():
    send_pan_tilt_command(current_pan - 10, current_tilt)

def manual_pan_right():
    send_pan_tilt_command(current_pan + 10, current_tilt)

def manual_tilt_up():
    send_pan_tilt_command(current_pan, current_tilt - 10) # Tilt might be inverted depending on setup

def manual_tilt_down():
    send_pan_tilt_command(current_pan, current_tilt + 10)

def center_servos():
    send_pan_tilt_command(90, 90)


def update_angle_labels():
    pan_angle_label.config(text=f"Pan: {current_pan}°")
    tilt_angle_label.config(text=f"Tilt: {current_tilt}°")


def on_closing():
    """Handles window close event."""
    global stop_threads
    if messagebox.askokcancel("Quit", "Do you want to quit?"):
        stop_threads = True
        # Wait for thread to finish if it's running
        if video_thread and video_thread.is_alive():
            video_thread.join(timeout=2) # Wait max 2 seconds
        app.destroy()

# --- Main Application Setup ---
app = tk.Tk()
app.title("ESP32-CAM Pan/Tilt Tracker")

# --- Video Display ---
video_label = ttk.Label(app)
video_label.grid(row=0, column=0, columnspan=4, padx=10, pady=10)

# --- Controls Frame ---
controls_frame = ttk.LabelFrame(app, text="Controls")
controls_frame.grid(row=1, column=0, columnspan=4, padx=10, pady=10, sticky="ew")

# Auto Tracking Button
tracking_button = ttk.Button(controls_frame, text="Toggle Auto Tracking", command=toggle_auto_tracking)
tracking_button.grid(row=0, column=0, columnspan=2, padx=5, pady=5)
tracking_status_label = ttk.Label(controls_frame, text="Auto Tracking: OFF")
tracking_status_label.grid(row=0, column=2, columnspan=2, padx=5, pady=5)

# Manual Controls
manual_label = ttk.Label(controls_frame, text="Manual Control:")
manual_label.grid(row=1, column=0, columnspan=4, pady=(10,0))

btn_tilt_up = ttk.Button(controls_frame, text="Tilt Up (↑)", command=manual_tilt_up)
btn_tilt_up.grid(row=2, column=1, padx=5, pady=2)

btn_pan_left = ttk.Button(controls_frame, text="Pan Left (←)", command=manual_pan_left)
btn_pan_left.grid(row=3, column=0, padx=5, pady=2)

btn_center = ttk.Button(controls_frame, text="Center (C)", command=center_servos)
btn_center.grid(row=3, column=1, padx=5, pady=2)

btn_pan_right = ttk.Button(controls_frame, text="Pan Right (→)", command=manual_pan_right)
btn_pan_right.grid(row=3, column=2, padx=5, pady=2)

btn_tilt_down = ttk.Button(controls_frame, text="Tilt Down (↓)", command=manual_tilt_down)
btn_tilt_down.grid(row=4, column=1, padx=5, pady=2)

# Angle display
angle_frame = ttk.LabelFrame(app, text="Current Angles")
angle_frame.grid(row=2, column=0, columnspan=4, padx=10, pady=10, sticky="ew")
pan_angle_label = ttk.Label(angle_frame, text=f"Pan: {current_pan}°")
pan_angle_label.pack(side=tk.LEFT, padx=10)
tilt_angle_label = ttk.Label(angle_frame, text=f"Tilt: {current_tilt}°")
tilt_angle_label.pack(side=tk.LEFT, padx=10)


# --- Keyboard Bindings for Manual Control ---
app.bind('<Left>', lambda event: manual_pan_left())
app.bind('<Right>', lambda event: manual_pan_right())
app.bind('<Up>', lambda event: manual_tilt_up())
app.bind('<Down>', lambda event: manual_tilt_down())
app.bind('c', lambda event: center_servos())
app.bind('C', lambda event: center_servos())
app.bind('t', lambda event: toggle_auto_tracking())
app.bind('T', lambda event: toggle_auto_tracking())


# --- Start Application ---
if __name__ == "__main__":
    if not load_yolo():
        app.destroy() # Close app if YOLO fails to load
    else:
        # Initialize servos to center on startup (optional)
        # send_pan_tilt_command(current_pan, current_tilt)

        # Start video stream in a separate thread
        video_thread = threading.Thread(target=video_stream_thread, daemon=True)
        video_thread.start()

        app.protocol("WM_DELETE_WINDOW", on_closing) # Handle window close button
        app.mainloop()

        # Ensure thread is stopped if mainloop exits unexpectedly
        stop_threads = True
        if video_thread.is_alive():
            video_thread.join(timeout=1)
        print("Application closed.")
