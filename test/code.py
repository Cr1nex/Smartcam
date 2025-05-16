import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from tkinter import simpledialog 
from PIL import Image, ImageTk, ImageDraw, ImageFont 
import cv2
import numpy as np
import requests 
import threading
import time
import socket 
import csv
import os
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.base import MIMEBase
from email import encoders
from zeroconf import ServiceBrowser, Zeroconf, ServiceInfo 
from dotenv import load_dotenv
from Crypto.Cipher import AES
from Crypto.Util.Padding import pad, unpad 
import base64

load_dotenv()

#Configuration
ESP32_CAM_IP_ADDRESS = None 
ESP32_COMMAND_URL_BASE = None 

MDNS_SERVICE_TYPE = "_http._tcp.local."
MDNS_SERVICE_NAME = "esp32-cam-tracker._http._tcp.local." 
DISCOVERY_TIMEOUT = 10
#Encryption Configuration
AES_KEY = None
AES_IV = None
AES_BLOCK_SIZE = 16 # AES block size is 16 bytes

def load_encryption_config():
    global AES_KEY, AES_IV
    key_str = os.getenv("AES_KEY_BYTES")
    iv_str = os.getenv("AES_IV_BYTES")

    if not key_str or not iv_str:
        messagebox.showerror("Encryption Error", "AES_KEY_BYTES or AES_IV_BYTES not found in .env file!\nPlease create a .env file with these values (comma-separated decimal bytes).")
        return False
    
    try:
        AES_KEY = bytes([int(b.strip()) for b in key_str.split(',')])
        AES_IV = bytes([int(b.strip()) for b in iv_str.split(',')])

        if len(AES_KEY) != 16:
            messagebox.showerror("Encryption Error", f"AES_KEY from .env file must be 16 bytes long, found {len(AES_KEY)} bytes.")
            return False
        if len(AES_IV) != 16:
            messagebox.showerror("Encryption Error", f"AES_IV from .env file must be 16 bytes long, found {len(AES_IV)} bytes.")
            return False
        
        print("Python: AES Key and IV loaded successfully from .env file.")
        return True
    except Exception as e:
        messagebox.showerror("Encryption Error", f"Error parsing AES_KEY_BYTES or AES_IV_BYTES from .env file: {e}\nEnsure they are comma-separated decimal byte values.")
        return False
UDP_VIDEO_IP_ADDRESS = "0.0.0.0"  
UDP_VIDEO_PORT = 12345            
UDP_VIDEO_BUFFER_SIZE = 65507     
START_OF_FRAME_MARKER = b"<SOF>"
END_OF_FRAME_MARKER = b"<EOF>"

UDP_LOG_IP_ADDRESS = "0.0.0.0"
UDP_LOG_PORT = 12346 
UDP_LOG_BUFFER_SIZE = 256 

YOLO_CONFIG_PATH = "yolov3-tiny.cfg"
YOLO_WEIGHTS_PATH = "yolov3-tiny.weights"
YOLO_CLASSES_PATH = "coco.names"

#Global Variables
yolo_net = None; output_layers = None; classes = []
current_pan = 90; current_tilt = 90
auto_tracking_enabled = False
stop_threads = False 
last_command_time = 0 
COMMAND_SEND_INTERVAL = 0.5 

udp_video_socket = None
video_frame_buffer = bytearray()
receiving_video_frame = False
last_valid_frame_time = 0 
NO_SIGNAL_TIMEOUT = 5.0 #Seconds before showing "No Signal"
decode_error_count = 0 

udp_log_socket = None
activity_log_entries = [] 
log_lock = threading.Lock() 
recipient_email_entry = None 
placeholder_image_tk = None 

#Placeholder Image Creation
def create_placeholder_image(width, height, text="Waiting for Video Signal..."):
    global placeholder_image_tk 
    try:
        img = Image.new('RGB', (width, height), color = (70, 70, 70)) 
        d = ImageDraw.Draw(img)
        try: font = ImageFont.truetype("arial.ttf", 20)
        except IOError: font = ImageFont.load_default(); print("Arial font not found, using default.")
        text_bbox = d.textbbox((0,0), text, font=font)
        text_width = text_bbox[2] - text_bbox[0]; text_height = text_bbox[3] - text_bbox[1]
        text_x = (width - text_width) / 2; text_y = (height - text_height) / 2
        d.text((text_x, text_y), text, fill=(200,200,200), font=font) 
        placeholder_image_tk = ImageTk.PhotoImage(image=img) 
        return placeholder_image_tk
    except Exception as e:
        print(f"Error creating placeholder image: {e}")
        img = Image.new('RGB', (width, height), color = (70, 70, 70)) 
        placeholder_image_tk = ImageTk.PhotoImage(image=img)
        return placeholder_image_tk

#Encryption Function
def encrypt_command(plaintext_command_str):
    try:
        cipher = AES.new(AES_KEY, AES.MODE_CBC, AES_IV)
        padded_command_bytes = pad(plaintext_command_str.encode('utf-8'), AES_BLOCK_SIZE)
        ciphertext_bytes = cipher.encrypt(padded_command_bytes)
        base64_ciphertext_str = base64.b64encode(ciphertext_bytes).decode('utf-8')
        return base64_ciphertext_str
    except Exception as e: print(f"Python: Encryption failed: {e}"); return None

#mDNS Discovery Listener
class MyServiceListener:
    def __init__(self): self.discovered_ip = None; self.discovered_port = None
    def remove_service(self, zeroconf, type, name): print(f"Python: Service {name} removed")
    def add_service(self, zeroconf, type, name):
        info = zeroconf.get_service_info(type, name)
        if info and name == MDNS_SERVICE_NAME:
            try:
                ipv4_address = None
                for addr_bytes in info.addresses:
                    if len(addr_bytes) == 4: ipv4_address = socket.inet_ntoa(addr_bytes); break
                if ipv4_address: self.discovered_ip = ipv4_address
                elif info.addresses: self.discovered_ip = socket.inet_ntoa(info.addresses[0]) 
                self.discovered_port = info.port
                if self.discovered_ip: print(f"Python: Discovered ESP32-CAM '{name}' at {self.discovered_ip}:{self.discovered_port}")
            except Exception as e: print(f"Error processing service info for {name}: {e}"); print(f"Raw addresses: {info.addresses}")
    def update_service(self, zeroconf, type, name): pass

def discover_esp32_cam_ip():
    global ESP32_CAM_IP_ADDRESS, ESP32_COMMAND_URL_BASE
    zeroconf = None; browser = None
    try:
        zeroconf = Zeroconf(); listener = MyServiceListener()
        browser = ServiceBrowser(zeroconf, MDNS_SERVICE_TYPE, listener)
        print(f"Python: Starting mDNS discovery for '{MDNS_SERVICE_NAME}' for {DISCOVERY_TIMEOUT} seconds...")
        time.sleep(DISCOVERY_TIMEOUT) 
        if listener.discovered_ip and listener.discovered_port == 8081:
            ESP32_CAM_IP_ADDRESS = listener.discovered_ip
            ESP32_COMMAND_URL_BASE = f"http://{ESP32_CAM_IP_ADDRESS}:{listener.discovered_port}"
            print(f"Python: ESP32-CAM IP automatically set to: {ESP32_CAM_IP_ADDRESS}")
            return True
        else: print("Python: ESP32-CAM not found via mDNS."); return False
    except Exception as e: print(f"Python: mDNS discovery error: {e}"); return False
    finally:
        if browser: browser.cancel()
        if zeroconf: zeroconf.close()

#Register Python Client IP with ESP32-CAM
def register_python_client_with_esp32():
    global ESP32_COMMAND_URL_BASE
    if ESP32_COMMAND_URL_BASE is None:
        print("Python: Cannot register client, ESP32-CAM IP not known.")
        return False
    register_url = f"{ESP32_COMMAND_URL_BASE}/register_python_client"
    try:
        print(f"Python: Registering this client's IP with ESP32-CAM at {register_url}")
        response = requests.get(register_url, timeout=3.0) 
        if response.status_code == 200:
            print(f"Python: Successfully registered IP with ESP32-CAM. ESP Response: {response.text}")
            return True
        else:
            print(f"Python: Failed to register IP with ESP32-CAM. Status: {response.status_code}, Response: {response.text}")
            messagebox.showwarning("ESP32-CAM Registration", f"Failed to register with ESP32-CAM (for UDP target).\nStatus: {response.status_code}\n{response.text}")
            return False
    except Exception as e:
        print(f"Python: Error registering IP with ESP32-CAM: {e}")
        messagebox.showerror("ESP32-CAM Registration Error", f"Error registering with ESP32-CAM:\n{e}")
        return False

#YOLOv3-tiny Initialization
def load_yolo(): 
    global yolo_net, output_layers, classes
    try:
        yolo_net = cv2.dnn.readNet(YOLO_WEIGHTS_PATH, YOLO_CONFIG_PATH)
        if cv2.cuda.getCudaEnabledDeviceCount() > 0: yolo_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA); yolo_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA); print("YOLO: Using CUDA backend.")
        else: print("YOLO: Using CPU backend.")
        layer_names = yolo_net.getLayerNames()
        try: output_layers = [layer_names[i - 1] for i in yolo_net.getUnconnectedOutLayers().flatten()]
        except AttributeError: output_layers = [layer_names[i[0] - 1] for i in yolo_net.getUnconnectedOutLayers()]
        with open(YOLO_CLASSES_PATH, "r") as f: classes = [line.strip() for line in f.readlines()]
        print("YOLO model loaded successfully."); return True
    except Exception as e: messagebox.showerror("YOLO Load Error", f"Failed to load YOLO: {e}"); return False

#Servo Control Communication (TCP/HTTP)
def send_pan_tilt_command(pan, tilt): 
    global current_pan, current_tilt, last_command_time, ESP32_COMMAND_URL_BASE 
    if ESP32_COMMAND_URL_BASE is None: messagebox.showwarning("Network Error", "ESP32-CAM IP not set."); return False
    current_time = time.time()
    if current_time - last_command_time < COMMAND_SEND_INTERVAL: return False 
    pan = int(np.clip(pan, 0, 180)); tilt = int(np.clip(tilt, 30, 150)) 
    plaintext_command = f"S{pan}:{tilt}" 
    encrypted_command_b64 = encrypt_command(plaintext_command)
    if encrypted_command_b64 is None: print("Python: Command encryption failed."); return False
    actual_command_url = f"{ESP32_COMMAND_URL_BASE}/command"
    response_obj = None 
    try:
        headers = {'Content-Type': 'text/plain'} 
        print(f"Python: Sending to {actual_command_url}, Data (Encrypted B64): {encrypted_command_b64[:20]}...") 
        response_obj = requests.post(actual_command_url, data=encrypted_command_b64, headers=headers, timeout=2.0) 
        print(f"Python: HTTP Status: {response_obj.status_code}, ESP Response: '{response_obj.text}' for command {plaintext_command}")
        response_obj.raise_for_status() 
        current_pan = pan; current_tilt = tilt
        update_angle_labels(); last_command_time = current_time 
        return True
    except requests.exceptions.HTTPError as http_err: 
        print(f"Python: HTTP error sending command to {actual_command_url}: {http_err}")
        if response_obj is not None: print(f"Python: (Error Detail) Status: {response_obj.status_code}, Body: '{response_obj.text}'")
        else: print("Python: (Error Detail) No response object available in HTTPError.")
    except requests.exceptions.Timeout: print(f"Python: Error sending command to {actual_command_url}: Timeout")
    except requests.exceptions.RequestException as e: print(f"Python: Error sending command to {actual_command_url}: {e}")
    last_command_time = current_time 
    return False

#Video Processing and Object Detection (YOLO)
def process_frame_yolo(frame_input): 
    global current_pan, current_tilt, auto_tracking_enabled
    if frame_input is None or yolo_net is None: return frame_input 
    frame = frame_input.copy(); height, width, _ = frame.shape
    frame_center_x, frame_center_y = width // 2, height // 2
    blob = cv2.dnn.blobFromImage(frame, 1/255.0, (320,320), swapRB=True, crop=False)
    yolo_net.setInput(blob)
    try: outs = yolo_net.forward(output_layers)
    except Exception as e: print(f"Python: YOLO error: {e}"); return frame_input
    class_ids_detected, confidences_detected, boxes_detected = [], [], []
    TRACK_TARGET_CLASS = "person"; best_target_box = None; largest_target_area = 0
    for out in outs:
        for detection in out:
            scores = detection[5:]; class_id = np.argmax(scores); confidence = scores[class_id]
            if confidence > 0.4: 
                cx, cy = int(detection[0]*width), int(detection[1]*height); w, h = int(detection[2]*width), int(detection[3]*height)
                x, y = int(cx - w/2), int(cy - h/2)
                boxes_detected.append([x,y,w,h]); confidences_detected.append(float(confidence)); class_ids_detected.append(class_id)
                if classes[class_id] == TRACK_TARGET_CLASS:
                    area = w*h
                    if area > largest_target_area: largest_target_area=area; best_target_box=(cx,cy,w,h)
    indexes = cv2.dnn.NMSBoxes(boxes_detected, confidences_detected, 0.4, 0.3) 
    if indexes is not None and len(indexes) > 0:
        for i in indexes.flatten():
            label = str(classes[class_ids_detected[i]])
            if label == TRACK_TARGET_CLASS: 
                x,y,w,h = boxes_detected[i]; conf_score = confidences_detected[i]
                cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
                cv2.putText(frame, f"{label} {conf_score:.2f}", (x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),2)
    if auto_tracking_enabled and best_target_box:
        tcx, tcy, _, _ = best_target_box
        ex, ey = tcx - frame_center_x, tcy - frame_center_y
        gP, gT, dZ = 0.04, -0.04, 25; dP, dT = 0,0 
        if abs(ex)>dZ: dP = gP*ex
        if abs(ey)>dZ: dT = gT*ey
        dP = np.clip(dP, -2,2); dT = np.clip(dT, -2,2) 
        if abs(dP)>0.2 or abs(dT)>0.2: send_pan_tilt_command(current_pan+dP, current_tilt+dT)
        cv2.circle(frame, (int(tcx), int(tcy)), 7, (0,0,255), -1) 
    cv2.circle(frame, (frame_center_x, frame_center_y), 5, (255,0,0), -1) 
    return frame


#UDP Video Receiver Thread
def udp_video_receiver_thread(): 
    global stop_threads, udp_socket, video_frame_buffer, receiving_video_frame, last_valid_frame_time, decode_error_count, placeholder_image_tk, ESP32_COMMAND_URL_BASE
    
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        udp_socket.bind((UDP_VIDEO_IP_ADDRESS, UDP_VIDEO_PORT))
        print(f"Python: UDP Video Receiver Listening on {UDP_VIDEO_IP_ADDRESS}:{UDP_VIDEO_PORT}")
        udp_socket.settimeout(0.1) # Timeout for recvfrom
    except socket.error as e: messagebox.showerror("UDP Socket Error", f"Video socket bind failed: {e}"); stop_threads = True; return
    
    if ESP32_COMMAND_URL_BASE:
        try:
            ready_url = f"{ESP32_COMMAND_URL_BASE}/signal_video_ready"; 
            print(f"Python: Signaling ESP32 video ready at {ready_url}")
            response = requests.get(ready_url, timeout=2.0)
            if response.status_code == 200: print(f"Python: ESP32 acknowledged video ready: {response.text}")
            else: print(f"Python: ESP32 responded to video ready with {response.status_code}: {response.text}")
        except Exception as e: 
            print(f"Python: Failed to send video ready signal: {e}"); 
            messagebox.showwarning("ESP32 Comms", "Could not send 'video ready' signal.\nVideo stream might not start if ESP32 IP was not registered.")
    else: 
        print("Python: ESP32 IP not set, cannot send video ready signal."); 
        messagebox.showwarning("ESP32 Comms", "ESP32 IP not set.\nVideo stream might not start if ESP32 IP was not registered.")

    frame_width_display, frame_height_display = 640, 480
    if placeholder_image_tk is None: 
        placeholder_image_tk = create_placeholder_image(frame_width_display, frame_height_display)
    
    last_valid_frame_time = time.time() 

    while not stop_threads:
        try:
            data, addr = udp_socket.recvfrom(UDP_VIDEO_BUFFER_SIZE)
            if data == START_OF_FRAME_MARKER: 
                video_frame_buffer = bytearray()
                receiving_video_frame = True
                continue
            
            if data == END_OF_FRAME_MARKER:
                receiving_video_frame = False #Process frame after EOF
                if len(video_frame_buffer) > 0:
                    try:
                        np_arr = np.frombuffer(video_frame_buffer, np.uint8)
                        img_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                        if img_np is not None:
                            last_valid_frame_time = time.time() 
                            decode_error_count = 0 
                            processed_frame = process_frame_yolo(cv2.resize(img_np, (frame_width_display, frame_height_display)))
                            img_rgb = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
                            img_pil = Image.fromarray(img_rgb)
                            img_tk = ImageTk.PhotoImage(image=img_pil)
                            video_label.imgtk = img_tk #Keep reference
                            video_label.configure(image=img_tk)
                        else: 
                            decode_error_count += 1
                            # print(f"Python: UDP Video - cv2.imdecode returned None (Error count: {decode_error_count})") 
                    except Exception as e: 
                        decode_error_count += 1
                        print(f"Python: UDP Video - Error processing frame: {e}")
                video_frame_buffer = bytearray() # Clear buffer for next frame
                continue # Go back to recvfrom

            if receiving_video_frame:
                video_frame_buffer.extend(data)
        
        except socket.timeout: 
            #This timeout is for udp_socket.recvfrom()
            if receiving_video_frame and len(video_frame_buffer) > 0:
                #If we were in the middle of receiving a frame and timed out (0.1s with no new packets for this frame),
                #assume the rest of the frame (including EOF) is lost.
                print("Python: UDP Video - Timeout while actively receiving frame data. Discarding partial frame.")
                video_frame_buffer = bytearray()
                receiving_video_frame = False #Reset state to wait for a new SOF
            
            #Check if it's time to show "No Signal" placeholder (overall timeout)
            if time.time() - last_valid_frame_time > NO_SIGNAL_TIMEOUT:
                if 'video_label' in globals() and video_label.winfo_exists():
                    is_placeholder_showing = False
                    try:
                        if video_label.imgtk == placeholder_image_tk:
                            is_placeholder_showing = True
                    except AttributeError: pass #imgtk might not exist if no frame ever shown
                    
                    if not is_placeholder_showing:
                        print(f"Python: No valid video signal for {NO_SIGNAL_TIMEOUT}s. Displaying placeholder.")
                        if placeholder_image_tk is None: #Should have been created at startup
                             placeholder_image_tk = create_placeholder_image(frame_width_display,frame_height_display)
                        video_label.imgtk = placeholder_image_tk 
                        video_label.configure(image=placeholder_image_tk)
            continue 
        except Exception as e:
            if not stop_threads: print(f"Python: UDP Video Receiver Error: {e}"); time.sleep(0.05) 
    
    if udp_socket: udp_socket.close()
    print("Python: UDP Video Receiver thread stopped.")

#UDP Log Receiver Thread
def udp_log_receiver_thread(): # ... (same as before) ...
    global stop_threads, udp_log_socket, activity_log_entries
    udp_log_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        udp_log_socket.bind((UDP_LOG_IP_ADDRESS, UDP_LOG_PORT))
        print(f"Python: UDP Log Receiver Listening on {UDP_LOG_IP_ADDRESS}:{UDP_LOG_PORT}")
        udp_log_socket.settimeout(1.0) 
    except socket.error as e: messagebox.showerror("UDP Socket Error", f"Log socket bind failed: {e}"); print(f"Python: UDP Log Receiver - Failed to bind socket. Error: {e}"); return
    while not stop_threads:
        try:
            data, addr = udp_log_socket.recvfrom(UDP_LOG_BUFFER_SIZE)
            log_message = data.decode('utf-8').strip(); print(f"Python: UDP Log Received: {log_message}")
            if log_message.startswith("LOG_START:") or log_message.startswith("LOG_STOP:"):
                parts = log_message.split(':', 1); event_type = parts[0]; timestamp_str = parts[1] 
                if ',' in timestamp_str: 
                    date_part, time_part = timestamp_str.split(',', 1)
                    with log_lock: activity_log_entries.append((event_type, date_part, time_part))
                else: print(f"Python: Received malformed log timestamp: {timestamp_str}")
        except socket.timeout: continue 
        except Exception as e:
            if not stop_threads: print(f"Python: UDP Log Receiver Error: {e}"); time.sleep(0.1) 
    if udp_log_socket: udp_log_socket.close()
    print("Python: UDP Log Receiver thread stopped.")


#Email and CSV Functionality
def trigger_generate_csv_and_email(): 
    global recipient_email_entry
    if recipient_email_entry is None: messagebox.showerror("GUI Error", "Recipient email entry field not found."); return
    recipient_email = recipient_email_entry.get()
    if not recipient_email or "@" not in recipient_email or "." not in recipient_email: messagebox.showerror("Email Error", "Valid recipient email needed."); return
    generate_csv_and_email(recipient_email)

def generate_csv_and_email(recipient_email_addr): 
    if not activity_log_entries: messagebox.showinfo("Log Email", "No activity logs to send."); return
    filepath = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV files", "*.csv")], title="Save Activity Log As CSV")
    if not filepath: return 
    try:
        with open(filepath, 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile); csvwriter.writerow(['Event Type', 'Date', 'Time']) 
            with log_lock: 
                for entry in activity_log_entries: csvwriter.writerow(entry)
        print(f"Python: Activity log saved to {filepath}")
        if not messagebox.askyesno("Email Log", f"Log saved to {os.path.basename(filepath)}.\nEmail to {recipient_email_addr}?"): return
    except Exception as e: messagebox.showerror("CSV Error", f"Failed to save CSV: {e}"); return
    sender_email = simpledialog.askstring("Email Config", "Your Email (e.g., you@gmail.com):", parent=app)
    if not sender_email: return
    sender_password = simpledialog.askstring("Email Config", "Your Email Password/App Password:", show='*', parent=app)
    if not sender_password: return
    smtp_server = ""; smtp_port = 587 
    if "gmail.com" in sender_email.lower(): smtp_server = "smtp.gmail.com"
    elif "outlook.com" in sender_email.lower() or "hotmail.com" in sender_email.lower(): smtp_server = "smtp.office365.com"
    else:
        smtp_server = simpledialog.askstring("Email Config", "SMTP Server:", parent=app)
        if not smtp_server: return
        port_str = simpledialog.askstring("Email Config", "SMTP Port (587 TLS, 465 SSL):", initialvalue="587", parent=app)
        try: smtp_port = int(port_str)
        except: messagebox.showerror("Email Error", "Invalid SMTP port."); return
    subject = "ESP32-CAM Activity Log"; body = "Attached is the ESP32-CAM activity log."
    try:
        msg = MIMEMultipart(); msg['From'] = sender_email; msg['To'] = recipient_email_addr; msg['Subject'] = subject
        msg.attach(MIMEText(body, 'plain'))
        with open(filepath, "rb") as attachment: part = MIMEBase('application', 'octet-stream'); part.set_payload(attachment.read())
        encoders.encode_base64(part); part.add_header('Content-Disposition', f"attachment; filename={os.path.basename(filepath)}"); msg.attach(part)
        server = smtplib.SMTP(smtp_server, smtp_port); server.starttls(); server.login(sender_email, sender_password)
        server.sendmail(sender_email, recipient_email_addr, msg.as_string()); server.quit()
        messagebox.showinfo("Email Sent", f"Log sent to {recipient_email_addr}")
        print(f"Python: Email sent to {recipient_email_addr}")
    except Exception as e: messagebox.showerror("Email Error", f"Failed to send email: {e}"); print(f"Python: Failed to send email: {e}")

#GUI Functions
def toggle_auto_tracking(): global auto_tracking_enabled; auto_tracking_enabled = not auto_tracking_enabled; tracking_status_label.config(text=f"Auto Tracking: {'ON' if auto_tracking_enabled else 'OFF'}")
def manual_pan_left(): send_pan_tilt_command(current_pan - 10, current_tilt)
def manual_pan_right(): send_pan_tilt_command(current_pan + 10, current_tilt)
def manual_tilt_up(): send_pan_tilt_command(current_pan, current_tilt - 10)
def manual_tilt_down(): send_pan_tilt_command(current_pan, current_tilt + 10)
def center_servos(): send_pan_tilt_command(90, 90)
def update_angle_labels():
    if 'pan_angle_label' in globals() and 'tilt_angle_label' in globals():
        pan_angle_label.config(text=f"Pan: {current_pan}°"); tilt_angle_label.config(text=f"Tilt: {current_tilt}°")

def on_closing(): 
    global stop_threads, udp_socket, udp_log_socket 
    quit_confirmed = False
    if activity_log_entries:
        if messagebox.askyesno("Quit", "Save and email activity log before quitting?"): trigger_generate_csv_and_email(); quit_confirmed = messagebox.askokcancel("Quit", "Proceed to quit application?")
        elif messagebox.askokcancel("Quit", "Log not saved/emailed. Quit anyway?"): quit_confirmed = True
    else: 
        if messagebox.askokcancel("Quit", "Do you really want to quit?"): quit_confirmed = True
    if not quit_confirmed: return 
    print("Python: Closing application..."); stop_threads = True
    if 'udp_video_thread' in globals() and udp_video_thread.is_alive(): udp_video_thread.join(timeout=1) 
    if 'udp_log_thread' in globals() and udp_log_thread.is_alive(): udp_log_thread.join(timeout=1)
    if udp_socket: udp_socket.close(); print("Python: UDP video socket closed.")
    if udp_log_socket: udp_log_socket.close(); print("Python: UDP log socket closed.") 
    app.destroy()

#Main Application Setup
app = tk.Tk(); app.title("ESP32-CAM UDP Tracker with YOLO & mDNS (Encrypted Commands)")
video_label = ttk.Label(app); video_label.grid(row=0, column=0, columnspan=5, padx=10, pady=10, sticky="nsew") 
app.grid_rowconfigure(0, weight=1); app.grid_columnconfigure(0, weight=1); app.grid_columnconfigure(1, weight=1)
app.grid_columnconfigure(2, weight=1); app.grid_columnconfigure(3, weight=1); app.grid_columnconfigure(4, weight=1) 
controls_frame = ttk.LabelFrame(app, text="Controls & Logging"); controls_frame.grid(row=1, column=0, columnspan=5, padx=10, pady=5, sticky="ew") 
tracking_button = ttk.Button(controls_frame, text="Toggle Auto Tracking (T)", command=toggle_auto_tracking)
tracking_button.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
tracking_status_label = ttk.Label(controls_frame, text="Auto Tracking: OFF")
tracking_status_label.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
recipient_email_label = ttk.Label(controls_frame, text="Recipient Email:")
recipient_email_label.grid(row=0, column=2, padx=(10,0), pady=5, sticky="e")
recipient_email_entry = ttk.Entry(controls_frame, width=30)
recipient_email_entry.grid(row=0, column=3, padx=5, pady=5, sticky="ew")
send_email_button = ttk.Button(controls_frame, text="Send Log to Email", command=trigger_generate_csv_and_email)
send_email_button.grid(row=0, column=4, padx=5, pady=5, sticky="ew")
manual_label = ttk.Label(controls_frame, text="Manual Control:")
manual_label.grid(row=1, column=0, columnspan=5, pady=(5,0)) 
btn_tilt_up = ttk.Button(controls_frame, text="Tilt Up (↑)", command=manual_tilt_up)
btn_tilt_up.grid(row=2, column=2, padx=2, pady=1, sticky="ew") 
btn_pan_left = ttk.Button(controls_frame, text="Pan Left (←)", command=manual_pan_left)
btn_pan_left.grid(row=3, column=1, padx=2, pady=1, sticky="ew")
btn_center = ttk.Button(controls_frame, text="Center (C)", command=center_servos)
btn_center.grid(row=3, column=2, padx=2, pady=1, sticky="ew")
btn_pan_right = ttk.Button(controls_frame, text="Pan Right (→)", command=manual_pan_right)
btn_pan_right.grid(row=3, column=3, padx=2, pady=1, sticky="ew")
btn_tilt_down = ttk.Button(controls_frame, text="Tilt Down (↓)", command=manual_tilt_down)
btn_tilt_down.grid(row=4, column=2, padx=2, pady=1, sticky="ew") 
controls_frame.grid_columnconfigure(0, weight=1); controls_frame.grid_columnconfigure(1, weight=1)
controls_frame.grid_columnconfigure(2, weight=1); controls_frame.grid_columnconfigure(3, weight=2) 
controls_frame.grid_columnconfigure(4, weight=1)
angle_frame = ttk.LabelFrame(app, text="Current Angles")
angle_frame.grid(row=2, column=0, columnspan=5, padx=10, pady=5, sticky="ew") 
pan_angle_label = ttk.Label(angle_frame, text=f"Pan: {current_pan}°")
pan_angle_label.pack(side=tk.LEFT, padx=10, pady=2)
tilt_angle_label = ttk.Label(angle_frame, text=f"Tilt: {current_tilt}°")
tilt_angle_label.pack(side=tk.LEFT, padx=10, pady=2)
app.bind('<Left>', lambda e: manual_pan_left()); app.bind('<Right>', lambda e: manual_pan_right())
app.bind('<Up>', lambda e: manual_tilt_up()); app.bind('<Down>', lambda e: manual_tilt_down())
app.bind('c', lambda e: center_servos()); app.bind('C', lambda e: center_servos())
app.bind('t', lambda e: toggle_auto_tracking()); app.bind('T', lambda e: toggle_auto_tracking())

#Start Application
if __name__ == "__main__":
    if not discover_esp32_cam_ip():
        print("Python: mDNS discovery failed. Prompting for IP address.")
        initial_ip_for_prompt = ESP32_CAM_IP_ADDRESS if ESP32_CAM_IP_ADDRESS else "192.168.1.100" 
        user_prompt_ip = simpledialog.askstring("ESP32-CAM IP Configuration", 
                                           f"Could not find ESP32-CAM via mDNS.\n"
                                           f"Current IP: {initial_ip_for_prompt}\n\n"
                                           "Enter ESP32-CAM IP Address or leave blank to use current:", 
                                           parent=app, initialvalue=initial_ip_for_prompt)
        if user_prompt_ip is not None: 
            if user_prompt_ip.strip() == "": ESP32_CAM_IP_ADDRESS = initial_ip_for_prompt
            else: ESP32_CAM_IP_ADDRESS = user_prompt_ip.strip()
            ESP32_COMMAND_URL_BASE = f"http://{ESP32_CAM_IP_ADDRESS}:8081"
            print(f"Python: ESP32-CAM IP Address set to: {ESP32_CAM_IP_ADDRESS}")
        else: messagebox.showerror("Config Error", "ESP32-CAM IP Address config cancelled. Exiting."); app.destroy(); exit()
    
    if ESP32_CAM_IP_ADDRESS is None: messagebox.showerror("Config Error", "ESP32-CAM IP Address not set. Exiting."); app.destroy(); exit()

    if not register_python_client_with_esp32():
        messagebox.showwarning("ESP32-CAM UDP Target", "Could not register this PC's IP with ESP32-CAM.\n"
                                                       "ESP32-CAM might send UDP packets to a wrong/old IP if it doesn't have a default.")

    #Create and set placeholder image before starting threads
    placeholder_image_tk = create_placeholder_image(640, 480) 
    if 'video_label' in globals() and video_label: # Ensure video_label is created
        video_label.imgtk = placeholder_image_tk # Initialize this attribute
        video_label.configure(image=placeholder_image_tk) 

    if not load_yolo(): app.destroy(); exit()
    else:
        update_angle_labels() 
        udp_video_thread = threading.Thread(target=udp_video_receiver_thread, daemon=True)
        udp_video_thread.start()
        udp_log_thread = threading.Thread(target=udp_log_receiver_thread, daemon=True)
        udp_log_thread.start()
        app.protocol("WM_DELETE_WINDOW", on_closing) 
        app.mainloop()
        stop_threads = True 
        if 'udp_video_thread' in globals() and udp_video_thread.is_alive(): udp_video_thread.join(timeout=1)
        if 'udp_log_thread' in globals() and udp_log_thread.is_alive(): udp_log_thread.join(timeout=1) 
        if udp_socket: udp_socket.close()
        if udp_log_socket: udp_log_socket.close() 
        print("Python: Application closed.")
