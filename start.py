import subprocess
import sys
import os
import signal
import time
from pathlib import Path
import threading
import queue
import tkinter as tk
from tkinter import scrolledtext, Frame, Label, Button, Toplevel, messagebox
from tkinter.constants import END, BOTH, X, Y, TOP, BOTTOM, LEFT, RIGHT, W, E, N, S

# Default port for the WebSocket bridge (direct_bridge.py)
DEFAULT_WS_BRIDGE_PORT = 9003
# Default port for a potential API backend (like the FastAPI in start_system.bat)
DEFAULT_API_PORT = 9004

# Global list to keep track of processes to be managed by the GUI
managed_processes = []
# Global queue for all output
output_queue = queue.Queue()

class AppGUI:
    def __init__(self, root_tk):
        self.root = root_tk
        self.root.title("System Runner")
        self.root.geometry("800x600")

        # Main frame
        main_frame = Frame(self.root)
        main_frame.pack(fill=BOTH, expand=True, padx=5, pady=5)

        # Control frame
        control_frame = Frame(main_frame, pady=5)
        control_frame.pack(fill=X, side=TOP)

        self.start_button = Button(control_frame, text="Start All", command=self.start_all_servers_thread)
        self.start_button.pack(side=LEFT, padx=5)

        self.stop_button = Button(control_frame, text="Stop All", command=self.confirm_stop_all_servers_thread, state=tk.DISABLED)
        self.stop_button.pack(side=LEFT, padx=5)

        # Output Area Frame
        output_area_frame = Frame(main_frame)
        output_area_frame.pack(fill=BOTH, expand=True)
        output_area_frame.columnconfigure(0, weight=1)
        output_area_frame.columnconfigure(1, weight=1)
        output_area_frame.rowconfigure(1, weight=1)

        Label(output_area_frame, text="Backend Output", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky=W, pady=(0,2))
        self.backend_output_text = scrolledtext.ScrolledText(output_area_frame, wrap=tk.WORD, height=15, font=("Consolas", 9))
        self.backend_output_text.grid(row=1, column=0, sticky=N+S+E+W, padx=(0,2))
        self.backend_output_text.configure(state='disabled') # Start disabled

        Label(output_area_frame, text="Frontend Output", font=("Arial", 10, "bold")).grid(row=0, column=1, sticky=W, pady=(0,2))
        self.frontend_output_text = scrolledtext.ScrolledText(output_area_frame, wrap=tk.WORD, height=15, font=("Consolas", 9))
        self.frontend_output_text.grid(row=1, column=1, sticky=N+S+E+W, padx=(2,0))
        self.frontend_output_text.configure(state='disabled') # Start disabled

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing_thread)
        self.check_output_queue()

        self.threads = []
        self.stop_event = threading.Event() # Event to signal threads to stop

    def update_output(self, process_name, line):
        if "Backend" in process_name:
            widget = self.backend_output_text
        elif "Frontend" in process_name:
            widget = self.frontend_output_text
        elif "System" in process_name: # For system messages from start.py itself
            # Decide if you want a separate system log or append to one of the existing
            # For now, let's append to backend as it's more of a control log
            widget = self.backend_output_text 
            line = f"[System] {line}" # Prefix system messages clearly
        else:
            print(f"[UnknownProcess-{process_name}] {line}") # Fallback to console
            return
        
        widget.configure(state='normal')
        widget.insert(END, line + "\n")
        widget.configure(state='disabled')
        widget.see(END)

    def check_output_queue(self):
        try:
            while True: 
                source_process, line_content = output_queue.get_nowait()
                self.update_output(source_process, line_content)
        except queue.Empty:
            pass 
        
        self.root.after(100, self.check_output_queue)
    
    def start_all_servers_thread(self):
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.backend_output_text.configure(state='normal')
        self.backend_output_text.delete(1.0, END)
        self.backend_output_text.configure(state='disabled')
        self.frontend_output_text.configure(state='normal')
        self.frontend_output_text.delete(1.0, END)
        self.frontend_output_text.configure(state='disabled')
        
        self.stop_event.clear() 
        thread = threading.Thread(target=start_servers, args=(self.stop_event,))
        self.threads.append(thread)
        thread.daemon = True
        thread.start()

    def confirm_stop_all_servers_thread(self):
        if messagebox.askyesno("Confirm Stop", "Are you sure you want to stop all servers?"):
            self.stop_all_servers_thread()

    def stop_all_servers_thread(self):
        self.update_output("System", "Stop All button pressed. Initiating shutdown...")
        self.stop_button.config(state=tk.DISABLED) 
        self.stop_event.set()
        
        stop_thread = threading.Thread(target=stop_servers, args=("GUI Stop Button", self.start_button))
        stop_thread.daemon = True
        stop_thread.start()

    def on_closing_thread(self):
        if messagebox.askokcancel("Quit", "Do you want to quit? This will stop all running servers."):
            self.update_output("System", "Window close requested. Initiating shutdown...")
            self.stop_event.set() 
            thread = threading.Thread(target=stop_servers, args=("Window Close", self.start_button))
            thread.daemon = True 
            thread.start()
            self.root.after(1000, self.root.destroy) 

# --- Backend and Frontend Process Management --- (Outside GUI class)

def enqueue_output_gui(stream, q, prefix):
    try:
        # When text=True in Popen, readline() returns str, no decode needed.
        for line in iter(stream.readline, ''): 
            if line: # Ensure line is not empty
                q.put((prefix, line.strip())) 
            else: # Empty line might mean EOF or stream closed
                break
        stream.close()
    except ValueError: # Stream might be closed already
        pass
    except Exception as e:
        # Ensure the exception itself is a string for the queue
        q.put((prefix + "-ERR", f"Error reading stream: {str(e)}")) 

def run_command_gui(command, cwd=None):
    # text=True makes stdout/stderr text streams, encoding specifies how to decode bytes.
    common_params = {
        "shell": True,
        "cwd": cwd,
        "stdout": subprocess.PIPE,
        "stderr": subprocess.PIPE,
        "text": True, 
        "encoding": 'utf-8', 
        "errors": 'replace' # Replace undecodable characters
    }
    if sys.platform == "win32":
        process = subprocess.Popen(
            command,
            **common_params,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP
        )
    else:
        process = subprocess.Popen(
            command,
            **common_params,
            preexec_fn=os.setsid
        )
    return process

def check_and_create_frontend_env(frontend_dir):
    frontend_env_file = frontend_dir / ".env"
    if not frontend_env_file.exists():
        output_queue.put(("System", f"Warning: Frontend .env file not found at {frontend_env_file}. Creating default..."))
        try:
            with open(frontend_env_file, "w") as f:
                f.write("# Frontend environment variables (auto-generated by start.py)\n")
                f.write(f"REACT_APP_WS_BRIDGE_URL=ws://localhost:{DEFAULT_WS_BRIDGE_PORT}\n")
                f.write(f"REACT_APP_API_URL=http://localhost:{DEFAULT_API_PORT}\n")
                f.write(f"REACT_APP_WS_URL=ws://localhost:{DEFAULT_API_PORT}/ws\n")
            output_queue.put(("System", f"Created default {frontend_env_file}"))
        except Exception as e:
            output_queue.put(("System-ERR", f"Error creating default {frontend_env_file}: {e}"))

def start_servers(stop_event_ref):
    global managed_processes
    managed_processes.clear()
    
    threads = []

    project_root = Path(__file__).resolve().parent
    backend_dir = project_root / "back"
    frontend_dir = project_root / "front"

    if not backend_dir.is_dir():
        output_queue.put(("System-ERR", f"Backend directory not found at {backend_dir}"))
        return
    if not frontend_dir.is_dir():
        output_queue.put(("System-ERR", f"Frontend directory not found at {frontend_dir}"))
        return

    check_and_create_frontend_env(frontend_dir)

    output_queue.put(("System", "Starting backend server (direct_bridge.py)..."))
    backend_process = run_command_gui("python direct_bridge.py", cwd=backend_dir)
    if backend_process.stdout is None or backend_process.stderr is None:
        output_queue.put(("System-ERR", "Failed to get stdout/stderr for backend process."))
        return
    managed_processes.append(("Backend", backend_process))
    
    be_out_thread = threading.Thread(target=enqueue_output_gui, args=(backend_process.stdout, output_queue, "Backend"))
    be_err_thread = threading.Thread(target=enqueue_output_gui, args=(backend_process.stderr, output_queue, "Backend-ERR"))
    threads.extend([be_out_thread, be_err_thread])
    be_out_thread.daemon = True
    be_err_thread.daemon = True
    be_out_thread.start()
    be_err_thread.start()

    output_queue.put(("System", "Waiting for backend to initialize (3 seconds)..."))
    for _ in range(30):
        if stop_event_ref.is_set(): 
            output_queue.put(("System", "Backend initialization interrupted by stop signal."))
            return 
        time.sleep(0.1)

    output_queue.put(("System", "Starting frontend server..."))
    frontend_command = "npm start"
    frontend_process = run_command_gui(frontend_command, cwd=frontend_dir)
    if frontend_process.stdout is None or frontend_process.stderr is None:
        output_queue.put(("System-ERR", "Failed to get stdout/stderr for frontend process."))
        return
    managed_processes.append(("Frontend", frontend_process))

    fe_out_thread = threading.Thread(target=enqueue_output_gui, args=(frontend_process.stdout, output_queue, "Frontend"))
    fe_err_thread = threading.Thread(target=enqueue_output_gui, args=(frontend_process.stderr, output_queue, "Frontend-ERR"))
    threads.extend([fe_out_thread, fe_err_thread])
    fe_out_thread.daemon = True
    fe_err_thread.daemon = True
    fe_out_thread.start()
    fe_err_thread.start()

    output_queue.put(("System", "--- Servers Initialized ---"))

    while not stop_event_ref.is_set():
        all_exited_gracefully = True
        active_processes = False
        for name, process in managed_processes:
            if process.poll() is None:
                active_processes = True
                all_exited_gracefully = False
                break
        
        if not active_processes and all_exited_gracefully:
            output_queue.put(("System", "All server processes have exited."))
            break
        elif not active_processes and not all_exited_gracefully:
            output_queue.put(("System", "Some server processes may have exited prematurely or are in an unknown state."))
            break

        time.sleep(0.5)
    
    output_queue.put(("System", "Server starting/monitoring thread finished."))

def stop_servers(source_description="Unknown", start_button_ref=None):
    global managed_processes
    output_queue.put(("System", f"Shutting down servers (triggered by: {source_description})..."))
    
    for name, process in list(managed_processes):
        if process.poll() is None:
            output_queue.put(("System", f"Stopping {name} server (PID: {process.pid})..."))
            try:
                if sys.platform == "win32":
                    kill_command = f"taskkill /F /T /PID {process.pid}"
                    result = subprocess.run(kill_command, shell=True, capture_output=True, text=True, timeout=10)
                    if result.returncode == 0:
                        output_queue.put(("System", f"{name} (PID: {process.pid}) taskkill signaled."))
                    else:
                        output_queue.put(("System-ERR", f"taskkill for {name} (PID: {process.pid}) failed or process already gone. Stderr: {result.stderr.strip()}"))
                else:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    output_queue.put(("System", f"{name} (PID: {process.pid}) group signaled SIGTERM."))
                
                process.wait(timeout=10)
                output_queue.put(("System", f"{name} server stopped (waited)."))
            except subprocess.TimeoutExpired:
                output_queue.put(("System-ERR", f"Timeout waiting for {name} to stop. Forcing kill."))
                process.kill()
                process.wait()
                output_queue.put(("System", f"{name} server killed."))
            except Exception as e_kill:
                output_queue.put(("System-ERR", f"Error stopping {name}: {e_kill}. Forcing kill."))
                process.kill()
                process.wait()
                output_queue.put(("System", f"{name} server killed (due to error)."))
        else:
            output_queue.put(("System", f"{name} server was already stopped (PID: {process.pid}, Code: {process.returncode})."))
    
    managed_processes.clear()
    output_queue.put(("System", "All server stopping procedures completed."))
    if start_button_ref and isinstance(start_button_ref, tk.Button):
        try:
            start_button_ref.config(state=tk.NORMAL)
        except tk.TclError:
            pass

if __name__ == "__main__":
    root = tk.Tk()
    app = AppGUI(root)
    root.mainloop()
    
    print("Tkinter mainloop exited. Ensuring all processes are stopped as a fallback.")
    if any(p.poll() is None for _, p in managed_processes):
        print("Some processes might still be running post-GUI. Attempting final stop...")
        # Calling stop_servers here without a valid start_button_ref, so pass None or handle it.
        stop_servers("Post-mainloop-exit-fallback", None)
    
    print("Application finished.")
