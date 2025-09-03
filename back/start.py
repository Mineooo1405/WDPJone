import subprocess
import sys
import os
import signal
import time
from pathlib import Path
import threading
import queue
import tkinter as tk
from tkinter import scrolledtext, Frame, Label, Button, messagebox
from tkinter.constants import END, BOTH, X, Y, TOP, BOTTOM, LEFT, RIGHT, W, E, N, S
import re # Added for regex parsing

# Default port for the WebSocket bridge (direct_bridge.py)
DEFAULT_WS_BRIDGE_PORT = 9003
# Default port for a potential API backend (like the FastAPI in start_system.bat)
DEFAULT_API_PORT = 9004 # Currently unused by start_servers

# Ports expected to be used by the applications started by this script
FRONTEND_DEV_PORT = 3001 # As hardcoded in frontend_command in start_servers
DIRECT_BRIDGE_TCP_PORT = 12346 # Default from direct_bridge.py for control
DIRECT_BRIDGE_OTA_PORT = 12345 # Default from direct_bridge.py for OTA

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
            widget = self.backend_output_text 
            line = f"[System] {line}" 
        else: # Fallback for unknown prefixes like System-DEBUG, System-ERR
            widget = self.backend_output_text
            line = f"[{process_name}] {line}"

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
        
        # Pass the global output_queue to stop_servers
        stop_thread = threading.Thread(target=stop_servers, args=("GUI Stop Button", self.start_button, output_queue))
        stop_thread.daemon = True
        stop_thread.start()

    def on_closing_thread(self):
        # First, ask the user if they want to quit.
        # This messagebox needs a valid root window to operate.
        try:
            user_wants_to_quit = messagebox.askokcancel("Quit", "Do you want to quit? This will stop all running servers.")
        except tk.TclError as e:
            # This can happen if the window is already being destroyed by some other means
            # or if Tkinter's internal state is inconsistent.
            print(f"Tkinter error during messagebox: {e}. Assuming shutdown.")
            user_wants_to_quit = True # Proceed with shutdown if messagebox fails

        if user_wants_to_quit:
            self.update_output("System", "Window close requested. Initiating shutdown...")
            self.stop_event.set() 
            
            # Disable buttons to prevent further interaction during shutdown
            if self.start_button:
                try:
                    self.start_button.config(state=tk.DISABLED)
                except tk.TclError: pass # Ignore if already destroyed
            if self.stop_button:
                try:
                    self.stop_button.config(state=tk.DISABLED)
                except tk.TclError: pass # Ignore if already destroyed

            # Start the stop_servers thread
            thread = threading.Thread(target=stop_servers, args=("Window Close", self.start_button, output_queue))
            thread.daemon = True 
            thread.start()
            
            # Schedule the root window to be destroyed.
            # This should be one of the last things related to the GUI.
            # The 2000ms delay is to allow stop_servers to run and potentially send
            # its initial messages to the queue before the GUI is completely gone.
            self.root.after(2000, self.root.destroy)
        else:
            # User clicked "Cancel" on the messagebox.
            # Do nothing, allowing the window to remain open.
            # The default WM_DELETE_WINDOW action is prevented because this handler exists.
            self.update_output("System", "Quit cancelled by user.")
            return 

# --- Backend and Frontend Process Management --- (Outside GUI class)

def enqueue_output_gui(stream, q, prefix):
    try:
        for line in iter(stream.readline, ''): 
            if line: 
                q.put((prefix, line.strip())) 
            else: 
                break
        stream.close()
    except ValueError: 
        pass
    except Exception as e:
        q.put((prefix + "-ERR", f"Error reading stream: {str(e)}")) 

def run_command_gui(command, cwd=None):
    common_params = {
        "shell": True,
        "cwd": cwd,
        "stdout": subprocess.PIPE,
        "stderr": subprocess.PIPE,
        "text": True, 
        "encoding": 'utf-8', 
        "errors": 'replace' 
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
            preexec_fn=os.setsid # Important for process group killing
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
                # f.write(f"REACT_APP_API_URL=http://localhost:{DEFAULT_API_PORT}\n") # If you have an API backend
                # f.write(f"REACT_APP_WS_URL=ws://localhost:{DEFAULT_API_PORT}/ws") # If API backend has WebSocket
                f.write(f"PORT={FRONTEND_DEV_PORT}\n") # To suggest a port if Create React App respects it
            output_queue.put(("System", f"Created default {frontend_env_file}"))
        except Exception as e:
            output_queue.put(("System-ERR", f"Error creating default {frontend_env_file}: {e}"))

def start_servers(stop_event_ref):
    global managed_processes
    managed_processes.clear()
    
    threads = []

    script_file_path = Path(__file__)
    resolved_script_path = script_file_path.resolve()
    current_script_dir = resolved_script_path.parent
    
    backend_dir = current_script_dir 
    project_root_dir = current_script_dir.parent 
    frontend_dir = project_root_dir / "front"

    # Debug logs removed for cleanliness

    if not backend_dir.is_dir():
        output_queue.put(("System-ERR", f"Backend directory not found at {backend_dir}"))
        return
    if not frontend_dir.is_dir():
        output_queue.put(("System-ERR", f"Frontend directory not found at {frontend_dir}"))
        return

    check_and_create_frontend_env(frontend_dir)

    # --- Proactively clear known critical ports before starting servers ---
    do_preclear = os.environ.get("BRIDGE_PORT_PRECLEAR", "1").lower() not in ("0", "false", "no")
    if do_preclear:
        output_queue.put(("System", "Attempting to pre-clear known critical ports..."))
        ports_to_pre_clear = [
            DIRECT_BRIDGE_TCP_PORT,
            DIRECT_BRIDGE_OTA_PORT,
            DEFAULT_WS_BRIDGE_PORT, # direct_bridge.py also uses this
            FRONTEND_DEV_PORT       # For the frontend server
        ]
        for port_num in ports_to_pre_clear:
            force_close_port(port_num, output_queue)
        output_queue.put(("System", "Pre-clearing of ports attempt finished."))
    else:
        output_queue.put(("System", "Port pre-clear skipped (BRIDGE_PORT_PRECLEAR disabled)."))
    # --- End of proactive port clearing ---

    output_queue.put(("System", "Starting backend server (direct_bridge.py)..."))
    backend_process = run_command_gui(f"{sys.executable} direct_bridge.py", cwd=backend_dir)
    if backend_process.stdout is None or backend_process.stderr is None:
        output_queue.put(("System-ERR", "Failed to get stdout/stderr for backend process."))
        # Even if backend fails to start, proceed to attempt frontend start if desired,
        # or return here if backend is critical. For now, let's assume we might want to see frontend logs.
    else:
        managed_processes.append(("Backend", backend_process))
        be_out_thread = threading.Thread(target=enqueue_output_gui, args=(backend_process.stdout, output_queue, "Backend"))
        be_err_thread = threading.Thread(target=enqueue_output_gui, args=(backend_process.stderr, output_queue, "Backend-ERR"))
        threads.extend([be_out_thread, be_err_thread])
        be_out_thread.daemon = True; be_err_thread.daemon = True
        be_out_thread.start(); be_err_thread.start()

    output_queue.put(("System", "Waiting for backend to initialize (3 seconds)..."))
    for _ in range(30): # 3 seconds with 0.1 sleep
        if stop_event_ref.is_set(): 
            output_queue.put(("System", "Backend initialization interrupted by stop signal."))
            # If backend init is interrupted, we might not want to start frontend
            # or we might want to proceed to stop_servers logic.
            # For now, just return from start_servers.
            return 
        time.sleep(0.1)

    output_queue.put(("System", "Starting frontend server..."))
    frontend_command = f"PORT={FRONTEND_DEV_PORT} npm start"
    if sys.platform == "win32": 
        frontend_command = f"set PORT={FRONTEND_DEV_PORT} && npm start"

    frontend_process = run_command_gui(frontend_command, cwd=frontend_dir)
    if frontend_process.stdout is None or frontend_process.stderr is None:
        output_queue.put(("System-ERR", "Failed to get stdout/stderr for frontend process."))
    else:
        managed_processes.append(("Frontend", frontend_process))
        fe_out_thread = threading.Thread(target=enqueue_output_gui, args=(frontend_process.stdout, output_queue, "Frontend"))
        fe_err_thread = threading.Thread(target=enqueue_output_gui, args=(frontend_process.stderr, output_queue, "Frontend-ERR"))
        threads.extend([fe_out_thread, fe_err_thread])
        fe_out_thread.daemon = True; fe_err_thread.daemon = True
        fe_out_thread.start(); fe_err_thread.start()

    output_queue.put(("System", "--- Servers Initialized ---"))

    while not stop_event_ref.is_set():
        all_processes_exited = True
        # Check only processes that were successfully added to managed_processes
        active_managed_processes = [p for _, p in managed_processes if p is not None]

        if not active_managed_processes and managed_processes: # If list had entries but all are None (failed to start)
             output_queue.put(("System", "No managed processes were successfully started or all have exited prematurely."))
             break # Exit monitoring loop

        if not active_managed_processes and not managed_processes: # No processes were even attempted to be managed
            output_queue.put(("System", "No processes were configured to be managed."))
            break


        for process in active_managed_processes:
            if process.poll() is None:
                all_processes_exited = False
                break
        
        if all_processes_exited and active_managed_processes: # Ensure there were active processes to begin with
            output_queue.put(("System", "All managed server processes have exited."))
            break
        elif not active_managed_processes and managed_processes: # All initial attempts failed
            pass # Message already printed above
        
        time.sleep(0.5)
    
    output_queue.put(("System", "Server starting/monitoring thread finished."))


def force_close_port(port, q_ref):
    q_ref.put(("System", f"Attempting to force close port {port}..."))
    killed_pids_count = 0
    try:
        if sys.platform == "win32":
            cmd_find = f'netstat -ano -p TCP | findstr ":{port}" | findstr "LISTENING"'
            result_find = subprocess.run(cmd_find, shell=True, capture_output=True, text=True, timeout=10)
            if result_find.stdout:
                for line in result_find.stdout.strip().split('\n'):
                    if not line.strip(): continue
                    parts = line.strip().split()
                    if len(parts) >= 4: # TCP Address Address State PID
                        pid_str = parts[-1]
                        local_address_part = parts[1]
                        if f":{port}" == local_address_part.split(':')[-1]: # Check exact port match
                            try:
                                pid = int(pid_str)
                                q_ref.put(("System", f"Found PID {pid} on port {port}. Attempting taskkill /F /PID {pid}"))
                                cmd_kill = f"taskkill /F /PID {pid}"
                                result_kill = subprocess.run(cmd_kill, shell=True, capture_output=True, text=True, timeout=5)
                                if result_kill.returncode == 0 or \
                                   ("reason: there is no running instance of the task" in result_kill.stderr.lower()):
                                    q_ref.put(("System", f"Kill signal for PID {pid} on port {port} sent or process already gone."))
                                    killed_pids_count +=1
                                else:
                                    q_ref.put(("System-ERR", f"Failed to kill PID {pid} on port {port}. RC: {result_kill.returncode}, Err: {result_kill.stderr.strip()}"))
                            except ValueError:
                                q_ref.put(("System-WARN", f"Non-integer PID '{pid_str}' from netstat: {line}"))
                            except Exception as e_kill:
                                q_ref.put(("System-ERR", f"Exception killing PID {pid_str} for port {port}: {e_kill}"))
            else:
                q_ref.put(("System", f"No processes found listening on port {port} (Windows)."))
        else: # Linux / macOS
            # Try 'ss' first
            cmd_ss = f"ss -tulnp | grep ':{port}'"
            found_by_ss = False
            try:
                result_ss = subprocess.run(cmd_ss, shell=True, capture_output=True, text=True, timeout=10)
                if result_ss.stdout:
                    pid_pattern = re.compile(r'pid=(\d+)')
                    for line in result_ss.stdout.strip().split('\n'):
                        if "LISTEN" not in line.upper(): continue
                        match = pid_pattern.search(line)
                        if match:
                            pid = int(match.group(1))
                            q_ref.put(("System", f"Found PID {pid} on port {port} (via ss). Attempting kill -9 {pid}"))
                            try:
                                os.kill(pid, signal.SIGKILL)
                                q_ref.put(("System", f"SIGKILL sent to PID {pid} for port {port}."))
                                killed_pids_count += 1
                            except ProcessLookupError:
                                q_ref.put(("System", f"PID {pid} for port {port} already gone (ProcessLookupError)."))
                            except Exception as e_kill:
                                q_ref.put(("System-ERR", f"Exception sending SIGKILL to PID {pid} for port {port}: {e_kill}"))
                            found_by_ss = True
            except FileNotFoundError:
                q_ref.put(("System-WARN", "'ss' command not found. Will try 'netstat'."))
            except Exception as e_ss_cmd:
                q_ref.put(("System-ERR", f"Error executing 'ss' for port {port}: {e_ss_cmd}"))

            if not found_by_ss:
                cmd_netstat = f"netstat -tulnp | grep ':{port}'"
                try:
                    result_netstat = subprocess.run(cmd_netstat, shell=True, capture_output=True, text=True, timeout=10)
                    if result_netstat.stdout:
                        pid_pattern_netstat = re.compile(r'(\d+)/')
                        for line in result_netstat.stdout.strip().split('\n'):
                            if "LISTEN" not in line.upper(): continue
                            match = pid_pattern_netstat.search(line)
                            if match:
                                pid = int(match.group(1))
                                q_ref.put(("System", f"Found PID {pid} on port {port} (via netstat). Attempting kill -9 {pid}"))
                                try:
                                    os.kill(pid, signal.SIGKILL)
                                    q_ref.put(("System", f"SIGKILL sent to PID {pid} for port {port}."))
                                    killed_pids_count += 1
                                except ProcessLookupError:
                                    q_ref.put(("System", f"PID {pid} for port {port} already gone (ProcessLookupError)."))
                                except Exception as e_kill:
                                    q_ref.put(("System-ERR", f"Exception sending SIGKILL to PID {pid} for port {port}: {e_kill}"))
                    elif not found_by_ss : # Only say no processes if ss also found none
                        q_ref.put(("System", f"No processes found listening on port {port} (Linux/macOS - ss/netstat)."))
                except FileNotFoundError:
                    q_ref.put(("System-ERR", "'netstat' command not found (and 'ss' failed or also not found)."))
                except Exception as e_netstat_cmd:
                    q_ref.put(("System-ERR", f"Error executing 'netstat' for port {port}: {e_netstat_cmd}"))
        
    except subprocess.TimeoutExpired:
        q_ref.put(("System-ERR", f"Timeout executing command to find/kill process on port {port}."))
    except FileNotFoundError as e_fnf: # For netstat/ss/taskkill itself
        q_ref.put(("System-ERR", f"A required command for port cleanup not found: {e_fnf}"))
    except Exception as e:
        q_ref.put(("System-ERR", f"Unexpected error in force_close_port for port {port}: {e}"))
    
    if killed_pids_count > 0:
        q_ref.put(("System", f"Force close for port {port}: {killed_pids_count} process(es) targeted."))
    else:
        q_ref.put(("System", f"Force close for port {port}: No new processes killed (either none found or errors occurred)."))


def stop_servers(source_description="Unknown", start_button_ref=None, q_ref=None): # Added q_ref
    global managed_processes
    if q_ref is None: # Fallback if not passed, though AppGUI methods should pass it
        q_ref = output_queue

    q_ref.put(("System", f"Shutting down servers (triggered by: {source_description})..."))
    
    for name, process in list(managed_processes): # Iterate over a copy
        if process.poll() is None: # Check if process is still running
            q_ref.put(("System", f"Stopping {name} server (PID: {process.pid})..."))
            try:
                if sys.platform == "win32":
                    # Terminate the entire process group/tree
                    kill_command = f"taskkill /F /T /PID {process.pid}"
                    result = subprocess.run(kill_command, shell=True, capture_output=True, text=True, timeout=10)
                    # Check if successful or if process was already gone
                    if result.returncode == 0 or \
                       ("reason: there is no running instance of the task" in result.stderr.lower()):
                        q_ref.put(("System", f"{name} (PID: {process.pid}) taskkill signaled or process already gone."))
                    else:
                        q_ref.put(("System-ERR", f"taskkill for {name} (PID: {process.pid}) failed. RC:{result.returncode} Stderr: {result.stderr.strip()}"))
                else: # Linux / macOS
                    pgid = os.getpgid(process.pid)
                    os.killpg(pgid, signal.SIGTERM) # Send SIGTERM to the process group
                    q_ref.put(("System", f"{name} (PID: {process.pid}, PGID: {pgid}) group signaled SIGTERM."))
                
                process.wait(timeout=10) # Wait for graceful termination
                q_ref.put(("System", f"{name} server stopped (waited)."))
            except subprocess.TimeoutExpired:
                q_ref.put(("System-WARN", f"Timeout waiting for {name} (PID: {process.pid}) to stop gracefully. Forcing kill."))
                try:
                    if sys.platform == "win32":
                        # Force kill the process tree again
                        subprocess.run(f"taskkill /F /T /PID {process.pid}", shell=True, capture_output=True, text=True, timeout=5)
                    else:
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL) # Force kill process group
                    process.wait(timeout=5) # Wait for kill to complete
                    q_ref.put(("System", f"{name} server killed (forcefully)."))
                except ProcessLookupError: # Process might have died in the meantime
                     q_ref.put(("System", f"{name} (PID: {process.pid}) process already gone before forceful kill."))
                except Exception as e_force_kill:
                    q_ref.put(("System-ERR", f"Error during forceful kill of {name} (PID: {process.pid}): {e_force_kill}"))
            except ProcessLookupError: # if process died before os.killpg or during wait
                 q_ref.put(("System", f"{name} (PID: {process.pid}) process already gone before explicit stop/kill action completed."))
            except Exception as e_kill: # Other errors during initial stop attempt
                q_ref.put(("System-ERR", f"Error stopping {name} (PID: {process.pid}): {e_kill}. Attempting final kill."))
                try:
                    process.kill() # Fallback to simple kill on the process object
                    process.wait(timeout=5)
                    q_ref.put(("System", f"{name} server killed (fallback)."))
                except Exception as e_final_kill:
                    q_ref.put(("System-ERR", f"Final kill attempt for {name} (PID: {process.pid}) also failed: {e_final_kill}"))
        else:
            q_ref.put(("System", f"{name} server (PID: {process.pid}) was already stopped (Code: {process.returncode})."))
    
    managed_processes.clear() # Clear the list of managed processes
    q_ref.put(("System", "Initial server process shutdown completed."))
    q_ref.put(("System", "Now attempting to force free known ports if still in use..."))

    ports_to_force_close = [
        DEFAULT_WS_BRIDGE_PORT, 
        FRONTEND_DEV_PORT, 
        DIRECT_BRIDGE_TCP_PORT, 
        DIRECT_BRIDGE_OTA_PORT
    ]

    for port_num in ports_to_force_close:
        force_close_port(port_num, q_ref)

    q_ref.put(("System", "All server stopping procedures completed, including port cleanup attempts."))
    if start_button_ref and isinstance(start_button_ref, tk.Button):
        try:
            # Ensure GUI updates happen on the main thread if called from a different thread
            start_button_ref.master.after(0, lambda: start_button_ref.config(state=tk.NORMAL))
        except tk.TclError: # Widget might be destroyed
            pass # Silently ignore if widget is gone

if __name__ == "__main__":
    root = tk.Tk()
    app = AppGUI(root)
    root.mainloop()
    
    # This part runs after the Tkinter mainloop has exited
    print("Tkinter mainloop exited. Ensuring all processes are stopped as a fallback.")
    # Check if any managed processes were somehow left (should be cleared by stop_servers)
    # or if stop_event was not set properly before exit.
    # A more direct check might be needed if stop_servers wasn't guaranteed to run on exit.
    # For now, assume stop_servers was called by on_closing_thread.
    # If we want a truly robust fallback, we might need to re-check PIDs if stored, or ports.
    # The current on_closing_thread calls stop_servers, so this fallback here is mostly a safety net print.
    
    # If stop_servers needs to be called here, it needs the output_queue.
    # However, the GUI is gone, so output_queue might not be processed.
    # For simplicity, we rely on on_closing_thread to have handled it.
    # If you need a more robust non-GUI fallback, it would look different.
    
    print("Application finished.")
