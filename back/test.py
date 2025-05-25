import asyncio
import json
import random
import time
import logging
import socket

# --- Configuration ---
SERVER_IP = "127.0.0.1"  # IP address of the machine running direct_bridge.py
SERVER_PORT = 12346       # The TCP Control Port of direct_bridge.py
ROBOT_ID = "sim_robot_test_01" # An identifier for this simulated robot
SEND_INTERVAL_ENCODER = 2.0  # Seconds between sending encoder data
SEND_INTERVAL_IMU = 2.0     # Seconds between sending IMU data
SEND_INTERVAL_LOG = 5.0     # Seconds between sending log messages
RECONNECT_DELAY = 5.0      # Seconds to wait before attempting reconnection

# --- Logging Setup ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("RobotSim")

# --- Data Simulation Functions ---

def generate_encoder_data():
    """Generates a simulated encoder reading."""
    rpm1 = random.uniform(10.0, 60.0) * random.choice([-1, 1, 0.5, 0])
    rpm2 = random.uniform(10.0, 60.0) * random.choice([-1, 1, 0.5, 0])
    rpm3 = random.uniform(10.0, 60.0) * random.choice([-1, 1, 0.5, 0])
    return {
        "type": "encoder", 
        "id": ROBOT_ID,
        "data": [round(rpm1, 2), round(rpm2, 2), round(rpm3, 2)]
    }

def generate_imu_data():
    """Generates simulated IMU data with raw accelerometer values."""
    heading = (time.time() * 10) % 360 # Simulate changing heading
    pitch = random.uniform(-5.0, 5.0)
    roll = random.uniform(-5.0, 5.0)

    qx = random.uniform(-0.1, 0.1)
    qy = random.uniform(-0.1, 0.1)
    qz = random.uniform(-0.1, 0.1)
    qw = (1.0 - qx**2 - qy**2 - qz**2)**0.5 # Ensure it's a valid quaternion component

    # Simulate components of linear acceleration
    sim_lin_accel_x = random.gauss(0, 0.2)
    sim_lin_accel_y = random.gauss(0, 0.2)
    sim_lin_accel_z = random.gauss(0, 0.1)

    # Simulate components of gravity vector (simplified, assumes Z is mostly down)
    # In a real scenario, these would be derived from orientation if IMU provides them
    # For simulation, we can make them small if the robot is mostly level
    sim_grav_x = roll * 0.1 # Simplified: gravity component due to roll
    sim_grav_y = pitch * 0.1 # Simplified: gravity component due to pitch
    sim_grav_z = -9.8 + sim_lin_accel_z * 0.05 # Z-gravity dominant, slightly affected by Z linear accel noise

    # Raw accelerometer data is the sum of linear acceleration and gravity effect
    raw_accel_x = sim_lin_accel_x + sim_grav_x
    raw_accel_y = sim_lin_accel_y + sim_grav_y
    raw_accel_z = sim_lin_accel_z + sim_grav_z


    data_payload = {
            "time": int(time.time() * 1_000_000), # Microseconds
            "euler": [round(heading, 2), round(pitch, 2), round(roll, 2)],
            "quaternion": [round(qw, 4), round(qx, 4), round(qy, 4), round(qz, 4)],
    }

    return {
        "id": ROBOT_ID,
        "type": "bno055", # Corrected type
        "data": data_payload
    }

def generate_log_data():
    """Generates a simulated log message."""
    levels = ["INFO", "WARN", "ERROR", "DEBUG"]
    messages = [
        "Motor 1 speed nominal.",
        "Approaching waypoint.",
        "Battery level low.",
        "Obstacle detected.",
        "Path clear.",
        f"Current heading: {random.uniform(0, 360):.1f}",
    ]
    return {
        "type": "log_data", # Corrected type
        "id": ROBOT_ID,
        "message": random.choice(messages),
        "level": random.choice(levels)
    }

async def send_sensor_data(writer):
    """Periodically sends simulated sensor data."""
    last_sent_encoder = 0
    last_sent_imu = 0
    last_sent_log = 0
    logger.info("Sensor data sending task started.")

    while True:
        now = time.monotonic()
        data_to_send = None
        
        # Determine which data to send based on intervals
        if now - last_sent_imu >= SEND_INTERVAL_IMU:
            data_to_send = generate_imu_data()
            last_sent_imu = now
        elif now - last_sent_encoder >= SEND_INTERVAL_ENCODER:
             data_to_send = generate_encoder_data()
             last_sent_encoder = now
        elif now - last_sent_log >= SEND_INTERVAL_LOG:
             data_to_send = generate_log_data()
             last_sent_log = now

        if data_to_send:
            try:
                json_str = json.dumps(data_to_send) + '\n'
                logger.info(f"Sending {data_to_send.get('type', 'UNKNOWN_TYPE').upper()} data: {json_str.strip()}")
                writer.write(json_str.encode('utf-8'))
                await writer.drain()
            except (ConnectionResetError, BrokenPipeError, OSError) as e:
                logger.error(f"Send Error: Connection lost while sending. {e}")
                return
            except Exception as e:
                logger.error(f"Send Error: Unexpected error during send: {e}", exc_info=True)
                return

        await asyncio.sleep(0.05) # Reduced sleep for more responsive check, actual send rate controlled by intervals


async def receive_commands(reader):
    """Listens for and logs commands from the server."""
    is_first_message = True
    logger.info("Command receiving task started.")
    while True:
        try:
            line_bytes = await reader.readline()
            if not line_bytes:
                logger.warning("Receive Error: Connection closed by server.")
                return

            command_str = line_bytes.decode('utf-8').strip()
            if not command_str:
                continue

            if is_first_message:
                try:
                    ack_data = json.loads(command_str)
                    if ack_data.get("type") == "connection_acknowledgement":
                        logger.info(f"Received SERVER ACK: {ack_data}")
                    else:
                        logger.info(f"Received First Server Msg (Expected Ack, got other JSON): {command_str}")
                except json.JSONDecodeError:
                     logger.info(f"Received First Server Msg (Non-JSON): {command_str}")
                is_first_message = False
            else:
                logger.info(f"Received Server Command: {command_str}")

        except (ConnectionResetError, BrokenPipeError, OSError) as e:
            logger.error(f"Receive Error: Connection lost. {e}")
            return
        except UnicodeDecodeError as e:
            logger.error(f"Receive Error: Could not decode received data: {line_bytes}. Error: {e}")
        except Exception as e:
            logger.error(f"Receive Error: Unexpected error during receive: {e}", exc_info=True)
            return

# --- Main Client Logic ---

async def robot_client_main():
    """Main loop to connect, run tasks, and reconnect."""
    while True:
        reader, writer = None, None
        try:
            logger.info(f"Attempting to connect to server {SERVER_IP}:{SERVER_PORT}...")
            reader, writer = await asyncio.open_connection(SERVER_IP, SERVER_PORT)
            peername = writer.get_extra_info('peername')
            logger.info(f"Successfully connected to server: {peername}")

            # Send registration packet
            registration_packet = {
                "type": "registration",
                "robot_id": ROBOT_ID,
                "capabilities": ["imu_data", "encoder_data", "log_data"], # Updated capabilities
                "timestamp": time.time()
            }
            reg_json_str = json.dumps(registration_packet) + '\n'
            logger.info(f"Sending REGISTRATION packet: {reg_json_str.strip()}")
            writer.write(reg_json_str.encode('utf-8'))
            await writer.drain()
            
            logger.info("Starting sensor data sending and command receiving tasks.")
            send_task = asyncio.create_task(send_sensor_data(writer))
            receive_task = asyncio.create_task(receive_commands(reader))

            done, pending = await asyncio.wait(
                [send_task, receive_task],
                return_when=asyncio.FIRST_COMPLETED,
            )

            for task in done:
                task_name = task.get_name() if hasattr(task, 'get_name') else 'Unknown Task'
                try:
                    task.result()
                    logger.info(f"Task '{task_name}' finished normally.")
                except asyncio.CancelledError:
                    logger.info(f"Task '{task_name}' was cancelled normally.")
                except Exception as e:
                    logger.error(f"Task '{task_name}' failed: {type(e).__name__} - {e}", exc_info=False)

            for task in pending:
                task_name = task.get_name() if hasattr(task, 'get_name') else 'Unknown Task'
                logger.info(f"Cancelling pending task '{task_name}'...")
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    logger.info(f"Task '{task_name}' successfully cancelled.")
                except Exception as e_cancel:
                    logger.error(f"Error during cancellation of task '{task_name}': {e_cancel}", exc_info=True)


        except ConnectionRefusedError:
            logger.error(f"Connection refused by server {SERVER_IP}:{SERVER_PORT}. Is direct_bridge.py running?")
        except socket.gaierror:
             logger.error(f"Could not resolve server IP: {SERVER_IP}. Check network or IP address.")
        except OSError as e:
             logger.error(f"Network/OS Error during connection: {e} (Error Code: {e.errno})")
        except Exception as e:
             logger.error(f"Network/OS Error during connection: {e} (Error Code: {e.errno})")
        except Exception as e:
            logger.error(f"An unexpected error occurred in the main client loop: {e}", exc_info=True)
        finally:
            if writer:
                try:
                    if not writer.is_closing():
                        writer.close()
                        await writer.wait_closed()
                        logger.info("Connection writer closed.")
                except Exception as e_close:
                    logger.error(f"Error closing writer: {e_close}", exc_info=True)
            else:
                 logger.info("No active writer to close (was None or connection failed).")
            
            # Ensure reader is also handled if it was opened
            # (Reader doesn't have a close method like writer, it closes when the socket does)
            # No explicit close needed for reader if writer.close() and wait_closed() succeeds.

            logger.info(f"Disconnected from server. Waiting {RECONNECT_DELAY} seconds before reconnecting...")
            await asyncio.sleep(RECONNECT_DELAY)

if __name__ == "__main__":
    try:
        logger.info(f"Starting Robot Simulator (ID: {ROBOT_ID}). Will connect to {SERVER_IP}:{SERVER_PORT}")
        asyncio.run(robot_client_main())
    except KeyboardInterrupt:
        logger.info("Robot Simulator stopped by user (KeyboardInterrupt).")
    except Exception as e:
         logger.critical(f"Robot Simulator crashed: {e}", exc_info=True)
