import sys
import random
from PyQt5 import QtCore, QtWidgets
from eggsIncubatorGUI import Ui_MainWindow  # Import the UI class directly

import serial
import re
import os
from datetime import datetime, timedelta
import csv
import time
import queue
import subprocess
from collections import deque
import uuid
import json
import shutil

'''
    SALVATAGGIO DEI PARAMETRI:
    non prevedo un bottone in GUI, ogni volta che modifico un parametro sensibile questo viene automaticamente salvato nel file di parametri.
    All'avvio, in automatico, se un parametro è salvato allora ok viene usato, altrimenti rimane il default scritto nel codice
'''

'''
    i comandi devono essere quanto più univoci possibile! perhé ho visto che se metto CW e CCW alla domanda indexof() arduino non li sa distinguere
    self.queue_command("ACK", "1") - self.queue_command("ACK", "1")
    self.queue_command("HTR01", self.current_heater_output_control)  # @<HTR01, True># @<HTR01, False>#
    self.queue_command("HUMER01", self.current_humidifier_output_control) # @<HUMER01, True># @<HUMER01, False>#
    self.queue_command("STPR01", "MCCW") # move_counter_clock_wise
    self.queue_command("STPR01", "MCW") # move_clock_wise 
    self.queue_command("STPR01", "STOP") # stop motor   
'''

'''
Siccome ci sono dei thread e si rischia concorrenza fra la chiamata delle funzioni di log, è necessario farne due separate. Un logging per il SerialThread e un logging per
il MainSoftwareThread

1. INFO
Informational messages that highlight the progress of the application at a coarse-grained level. These are useful for tracking the general flow of the application.

2. DEBUG
Detailed information, typically of interest only when diagnosing problems. These messages are useful for developers to understand the internal state of the application.

3. WARNING
Indications that something unexpected happened, or indicative of some problem in the near future (e.g., ‘disk space low’). The software is still working as expected.

4. ERROR
Due to a more serious problem, the software has not been able to perform some function. These messages indicate a failure in the application.

5. CRITICAL
A serious error, indicating that the program itself may be unable to continue running. These messages are used for severe errors that require immediate attention.

6. ALARM
Specific to your application, this could be used to indicate conditions that require immediate attention but are not necessarily errors (e.g., temperature out of range).

7. EXCEPTION
Used to log exceptions that occur in the application. This can include stack traces and other debugging information.

Example of usage:

log_message('INFO', 'This is an informational message.')
log_message('WARNING', 'This is a warning message.')
log_message('ERROR', 'This is an error message.')
'''

# ARDUINO serial communication - setup #
portSetup = "/dev/ttyUSB0"
portSetup = "/dev/ttyACM0"

baudrateSetup = 115200
timeout = 0.1

"""
	"EXTT" = riguarda il sensore di temperatura esterno.
"""
identifiers = ["TMP", "HUM", "HTP", "IND", "EXTT"]  # Global variable
command_tags = ["HTR01", "HUMER01", "STPR01"]


class SerialThread(QtCore.QThread):
    data_received = QtCore.pyqtSignal(list)

    def __init__(self, port = portSetup, baudrate = baudrateSetup):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.command_queue = queue.Queue()  # Queue for outgoing commands
        self.retry_interval = 1 #sleeping time in seconds
        self.max_retries = 5
        self.running = True
        self.serial_port = None  # To store the serial port object
        self.serial_port_successfully_opened = False
        self.serial_thread_ready_to_go = False
        
        self.awaiting_ack = None
        self.ack_received_flag = False
        self.failed_commands = []
        self.max_retries = 3
        self.ack_timeout = 0.5 # time.time() returns seconds. Suggested is 300-500ms
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # Create a Log folder if it doesn't exist
        self.serial_thread_log_folder_path = os.path.join(script_dir, 'SerialThread_Log')
        if not os.path.exists(self.serial_thread_log_folder_path):
            os.makedirs(self.serial_thread_log_folder_path)
        '''
        self.pending_command = None
        self.failed_commands = []
        self.ack_timeout = 0.5  # secondi
        self.max_retries = 3
        self.last_sent_time = None
        '''
    def open_serial_port(self):
        retries = 0
        while retries < self.max_retries:
            try:
                self.serial_thread_log_message('INFO', 'Estabilishing serial connection')
                self.serial_port = serial.Serial(self.port, self.baudrate, timeout=1)
                time.sleep(3)
                self.serial_thread_log_message('INFO', 'Serial port opened successfully')
                # Discard any initial data to avoid decode errors
                self.serial_port.reset_input_buffer()
                return True
            except serial.SerialException as e:
                self.serial_thread_log_message('ERROR', f"Failed to open serial port: {e}. Retrying in {self.retry_interval} seconds...")
                time.sleep(self.retry_interval)
                retries += 1
        return False
 
    def run(self):
        self.serial_port_successfully_opened = self.open_serial_port()
        if not self.serial_port_successfully_opened:
            self.serial_thread_log_message('ERROR', 'Unable to open serial port after multiple attempts.')
            return
        sync_waiting_time_s = 5
        self.serial_thread_log_message('INFO', f"Waiting {sync_waiting_time_s} seconds for Arduino to setup")
        time.sleep(sync_waiting_time_s)
        self.serial_thread_ready_to_go = True

        buffer = ''
        startRx = False
        saving = False
        identifiers_data_list = []

        decode_error_count = 0
        decode_error_threshold = 10  # Number of decode errors before resetting serial port

        '''
        def log_error(msg):
            print(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] ERROR: {msg}")
        '''
        
        try:
            while self.running:
                while self.serial_port.in_waiting > 0:
                    try:
                        dataRead = self.serial_port.read().decode('utf-8')
                        decode_error_count = 0
                    except UnicodeDecodeError as e:
                        decode_error_count += 1
                        self.serial_thread_log_message('ALARM', f"Decode error (#{decode_error_count}): {e}. Resetting buffer and flags.")
                        
                        startRx = False
                        buffer = ''
                        saving = False

                        if decode_error_count >= decode_error_threshold:
                            self.serial_thread_log_message('CRITICAL', 'Too many decode errors. Resetting serial port.')
                            self.close_serial_port()
                            time.sleep(1)
                            self.open_serial_port()
                            decode_error_count = 0
                        continue

                    if dataRead == '@':
                        startRx = True
                        buffer = ''
                        identifiers_data_list.clear()
                        saving = False
                    elif startRx:
                        buffer += dataRead

                        if dataRead == '#':
                            # Messaggio completo ricevuto, es: @<TMP01,17.5><HTR01, True, 08CA9775><STPR01, False, 1234ABCD>#
                            startRx = False
                            trimmed_buffer = buffer.strip('@').strip('#')
                            #print(f"🔍 Full buffer received: {trimmed_buffer!r}")
                            if trimmed_buffer:
                                self.decode_message(self, trimmed_buffer, identifiers_data_list, self)

                            buffer = ''

                            if identifiers_data_list:
                                self.data_received.emit(identifiers_data_list)
                                identifiers_data_list.clear()

                self._try_send_next_command()
                
                time.sleep(0.1)


        except serial.SerialException as e:
            self.serial_thread_log_message('ERROR', f"Serial error: {e}")
            
        finally:
            self.close_serial_port()
            
    def _try_send_next_command(self):
        # Se sto già aspettando un ACK, controllo timeout/flag
        if self.awaiting_ack:
            if self.ack_received_flag:
                # ACK arrivato: sblocco
                self.awaiting_ack = None
                self.ack_received_flag = False
                return
            elif time.time() - self.awaiting_ack["timestamp"] >= self.ack_timeout:
                cmd_obj = self.awaiting_ack["command"]
                retries = cmd_obj["retry_count"] + 1
                if retries <= self.max_retries:
                    cmd_obj["retry_count"] = retries
                    self.serial_port.write(cmd_obj["formatted"].encode('utf-8'))
                    # Stampiamo solo se non è ALIVE
                    if cmd_obj["cmd"] != "ALIVE":
                        self.serial_thread_log_message('WARNING', f"Retrying ({retries}) for: {cmd_obj['formatted']}")
                    self.awaiting_ack["timestamp"] = time.time()
                else:
                    # Stampiamo solo se non è ALIVE
                    if cmd_obj["cmd"] != "ALIVE":
                        self.serial_thread_log_message('ALARM', f"⚠️ NO ACK for command: {cmd_obj['formatted']}")
                    self.failed_commands.append(cmd_obj)
                    self.awaiting_ack = None
            return

        # Altrimenti, se non ho comandi in attesa, ne prendo uno nuovo
        if not self.command_queue.empty():
            cmd_obj = self.command_queue.get()
            formatted = cmd_obj["formatted"]
            uid = cmd_obj["id"]

            if uid is None:
                # Comando senza ACK
                self.serial_port.write(formatted.encode('utf-8'))
                # Stampiamo solo se non è ALIVE
                if cmd_obj["cmd"] != "ALIVE":
                    self.serial_thread_log_message('INFO', f"Sent (no ACK needed): {formatted}")
            else:
                # Comando con ACK
                self.serial_port.write(formatted.encode('utf-8'))
                # Stampiamo solo se non è ALIVE
                if cmd_obj["cmd"] != "ALIVE":
                    self.serial_thread_log_message('INFO', f"Sent to Arduino: {formatted}")
                cmd_obj["retry_count"] = 0
                self.awaiting_ack = {
                    "uid": uid,
                    "command": cmd_obj,
                    "timestamp": time.time()
                }
            
    def add_command(self, cmd, value):
        if cmd in command_tags:
            unique_id = uuid.uuid4().hex[:8].upper()  # ID breve, es. '3F7A91B2'
            formatted_command = f"@<{cmd}, {value}, {unique_id}>#"
        else:
            unique_id = None
            formatted_command = f"@<{cmd}, {value}>#"

        self.command_queue.put({
            "cmd": cmd,
            "value": value,
            "id": unique_id,
            "formatted": formatted_command,
            "retry_count": 0,
        })
    
    def stop(self):
        self.running = False
        self.wait()  # Ensure the thread finishes before returning
        self.close_serial_port()
        
    def close_serial_port(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                self.serial_thread_log_message('INFO', 'Serial port closed successfully')
            except Exception as e:
                self.serial_thread_log_message('ERROR', f"Error closing serial port: {e}")

    @staticmethod
    def decode_message(self, buffer, identifiers_data_list, serial_thread_instance):
        import re

        # Prende tutte le sottostringhe tra < e >
        messages = re.findall(r"<([^>]+)>", buffer)
        #print(f"🔍 Messages extracted: {messages}")
        for msg in messages:
            parts = [p.strip() for p in msg.split(',')]
            #print(f"🔍 Parsing parts: {parts}")
            #print(f"⏳ awaiting_ack right now: {serial_thread_instance.awaiting_ack!r}")

            if len(parts) == 3:
                cmd, value, uid = parts
                #print(f"🔍 Detected potential ACK - cmd={cmd}, value={value}, uid={uid}")
                # Se è un comando che richiede ACK e corrisponde all'atteso
                if cmd in command_tags:
                    if (serial_thread_instance.awaiting_ack and
                        serial_thread_instance.awaiting_ack["uid"] == uid):
                        self.serial_thread_log_message('INFO', f"✅ ACK ricevuto: {cmd}, {value}, ID={uid}")
                        serial_thread_instance.ack_received_flag = True
                else:
                    self.serial_thread_log_message('ALARM', f"⚠️ ACK {uid} non atteso o awaiting_ack differente")
                    # Non aggiungo ACK alla lista dati normali
                    continue

            elif len(parts) == 2:
                info_name, info_value = parts
                if info_value.upper() == "NAN":
                    number = 0.0
                elif SerialThread.is_number(info_value):
                    number = float(info_value) if '.' in info_value else int(info_value)
                else:
                    self.serial_thread_log_message('WARNING', f"Non-numeric data: {info_value}")
                    continue

                for identifier in identifiers:
                    if info_name.startswith(identifier):
                        identifiers_data_list.append({info_name: number})
                        break

            
    def handle_ack(self, tag, value, uid):
        if self.awaiting_ack and self.awaiting_ack["uid"] == uid:
            self.serial_thread_log_message('INFO', f"✅ ACK ricevuto: {tag}, {value}, ID={uid}")
            self.ack_received_flag = True
        else:
            self.serial_thread_log_message('ALARM', f"⚠️ ACK ricevuto ma non atteso o ID diverso: {uid}")
    
    @staticmethod
    def is_number(s):
        try:
            float(s)
            return True
        except ValueError:
            return False
        
    def serial_thread_log_message(self, error_type, message):
        """
        Logs a message to a log file named with the current date inside the Log folder.
        Format: [ERROR_TYPE] Message @ Timestamp

        Args:
            error_type (str): The type of error (e.g., 'INFO', 'WARNING', 'ERROR').
            message (str): The message to log.
        """
        print(message)
        current_date = datetime.now().strftime('%Y-%m-%d')
        log_file = os.path.join(self.serial_thread_log_folder_path, f"log_{current_date}.txt")
        
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        with open(log_file, 'a', encoding='utf-8') as file:
            file.write(f"[{error_type}] {message} @ {timestamp}\n")
        '''
        OLD VERSION FOR JSON FORMAT
        """
        Logs a message to a log file named with the current date inside the Log folder in plain text format with sections.

        Args:
            error_type (str): The type of error (e.g., 'INFO', 'WARNING', 'ERROR').
            message (str): The message to log.
        """
        print(message)
        current_date = datetime.now().strftime('%Y-%m-%d')
        log_file = os.path.join(self.serial_thread_log_folder_path, f"log_{current_date}.txt")
        
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        with open(log_file, 'a') as file:
            file.write(f"--- Log Entry ---\n")
            file.write(f"Timestamp: {timestamp}\n")
            file.write(f"Error Type: {error_type}\n")
            file.write(f"Message: {message}\n")
            file.write(f"-----------------\n\n")
        '''


class MainSoftwareThread(QtCore.QThread):
    update_view = QtCore.pyqtSignal(list)  # Signal to update the view (MainWindow)
    update_statistics = QtCore.pyqtSignal(list) # Signal to update the Statistics View Window
    update_spinbox_value = QtCore.pyqtSignal(str, float)  # Signal to update spinbox (name, value)
    update_motor = QtCore.pyqtSignal(list) # Signal to update the motor view
    
    def __init__(self):
        super().__init__()
        self.initialization_from_GUI_completed = False
        self.running = True
        self.serial_thread = SerialThread()
        self.serial_thread.data_received.connect(self.process_serial_data)
        self.current_data = []  # Holds the most recent data received from SerialThread
        
        self.alive_to_arduino_state = False
        self.alive_to_arduino_time_interval_sec = 2 # invio un ack ad arduino 2 secondi
        self.last_alive_to_arduino_time = time.time()
        
        self.saving_interval = 1 # default: every minute
        self.last_saving_time = datetime.now()
        self.arduino_readings_timestamp  = time.time() # per ricordarmi l'istante di tempo in cui arduino manda i dati
        
        self.command_list = [] # List to hold the pending commands
        
        # State variables to handle inputs from MainWindow
        self.current_button = None # notifica del pulsante premuto
        self.spinbox_values = {} # notifica dello spinbox cambiato
        
        # TEMPERATURE CONTROLLER
        # Constants
        self.INVALID_VALUES = [-127.0, 85.0]  # Known error values
        self.THRESHOLD = 0.5  # Acceptable variation from the valid range
        self.VALID_RANGE_TEMPERATURE = (5.0, 45.0)  # Expected temperature range
        self.VALID_RANGE_HUMIDITY = (0.0, 100.0) # Expected humidity range

        # ANTI-DEBOUNCE for thc temperature hysteresis controller
        self._last_output_change_time_thc = None
        self._last_output_state_thc = None
        self._debounced_heater_output_thc = None  # Stato effettivamente inviato
        
        # ANTI-DEBOUNCE for thc humidity hysteresis controller
        self._last_output_change_time_hhc = None
        self._last_output_state_hhc = None
        self._debounced_heater_output_hhc = None  # Stato effettivamente inviato
        
        self._debounce_duration = 1.0  # secondi
        
        
        
        # Error tracking
        self.ERROR_TIME_LIMIT = 10  # Tempo in secondi oltre il quale generare un warning
        # Stato del sistema
        self.error_counter = 0  # Conta tutti gli errori mai verificati
        self.error_timestamps = {}  # Memorizza il primo timestamp di errore per ogni sensore
        self.warned_sensors = set()  # Tiene traccia dei sensori già segnalati con un warning

        self.thc = self.HysteresisController(lower_limit = 37.5, upper_limit = 37.8) # temperature hysteresis controller
        self.current_heater_output_control = False # variabile che mi ricorda lo stato attuale dell'heater
        
        self.hhc = self.HysteresisController(lower_limit = 20.0, upper_limit = 50.0) # humidity hysteresis controller
        self.current_humidifier_output_control = False # variabile che mi ricorda lo stato attuale dell'heater
        
        self.eggTurnerMotor = self.StepperMotor("Egg_Turner_Stepper_Motor")
        self.last_turnsCounter = 0 # serve per ricordarmi il numero di turns counter
        
        # BUTTON HANDLING
        self.move_CW_motor_btn = False # = not pressed
        self.move_CCW_motor_btn = False # = not pressed
		
		# PLOT
        self.remove_erroneous_values_from_T_plot = True
        self.remove_erroneous_values_from_H_plot = True
        
        #--- Create Machine_Statistic folder ---#
        script_dir = os.path.dirname(os.path.abspath(__file__))
        machine_statistics_folder_path = os.path.join(script_dir, "Machine_Statistics") 
        if not os.path.exists(machine_statistics_folder_path):
                os.makedirs(machine_statistics_folder_path)

        # --- Create Statistics folder ---#
        temperatures_folder_path = os.path.join(machine_statistics_folder_path, 'Temperatures')
        if not os.path.exists(temperatures_folder_path):
            os.makedirs(temperatures_folder_path)

        humidity_folder_path = os.path.join(machine_statistics_folder_path, 'Humidity')
        if not os.path.exists(humidity_folder_path):
            os.makedirs(humidity_folder_path)
            
        heater_actuator_folder_path = os.path.join(machine_statistics_folder_path, 'Heater')
        if not os.path.exists(heater_actuator_folder_path):
            os.makedirs(heater_actuator_folder_path)

        humidifier_actuator_folder_path = os.path.join(machine_statistics_folder_path, 'Humidifier')
        if not os.path.exists(humidifier_actuator_folder_path):
            os.makedirs(humidifier_actuator_folder_path)
            
        # Create a Log folder if it doesn't exist
        self.main_software_thread_log_folder_path = os.path.join(script_dir, 'MainSoftwareThread_Log')
        if not os.path.exists(self.main_software_thread_log_folder_path):
            os.makedirs(self.main_software_thread_log_folder_path)

        #--- Create Parameters folder + file ---#
        script_dir = os.path.dirname(os.path.abspath(__file__))
        parameters_folder_path = os.path.join(script_dir, "Parameters") 
        if not os.path.exists(parameters_folder_path):
                os.makedirs(parameters_folder_path)

        self.parameters_file_path = os.path.join(parameters_folder_path, "parameters.json") # file where parameters are saved
        self.parameters = {} # dictionary storing all parameters in program memory
        self._load_all_parameters() # loading already existing parameters in the file

    def run(self):
        self.write_log_section_header()
        self.main_software_thread_log_message('INFO', f"Starting serial Thread")
        # Start the SerialThread
        self.serial_thread.start()
        
        while not self.serial_thread.serial_thread_ready_to_go:
            pass
        self.main_software_thread_log_message('INFO', f"Serial Thread started, waiting for GUI initialization")

        # prima di far partire il loop provo già a settare i paramteri corretti
        while not self.initialization_from_GUI_completed:
            pass
         #1x volta, inizializzatione dei paramteri da file
         # Initializing parameters from file:
        self.parameters_initialization_from_file()
        self.backup_parameters_file()
        self.main_software_thread_log_message('INFO', f"GUI initialization completed: loading parameters from file is done. Now the program starts!")

        while self.running:            
            if self.command_list:
                for cmd, value in self.command_list:
                    self.serial_thread.add_command(cmd, value)
                    self.main_software_thread_log_message('INFO', f"Added commad to serial thread queue: {cmd}, {value}")
                self.command_list.clear()
                
            # parte che gira periodica, quindi ci metto la gestione del motore siccome deve funzionare periodicamente per il timer.
            self.eggTurnerMotor.update()
            
            if self.eggTurnerMotor.getNewCommand() is not None: # checking if there's a command
                self.main_software_thread_log_message('INFO', f"Processing command: {self.eggTurnerMotor.getNewCommand()}")
                
                if (self.eggTurnerMotor.getNewCommand() == "automatic_CCW_rotation_direction"
                    or
                    self.eggTurnerMotor.getNewCommand() == "manual_CCW_rotation_direction"):
                    self.queue_command("STPR01", "MCCW") # move_counter_clock_wise
                    
                if (self.eggTurnerMotor.getNewCommand() == "automatic_CW_rotation_direction"
                    or
                    self.eggTurnerMotor.getNewCommand() == "manual_CW_rotation_direction"):
                    self.queue_command("STPR01", "MCW") # move_clock_wise           
                    
                if self.eggTurnerMotor.getNewCommand() == "stop":
                    self.queue_command("STPR01", "STOP") # stop motor 
                
                self.eggTurnerMotor.resetNewCommand()
            
            if self.eggTurnerMotor.getUpdateMotorData():
                all_values = []
                # [timePassed timeToNextTurn turnsCounter]
                all_values = [self.eggTurnerMotor.getTimeSinceLastRotation()] + \
                                [self.eggTurnerMotor.getTimeUntilNextRotation()] + \
                                [self.eggTurnerMotor.getTurnsCounter()] + \
                                [self.eggTurnerMotor.main_state] + \
                                [self.eggTurnerMotor.manual_state] + \
                                [self.eggTurnerMotor.rotation_state]
                self.update_motor.emit(all_values)

                if self.eggTurnerMotor.getTurnsCounter() != self.last_turnsCounter:
                    self.last_turnsCounter = self.eggTurnerMotor.getTurnsCounter() # salvo il numero nuovo di turns counter
                    self.save_parameter('TURNS_COUNTER', self.eggTurnerMotor.getTurnsCounter())
                
                self.eggTurnerMotor.resetUpdateMotorData()
            
            # GESTIONE DELL'ALIVE BIT verso arduino
            if (time.time() - self.last_alive_to_arduino_time) >= self.alive_to_arduino_time_interval_sec:
                self.last_alive_to_arduino_time = time.time()
                self.alive_to_arduino_state = not self.alive_to_arduino_state
                self.queue_command("ALIVE", self.alive_to_arduino_state)
              
            time.sleep(0.1)
            
    def check_errors(self,sensor_data):
        """Verifica errori, aggiorna il contatore e traccia il tempo degli errori persistenti."""

        current_time = time.time()
        new_errors = []  # Sensori che entrano in errore ora
        resolved_errors = []  # Sensori che sono tornati normali

        # Analizza il dizionario ricevuto
        #print(sensor_data)
        for sensor, value in sensor_data.items():
            # Verifica se il valore è un errore
            if any(abs(value - iv) < self.THRESHOLD for iv in self.INVALID_VALUES) or not (self.VALID_RANGE_TEMPERATURE[0] <= value <= self.VALID_RANGE_TEMPERATURE[1]):
                # Se è un nuovo errore, aggiorna il contatore e salva il timestamp
                if sensor not in self.error_timestamps:
                    self.error_timestamps[sensor] = current_time
                    self.error_counter += 1
                    new_errors.append(sensor)  # Sensore appena entrato in errore
            else:
                # Se il sensore era in errore ma ora non lo è più, lo rimuoviamo dal tracking
                if sensor in self.error_timestamps:
                    del self.error_timestamps[sensor]
                    self.warned_sensors.discard(sensor)  # Resetta anche il warning
                    resolved_errors.append(sensor)

        # Stampa solo se ci sono nuovi errori o warning per errori persistenti
        if new_errors:
            self.main_software_thread_log_message('ALARM', f"⚠ ERRORE: Sensori appena entrati in errore: {new_errors}")
            self.main_software_thread_log_message('INFO', f"Totale errori rilevati finora: {self.error_counter}")

        for sensor, start_time in self.error_timestamps.items():
            if current_time - start_time > self.ERROR_TIME_LIMIT and sensor not in self.warned_sensors:
                self.main_software_thread_log_message('WARNING', f"⚠ WARNING: Il sensore '{sensor}' è in errore da più di {self.ERROR_TIME_LIMIT} secondi!")
                self.warned_sensors.add(sensor)  # Segnala il warning solo una volta
            
    def filter_temperatures(self, temperatures):
        """Remove outliers and keep only valid temperature values."""
        return [
            temp for temp in temperatures
            if not any(abs(temp - iv) < self.THRESHOLD for iv in self.INVALID_VALUES)
            and self.VALID_RANGE_TEMPERATURE[0] <= temp <= self.VALID_RANGE_TEMPERATURE[1]
        ]
            
    def queue_command(self, cmd, value):
        self.command_list.append((cmd, value))

    def stop(self):
        self.running = False
        self.serial_thread.stop()
        self.serial_thread.wait()
        
    def handle_button_click(self, button_name):
        self.main_software_thread_log_message('INFO', f"[MainSoftwareThread] Processing button {button_name}")
        self.current_button = button_name
        
        if self.current_button == "forceEggsTurn_motor_btn":
            self.eggTurnerMotor.forceEggsRotation()
        
        if self.current_button == "move_CW_motor_btn":
            self.move_CW_motor_btn = not self.move_CW_motor_btn # ogni volta in cui lo premo, toggle dello stato
            
            if self.move_CW_motor_btn:
                self.eggTurnerMotor.moveCWContinuous()
            elif not self.move_CW_motor_btn:
                self.eggTurnerMotor.stop()
            
        if self.current_button == "move_CCW_motor_btn":
            self.move_CCW_motor_btn = not self.move_CCW_motor_btn
            
            if self.move_CCW_motor_btn:
                self.eggTurnerMotor.moveCCWContinuous()
            elif not self.move_CCW_motor_btn:
                self.eggTurnerMotor.stop()
				
        if self.current_button == "reset_statistics_T_btn":
            self.thc.reset_all_values()
            
        if self.current_button == "plotAllDays_temp_T_btn":
            script_dir = os.path.dirname(os.path.abspath(__file__))
            python_executable = self.get_python_executable()
            plot_script = os.path.join(script_dir, "appInteractivePlots.py")          
            command = [
                python_executable,
                plot_script,
                "PLOT_ALL_DAYS_DATA_TEMPERATURES",
                str(self.VALID_RANGE_TEMPERATURE[0]),
                str(self.VALID_RANGE_TEMPERATURE[1]),
                str(self.remove_erroneous_values_from_T_plot),
            ]
            self.main_software_thread_log_message('INFO', f"Command to plot data: {command}")
            process = subprocess.Popen(command)
            #print("Subprocess started and main program continues...")
            
        if self.current_button == "plotToday_temp_T_btn":
            script_dir = os.path.dirname(os.path.abspath(__file__))
            python_executable = self.get_python_executable()
            plot_script = os.path.join(script_dir, "appInteractivePlots.py") 
            command = [
                python_executable,
                plot_script,
                "PLOT_CURRENT_DAY_DATA_TEMPERATURES",
                str(self.VALID_RANGE_TEMPERATURE[0]),
                str(self.VALID_RANGE_TEMPERATURE[1]),
                str(self.remove_erroneous_values_from_T_plot),
            ]
            self.main_software_thread_log_message('INFO', f"Command to plot data: {command}")
            process = subprocess.Popen(command)
            #print("Subprocess started and main program continues...")
            
        if self.current_button == "plotAllDays_humidity_H_btn":
            script_dir = os.path.dirname(os.path.abspath(__file__))
            python_executable = self.get_python_executable()
            plot_script = os.path.join(script_dir, "appInteractivePlots.py") 
            command = [
                python_executable,
                plot_script,
                "PLOT_ALL_DAYS_DATA_HUMIDITY",
                str(self.VALID_RANGE_HUMIDITY[0]),
                str(self.VALID_RANGE_HUMIDITY[1]),
                str(self.remove_erroneous_values_from_H_plot),
            ]
            self.main_software_thread_log_message('INFO', f"Command to plot data: {command}")
            process = subprocess.Popen(command)
            #print("Subprocess started and main program continues...")
            
        if self.current_button == "plotToday_humidity_H_btn":
            script_dir = os.path.dirname(os.path.abspath(__file__))
            python_executable = self.get_python_executable()
            plot_script = os.path.join(script_dir, "appInteractivePlots.py") 
            command = [
                python_executable,
                plot_script,
                "PLOT_CURRENT_DAY_DATA_HUMIDITY",
                str(self.VALID_RANGE_HUMIDITY[0]),
                str(self.VALID_RANGE_HUMIDITY[1]),
                str(self.remove_erroneous_values_from_T_plot),
            ]
            self.main_software_thread_log_message('INFO', f"Command to plot data: {command}")
            process = subprocess.Popen(command)
            #print("Subprocess started and main program continues...")
            
        if self.current_button == "plotAllDays_cnt_T_btn":
            script_dir = os.path.dirname(os.path.abspath(__file__))
            python_executable = self.get_python_executable()
            plot_script = os.path.join(script_dir, "appInteractivePlots.py") 
            command = [
                python_executable,
                plot_script,
                "PLOT_ALL_DAYS_DATA_HEATER",
                str(0),
                str(1),
                str(False),
            ]
            self.main_software_thread_log_message('INFO', f"Command to plot data: {command}")
            process = subprocess.Popen(command)
            #print("Subprocess started and main program continues...")
            
        if self.current_button == "plotToday_cnt_T_btn":
            script_dir = os.path.dirname(os.path.abspath(__file__))
            python_executable = self.get_python_executable()
            plot_script = os.path.join(script_dir, "appInteractivePlots.py") 
            command = [
                python_executable,
                plot_script,
                "PLOT_CURRENT_DAY_DATA_HEATER",
                str(0),
                str(1),
                str(False),
            ]
            self.main_software_thread_log_message('INFO', f"Command to plot data: {command}")
            process = subprocess.Popen(command)
            #print("Subprocess started and main program continues...")
            
        if self.current_button == "plotAllDays_cnt_H_btn":
            script_dir = os.path.dirname(os.path.abspath(__file__))
            python_executable = self.get_python_executable()
            plot_script = os.path.join(script_dir, "appInteractivePlots.py") 
            command = [
                python_executable,
                plot_script,
                "PLOT_ALL_DAYS_DATA_HUMIDIFIER",
                str(0),
                str(1),
                str(False),
            ]
            self.main_software_thread_log_message('INFO', f"Command to plot data: {command}")
            process = subprocess.Popen(command)
            #print("Subprocess started and main program continues...")
            
        if self.current_button == "plotToday_cnt_H_btn":
            script_dir = os.path.dirname(os.path.abspath(__file__))
            python_executable = self.get_python_executable()
            plot_script = os.path.join(script_dir, "appInteractivePlots.py") 
            command = [
                python_executable,
                plot_script,
                "PLOT_CURRENT_DAY_DATA_HUMIDIFIER",
                str(0),
                str(1),
                str(False),
            ]
            self.main_software_thread_log_message('INFO', f"Command to plot data: {command}")
            process = subprocess.Popen(command)
            #print("Subprocess started and main program continues...")
            
    def get_python_executable(self):
        """
        Searches the current script directory for a non-hidden folder containing 'venv'
        and returns the path to its python3 executable if found.
        Otherwise, returns 'python3' (system-wide Python).
        """
        script_dir = os.path.dirname(os.path.abspath(__file__))
        venv_dir = None

        for entry in os.listdir(script_dir):
            if entry.startswith('.'):  # Skip hidden folders
                continue
            full_path = os.path.join(script_dir, entry)
            if os.path.isdir(full_path) and "venv" in entry:
                venv_dir = full_path
                break

        if venv_dir:
            return os.path.join(venv_dir, "bin", "python3")
        else:
            return "python3"
        
    def handle_float_spinBox_value(self, spinbox_name, value):
        rounded_value = round(value, 1)
        self.main_software_thread_log_message('INFO', f"[MainSoftwareThread] Processing spinbox {spinbox_name} of value: {rounded_value} ({type(rounded_value)})")
        self.spinbox_values[spinbox_name] = rounded_value       
        
        if spinbox_name == "rotation_interval_spinBox":
            self.eggTurnerMotor.setFunctionInterval(rounded_value * 60) # Set the rotation interval
            self.save_parameter('ROTATION_INTERVAL', rounded_value)

        elif spinbox_name == "maxHysteresisValue_temperature_spinBox":
            if rounded_value <= self.thc.get_lower_limit():
                self.thc.set_upper_limit(rounded_value)
                self.save_parameter('TEMPERATURE_HYSTERESIS_CONTROLLER_UPPER_LIMIT', rounded_value)

                self.thc.set_lower_limit(rounded_value)
                self.save_parameter('TEMPERATURE_HYSTERESIS_CONTROLLER_LOWER_LIMIT', rounded_value)
                self.update_spinbox_value.emit("minHysteresisValue_temperature_spinBox", rounded_value)
            else:
                self.thc.set_upper_limit(rounded_value)
                self.save_parameter('TEMPERATURE_HYSTERESIS_CONTROLLER_UPPER_LIMIT', rounded_value)
                
        elif spinbox_name == "minHysteresisValue_temperature_spinBox":
            if rounded_value >= self.thc.get_upper_limit():
                self.thc.set_upper_limit(rounded_value)
                self.save_parameter('TEMPERATURE_HYSTERESIS_CONTROLLER_UPPER_LIMIT', rounded_value)

                self.thc.set_lower_limit(rounded_value)
                self.save_parameter('TEMPERATURE_HYSTERESIS_CONTROLLER_LOWER_LIMIT', rounded_value)
                self.update_spinbox_value.emit("maxHysteresisValue_temperature_spinBox", rounded_value)
            else:
                self.thc.set_lower_limit(rounded_value)
                self.save_parameter('TEMPERATURE_HYSTERESIS_CONTROLLER_LOWER_LIMIT', rounded_value)
                
        elif spinbox_name == "maxHysteresisValue_humidity_spinBox":
            if rounded_value <= self.hhc.get_lower_limit():
                self.hhc.set_upper_limit(rounded_value)
                self.save_parameter('HUMIDITY_HYSTERESIS_CONTROLLER_UPPER_LIMIT', rounded_value)

                self.hhc.set_lower_limit(rounded_value)
                self.save_parameter('HUMIDITY_HYSTERESIS_CONTROLLER_LOWER_LIMIT', rounded_value)
                self.update_spinbox_value.emit("minHysteresisValue_humidity_spinBox", rounded_value)
            else:
                self.hhc.set_upper_limit(rounded_value)
                self.save_parameter('HUMIDITY_HYSTERESIS_CONTROLLER_UPPER_LIMIT', rounded_value)
                
        elif spinbox_name == "minHysteresisValue_humidity_spinBox":
            if rounded_value >= self.hhc.get_upper_limit():
                self.hhc.set_upper_limit(rounded_value)
                self.save_parameter('HUMIDITY_HYSTERESIS_CONTROLLER_UPPER_LIMIT', rounded_value)

                self.hhc.set_lower_limit(rounded_value)
                self.save_parameter('HUMIDITY_HYSTERESIS_CONTROLLER_LOWER_LIMIT', rounded_value)
                self.update_spinbox_value.emit("maxHysteresisValue_humidity_spinBox", rounded_value)
            else:
                self.hhc.set_lower_limit(rounded_value)
                self.save_parameter('HUMIDITY_HYSTERESIS_CONTROLLER_LOWER_LIMIT', rounded_value)
            
    def handle_intialization_step(self, value_name, value):
        rounded_value = round(value, 1)
        if value_name == "maxHysteresisValue_temperature_spinBox":
            self.thc.set_upper_limit(rounded_value)
        elif value_name == "minHysteresisValue_temperature_spinBox":
            self.thc.set_lower_limit(rounded_value)
        elif value_name == "maxHysteresisValue_humidity_spinBox":
            self.hhc.set_upper_limit(rounded_value)
        elif value_name == "minHysteresisValue_humidity_spinBox":
            self.hhc.set_lower_limit(rounded_value)
    
    def handle_initialization_done(self, parameter, value):
        # funzione che viene chiamata da MainWindow quando ha completato l'emit di tutti i parametri da default di GUI
        self.initialization_from_GUI_completed = True
            
    def handle_radio_button_toggle(self, value_name, value):
        if value_name == "heaterOFF_radioBtn":
            self.thc.set_control_mode("forceOFF")
        if value_name == "heaterAUTO_radioBtn":
            self.thc.set_control_mode("AUTO")
        if value_name == "heaterON_radioBtn":
            self.thc.set_control_mode("forceON")
            
        if value_name == "humidifierOFF_radioBtn":
            self.hhc.set_control_mode("forceOFF")
        if value_name == "humidifierAUTO_radioBtn":
            self.hhc.set_control_mode("AUTO")
        if value_name == "humidifierON_radioBtn":
            self.hhc.set_control_mode("forceON")
            
        if value_name == "removeErrors_from_T_plots":
            self.remove_erroneous_values_from_T_plot = value
            self.main_software_thread_log_message('INFO', f"{value_name} + {value}")
            
        if value_name == "removeErrors_from_H_plots":
            self.remove_erroneous_values_from_H_plot = value
            self.main_software_thread_log_message('INFO', f"{value_name} + {value}")
            
            
    def process_serial_data(self, new_data):
        arduino_time_difference = time.time() - self.arduino_readings_timestamp
        self.arduino_readings_timestamp = time.time()
        # Convert to milliseconds and format without commas
        ms = int(arduino_time_difference * 1000)
        self.main_software_thread_log_message('INFO', f"Data from serial now! Time passed wrt previous data: {ms} ms")
        
        self.current_data = new_data
        #print(new_data)
        # [{'TMP01': 19.5}, {'TMP02': 19.1}, {'TMP03': 19.8}, {'TMP04': 20.1}, {'HUM01': 19.5}, {'HTP01': 19.5}]
        # Extract data into specific categories - questi che seguono sono tutti dictionaries
        current_temperatures = {k: v for d in new_data for k, v in d.items() if k.startswith("TMP")} # {'TMP01': 22.3, 'TMP02': 22.2, 'TMP03': 22.3, 'TMP04': 22.4}
        current_humidities = {k: v for d in new_data for k, v in d.items() if k.startswith("HUM")}
        current_humidities_temperatures = {k: v for d in new_data for k, v in d.items() if k.startswith("HTP")}
        current_inductors_feedbacks = {k: v for d in new_data for k, v in d.items() if k.startswith("IND")} #{'IND_CCW': 1, 'IND_CW': 1}
        current_external_temperature = {k: v for d in new_data for k, v in d.items() if k.startswith("EXTT")} #{'EXTT': 25.2}
        
        
        # TEMPERATURE CONTROLLER SECTION
        # faccio l'update qui: ogni votla che arrivano dati nuovi li elaboro, anche nel controllore
        # al controllore di temperatura passo solo temperature filtrate, ovvero i valori dentro il range di temperatura corretto
        
        filtered_temperatures = self.filter_temperatures(list(current_temperatures.values()))
        if not filtered_temperatures:
            self.main_software_thread_log_message('WARNING', 'filtered_temperature list is empty! fault in the sensors')         
        self.thc.update(filtered_temperatures)
        #print(list(current_temperatures.values()))

        '''
            chatGPT: adding debounce logic.
            La scelta migliore è implementarlo nel codice che controlla l’output dell’heater, non all’interno della classe Heater
            La classe Heater dovrebbe occuparsi solo del controllo logico / PID.
            La decisione di inviare o meno un comando sulla seriale è una responsabilità applicativa, 
            quindi deve stare nel codice che usa Heater, cioè fuori da essa, per mantenere una buona separazione delle responsabilità (principio SOLID: Single Responsibility).
        '''
        current_state = self.thc.get_output_control()

        if current_state != self._last_output_state_thc:
            self._last_output_change_time_thc = time.time()
            self._last_output_state_thc = current_state

        # Se è cambiato e il nuovo stato è stabile da X secondi
        if (
            self._last_output_state_thc != self._debounced_heater_output_thc and
            self._last_output_change_time_thc is not None and
            (time.time() - self._last_output_change_time_thc) >= self._debounce_duration
        ):
            self._debounced_heater_output_thc = self._last_output_state_thc
            self.queue_command("HTR01", self._debounced_heater_output_thc)
            self.main_software_thread_log_message('INFO', f"⚙️ Debounced Heater state sent: {self._debounced_heater_output_thc}")

        # Check for persistent errors
        self.check_errors(current_temperatures)               
            
            
        # HUMIDITY CONTROLLER SECTION
        self.hhc.update(list(current_humidities.values()))
        current_state = self.hhc.get_output_control()

        if current_state != self._last_output_state_hhc:
            self._last_output_change_time_hhc = time.time()
            self._last_output_state_hhc = current_state

        # Se è cambiato e il nuovo stato è stabile da X secondi
        if (
            self._last_output_state_hhc != self._debounced_heater_output_hhc and
            self._last_output_change_time_hhc is not None and
            (time.time() - self._last_output_change_time_hhc) >= self._debounce_duration
        ):
            self._debounced_heater_output_hhc = self._last_output_state_hhc
            self.queue_command("HUMER01", self._debounced_heater_output_hhc)
            self.main_software_thread_log_message('INFO', f"⚙️ Debounced Humidifier state sent: {self._debounced_heater_output_hhc}")
        
        '''
        if current_humidities:
            self.hhc.update(list(current_humidities.values()))
            if self.current_humidifier_output_control != self.hhc.get_output_control():
                self.current_humidifier_output_control = self.hhc.get_output_control()
                self.queue_command("HUMER01", self.current_humidifier_output_control) # @<HUMER01, True># @<HUMER01, False>#
        '''    
            
        # Emit the data to update the view        
        # Collecting all values into a single list
        # questo all_values è semplicemente una lista [17.8, 17.9, 18.0, 17.9, 17.8, 17.9] dove SO IO ad ogni posto cosa è associato...passiamo solo i valori (non bellissimo...)
        # i mean value servono per pubblicare il valore che il controllore usa per fare effettivamente il controllo e lo metto nella sezione di isteresi
        # list() se passi un dizionario, mentre [] se vuoi aggiugnere alla lista elementi singoli
        if current_temperatures and current_humidities:
            # all_values for MAIN VIEW page
            all_values = []
            all_values = list(current_temperatures.values()) + \
                        list(current_humidities.values()) + \
                        list(current_humidities_temperatures.values()) + \
                        [self.thc.get_mean_value()] + \
                        [self.hhc.get_mean_value()] + \
			[self.thc.get_output_control()] + \
                        [self.hhc.get_output_control()] + \
			list(current_external_temperature.values())
                        
            self.update_view.emit(all_values)
            
            # all_values for STATISTICS page
            all_values = []
            all_values.append(self.thc.get_min_value())
            all_values.append(self.thc.get_mean_value())
            all_values.append(self.thc.get_max_value())
            all_values.append(self.thc.get_on_count())
            all_values.append(self.thc.get_off_count())
            all_values.append(self.thc.get_time_on())
            all_values.append(self.thc.get_time_off())
            
            all_values.append(self.hhc.get_min_value())
            all_values.append(self.hhc.get_mean_value())
            all_values.append(self.hhc.get_max_value())
            all_values.append(self.hhc.get_on_count())
            all_values.append(self.hhc.get_off_count())
            all_values.append(self.hhc.get_time_on())
            all_values.append(self.hhc.get_time_off())
            
            self.update_statistics.emit(all_values)

            # + saving in parameters file relevan statistics
            self.save_parameter('TEMPERATURE_HYSTERESIS_CONTROLLER_TIME_ON', self.thc.get_time_on())
            self.save_parameter('TEMPERATURE_HYSTERESIS_CONTROLLER_TIME_OFF', self.thc.get_time_off())
        
        # INDUCTOR SECTION
        """
            IND_CCW - IND_CW induttori posizioni limite counterClockWise - clockWise
            IND_HOR induttore posizionamento orizzontale
            IND_DR door (apertura/chiusura della porta)
        """
        if current_inductors_feedbacks: # that is checking if the dictionary is not empty
            #print(current_inductors_feedbacks)
            
            value = current_inductors_feedbacks.get("IND_CCW") # to handle cases where the key might not exist safely, you can use .get() - questa funzione tira fuori il valore, non la key!
            if value is not None and value == 1:
                # perché nella mia convenzione value == 1 significa rising edge della posizione ragginuta
                self.eggTurnerMotor.acknowledgeFromExternal("IND_CCW")
                
            value = current_inductors_feedbacks.get("IND_CW") # to handle cases where the key might not exist safely, you can use .get()
            if value is not None and value == 1:
                self.eggTurnerMotor.acknowledgeFromExternal("IND_CW")
        
        
        
             
        
        # SAVING DATA IN FILES
        time_difference = datetime.now() - self.last_saving_time
        
        if (time_difference >= timedelta(minutes = self.saving_interval)):
                start_time = time.perf_counter()                
                self.save_data_to_files('Temperatures', current_temperatures) #{'TMP01': 23.1, 'TMP02': 23.1, 'TMP03': 23.1}
                self.save_data_to_files('Humidity', current_humidities) #{'HUM01': 52.5}
                self.save_data_to_files('Heater', {'Heater_Status': self.thc.get_output_control()})  # need to pass a dictionary
                self.save_data_to_files('Humidifier', {'Humidifier_status': self.hhc.get_output_control()}) 
                self.last_saving_time = datetime.now()
                self.main_software_thread_log_message('SAVING', f"Saved data! {self.last_saving_time}")
                
                
                end_time = time.perf_counter()
                #print(f"Time requested for saving data [milli-seconds]: {(end_time - start_time)*1000}")

    def write_log_section_header(self):
        """
        Writes a section header in the existing daily log file to mark the start of a new thread run.
        """
        current_date = datetime.now().strftime('%Y-%m-%d')
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        log_file = os.path.join(self.main_software_thread_log_folder_path, f"log_{current_date}.txt")

        with open(log_file, 'a', encoding='utf-8') as file:
            file.write("\n")
            file.write("=====================================\n")
            file.write(f" New Run - {timestamp}\n")
            file.write("=====================================\n") 

    def main_software_thread_log_message(self, error_type, message, suppress_terminal_print = False):
        """
        Logs a message to a log file named with the current date inside the Log folder.
        Format: [ERROR_TYPE] Message @ Timestamp

        Args:
            error_type (str): The type of error (e.g., 'INFO', 'WARNING', 'ERROR').
            message (str): The message to log.
        """
        if not suppress_terminal_print:
            print(message)
        
        current_date = datetime.now().strftime('%Y-%m-%d')
        log_file = os.path.join(self.main_software_thread_log_folder_path, f"log_{current_date}.txt")
        
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        with open(log_file, 'a', encoding='utf-8') as file:
            file.write(f"[{error_type}] {message} @ {timestamp}\n")
        '''
        OLD VERSION FOR JSON FORMAT
        
        """
        Logs a message to a log file named with the current date inside the Log folder in plain text format with sections.

        Args:
            error_type (str): The type of error (e.g., 'INFO', 'WARNING', 'ERROR').
            message (str): The message to log.
        """
        print(message)
        current_date = datetime.now().strftime('%Y-%m-%d')
        log_file = os.path.join(self.main_software_thread_log_folder_path, f"log_{current_date}.txt")
        
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        with open(log_file, 'a') as file:
            file.write(f"--- Log Entry ---\n")
            file.write(f"Timestamp: {timestamp}\n")
            file.write(f"Error Type: {error_type}\n")
            file.write(f"Message: {message}\n")
            file.write(f"-----------------\n\n")
        '''
    def save_data_to_files(self, data_type, data_dictionary): #passo un dictionary di temperature/humidities, dimensione variabile per gestire più o meno sensori dinamicamente
        now = datetime.now()
        current_date = now.strftime('%Y-%m-%d')
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        machine_statistics_folder_path = os.path.join(script_dir, "Machine_Statistics") 
        # faccio selezione del folder da cui pescare i dati.
        if data_type == 'Temperatures':
            folder_path = os.path.join(machine_statistics_folder_path, 'Temperatures')
        elif data_type == 'Humidity':
            folder_path = os.path.join(machine_statistics_folder_path, 'Humidity')
        elif data_type == 'Heater':
            folder_path = os.path.join(machine_statistics_folder_path, 'Heater')
        elif data_type == 'Humidifier':
            folder_path = os.path.join(machine_statistics_folder_path, 'Humidifier')
        else:
            raise ValueError("Invalid path configuration")	
            
        file_path = os.path.join(folder_path, f"{current_date}.csv")

        # Initialize the CSV file with headers if it doesn't exist
        if not os.path.exists(file_path):
            with open(file_path, mode='w', newline='') as file: # write mode
                writer = csv.writer(file)

                # Initialize the list with 'Timestamp' as the first element
                result_list = ['Timestamp']

                # Append the keys from the dictionary to the list
                result_list.extend(data_dictionary.keys())

                writer.writerow(result_list)

        timestamp = now.strftime('%Y-%m-%d %H:%M:%S')

        with open(file_path, mode='a', newline='') as file: # append mode
            writer = csv.writer(file)

            # Initialize the list with the timestamp as the first element
            values_list = [timestamp]

            # Append the values from the dictionary to the list
            values_list.extend(data_dictionary.values())
            writer.writerow(values_list)

    # === GESTIONE PARAMETRI === #
    def parameters_initialization_from_file(self):
        '''
        TEMPERATURE_HYSTERESIS_CONTROLLER_UPPER_LIMIT
        TEMPERATURE_HYSTERESIS_CONTROLLER_LOWER_LIMIT
        HUMIDITY_HYSTERESIS_CONTROLLER_UPPER_LIMIT
        HUMIDITY_HYSTERESIS_CONTROLLER_LOWER_LIMIT
        TEMPERATURE_HYSTERESIS_CONTROLLER_TIME_ON
        TEMPERATURE_HYSTERESIS_CONTROLLER_TIME_OFF
        HUMIDITY_HYSTERESIS_CONTROLLER_TIME_ON
        HUMIDITY_HYSTERESIS_CONTROLLER_TIME_OFF
        TURNS_COUNTER
        ROTATION_INTERVAL
        '''
        # TEMPERATURE SPINBOX MIN/MAX
        thc_upper_limit = self.load_parameter('TEMPERATURE_HYSTERESIS_CONTROLLER_UPPER_LIMIT')
        thc_lower_limit = self.load_parameter('TEMPERATURE_HYSTERESIS_CONTROLLER_LOWER_LIMIT')

        if (thc_upper_limit is None) or (thc_lower_limit is None):
            # do nothing, leave default values
            pass        
        elif (thc_upper_limit < thc_lower_limit):
            # check for errors in parameters
            pass
        else:
            # set SW
            self.thc.set_upper_limit(thc_upper_limit)
            self.thc.set_lower_limit(thc_lower_limit)
            # set GUI
            self.update_spinbox_value.emit("maxHysteresisValue_temperature_spinBox", thc_upper_limit)
            self.update_spinbox_value.emit("minHysteresisValue_temperature_spinBox", thc_lower_limit)

        # HUMIDITY SPINBOX MIN/MAX
        hhc_upper_limit = self.load_parameter('HUMIDITY_HYSTERESIS_CONTROLLER_UPPER_LIMIT')
        hhc_lower_limit = self.load_parameter('HUMIDITY_HYSTERESIS_CONTROLLER_LOWER_LIMIT')

        if (hhc_upper_limit is None) or (hhc_lower_limit is None):
            # do nothing, leave default values
            pass        
        elif (hhc_upper_limit < hhc_lower_limit):
            # check for errors in parameters
            pass
        else:
            # set SW
            self.hhc.set_upper_limit(hhc_upper_limit)
            self.hhc.set_lower_limit(hhc_lower_limit)
            # set GUI
            self.update_spinbox_value.emit("maxHysteresisValue_humidity_spinBox", hhc_upper_limit)
            self.update_spinbox_value.emit("minHysteresisValue_humidity_spinBox", hhc_lower_limit)

        # TEMPERATURE TIMINGS
        thc_time_on = self.load_parameter('TEMPERATURE_HYSTERESIS_CONTROLLER_TIME_ON')
        thc_time_off = self.load_parameter('TEMPERATURE_HYSTERESIS_CONTROLLER_TIME_OFF')
        if (thc_time_on is None) or (thc_time_off is None):
            pass
        else:
            # la visualizzazione si aggiornerà da sola periodicamente
            self.thc.set_time_on(thc_time_on) 
            self.thc.set_time_off(thc_time_off)             

        # HUMIDITY TIMINGS
        hhc_time_on = self.load_parameter('HUMIDITY_HYSTERESIS_CONTROLLER_TIME_ON')
        hhc_time_off = self.load_parameter('HUMIDITY_HYSTERESIS_CONTROLLER_TIME_OFF')
        if (hhc_time_on is None) or (hhc_time_off is None):
            pass
        else:
            # la visualizzazione si aggiornerà da sola periodicamente
            self.hhc.set_time_on(hhc_time_on) 
            self.hhc.set_time_off(hhc_time_off)     

        # EGG TURNS COUNTER
        turns_counter = self.load_parameter('TURNS_COUNTER')
        if turns_counter is not None:
            self.eggTurnerMotor.setTurnsCounter(turns_counter) # la visualizzazione si aggiornerà da sola periodicamente

        # ROTATION INTERVAL
        rotation_interval = self.load_parameter('ROTATION_INTERVAL')
        if rotation_interval is not None:
            self.eggTurnerMotor.setFunctionInterval(rotation_interval * 60) # Set the rotation interval
            self.update_spinbox_value.emit("rotation_interval_spinBox", rotation_interval) # aggiorno la visualizzazione


    def _load_all_parameters(self):
        if os.path.exists(self.parameters_file_path):
            try:
                with open(self.parameters_file_path, "r") as f:
                    self.parameters = json.load(f)
            except Exception as e:
                print(f"Errore nel caricamento dei parametri: {e}")
                self.parameters = {}
        else:
            self.parameters = {}
    
    def _save_all_parameters(self):
        """Salva tutti i parametri nel file, con backup automatico."""
        try:
            # Se il file originale esiste, crea una copia di backup
            '''
            if os.path.exists(self.parameters_file_path):
                backup_path = self.parameters_file_path + ".bak"
                shutil.copy2(self.parameters_file_path, backup_path)
            '''

            # Ora salva il nuovo contenuto
            with open(self.parameters_file_path, "w") as f:
                json.dump(self.parameters, f, indent=4)
        except Exception as e:
            print(f"Errore nel salvataggio dei parametri: {e}")

    def load_parameter(self, key):
        """Restituisce il valore del parametro, o None se non esiste."""
        return self.parameters.get(key, None)

    def save_parameter(self, key, value):
        """Salva o aggiorna un parametro e lo scrive su file."""
        self.parameters[key] = value
        self.main_software_thread_log_message('INFO', f"Saving parameter {key}: {value}", suppress_terminal_print = True)
        self._save_all_parameters()

    def _load_from_backup(self):
        backup_path = self.parameters_file_path + ".bak"
        if os.path.exists(backup_path):
            try:
                with open(backup_path, "r") as f:
                    self.parameters = json.load(f)
                print("Parametri caricati dal backup.")
            except Exception as e:
                print(f"Errore nel caricamento del backup: {e}")

    def backup_parameters_file(self):
        """Crea una copia di backup di parameters.json con data e ora nel nome, estensione .bak."""
        if not os.path.exists(self.parameters_file_path):
            print("Nessun file di parametri da salvare.")
            return

        # Estrai nome base e cartella
        dir_path = os.path.dirname(self.parameters_file_path)
        base_name = os.path.splitext(os.path.basename(self.parameters_file_path))[0]

        # Costruisci nome con timestamp
        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        backup_filename = f"{base_name}_{ts}.json.bak"
        backup_path = os.path.join(dir_path, backup_filename)

        try:
            shutil.copy2(self.parameters_file_path, backup_path)
            print(f"Backup creato: {backup_filename}")
        except Exception as e:
            print(f"Errore durante il backup: {e}")
            
    class HysteresisController:
        def __init__(self, lower_limit, upper_limit):            
            self.lower_limit = lower_limit
            self.upper_limit = upper_limit
            self._min_value = float('inf')  # Track the minimum value observed
            self._max_value = float('-inf') # Track the maximum value observed
            self._mean_value = None         # Mean value of the monitored inputs
            self._output_control = False    # False = OFF, True = ON
            
            # Statistics tracking
            self.on_count = 0
            self.off_count = 0
            
            self.reset_timer_to_do = True
            self.measuring_time_interval_sec = 1 # ogni n secondi aggiorno i conteggi di tempo on/off dell'attuatore
            self.last_measuring_time = time.time() # si ricorda appena ho preso il campione precedente
            self.effective_time_difference = 0.0
            # questi sono i contatori di timer
            self.time_on = 0.0
            self.time_off = 0.0
            
            self._last_switch_time = time.time()
            
            self.forceON = False
            self.forceOFF = False

        def set_time_on(self, time_on):
            self.time_on = time_on

        def set_time_off(self, time_off):
            self.time_off = time_off

        # Method to set the lower limit
        def set_lower_limit(self, lower_limit):
            self.lower_limit = lower_limit
            if self.lower_limit == self.upper_limit:
                self._output_control = False  # Safety enforcement
        
        # Method to get the lower limit
        def get_lower_limit(self):
            return self.lower_limit
        
        # Method to set the upper limit
        def set_upper_limit(self, upper_limit):
            self.upper_limit = upper_limit
            if self.lower_limit == self.upper_limit:
                self._output_control = False  # Safety enforcement
                
        def set_control_mode(self, control_mode):
            if control_mode == "forceON":
                self.forceON = True
                self.forceOFF = False
            if control_mode == "forceOFF":
                self.forceON = False
                self.forceOFF = True
            if control_mode == "AUTO":
                self.forceON = False
                self.forceOFF = False
        
        # Method to get the upper limit
        def get_upper_limit(self):
            return self.upper_limit

        # Update method to process input values and apply hysteresis logic
        def update(self, values):
            # reset timers all'avvio
            if self.reset_timer_to_do:
                self.last_measuring_time = time.time()
                self.reset_timer_to_do = False
                
            '''
            # Safety check: If limits are equal, force OFF state
            if self.lower_limit == self.upper_limit:
                self._output_control = False
                return
            '''
        
            # Ensure values is a list for consistency
            if not isinstance(values, list):
                values = [values]
            
            
            new_output = self._output_control
            
            if not values:
                new_output = False # FOR SAFETY!! no values in input, means no good temperatures are passed, then OFF the actuator
            else:            
                # Calculate the mean of the values
                self._mean_value = round(sum(values) / len(values), 1)
                            
                # Update the max and min values reached
                for value in values:
                    self._max_value = max(self._max_value, value)
                    self._min_value = min(self._min_value, value)                
                
                # Check if output state changes
                if self.lower_limit == self.upper_limit:
                    new_output = False
                elif self.forceON: 
                    new_output = True               
                elif self.forceOFF:  
                    new_output = False              
                else: # AUTO                
                    if self._mean_value >= self.upper_limit:
                        new_output = False  # Turn OFF if mean is above upper limit
                    elif self._mean_value <= self.lower_limit:
                        new_output = True   # Turn ON if mean is below lower limit
                
            # update time tracking (continuous)
            elapsed_time = time.time() - self.last_measuring_time
            if elapsed_time >= self.measuring_time_interval_sec:
                if self._output_control:
                    self.time_on += elapsed_time
                else:
                    self.time_off += elapsed_time
                self.last_measuring_time = time.time()
                
            # If state changed, update count transitions
            if new_output != self._output_control:
                elapsed_time = time.time() - self.last_measuring_time
                if self._output_control:
                    self.time_on += elapsed_time
                    self.off_count += 1
                else:
                    self.time_off += elapsed_time
                    self.on_count += 1  
                self.last_measuring_time = time.time()                
                
            self._output_control = new_output
            
                
        
        # Method to get the output control (True or False)
        def get_output_control(self):
            return self._output_control
        
        # Method to get the maximum value reached
        def get_max_value(self):
            return self._max_value
        
        # Method to get the minimum value reached
        def get_min_value(self):
            return self._min_value

        # Method to get the mean value of the last monitored inputs
        def get_mean_value(self):
            return self._mean_value

        # Method to reset min/max values
        def reset_max_value(self):
            self._max_value = float('-inf')
        
        def reset_min_value(self):
            self._min_value = float('inf')

        def reset_all_values(self):
            self._min_value = float('inf')
            self._max_value = float('-inf')

        # Method to get the number of ON/OFF transitions
        def get_on_count(self):
            return self.on_count

        def get_off_count(self):
            return self.off_count

        # Method to get the total time spent ON/OFF
        def get_time_on(self):
            return round(self.time_on, 0)

        def get_time_off(self):
            return round(self.time_off, 0)

        # Method to reset statistics
        def reset_time_statistics(self):
            self.on_count = 0
            self.off_count = 0
            self.time_on = 0.0
            self.time_off = 0.0
            self._last_switch_time = time.time()
        
        def reset_absolute_temperatures_statistics(self):
            self._max_value = float('-inf')
            self._min_value = float('inf')
            
    class StepperMotor:
        def __init__(self, name="Stepper1"):
            self.name = name
            self.running = False
            
            """
                self.main_state:
                1 = INITIALIZATION
                10 MANUAL   _MODE
                30 AUTOMATIC_MODE
            
            """
            self.main_state = "INITIALIZATION"
            
            self.manual_state = "WAITING_FOR_COMMAND"
            
            '''
                Vogio gestire tutto con una variabile...anziché avere direzione e movimento, ne ho una sola
                Il tribolo è lo stop. Not moving, ma in che direzione? allora faccio due valori della variabile, che sono loro che si ricordano
                da dove mi stavo fermando
                stopped_from_CCW_rotation_direction
                stopped_from_CW_rotation_direction
                
                self.rotation_state 
                not_defined
                CCW_reached
                CW_reached
                CCW_rotation_direction
                CW_rotation_direction
                laying_horizontal_position
                
            '''
            self.rotation_state = "not_defined"
            
            
            self.last_execution_time = time.time()
            self.auto_function_interval_sec = 3600  # Default to 1 hour
            self.last_ack_time = None
            self.alarm_triggered = False
            self.turnsCounter = 0
            
            self.stop_command = False
            
            self.new_command = None
            self.update_motor_data = False
            self.acknowledge_from_external = None            
            
            self.force_change_rotation_flag = False
            
            self.rotation_in_progress_timeout_sec = 120 # 2min timeout
            
            # TIMING
            self.last_motor_data_update_sec = time.time()
            self.motor_data_update_interval_sec = 15 # indica ogni quanti secondi viene fatto l'update dei dati relativi al motore

            # Variabili che uso per il salvataggio dei dati: mi servono per: 
            # 1) fare la foto allo stato stabile di rotazione che raggiungo
            # 2) fare la foto all'istante (datetime) in cui questa cosa succede

            self.last_time_stable_position_is_reached = None
            self.last_stable_position = None

        def setTurnsCounter(self, value):
            self.turnsCounter = value

        def moveCWContinuous(self):
            self.main_state = "MANUAL_MODE"
            self.manual_state = "MOVING_CW_CONTINUOUSLY"
            print(f"{self.name}: Moving cw continuously.")

        def moveCCWContinuous(self):
            self.main_state = "MANUAL_MODE"
            self.manual_state = "MOVING_CCW_CONTINUOUSLY"
            print(f"{self.name}: Moving ccw continuously.")

        def stop(self):
            self.stop_command = True
            print(f"{self.name}: Stopping motor.")
        
        def setFunction1(self): # ?? non la usa nessuno
            self.main_state = "AUTOMATIC_MODE"
            self.last_ack_time = time.time()
            self.alarm_triggered = False
            print(f"{self.name}: Automatic function 1 enabled.")
        
        def stopFunction1(self): # ?? non la usa nessuno
            self.main_state = "STOPPED"
            print(f"{self.name}: Stopping automatic function 1.")
        
        def setFunctionInterval(self, interval):
            self.auto_function_interval_sec = interval
            print(f"{self.name}: Automatic function interval set to {interval} seconds.")
        
        def acknowledgeFromExternal(self, ack):
            self.last_ack_time = time.time()
            self.acknowledge_from_external = ack  
            print(f"Acknowledge received from external:{ack}")          
            
        def resetNewCommand(self):
            self.new_command = None
            
        def resetUpdateMotorData(self):
            self.update_motor_data = None
        
        def getRotationDirection(self):
            return self.rotation_direction
        
        def getTimeSinceLastRotation(self):
            time_result = round(time.time() - self.last_execution_time, 0) # in seconds
            return time_result
        
        def getTimeUntilNextRotation(self):
            time_result = round(max(0, self.auto_function_interval_sec - (time.time() - self.last_execution_time)), 0) # in seconds
            #print(f"Time until next rotation:{time_result} seconds")  
            return time_result
        
        def getTurnsCounter(self):
            return self.turnsCounter
        
        def getUpdateMotorData(self):
            return self.update_motor_data
        
        def getNewCommand(self):
            return self.new_command
        
        def forceEggsRotation(self):
            self.force_change_rotation_flag = True
            self.main_state = "AUTOMATIC_MODE" # go back in automatic, if you were in manual
        
        def update(self):
            current_time = time.time()
            previous_rotation_state = self.rotation_state
            
            if self.main_state == "INITIALIZATION":
                if self.acknowledge_from_external is not None:                    
                    if self.acknowledge_from_external == "IND_CCW": 
                        self.rotation_state = "CCW_reached"
                        print("Homing done. Reached the Counter Clock Wise limit switch")
                        self.acknowledge_from_external = None # reset
                        
                    if self.acknowledge_from_external == "IND_CW":
                        self.rotation_state = "CW_reached"
                        print("Homing done. Reached the Clock Wise limit switch")                        
                        self.acknowledge_from_external = None # reset
                        
                    self.last_execution_time = current_time # salvo il tempo, perché così la prossima FULL TURN avviene contando il tempo da quando ho finito l'homing
                    self.main_state = "AUTOMATIC_MODE"
                else:
                    pass
                
            elif self.main_state == "MANUAL_MODE":
                    if self.manual_state == "WAITING_FOR_COMMAND": #waiting for command
                        pass
                        
                    if self.manual_state == "MOVING_CW_CONTINUOUSLY": # CW continuous moving
                        self.rotation_state = "CW_rotation_direction"
                        self.new_command = "manual_" + self.rotation_state
                        self.manual_state = "ROTATION_IN_PROGRESS" 
                        
                    if self.manual_state == "MOVING_CCW_CONTINUOUSLY": # CCW continuous moving
                        self.rotation_state = "CCW_rotation_direction"
                        self.new_command = "manual_" + self.rotation_state
                        self.manual_state = "ROTATION_IN_PROGRESS" 
                        
                    if self.manual_state == "ROTATION_IN_PROGRESS": # rotation in progress
                        # IF ack from limit switch --> comunica che è arrivato ack, ma di fatto si ferma da solo per Arduino
                        if self.acknowledge_from_external is not None:         
                            if self.acknowledge_from_external == "IND_CCW" and self.rotation_state == "CCW_rotation_direction":
                                self.rotation_state = "CCW_reached"
                                print("Reached the IND_CCW limit switch")
                                self.acknowledge_from_external = None # reset
                                self.manual_state = "WAITING_FOR_COMMAND"
                                
                            if self.acknowledge_from_external == "IND_CW" and self.rotation_state == "CW_rotation_direction":
                                self.rotation_state = "CW_reached"
                                print("Reached the IND_CW limit switch")
                                self.acknowledge_from_external = None # reset
                                self.manual_state = "WAITING_FOR_COMMAND"
                        
                        # IF stop command, then stop
                        if self.stop_command:
                            self.manual_state = "STOPPED"
                            self.stop_command = False
                        
                    if self.manual_state == "STOPPED": 
                        self.new_command = "stop"
                        
                        if self.rotation_state == "CCW_rotation_direction":
                            self.rotation_state = "stopped_from_CCW_rotation_direction"
                            
                        if self.rotation_state == "CW_rotation_direction":
                            self.rotation_state = "stopped_from_CW_rotation_direction"
                            
                        self.manual_state = "WAITING_FOR_COMMAND"
            
            elif self.main_state == "AUTOMATIC_MODE":
                if (current_time - self.last_execution_time >= self.auto_function_interval_sec or self.force_change_rotation_flag):
                    
                    if self.rotation_state == "CCW_reached" or self.rotation_state == "CCW_rotation_direction" or self.rotation_state == "stopped_from_CCW_rotation_direction":
                        self.rotation_state = "CW_rotation_direction"
                        
                    elif self.rotation_state == "CW_reached" or self.rotation_state == "CW_rotation_direction" or self.rotation_state == "stopped_from_CW_rotation_direction":
                        self.rotation_state = "CCW_rotation_direction"
                        
                    print(f"{self.name}: Changing rotation direction to {self.rotation_state}.")                    
                    self.last_execution_time = current_time
                    self.turnsCounter += 1
                    self.new_command = "automatic_" + self.rotation_state
                    print(f"{'Activating' if (self.rotation_state == "CW_rotation_direction" or self.rotation_state == "CCW_rotation_direction") else 'Deactivating'} diagnostics")
                    
                
                    
                    
                if self.force_change_rotation_flag:
                    self.force_change_rotation_flag = False # resetting
                    
                    
                if (self.rotation_state == "CCW_rotation_direction" or self.rotation_state == "CW_rotation_direction"):
                    # diagnostic: attesa del feedback
                    if (time.time() - self.last_execution_time) >= self.rotation_in_progress_timeout_sec:
                        # manda comando di STOP
                        #self.new_command = "stop"
                        pass
                        
                    if self.acknowledge_from_external is not None:         
                        if self.acknowledge_from_external == "IND_CCW" and self.rotation_state == "CCW_rotation_direction":
                            self.rotation_state = "CCW_reached"
                            self.last_stable_position = self.rotation_state
                            self.last_time_stable_position_is_reached = datetime.now()
                            print(f"{'Activating' if (self.rotation_state == "CW_rotation_direction" or self.rotation_state == "CCW_rotation_direction") else 'Deactivating'} diagnostics")
                            print("Reached the IND_CCW limit switch")
                            self.acknowledge_from_external = None # reset
                            
                        if self.acknowledge_from_external == "IND_CW" and self.rotation_state == "CW_rotation_direction":
                            self.rotation_state = "CW_reached"
                            self.last_stable_position = self.rotation_state
                            self.last_time_stable_position_is_reached = datetime.now()
                            print(f"{'Activating' if (self.rotation_state == "CW_rotation_direction" or self.rotation_state == "CCW_rotation_direction") else 'Deactivating'} diagnostics")
                            print("Reached the IND_CW limit switch")
                            self.acknowledge_from_external = None # reset
                            
                else:
                    pass
					
		# ogni tot diciamo che è passato il tempo per fare un update dei dati del motore: il programma, da fuori, riceve il segnale e prende i dati
                #print(time.time() - self.last_motor_data_update_sec)
                if ((time.time() - self.last_motor_data_update_sec) >= self.motor_data_update_interval_sec # update periodico
                    or 
                    self.force_change_rotation_flag # forzatura dell'update quando forzo un giro a mano
                    or
                    (self.new_command is not None and "automatic_" in self.new_command) # forzatura dell'update se non è scaduto il tempo ma se è scattato il comando di turn
		    or
		    (previous_rotation_state == "CCW_rotation_direction" and self.rotation_state == "CCW_reached") # forzatura update della visu se non è scaduto il tempo ma ho raggiunto il finecorsa
		    or
		    (previous_rotation_state == "CW_rotation_direction" and self.rotation_state == "CW_reached")
		    ):
                    self.update_motor_data = True
                    self.last_motor_data_update_sec = time.time()
                
            #print(f"{self.main_state} {self.manual_state} {self.rotation_state} {self.new_command}")
                            
                        
                    
                
                
            '''
            if self.last_ack_time and (current_time - self.last_ack_time > 180):  # 3 minutes timeout
                if not self.alarm_triggered:
                    self.stop()
                    print(f"{self.name}: ERROR! No acknowledgment received for 2 minutes. Stopping motor and raising alarm!")
                    self.alarm_triggered = True
            '''
            
            if self.acknowledge_from_external is not None:   
                '''
                    I feedback degli induttori sono msi asincroni dal process serial data dentro qusta variabile. Se arriva per qualche motivo mentre
                    sono in uno stato dove non me lo aspetto, allora non viene resettato, rimane in variabile e dopo apena entro in modalità automatica
                    la variabile l'ha ancora in pancia e quindi dà una falsa lettura. Allora, alla fine del ciclo, se non l'ho usato, lo resetto.
                '''                  
                self.acknowledge_from_external = None # reset 

class MainWindow(QtWidgets.QMainWindow):
    # Define custom signals - this is done to send button/spinBox and other custom signals to other thread MainSoftwareThread: use Qt Signals
    button_clicked = QtCore.pyqtSignal(str)  # Emits button name
    float_spinBox_value_changed = QtCore.pyqtSignal(str, float)  # Emits spinbox value  // METTI INT se intero
    initialization_step = QtCore.pyqtSignal(str, float)
    initialization_done = QtCore.pyqtSignal(str, bool)
    radio_button_toggled = QtCore.pyqtSignal(str, bool)
    
    def __init__(self, main_software_thread):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.main_software_thread = main_software_thread
        self.main_software_thread.update_view.connect(self.update_display_data)
        self.main_software_thread.update_statistics.connect(self.update_statistics_data)
        self.main_software_thread.update_motor.connect(self.update_display_motor_data)
        
        # Connect signals to main software thread slots
        self.button_clicked.connect(self.main_software_thread.handle_button_click)
        self.float_spinBox_value_changed.connect(self.main_software_thread.handle_float_spinBox_value)
        self.initialization_step.connect(self.main_software_thread.handle_intialization_step)
        self.initialization_done.connect(self.main_software_thread.handle_initialization_done) # signals that MainWindow has completed the initialization procedure (all emit signals have been sent)
        self.main_software_thread.update_spinbox_value.connect(self.update_spinbox)
        self.radio_button_toggled.connect(self.main_software_thread.handle_radio_button_toggle)


        # Connect buttons to handlers that emit signals
        self.ui.move_CW_motor_btn.clicked.connect(lambda: self.emit_button_signal(self.ui.move_CW_motor_btn.objectName()))
        self.ui.move_CCW_motor_btn.clicked.connect(lambda: self.emit_button_signal(self.ui.move_CCW_motor_btn.objectName()))
        self.ui.layHorizontal_motor_btn.clicked.connect(lambda: self.emit_button_signal(self.ui.layHorizontal_motor_btn.objectName()))
        self.ui.forceEggsTurn_motor_btn.clicked.connect(lambda: self.emit_button_signal(self.ui.forceEggsTurn_motor_btn.objectName()))
        self.ui.reset_statistics_T_btn.clicked.connect(lambda: self.emit_button_signal(self.ui.reset_statistics_T_btn.objectName()))
        
        self.ui.plotAllDays_temp_T_btn.clicked.connect(lambda: self.emit_button_signal(self.ui.plotAllDays_temp_T_btn.objectName()))
        self.ui.plotToday_temp_T_btn.clicked.connect(lambda: self.emit_button_signal(self.ui.plotToday_temp_T_btn.objectName()))
        self.ui.plotAllDays_humidity_H_btn.clicked.connect(lambda: self.emit_button_signal(self.ui.plotAllDays_humidity_H_btn.objectName()))
        self.ui.plotToday_humidity_H_btn.clicked.connect(lambda: self.emit_button_signal(self.ui.plotToday_humidity_H_btn.objectName()))
        
        self.ui.plotToday_cnt_T_btn.clicked.connect(lambda: self.emit_button_signal(self.ui.plotToday_cnt_T_btn.objectName()))
        self.ui.plotAllDays_cnt_T_btn.clicked.connect(lambda: self.emit_button_signal(self.ui.plotAllDays_cnt_T_btn.objectName()))
        
        self.ui.plotToday_cnt_H_btn.clicked.connect(lambda: self.emit_button_signal(self.ui.plotToday_cnt_H_btn.objectName()))
        self.ui.plotAllDays_cnt_H_btn.clicked.connect(lambda: self.emit_button_signal(self.ui.plotAllDays_cnt_H_btn.objectName()))

        # Connect radio buttons to emit its values
        # Connect radio buttons to emit signals
        radio_buttons = [
            self.ui.heaterOFF_radioBtn,
            self.ui.heaterAUTO_radioBtn,
            self.ui.heaterON_radioBtn,
            self.ui.humidifierOFF_radioBtn,
            self.ui.humidifierAUTO_radioBtn,
            self.ui.humidifierON_radioBtn,
            self.ui.removeErrors_from_T_plots,
            self.ui.removeErrors_from_H_plots,
        ]
        for radio_button in radio_buttons:
            radio_button.toggled.connect(lambda state, btn=radio_button: self.emit_radio_button_signal(btn.objectName(), state))
        
        # Connect spinBox to emit its values
        self.ui.rotation_interval_spinBox.valueChanged.connect(lambda value: self.emit_float_spinbox_signal(self.ui.rotation_interval_spinBox.objectName(), value))
        
        # Temperature Hysteresis
        self.ui.maxHysteresisValue_temperature_spinBox.valueChanged.connect(lambda value: self.emit_float_spinbox_signal(self.ui.maxHysteresisValue_temperature_spinBox.objectName(), value))
        self.ui.minHysteresisValue_temperature_spinBox.valueChanged.connect(lambda value: self.emit_float_spinbox_signal(self.ui.minHysteresisValue_temperature_spinBox.objectName(), value))
        
        # Humidity Hysteresis
        self.ui.maxHysteresisValue_humidity_spinBox.valueChanged.connect(lambda value: self.emit_float_spinbox_signal(self.ui.maxHysteresisValue_humidity_spinBox.objectName(), value))
        self.ui.minHysteresisValue_humidity_spinBox.valueChanged.connect(lambda value: self.emit_float_spinbox_signal(self.ui.minHysteresisValue_humidity_spinBox.objectName(), value))
        
        # Connect to send initialization values to the mainSoftwareThread
        self.emit_initialization_values(self.ui.maxHysteresisValue_temperature_spinBox.objectName(), self.ui.maxHysteresisValue_temperature_spinBox.value())
        self.emit_initialization_values(self.ui.minHysteresisValue_temperature_spinBox.objectName(), self.ui.minHysteresisValue_temperature_spinBox.value())
        
        self.emit_initialization_values(self.ui.maxHysteresisValue_humidity_spinBox.objectName(), self.ui.maxHysteresisValue_humidity_spinBox.value())
        self.emit_initialization_values(self.ui.minHysteresisValue_humidity_spinBox.objectName(), self.ui.minHysteresisValue_humidity_spinBox.value())
        
        # signaling that ManWindow initialization procedure has been completed
        self.initialization_done.emit("GUI_initialization_procedure", True)


    def emit_initialization_values(self, spinbox_name, value):
        self.initialization_step.emit(spinbox_name, value)
        
        
    def emit_button_signal(self, button_name):
        self.button_clicked.emit(button_name)
         #print(f"Button clicked: {button_name}")

    def emit_float_spinbox_signal(self, spinbox_name, value):
        #value = float(value)  # Cast value to float explicitly
        self.float_spinBox_value_changed.emit(spinbox_name, value)
        #print(f"[MainWindow] Emitting signal from {spinbox_name} with value: {value}")
        
    def emit_radio_button_signal(self, radio_button_name, state):
        self.radio_button_toggled.emit(radio_button_name, state)

    def update_display_data(self, all_data):
        # Update the temperature labels in the GUI
        if len(all_data) >= 6: # perché il numero??
            self.ui.temperature1_T.setText(f"{all_data[0]} °C")
            self.ui.temperature2_T.setText(f"{all_data[1]} °C")
            self.ui.temperature3_T.setText(f"{all_data[2]} °C")
            self.ui.temperature4_T.setText(f"{all_data[3]} °C")
            self.ui.humidity1_H.setText(f"{all_data[4]} %")
            self.ui.temperatureFromHumidity1.setText(f"{all_data[5]} °C")
            self.ui.heatCtrlVal.setText(f"{all_data[6]} °C")
            self.ui.humCtrlVal.setText(f"{all_data[7]} %")
            if all_data[8] == True:
                self.ui.heaterStatus.setText(f"Heating ON!")
            else:
                self.ui.heaterStatus.setText(f"OFF")
            if all_data[9] == True:
                self.ui.humidifierStatus.setText(f"Humidifying ON!")
            else:
                self.ui.humidifierStatus.setText(f"OFF")
            self.ui.externalTemperature.setText(f"{all_data[10]} °C")
            #self.ui.temperature4_2.setText(f"{all_data[3]} °C") PER TEMPERATURA DA UMIDITA
            
    def update_statistics_data(self, all_data):
        # da implementare la parte di update delle statistiche
        if len(all_data) > 0:
            self.ui.minTemp_T.setText(f"{all_data[0]} °C")
            self.ui.meanTemp_T.setText(f"{all_data[1]} °C")
            self.ui.maxTemp_T.setText(f"{all_data[2]} °C")
            self.ui.onCounter_T.setText(f"{all_data[3]}")
            self.ui.offCounter_T.setText(f"{all_data[4]}")
			# VISUALIZZAZIONE DEI TEMPI: il programma di base mi manda dei secondi. E' qui che stampo la stringa opportunamente in min o h
            self.ui.timeOn_T.setText(self.format_time(all_data[5]))
            self.ui.timeOFF_T.setText(self.format_time(all_data[6]))
            self.ui.minHum_H.setText(f"{all_data[7]} %")
            self.ui.meanHum_H.setText(f"{all_data[8]} %")
            self.ui.maxHum_H.setText(f"{all_data[9]} %")
            self.ui.onCounter_H.setText(f"{all_data[10]}")
            self.ui.offCounter_H.setText(f"{all_data[11]}")
            self.ui.timeOn_H.setText(self.format_time(all_data[12]))
            self.ui.timeOFF_H.setText(self.format_time(all_data[13]))
            
        pass
    
    def format_time(self, value, unit = None, simple_format = False):
        """
            Unit argument is optional:
                If unit is "sec", it returns only seconds.
                If unit is "min", it returns only minutes.
                If unit is "hour", it returns only hours.
                If unit is None (default), it follows the mixed format.
        """
        
        if unit == "sec":
            return f"{value} sec"
        elif unit == "min":
            return f"{value // 60} min"
        elif unit == "hour":
            return f"{value // 3600} h"
        
        # Simple format: Only minutes if 60 ≤ value < 3600, only hours if value ≥ 3600
        if simple_format:
            if value >= 3600:
                return f"{value // 3600} h"
            elif value >= 60:
                return f"{value // 60} min"
        
        # Default behavior (detailed format)
        if value < 60:
            return f"{value} sec"
        elif value < 3600:
            minutes = value // 60
            seconds = value % 60
            return f"{minutes} min {seconds} sec" if seconds else f"{minutes} min"
        else:
            hours = value // 3600
            minutes = (value % 3600) // 60
            return f"{hours} h {minutes} min" if minutes else f"{hours} h"

            
    def update_display_motor_data(self, all_data):
        if len(all_data) > 0:
            self.ui.timePassed.setText(self.format_time(all_data[0]))
            self.ui.timeToNextTurn.setText(self.format_time(all_data[1]))
            self.ui.turnsCounter.setText(f"{all_data[2]}")
            self.ui.main_state.setText(f"{all_data[3]}")
            self.ui.manual_state.setText(f"{all_data[4]}")
            self.ui.rotation_state.setText(f"{all_data[5]}")

    def update_spinbox(self, spinbox_name, value):
        """ Update the spinbox in the GUI safely from another thread """
        spinbox = getattr(self.ui, spinbox_name, None)  # Get the spinbox dynamically

        if spinbox:  # Ensure the spinbox exists
            spinbox.setValue(value)  # Set the new value safely in the GUI thread
    '''       
    def handle_radio_button(self):
        sender = self.sender()
        if sender.isChecked():
            print(f"Radio button '{sender.text()}' selected")
    '''
    def closeEvent(self, event):
        # Ensure the threads are stopped when the window is closed
        self.main_software_thread.stop()
        self.main_software_thread.wait()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)

    # Initialize the main software thread
    main_software_thread = MainSoftwareThread()

    # Initialize and show the main window
    window = MainWindow(main_software_thread)
    window.show()

    # Start the main software thread
    main_software_thread.start()

    sys.exit(app.exec_())
