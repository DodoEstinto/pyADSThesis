import sys
import subprocess
from prometheus_req_py.ADS.utils import inputType
import prometheus_req_py.ADS.constants as constants
import re
from rosidl_runtime_py import message_to_ordereddict

def print_menu():
    '''
    Print the CLI menu options.
    '''

    print("=== ROS2 CLI Menu ===","\n",
          "1. Send Command","\n",
          "2. Show last request","\n",
          "3. Get Status","\n",
          "0. Exit","\n")


def select_input_type():
    '''
    Prompt the user to select an input type.
    :return: Tuple of (input type name, input type value) or (None, None) if invalid.
    '''

    print("Select input type:")
    members = list(inputType)
    stringToPrint=""
    for idx, member in enumerate(members, start=1):
        stringToPrint+=f"{idx}. {member.name}\n"

    print(stringToPrint)

    choice = input("Enter your choice: ")
    try:
        idx = int(choice) - 1
        selected = members[idx]
        return selected.name, selected.value
    except (ValueError, IndexError):
        print("Invalid choice.")
        return None, None

def select_callblock():
    '''
    Prompt the user to select a callblock from predefined building blocks.
    :return: Selected callblock string or None if invalid.
    '''
    print("Select callblock:")
    stringToPrint=""
    for idx, block in enumerate(constants.BUILDING_BLOCKS, start=1):
        stringToPrint+=f"{idx}. {block}\n"
    print(stringToPrint)
    choice = input("Enter your choice: ")
    try:
        if(choice=="0"):
            print("Exiting callblock selection.")
            return None
        idx = int(choice) - 1
        selected = constants.BUILDING_BLOCKS[idx]
        return selected
    except (ValueError, IndexError):
        print("Invalid choice.")
        return None

def send_command():
    '''
    Send a command to the ROS2 topic based on user input.
    '''

    key, value = select_input_type()
    print(f"Selected input type: {key} ({value})")
    if key is None:
        return
    if value == inputType.CALLBLOCK:
        message = select_callblock()
    elif value not in [inputType.OK, inputType.OK]:
        message = input("Enter message string: ")
    else:
        message = "None"
    if message is None:
        return
    
    ros2_cmd = [
        "ros2", "topic", "pub", "--once",
        "--qos-reliability", "reliable",
        "--qos-durability","transient_local",
        "--qos-history","keep_last",
        "--qos-depth","5",
        "receiveinput",
        "prometheus_req_interfaces/msg/InputOutput",
        f'{{type: {value}, message: "{message}"}}'
    ]
    print("Executing:", " ".join(ros2_cmd))
    try:
        subprocess.run(ros2_cmd, check=True)
    except subprocess.CalledProcessError as e:
        print("Error executing ROS2 command:", e)

def receive_last_feedback():
    '''
    Receive the last feedback message from the ROS2 topic "askinput".
    '''

    ros2_cmd = [
        "timeout","5s","ros2", "topic", "echo", "--once",
        "--qos-reliability", "reliable",
        "--qos-durability","transient_local",
        "--qos-history","keep_last",
        "--qos-depth","1",
        "askinput",
        "prometheus_req_interfaces/msg/InputOutput"
    ]
    print("Executing:", " ".join(ros2_cmd))
    try:
        subprocess.run(ros2_cmd, check=True)
    except subprocess.CalledProcessError as e:
        if e.returncode == 124:
            print("No message received within timeout.")
        else:
            print("Error executing ROS2 command:", e)

def get_status():

    ros2_cmd = [
        "timeout","5s","ros2", "service", "call",
        "request_state",
        "prometheus_req_interfaces/srv/RequestState"
    ]
    #print("Executing:", " ".join(ros2_cmd))
    try:
        result= subprocess.run(ros2_cmd, check=True,capture_output=True,
            text=True)
        
        output = result.stdout.strip()
        #remove all useless lines
        formatted_msg = re.sub(r',\s*', ',\n    ', output)
        #adds \n after params opening parenthesis
        formatted_msg = re.sub(r'\(', '(\n    ', formatted_msg)
        #adds \n before closing parenthesis
        formatted_msg = re.sub(r'\)', '\n)', formatted_msg)
        # Remove extra blank lines
        formatted_msg = re.sub(r'    (?!max|next)','', formatted_msg)
        #Remove useless lines
        formatted_msg = re.sub(r'^requester.*(\n).*(\n).*(\n)*.(\n).*(\n).*(\n).*(\n).*(\n)','\n', formatted_msg)
        formatted_msg = re.sub(r'prometheus_req_interfaces.msg.ScrewSlot','ScrewSlot', formatted_msg)
        formatted_msg = re.sub(r'screw_bay=\[','', formatted_msg)
        print(formatted_msg)
        
    except subprocess.CalledProcessError as e:
        if e.returncode == 124:
            print("No message received within timeout.")
        else:
            print("Error executing ROS2 command:", e)

def main():
    '''
    Main function to run the CLI menu loop.
    '''
    while True:
        print_menu()
        choice = input("Select option: ")
        if choice == "1":
            send_command()
        elif choice == "2":
            receive_last_feedback()
        elif choice == "3":
            get_status()
        elif choice == "0":
            print("Exiting.")
            sys.exit(0)
        else:
            print("Option not implemented or invalid.")

if __name__ == "__main__":
    main()