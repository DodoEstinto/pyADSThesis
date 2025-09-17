import sys
import subprocess
from prometheus_req_py.ADS.utils import inputType

def print_menu():
    print("=== ROS2 CLI Menu ===")
    print("1. Send Command")
    print("2. Receive Last Feedback (not implemented)")
    print("3. Get Status (not implemented)")
    print("0. Exit")

def select_input_type():
    print("Select input type:")
    members = list(inputType)
    for idx, member in enumerate(members, start=1):
        print(f"{idx}. {member.name}")

    choice = input("Enter choice number: ")
    try:
        idx = int(choice) - 1
        selected = members[idx]
        return selected.name, selected.value
    except (ValueError, IndexError):
        print("Invalid choice.")
        return None, None
    
def send_command():
    key, value = select_input_type()
    if key is None:
        return
    message = input("Enter message string: ")
    ros2_cmd = [
        "ros2", "topic", "pub", "--once",
        "receiveinput",
        "prometheus_req_interfaces/msg/InputOutput",
        f'{{type: {value}, message: "{message}"}}'
    ]
    print("Executing:", " ".join(ros2_cmd))
    try:
        subprocess.run(ros2_cmd, check=True)
    except subprocess.CalledProcessError as e:
        print("Error executing ROS2 command:", e)

def main():
    while True:
        print_menu()
        choice = input("Select option: ")
        if choice == "1":
            send_command()
        elif choice == "0":
            print("Exiting.")
            sys.exit(0)
        else:
            print("Option not implemented or invalid.")

if __name__ == "__main__":
    main()