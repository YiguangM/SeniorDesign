import os
import time
import tkinter as tk
import tkinter.ttk as ttk
import subprocess


# Define the function to execute the command
# def run_command():
#     command = "echo 'Hello, world!'"  # Replace this with your desired command
#     subprocess.call(command, shell=True)

def Show_Menu():
    start_button.pack_forget()
    scenario_button.pack_forget()
    
    parking_btn.pack()
    driving_btn.pack()

def startserver():
    cmd1 = 'cd C:\\Users\\b00083281\\Desktop\\PythonAPI && .\\CarlaUE4.exe -carla-server '
    # Run the commands in separate Command Prompt windows
    subprocess.Popen(['start', 'cmd', '/k', cmd1], shell=True)
    
def Park():
    
    cmd2 = 'cd C:\\Users\\b00083281\\Desktop\\PythonAPI\\examplesOURWORK && python Easy_New_Steering_Wheel_Parking.py -n 12'
    subprocess.Popen(['start', 'cmd', '/k', cmd2], shell=True)

def Driving_Menu():
    parking_btn.pack_forget()
    driving_btn.pack_forget()
    drive_easy.pack()
    drive_hard.pack()


def Drive_Easy():
    # cmd1 = 'cd C:\\Users\\b00083281\\Desktop\\PythonAPI && .\\CarlaUE4.exe -carla-server'
    cmd2 = 'cd C:\\Users\\b00083281\\Desktop\\PythonAPI\\scenario_runner-0.9.13 && python TRIAL_mc_sw.py'
    cmd3 = 'cd C:\\Users\\b00083281\\Desktop\\PythonAPI\\scenario_runner-0.9.13 && python scenario_runner.py --route srunner/data/parking_routes_debug.xml srunner/data/all_towns_traffic_scenarios1_3_4.json 22 --agent srunner/autoagents/human_agent.py --output --debug'
    # Run the commands in separate Command Prompt windows
    #subprocess.Popen(['start', 'cmd', '/k', cmd1], shell=True)
    #time.sleep(5)
    subprocess.Popen(['start', 'cmd', '/k', cmd2], shell=True)
    #time.sleep(5)
    subprocess.Popen(['start', 'cmd', '/k', cmd3], shell=True)


def Drive_Hard():
    #cmd1 = 'cd C:\\Users\\b00083281\\Desktop\\PythonAPI && .\\CarlaUE4.exe -carla-server'
    cmd2 = 'cd C:\\Users\\b00083281\\Desktop\\PythonAPI\\scenario_runner-0.9.13 && python TRIAL_mc_sw.py'
    cmd3 = 'cd C:\\Users\\b00083281\\Desktop\\PythonAPI\\scenario_runner-0.9.13 && python scenario_runner.py --route srunner/data/parking_routes_debug.xml srunner/data/all_towns_traffic_scenarios1_3_4.json 23 --agent srunner/autoagents/human_agent.py --output --debug'
    # Run the commands in separate Command Prompt windows
    #subprocess.Popen(['start', 'cmd', '/k', cmd1], shell=True)
    time.sleep(5)
    subprocess.Popen(['start', 'cmd', '/k', cmd2], shell=True)
    time.sleep(5)
    subprocess.Popen(['start', 'cmd', '/k', cmd3], shell=True)

def go_back():
    if(parking_btn.winfo_viewable()):
        parking_btn.pack_forget()
        driving_btn.pack_forget()
        start_button.pack()
        scenario_button.pack()
    elif(drive_easy.winfo_viewable()):
        parking_btn.pack()
        driving_btn.pack()
        drive_easy.pack_forget()
        drive_hard.pack_forget()
    else:
        pass

def end():
    pid = os.getpid()
    os.system(f"taskkill /PID {pid} /F")
    os.system('taskkill /f /im notepad.exe') ## Replace Notepad.exe with the name of the server and the pygame windows


# Create the GUI window
root = tk.Tk()
root.title("Start Menu")

screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()

window_width = int(screen_width * 0.5)  # Set the width to 50% of the screen width
window_height = int(screen_height * 0.5)  # Set the height to 50% of the screen height

# Set the window size
root.geometry("{}x{}".format(window_width, window_height))

# # Load the bg image file
# image = tk.PhotoImage(file=r"C:\\Users\\b00083281\\Desktop\\CARLA.png")
# # Set the window background image
# root.configure(background=image)

# Create a label with the header text
header = tk.Label(root, text="Welcome to CARLA Driving Simulator \n", font=("Arial", 36))

# Add the label to the window
header.pack()

# Create a Style object
style = ttk.Style()
# Set the styling options for buttons
style.configure(
    "TButton",
    foreground="black",
    font=("Arial", 16),
    padding=10,
    relief="sunken",
)

# Create the start buttons
scenario_button = ttk.Button(root, text="Scenarios", command=Show_Menu)
start_button = ttk.Button(root, text="Start Server", command=startserver)
start_button.pack()
scenario_button.pack()

#Create the Back Button
back_button = ttk.Button(root, text="Back", command=go_back)
back_button.pack(side=tk.LEFT)

#Create the End Button
end_button = ttk.Button(root, text="Exit", command=end)
end_button.pack(side=tk.RIGHT)


# Create the Scenario buttons
parking_btn = ttk.Button(root, text="Parking", command=Park)
driving_btn = ttk.Button(root, text="Driving", command=Driving_Menu)

# Creating the driving buttons
drive_easy = ttk.Button(root, text="Easy", command=Drive_Easy)
drive_hard = ttk.Button(root, text="Hard", command=Drive_Hard)


# Run the GUI loop
root.mainloop()
