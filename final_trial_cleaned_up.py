import PySimpleGUI as sg
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import serial

sg.theme('Black')

ser = serial.Serial('COM3', 9600) #Define Arduino port and baud rate


# Define the UI layout for initial window

layout = [ [sg.Text("Current/ mA"), sg.InputText()],
           [sg.Button("Start"), sg.Button("Cancel")],  ] #create box to take an input from user and required buttons
           
# Create the initial UI window for the above layout 
           
window = sg.Window('Electrochemical Etching', layout, margins=(100,100)) #Define window title and size

#Run the event loop for the initial window
while True:
    event, values = window.read()
    if event == "Cancel" or event == sg.WIN_CLOSED: #if cancel button pressed or the window is exited the code will terminate
        break
    if event == "Start":
        current_setpoint = float(values[0]) #store input from user as a variable
        window.close()

        # Define the UI layout for new plotting window
        layout = [[sg.Canvas(size=(400, 400), key="-CANVAS-")], 
                  [sg.Button("Exit")], ]

        # Create plotting window from above layout
        window = sg.Window("Electrochemical Etching", layout, finalize=True)


        # Define the Matplotlib figure and axis
        
        fig, ax = plt.subplots()
        plt.close()
        xs = [] #create variables required for graph plotting
        x = []
        ys = []
        y = []
        ax.set_ylim(0, 5) # Set the y-axis limits(0-5 V as determined by the Arduino - currently plotting voltage not current)
        ax.set_xlabel('Time/ s')
        ax.set_ylabel('Voltage/ V')
        line, = ax.plot(xs, ys, 'b-', label="Raw Data", color="red") # plot live data 
        set_point, = ax.plot(x, y, label="Set Point", color="black") #plot set point inputted by user
        ax.legend()


        # Create the Matplotlib canvas
        canvas = FigureCanvasTkAgg(fig, master=window["-CANVAS-"].TKCanvas)
        canvas.draw()
        canvas.get_tk_widget().pack(side="top", fill="both", expand=True)

        # Run event loop for plotting window 
        while True:
            event, values = window.read(timeout=0)
            if event == sg.WIN_CLOSED or event == "Exit": #if cancel button pressed or the window is exited the code will terminate
                break
            try:
                voltage = float(ser.readline().decode().strip()) # Read the value from the analogue pin - as defined in the Arduino code
                xs.append((len(xs) + 1)/ 10) # Append the x value to the xs list
                x.append((len(x) + 1)/ 10) # /10 to convert number of data points to seconds as a data point is read approx every 0.1s
                ys.append(voltage) # Append the y value to the ys list
                y.append(current_setpoint)
                if len(xs) > 18000: # If the plot has more than 18000 data points/samples (30 minutes) abort etching
                    break

                line.set_data(xs, ys) # Update the plot
                set_point.set_data(x, current_setpoint)
                ax.relim()
                ax.autoscale_view() #x axis will autoscale as new data is collected
                canvas.draw()
                plt.pause(0.001)
            except KeyboardInterrupt:
                break
ser.close()
window.close()

#mili function to count seconds?
