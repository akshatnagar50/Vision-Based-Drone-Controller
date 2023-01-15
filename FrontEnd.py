import tkinter as tk

def button_click(number):
    if number==1:
        #arm()
        pass
    if number==10:
        #disarm()
        pass
    if number==2:
        #throttle(Higher_val)
        pass
    if number==1:
        #arm()
        pass
    if number==10:
        #disarm()
        pass
    if number==2:
        #throttle(Higher_val)
        pass
    if number==4:
        #arm()
        pass
    if number==7:
        #disarm()
        pass
    if number==5:
        #yaw(right_val)
        pass
    if number==6:
        #yaw(left_val)
        pass

root = tk.Tk()
root.title("Tkinter 10 Buttons")
root.geometry('300x200')

# Create individual buttons
button1 = tk.Button(root, text="Arm", command=lambda: button_click(1))
button2 = tk.Button(root, text="Throttle up", command=lambda: button_click(2))
button3 = tk.Button(root, text="Pitch forward", command=lambda: button_click(3))
button4 = tk.Button(root, text="Roll left", command=lambda: button_click(4))
button5 = tk.Button(root, text="Roll right", command=lambda: button_click(5))
button6 = tk.Button(root, text="Yaw left", command=lambda: button_click(6))
button7 = tk.Button(root, text="Yaw right", command=lambda: button_click(7))
button8 = tk.Button(root, text="Pitch back", command=lambda: button_click(8))
button9 = tk.Button(root, text="Throttle Down", command=lambda: button_click(9))
button10 = tk.Button(root, text="Disarm", command=lambda: button_click(10))

# Arrange buttons symmetrically in a 2x5 grid
button1.grid(row=0, column=0)
button2.grid(row=0, column=1)
button3.grid(row=0, column=2)
button4.grid(row=0, column=3)
button5.grid(row=0, column=4)
button6.grid(row=1, column=4)
button7.grid(row=1, column=3)
button8.grid(row=1, column=2)
button9.grid(row=1, column=1)
button10.grid(row=1, column=0)

root.mainloop()


