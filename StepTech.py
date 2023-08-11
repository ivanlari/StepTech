import numpy as np
import control
from scipy.signal import lti
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import sympy as sy
from tkinter import *
import sys

# ----------------- Useful Functions -------------------- 

# View a Transfer Function
def prettysys(sys):
    s = sy.symbols('s')
    num = sy.Poly(sys.num, s)
    den = sy.Poly(sys.den, s)
    return print(sy.simplify(num/den))

# Multiplication between TFs
def mult(sys1, sys2):
    num = np.polymul(sys1.num, sys2.num)
    den = np.polymul(sys1.den, sys2.den)
    return lti(num, den)

# Closed Loop function
def feedback(sysL): # sys would be L
    num = sysL.num
    den = np.polyadd(sysL.den, sysL.num)
    return lti(num, den)

# ----------------- Core --------------------------------

print("------------------------------------------------------")
print("------------------------------------------------------")
print("---------------------- StepTech-----------------------")
print("------------------------------------------------------")
print("-------------- Developed by Ivan Lari ----------------")
print("-------------------- January 2023 --------------------")
print("------------------------------------------------------")
print("------------------------------------------------------\n")
print("      r --->O---->[ PID ]---->[ G ]---+---> y      ")
print("            |                         |            ") 
print("            |                         |            ")
print("            +-------------------------+            ")

d = 1
while d>0:
    print("\n----- Define the System Transfer Function [G] -----\n")
    G_num = list(map(float, input("Enter numerator coefficients (separated by a space): ").split()))
    G_den = list(map(float, input("Enter denumerator coefficients (separated by a space): ").split()))

    # System TF
    #G = lti(10, [1, 6, 11, 6])
    G = lti(G_num, G_den)

    print("\nThe entered system transfer function is: ")
    print(control.tf(G_num, G_den))

    ol_zeros = control.zero(control.tf(G_num, G_den))
    ol_poles = control.pole(control.tf(G_num, G_den))

    for i in range(len(ol_poles)):
        ol_poles[i] = f'{ol_poles[i]:.1f}'

    for i in range(len(ol_zeros)):
        ol_zeros[i] = f'{ol_zeros[i]:.1f}'

    print("The properties of this system are:\n")
    print("DC gain: " + f'{control.dcgain(control.tf(G_num, G_den)):.1f}')
    print("Zeros: " + str(ol_zeros))
    print("Poles: " + str(ol_poles))
    
    while True:
        c = int(input("\nWhat would you like to do? \n1 ---> Bode plot \n2 ---> Root Locus \n3 ---> Nyquist plot \n4 ---> PID tuner \n5 ---> Exit \nEnter your choice: "))
        if c == 1: # Bode
            print("\nPlotting Bode.")
            plt.figure(figsize=(10,6))
            w = np.logspace(-1.5,1,200) # intervallo delle frequenze
            mag, phase, freq = control.bode_plot(control.tf(G.num, G.den), dB=True, deg=True)
            plt.show()
            print("\nBode is closed.")
            
        elif c == 2: # Root Locus
            print("\nPlotting the Root Locus.")
            plt.figure(figsize=(10,6))
            control.root_locus(control.tf(G.num, G.den))
            plt.show()
            print("\nRoot Locus is closed.")

        elif c == 3: # Nyquist
            print("\nPlotting Nyquist.")
            plt.figure(figsize=(10,6))
            control.nyquist_plot(control.tf(G.num, G.den))
            plt.show()
            print("\nNyquist is closed.")

        elif c == 4: # PID Tuner
            print("\nPress Enter to open the PID tuner.")
            input()
            print("Tuning is in progress...")

            # PID Initial Gains
            kp = 1
            ki = 1
            kd = 1

            # Filter
            N = 1

            # Controller TF
            Kpid = lti([kp*kd/N+kp*kd, kp**2+ki*kd/N, kp*ki], [kd/N, kp, 0])

            L = mult(Kpid, G) # Open Loop function
            T = feedback(L) # Closed Loop function

            t_in = 0
            t_fin = 10
            t_span = np.linspace(t_in , t_fin, 1000)
            t, y = G.step(T=t_span) # step response in open loop
            t, y2 = T.step(T=t_span) # step response in closed loop
            cl_zeros = control.zero(control.tf(T.num, T.den)) # zeros in closed loop
            cl_poles = control.pole(control.tf(T.num, T.den)) # poles in closed loop
            cl_dcgain = control.dcgain(control.tf(T.num, T.den)) # dc gain in closed loop
            (gm, pm, wgc, wpc) = control.margin(control.tf(L.num, L.den)) # stability margins
            gm_dB = 20*np.log10(gm)

            # ----------------- Plots -------------------------

            # Plots
            fig = plt.figure("StepTech", figsize=(9, 6))
            plt.subplots_adjust(bottom=0.4)
            ax = fig.subplots()
            plt.suptitle('PID Tuner', fontsize=13)
            ax.plot([t_in, t_fin], [1, 1], color='k', ls='--', label="Reference ($r$)")
            ax.plot([t_in, t_in], [0, 1], color='k', ls='--')
            ax.plot([t_in-1, t_in], [0, 0], color='k', ls='--')
            ax.plot(t, y, color='b', label="Open Loop")
            cl_plot = ax.plot(t, y2, color='r', label="Closed Loop")[0]
            ax.grid(ls=':', lw=0.8)
            ax.legend(loc="best")
            plt.xlabel('$t$', fontsize=11)
            plt.ylabel('$y (t)$', fontsize=12, rotation=0)
            plt.title("Step Response", fontsize=11, style='italic')

            for i in range(len(cl_poles)):
                    cl_poles[i] = f'{cl_poles[i]:.1f}'
            text_poles = fig.text(0.15, 0.04, 'Closed Loop poles: ' + str(cl_poles), fontsize = 10, 
                                  color = "black")

            for i in range(len(cl_zeros)):
                cl_zeros[i] = f'{cl_zeros[i]:.1f}'
            text_zeros = fig.text(0.15, 0.08, 'Closed Loop zeros: ' + str(cl_zeros), fontsize = 10, 
                                  color = "black")

            text_gm = fig.text(0.65, 0.25, 'Gain Margin (abs): ' + f'{gm:.1f}', fontsize = 10, color = "black")

            text_pm = fig.text(0.65, 0.21, 'Phase Margin (deg): ' + f'{pm:.1f}', fontsize = 10, color = "black")

            text_wpc = fig.text(0.65, 0.17, 'Phase Crossover freq. (rad/s): ' + f'{wpc:.1f}', fontsize = 10, 
                                color = "black")

            text_wgc = fig.text(0.65, 0.13, 'Gain Crossover freq. (rad/s): ' + f'{wgc:.1f}', fontsize = 10, 
                                color = "black")

            plt.xlim((t_in-1, t_fin))
            plt.ylim((0-0.25, 1+0.5))

            fig.text(0.32, 0.30, 'Gains:', style = 'italic', fontsize = 11, color = "black")
            
            fig.text(0.7, 0.30, 'Stability Margins:', style = 'italic', fontsize = 11, color = "black")

            # Sliders
            kp_slider_pos = plt.axes([0.15, 0.27, 0.4, 0.02])
            slider_kp = Slider(kp_slider_pos, '$K_P$', 0.0000000001, 50, valinit=1, valstep=0.00001, color="red")
            slider_kp.label.set_size(14)

            ki_slider_pos = plt.axes([0.15, 0.23, 0.4, 0.02])
            slider_ki = Slider(ki_slider_pos, '$K_I$', 0.0000000001, 50, valinit=1, valstep=0.00001, color="red")
            slider_ki.label.set_size(14)

            kd_slider_pos = plt.axes([0.15, 0.19, 0.4, 0.02])
            slider_kd = Slider(kd_slider_pos, '$K_D$', 0.0000000001, 50, valinit=1, valstep=0.00001, color="red")
            slider_kd.label.set_size(14)

            N_slider_pos = plt.axes([0.15, 0.15, 0.4, 0.02])
            slider_N = Slider(N_slider_pos, '$N$', 0.0000000001, 50, valinit=1, valstep=0.00001, color="red")
            slider_N.label.set_size(14)

            # Update plot
            def update(val):
                x = [slider_kp.val, slider_ki.val, slider_kd.val, slider_N.val]
                current_kp = x[0]
                current_ki = x[1]
                current_kd = x[2]
                current_N  = x[3]
                kp = current_kp
                ki = current_ki
                kd = current_kd
                N  = current_N
                Kpid = lti([kp*kd/N+kp*kd, kp**2+ki*kd/N, kp*ki], [kd/N, kp, 0])
                L = mult(Kpid, G)
                T = feedback(L)

                (gm, pm, wgc, wpc) = control.margin(control.tf(L.num, L.den))
                gm_dB = 20*np.log10(gm)
                
                cl_dcgain = control.dcgain(control.tf(T.num, T.den))

                cl_poles = control.pole(control.tf(T.num, T.den))
                for i in range(len(cl_poles)):
                    cl_poles[i] = f'{cl_poles[i]:.1f}'
                text_poles.set_text('Closed Loop poles: ' + str(cl_poles))

                cl_zeros = control.zeros(control.tf(T.num, T.den))
                for i in range(len(cl_zeros)):
                    cl_zeros[i] = f'{cl_zeros[i]:.1f}'
                text_zeros.set_text('Closed Loop zeros: ' + str(cl_zeros))

                text_gm.set_text('Gain Margin (abs): ' + f'{gm:.1f}')
                text_pm.set_text('Phase Margin (deg): ' + f'{pm:.1f}')
                text_wpc.set_text('Phase Crossover freq. (rad/s): ' + f'{wpc:.1f}')
                text_wgc.set_text('Gain Crossover freq. (rad/s): ' + f'{wgc:.1f}')

                t, y2_new = T.step(T=t_span) # step response in closed loop

                cl_plot.set_data(t,y2_new)

                # redrawing the figure
                fig.canvas.draw()
                return [kp, ki, kd, cl_poles]

            slider_kp.on_changed(update)
            slider_ki.on_changed(update)
            slider_kd.on_changed(update)
            slider_N.on_changed(update)
            plt.show()
            print("\nTuner is closed.")

        elif c == 5: # Exit ifs
            print("\nExiting.")
            break
        else:
            print("Response not allowed. Press Enter to close the program.")
            input()
            sys.exit()

    q = input("Do you want to enter a new transfer function? (y/n): ")
    if q == "y":
        pass
    elif q == "n":
        d = 0
        print("Press Enter to close the program.")
        input()
    else:
        print("Response not allowed. Press Enter to close the program.")
        d = 0
        input()