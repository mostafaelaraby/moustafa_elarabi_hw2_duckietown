import matplotlib.pyplot as plt
import re

readings_pattern_regex = r'.+\[info\].+pure_pursuit_lf_node.+([0-9]+\.[0-9]+)\t([0-9]+\.[0-9]+)\t([0-9]+\.[0-9]+)\t([0-9]+\.[0-9]+).+'
speed = []
omega = []
crosstrack_error = []
angle_error = []
with open('logs.txt','r') as logs:
    log_lines = logs.readlines()
    for line in log_lines:
        match = re.search(readings_pattern_regex, line.strip(), re.I)
        if match:
            speed.append(float(match.group(1)))
            omega.append(float(match.group(2)))
            crosstrack_error.append(float(match.group(3)))
            angle_error.append(float(match.group(4)))

input_val = [i*0.1 for i in range(len(angle_error))]
plt.plot(input_val, angle_error, 'b-')
plt.legend(['Angle Error'])
plt.xlabel('time')
plt.ylabel('Angle Error')
plt.axis('tight')
plt.savefig('angle_error.png')
plt.close()
 
plt.plot(input_val, crosstrack_error, 'r-')
plt.legend(['Cross Track Error'])
plt.xlabel('time')
plt.ylabel('Cross Track Error')
plt.axis('tight')
plt.savefig('crosstrack_error.png')
plt.close()

plt.plot(input_val, omega, 'g-')
plt.plot(input_val, speed, 'b-')
plt.legend(['Omega','Linear Speed'])
plt.xlabel('time')
plt.ylabel('Cross Track Error')
plt.axis('tight')
plt.savefig('omega_speed.png')
plt.close()