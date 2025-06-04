import matplotlib.pyplot as plt
import numpy as np


with open("/home/ziv/ros2_ws/src/my_py_pkg/my_py_pkg/acceleration_measure.txt", "r") as f: 
    data = np.array([line.strip().split(" ") for line in f],  dtype=float)

start = 0
end = 60  ###### time from starting offboard -> this is part of the time that was took from accelertion_frd file from printing to the file

################ step number from starting offboard ################################

until_start_offboard = 0  # brfore offboard
until_first_step = 980 #after finish 0 step
until_second_step = 1443 #after finish 1 step
until_third_step = 1975 #after finish 2 step
until_fourth_step = 2373 #after finish 3 step

#########  STEP ZERO ##########################
trajectory_step_zero = [0.0, 0.0, 14.0]
######## acceleration array ######################################
acceleration_step_0_x =  np.full(until_first_step - until_start_offboard, 0)
acceleration_step_0_y =  np.full(until_first_step - until_start_offboard, 0)
acceleration_step_0_z =  np.full(until_first_step - until_start_offboard, 14)

######## for error ###############################
x_error_step_zero = np.abs((np.array([x - trajectory_step_zero[0] for x in data[until_start_offboard:until_first_step, 0]]))*100/(1+0.0))
y_error_step_zero = np.abs(np.array([y - trajectory_step_zero[1] for y in data[until_start_offboard:until_first_step, 1]])*100/(1+0.0))
#z_error = np.abs((np.array([z - trajectory_step_zero[2] for z in data[:, 2]])/trajectory_step_zero[2])*100)
########  STEP ONE ########################
trajectory_step_one = [1.5, 0.0, 10.0]
#######  acceleration array ######################################
acceleration_step_1_x = np.full(until_second_step - until_first_step, 1.5)
acceleration_step_1_y = np.full(until_second_step - until_first_step, 0)
acceleration_step_1_z = np.full(until_second_step - until_first_step, 10)

######## for error ###############################
x_error_step_one = np.abs((np.array([x - trajectory_step_one[0] for x in data[until_first_step:until_second_step, 0]]))*100/(1+2.0))
y_error_step_one = np.abs((np.array([y - trajectory_step_one[1] for y in data[until_first_step:until_second_step, 1]]))*100/(1+0.0))
#z_error = np.abs((np.array([z - trajectory_step_zero[2] for z in data[:, 2]])/trajectory_step_zero[2])*100)

########  STEP TWO ########################
trajectory_step_two = [0.0, 0.0, 14.0]
#######  acceleration array ######################################
acceleration_step_2_x = np.full(until_third_step - until_second_step, 0)
acceleration_step_2_y = np.full(until_third_step - until_second_step, 0)
acceleration_step_2_z = np.full(until_third_step - until_second_step, 14)

######## for error ###############################
x_error_step_two = np.abs((np.array([x - trajectory_step_two[0] for x in data[until_second_step:until_third_step, 0]]))*100/(1+0.0))
y_error_step_two = np.abs((np.array([y - trajectory_step_two[1] for y in data[until_second_step:until_third_step, 1]]))*100/(1+0.0))
#z_error = np.abs((np.array([z - trajectory_step_zero[2] for z in data[:, 2]])/trajectory_step_zero[2])*100)


########  STEP THREE ########################
trajectory_step_three = [0.0, 1.5, 10.0]
#######  acceleration array ######################################
acceleration_step_3_x = np.full(until_fourth_step - until_third_step, 0)
acceleration_step_3_y = np.full(until_fourth_step - until_third_step, 1.5)
acceleration_step_3_z = np.full(until_fourth_step - until_third_step, 10)

######## for error ###############################
x_error_step_three = np.abs((np.array([x - trajectory_step_three[0] for x in data[until_third_step:until_fourth_step, 0]]))*100/(1+0.0))
y_error_step_three = np.abs((np.array([y - trajectory_step_two[1] for y in data[until_third_step:until_fourth_step, 1]]))*100/(1.0+1.0))
#z_error = np.abs((np.array([z - trajectory_step_zero[2] for z in data[:, 2]])/trajectory_step_zero[2])*100)

########  STEP FOUR ########################
trajectory_step_four = [0.0, 0.0, 14.0]
#######  acceleration array ######################################
acceleration_step_4_x = np.full(data.shape[0] - until_fourth_step, 0)
acceleration_step_4_y = np.full(data.shape[0] - until_fourth_step, 0)
acceleration_step_4_z = np.full(data.shape[0] - until_fourth_step, 14)

######## for error ###############################
x_error_step_four = np.abs((np.array([x - trajectory_step_four[0] for x in data[until_fourth_step:, 0]]))*100/(1+0.0))
y_error_step_four = np.abs((np.array([y - trajectory_step_four[1] for y in data[until_fourth_step:, 1]]))*100/(1+0.0))
#z_error = np.abs((np.array([z - trajectory_step_zero[2] for z in data[:, 2]])/trajectory_step_zero[2])*100)


acceleration_desigred_0_x = np.append(acceleration_step_0_x, acceleration_step_1_x)
acceleration_desigred_1_x = np.append(acceleration_step_2_x, acceleration_step_3_x)
acceleration_desigred_2_x = np.append(acceleration_desigred_0_x, acceleration_desigred_1_x)
acceleration_desigred_x   = np.append(acceleration_desigred_2_x, acceleration_step_4_x)

acceleration_desigred_0_y = np.append(acceleration_step_0_y, acceleration_step_1_y)
acceleration_desigred_1_y = np.append(acceleration_step_2_y, acceleration_step_3_y)
acceleration_desigred_2_y = np.append(acceleration_desigred_0_y, acceleration_desigred_1_y)
acceleration_desigred_y   = np.append(acceleration_desigred_2_y, acceleration_step_4_y)

acceleration_desigred_0_z = np.append(acceleration_step_0_z, acceleration_step_1_z)
acceleration_desigred_1_z = np.append(acceleration_step_2_z, acceleration_step_3_z)
acceleration_desigred_2_z = np.append(acceleration_desigred_0_z, acceleration_desigred_1_z)
acceleration_desigred_z   = np.append(acceleration_desigred_2_z, acceleration_step_4_z)

#######  for error ######################################
#x_error_step_one = np.abs((np.array([x - trajectory_step_one[0] for x in data[until_first_step:, 0]])/trajectory_step_one[0])*100)
# y_error = np.abs((np.array([y - trajectory_maker_[1] for y in data[:, 1]])/trajectory_maker_[1])*100)
#z_error = np.abs((np.array([z - trajectory_step_one[2] for z in data[:, 2]])/trajectory_step_one[2])*100)
acceleration_measure_x = data[until_start_offboard:, 0]
acceleration_measure_y = data[until_start_offboard:, 1]
acceleration_measure_z = data[until_start_offboard:, 2]

# acceleration_measure_x = data[:, 0]
# acceleration_measure_y = data[:, 1]

####### TIME DETERMINE #########################


time = np.linspace(start, end, data.shape[0] - until_start_offboard) #time duration between two measure is 0.021 seconds 
# time = np.linspace(start, end, data.shape[0]) #time duration between two measure is 0.021 seconds 


# ###### Create a scatter error plot ########################
x_error_total = np.append(x_error_step_zero, x_error_step_one)
x_error_total_1 = np.append(x_error_step_two, x_error_step_three)
x_error_total_2 = np.append(x_error_total, x_error_total_1)
x_error_total_3 = np.append(x_error_total_2, x_error_step_four)

y_error_total = np.append(y_error_step_zero, y_error_step_one)
y_error_total_1 = np.append(y_error_step_two, y_error_step_three)
y_error_total_2 = np.append(y_error_total, y_error_total_1)
y_error_total_3 = np.append(y_error_total_2, y_error_step_four)

# plt.plot(time, x_error_total_3, label='error values')
# plt.xlabel('time [sec]')
# plt.ylabel('error [%]')
# plt.title('Error in Acceleration Compare X Axis')
# plt.xlim(start, end)  # Set x-axis limits
# # plt.ylim(-1.5, 3.5)  # Set y-axis limits
# plt.ylim(min(x_error_total_3), max(x_error_total_3))
# plt.legend()
# plt.grid(color = 'grey', linestyle = '--', linewidth = 0.5)

# plt.plot(time, y_error_total_3, label='error values')
# plt.xlabel('time [sec]')
# plt.ylabel('error [%]')
# plt.title('Error in Acceleration Compare Y Axis')
# plt.xlim(start, end)  # Set x-axis limits
# # plt.ylim(-1.5, 3.5)  # Set y-axis limits
# plt.ylim(min(y_error_total_3), max(y_error_total_3))
# plt.legend()
# plt.grid(color = 'grey', linestyle = '--', linewidth = 0.5)


plt.plot(time, acceleration_measure_x, label = 'measure')
plt.plot(time, acceleration_desigred_x,label = 'desigred', color = 'orange')
# plt.plot(time, x_error_total_3, label='error values')
plt.xlabel('time [sec]')
plt.ylabel('acceleration [m/s^2]')
plt.title('Acceleration Compare X Axis')
plt.xlim(start, end)  # Set x-axis limits
plt.ylim(-1.5, 3.5)  # Set y-axis limits
# plt.ylim(min(acceleration_measure_x), max(acceleration_measure_x))
plt.legend()
plt.grid(color = 'grey', linestyle = '--', linewidth = 0.5)


# plt.plot(time, acceleration_measure_y, label = 'measure')
# plt.plot(time, acceleration_desigred_y,label = 'desigred', color = 'orange')
# #plt.scatter(time, x_error, label='error values')
# plt.xlabel('time [sec]')
# plt.ylabel('acceleration [m/s^2]')
# plt.title('Acceleration Compare Y Axis')
# plt.xlim(start, end)  # Set x-axis limits
# plt.ylim(-1, 2.5)  # Set y-axis limits
# # plt.ylim(min(acceleration_measure_y), max(acceleration_measure_y))  # Set y-axis limits
# plt.legend()
# plt.grid(color = 'grey', linestyle = '--', linewidth = 0.5)


# plt.plot(time, acceleration_measure_z, label = 'measure')
# plt.plot(time, acceleration_desigred_z,label = 'desigred', color = 'orange')
# #plt.scatter(time, x_error, label='error values')
# plt.xlabel('time [sec]')
# plt.ylabel('acceleration [m/s^2]')
# plt.title('Acceleration Compare Z Axis')
# plt.xlim(start, end)  # Set x-axis limits
# plt.ylim(0, 21.5)  # Set y-axis limits
# plt.legend()
# plt.grid(color = 'grey', linestyle = '--', linewidth = 0.5)



###### Create a scatter plot for X error ########################
# plt.plot(time, x_error_total)
# #plt.scatter(time, x_error, label='error values')
# plt.xlabel('time')
# plt.ylabel('error of X values [%]')
# plt.title('X error Plot')
# plt.xlim(start, end)  # Set x-axis limits
# plt.ylim(0, max(x_error_total))  # Set y-axis limits
# plt.legend()


# # Create a scatter plot for Z error
# plt.scatter(time, z_error, label='error values')
# plt.xlabel('time')
# plt.ylabel('error of Z values [%]')
# plt.title('Z error Plot')
# plt.legend()

# Show the plot
plt.show()