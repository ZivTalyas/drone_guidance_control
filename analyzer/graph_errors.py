import matplotlib.pyplot as plt
import numpy as np
import argparse # For command-line arguments

DEFAULT_FILE_PATH = "acceleration_measure.txt"

def load_acceleration_data(filepath):
    """
    Loads acceleration data from the specified text file.

    Args:
        filepath (str): The path to the data file.

    Returns:
        numpy.ndarray: The loaded data as a NumPy array, or None if an error occurs.
    """
    try:
        with open(filepath, "r") as f:
            data = np.array([line.strip().split(" ") for line in f], dtype=float)
        print(f"Successfully loaded data from: {filepath}")
        return data
    except FileNotFoundError:
        print(f"Error: File not found at '{filepath}'.")
        return None
    except Exception as e:
        print(f"Error loading data from '{filepath}': {e}")
        return None

def main():
    """
    Main function to parse arguments, load data, process, and plot.
    """
    parser = argparse.ArgumentParser(description="Analyze and plot drone acceleration data.")
    parser.add_argument(
        "--filepath",
        type=str,
        default=DEFAULT_FILE_PATH,
        help=f"Path to the acceleration data file (default: {DEFAULT_FILE_PATH})"
    )
    parser.add_argument(
        "--axes",
        nargs='+',
        default=['x', 'y', 'z'],
        choices=['x', 'y', 'z'],
        help="Axes to plot (e.g., x y z)."
    )
    parser.add_argument(
        "--plot-types",
        nargs='+',
        default=['accel', 'error'],
        choices=['accel', 'error'],
        help="Types of plots to generate (e.g., accel error)."
    )
    parser.add_argument(
        "--save-plots",
        action='store_true',
        help="Save plots to files instead of showing them interactively."
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=".",
        help="Directory to save plots (default: current directory)."
    )
    # Add arguments for start and end time for plotting
    parser.add_argument(
        "--start-time-plot",
        type=float,
        default=0.0,
        help="Start time for the plotting range (seconds)."
    )
    parser.add_argument(
        "--end-time-plot",
        type=float,
        default=60.0,
        help="End time for the plotting range (seconds)."
    )
    args = parser.parse_args() # Parse all defined arguments once

    data = load_acceleration_data(args.filepath)
    if data is None:
        print("Exiting due to data loading failure.")
        return

    # Process data using step definitions
    processed_data = process_acceleration_data(data, args.start_time_plot, args.end_time_plot)


    # Extract plotting choices from args
    axes_to_plot = args.axes
    plot_types_to_generate = args.plot_types
    should_save_plots = args.save_plots
    plot_output_dir = args.output_dir
    # Determine if plots should be shown: show if not saving, unless overridden by a future --show-plots arg
    should_show_plots = not should_save_plots


    for axis_char in axes_to_plot:
        axis_name = axis_char.upper() # X, Y, Z

        measured = processed_data[f'measured_{axis_char}']
        desired = processed_data[f'desired_{axis_char}']
        error = processed_data[f'error_{axis_char}']
        time_vector = processed_data['time_vector']

        # Default y-limits (can be overridden by more specific args later)
        # Based on original script's fixed limits
        ylim_accel = (-1.5, 3.5) if axis_char == 'x' else (-1.5, 3.5) if axis_char == 'y' else (0, 21.5)
        ylim_error_max = max(error) * 1.1 if len(error) > 0 and max(error) > 0 else 100
        ylim_error = (0, ylim_error_max)

        if 'accel' in plot_types:
            plot_axis_data(
                time_vector=time_vector,
                measured_data=measured,
                desired_data=desired,
                error_data=None, # Not plotting error on the accel graph
                axis_label=axis_name,
                plot_type='Acceleration',
                y_limits=ylim_accel,
                start_time_plot=start_time_plot,
                end_time_plot=end_time_plot,
                save_plot=save_plots,
                output_dir=output_dir,
                show_plot=not save_plots # Show if not saving, or configure differently
            )

        if 'error' in plot_types:
             plot_axis_data(
                time_vector=time_vector,
                measured_data=None, # Not plotting measured on the error graph
                desired_data=None,  # Not plotting desired on the error graph
                error_data=error,
                axis_label=axis_name,
                plot_type='Error',
                y_limits=ylim_error,
                start_time_plot=start_time_plot,
                end_time_plot=end_time_plot,
                save_plot=save_plots,
                output_dir=output_dir,
                show_plot=not save_plots
            )

    if not save_plots: # If plots were saved, plt.show() might have been called in plot_axis_data.
                       # If only showing, ensure a final plt.show() if plots are managed globally.
                       # However, plot_axis_data calls plt.show() if not saving.
        pass


def process_acceleration_data(data, start_time_plot=0, end_time_plot=60):
    """
    Processes the raw acceleration data based on predefined step definitions
    to calculate desired accelerations and percentage errors.

    Args:
        data (numpy.ndarray): The raw data array (Nx3 for x,y,z).
        start_time_plot (float): The start time for the time vector for plotting.
        end_time_plot (float): The end time for the time vector for plotting.

    Returns:
        dict: A dictionary containing processed data arrays:
              'time_vector', 'measured_x/y/z', 'desired_x/y/z', 'error_x/y/z'.
    """
    step_definitions = [
        {"name": "Step Zero (Ascent)", "end_sample_point": 980, "trajectory": [0.0, 0.0, 14.0]},
        {"name": "Step One (Forward Accel)", "end_sample_point": 1443, "trajectory": [1.5, 0.0, 10.0]},
        {"name": "Step Two (Hold Z)", "end_sample_point": 1975, "trajectory": [0.0, 0.0, 14.0]},
        {"name": "Step Three (Rightward Accel)", "end_sample_point": 2373, "trajectory": [0.0, 1.5, 10.0]},
        {"name": "Step Four (Hold Z)", "end_sample_point": data.shape[0], "trajectory": [0.0, 0.0, 14.0]}
    ]

    # Store lists of arrays for each segment, then concatenate at the end
    desired_accel_segments = {'x': [], 'y': [], 'z': []}
    error_segments = {'x': [], 'y': [], 'z': []} # Z error can be added if needed

    current_sample_point = 0
    total_processed_samples = 0

    for step in step_definitions:
        start_point = current_sample_point
        end_point = step["end_sample_point"]

        # Ensure end_point does not exceed actual data length
        if end_point > data.shape[0]:
            print(f"Warning: Step '{step['name']}' end_sample_point ({end_point}) "
                  f"exceeds data length ({data.shape[0]}). Clamping to data length.")
            end_point = data.shape[0]

        num_samples_in_step = end_point - start_point

        if num_samples_in_step <= 0:
            if current_sample_point >= data.shape[0]: # All data processed
                break
            print(f"Warning: Step '{step['name']}' has no samples based on current data length. Skipping.")
            current_sample_point = end_point # Ensure progress if end_point was clamped
            continue

        total_processed_samples += num_samples_in_step

        traj_x, traj_y, traj_z = step["trajectory"]

        desired_accel_segments['x'].append(np.full(num_samples_in_step, traj_x))
        desired_accel_segments['y'].append(np.full(num_samples_in_step, traj_y))
        desired_accel_segments['z'].append(np.full(num_samples_in_step, traj_z))

        measured_x_step = data[start_point:end_point, 0]
        measured_y_step = data[start_point:end_point, 1]
        # measured_z_step = data[start_point:end_point, 2] # Uncomment if Z error needed

        # Denominator logic based on original script's specific values per step
        denom_x, denom_y = 1.0, 1.0 # Default
        if step["name"] == "Step Zero (Ascent)": denom_x, denom_y = 1.0, 1.0
        elif step["name"] == "Step One (Forward Accel)": denom_x, denom_y = 3.0, 1.0 # Orig: (1+2.0), (1+0.0)
        elif step["name"] == "Step Two (Hold Z)": denom_x, denom_y = 1.0, 1.0
        elif step["name"] == "Step Three (Rightward Accel)": denom_x, denom_y = 1.0, 2.0 # Orig: (1+0.0), (1.0+1.0) for Y
        elif step["name"] == "Step Four (Hold Z)": denom_x, denom_y = 1.0, 1.0

        error_segments['x'].append(np.abs(measured_x_step - traj_x) * 100 / denom_x)
        error_segments['y'].append(np.abs(measured_y_step - traj_y) * 100 / denom_y)
        # error_segments['z'].append(np.abs(measured_z_step - traj_z) * 100 / max(1.0, abs(traj_z)))) # Example Z error

        current_sample_point = end_point

    processed_data = {
        'desired_x': np.concatenate(desired_accel_segments['x']),
        'desired_y': np.concatenate(desired_accel_segments['y']),
        'desired_z': np.concatenate(desired_accel_segments['z']),
        'error_x': np.concatenate(error_segments['x']),
        'error_y': np.concatenate(error_segments['y']),
        # 'error_z': np.concatenate(error_segments['z']),
        'measured_x': data[:total_processed_samples, 0],
        'measured_y': data[:total_processed_samples, 1],
        'measured_z': data[:total_processed_samples, 2],
        'time_vector': np.linspace(start_time_plot, end_time_plot, total_processed_samples)
    }
    return processed_data

def plot_axis_data(time_vector, measured_data, desired_data, error_data,
                   axis_label, plot_type, y_limits,
                   start_time_plot, end_time_plot,
                   save_plot=False, output_dir=".", show_plot=True):
    """
    Generates and shows/saves a plot for a specific axis and type (acceleration or error).

    Args:
        time_vector (np.ndarray): Time data for x-axis.
        measured_data (np.ndarray, optional): Measured acceleration data.
        desired_data (np.ndarray, optional): Desired acceleration data.
        error_data (np.ndarray, optional): Calculated error data.
        axis_label (str): Label for the axis (e.g., 'X', 'Y', 'Z').
        plot_type (str): Type of plot ('Acceleration' or 'Error').
        y_limits (tuple): Y-axis limits (min, max).
        start_time_plot (float): Start time for x-axis limit.
        end_time_plot (float): End time for x-axis limit.
        save_plot (bool): Whether to save the plot to a file.
        output_dir (str): Directory to save plots.
        show_plot (bool): Whether to display the plot.
    """
    plt.figure(figsize=(12, 6))

    if plot_type == 'Acceleration':
        if measured_data is not None:
            plt.plot(time_vector, measured_data, label=f'Measured {axis_label}')
        if desired_data is not None:
            plt.plot(time_vector, desired_data, label=f'Desired {axis_label}', color='orange', linestyle='--')
        plt.ylabel('Acceleration [m/s^2]')
        plt.title(f'Acceleration Comparison - {axis_label} Axis')
    elif plot_type == 'Error':
        if error_data is not None:
            plt.plot(time_vector, error_data, label=f'{axis_label} Axis Error (%)', color='red' if axis_label=='X' else 'green' if axis_label=='Y' else 'blue')
        plt.ylabel('Error [%]')
        plt.title(f'Acceleration Error - {axis_label} Axis')
    else:
        print(f"Unknown plot type: {plot_type}")
        return

    plt.xlabel('Time [sec]')
    plt.xlim(start_time_plot, end_time_plot)
    if y_limits:
        plt.ylim(y_limits)
    plt.legend()
    plt.grid(True, linestyle='--', linewidth=0.5)

    if save_plot:
        import os
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        filename = os.path.join(output_dir, f"{plot_type.lower()}_compare_{axis_label.lower()}_axis.png")
        plt.savefig(filename)
        print(f"Plot saved to {filename}")

    if show_plot:
        plt.show()
    else:
        plt.close() # Close the figure if not showing to free memory


if __name__ == "__main__":
    main()

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