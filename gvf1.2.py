'''
Course: REHAB 599 F24 University of Alberta
Author: Annette Lau
Email: annette1@ualberta.ca

Example of implementing an on-policy General Value Function (GVF) learner.
Connects to a single Dynamixel XL330-M288-T servo motor to access position,
velocity, current, and pulse-width modulation (PWM) in real time.
The GVF learner predicts how much movement the motor will produce as it moves
in a back and forth fashion with a fixed gamma value using a TD(lambda) algorithm.

Policy = back and forth position of motor
Cumulant = goal position signal of motor
Gamma = 0.8
Alpha = 0.5
State = {position, velocity, current, pwm}
Lambda = 0.7
'''
import os, time, numpy as np
from dynamixel_sdk import * # Uses Dynamixel SDK library
from dynamic_plotter import * # Uses plotting in real time library

# ===========================================================================================================
# Control Table Addresses for Dynamixel XL330-M288-T
ADDR_PWM_LIMIT = 36
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_PWM = 100
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_PWM = 124
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132

COMM_SUCCESS = 0 # Communication Success result value
COMM_TX_FAIL = -1001 # Communication Tx Failed
PROTOCOL_VERSION = 2.0 # Check which protocol version is used in Dynamixel Wizard
DXL_ID = 1 # Default setting for new motor
BAUDRATE = 57600 # Most common baud rates are 57600 and 1000000 bps
DEVICENAME = 'COM8' # Check in device manager (USB port on the left side)
TORQUE_ENABLE = 1 # Value for enabling the torque
TORQUE_DISABLE = 0 # Value for disabling the torque
DXL_GOAL_PWM_VALUE = 177 # PWM of 20% for lower speed
DXL_PWM_LIMIT_VALUE = 354 # PWM limit of 40% so finger doesn't close too fast
DXL_MINIMUM_POSITION_VALUE = 600 # Min position limit for XL330 [pulse]
DXL_MAXIMUM_POSITION_VALUE = 2048 # Max position limit for XL330 [pulse]
DXL_MOVING_STATUS_THRESHOLD = 10 # Dynamixel moving status threshold [rev/min]

# ===========================================================================================================
# Set port path and protocol version
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Successfully opened the port!!")
else:
    print("Failed to open the port :(")
    input("Press any key to terminate...")
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Successfully changed the baudrate!!")
else:
    print("Failed to change the baudrate :(")
    input("Press any key to terminate...")
    quit()

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

# Set the PWM
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_PWM, DXL_GOAL_PWM_VALUE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("PWM limit set successfully")

# Create a file for logging
log_filename = "gvf_const_gamma.txt"
if os.path.exists(log_filename):
    os.remove(log_filename)
log_file = open(log_filename, "w")

# ===========================================================================================================
class GVF:
    def __init__(self, gamma, llama, alpha, state_size):
        self.gamma = gamma
        self.llama = llama
        self.alpha = alpha
        self.delta = 0 # TD error
        self.e = np.zeros(state_size) # Eligibility trace vector
        self.w = np.zeros(state_size) # Weight vector
        self.S = np.zeros(state_size) # Current state feature vector x(S)
        self.predict = 0.5
    def update(self, cumulant, S_prime, gamma):
        self.gamma = gamma
        self.predict = np.dot(self.w, self.S) # Prediction for the current state
        self.delta = cumulant + self.gamma*np.dot(self.w, S_prime) - self.predict # TD error update
        self.e = self.e*self.gamma*self.llama + self.S # Eligibility traces update
        self.w = self.w + self.alpha*self.delta*self.e # Weight vector update
        self.S = S_prime # Store current state for the next update

# Prepare the GVF variables
gamma = 0.8
llama = 0.7 # Trace decay parameter to compute eligibility traces to assign credit to previous states
alpha = 0.5 # Scales updates to the weights
state_size = 20**4 # S = {Position, Velocity, Current, PWM}
agent = GVF(gamma = gamma, llama = llama, alpha = alpha, state_size = state_size) # The GVF agent!

# Returning a raw state vector by getting info from the motors
def get_motor_info():
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    dxl_present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_VELOCITY)
    dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_CURRENT)
    dxl_present_pwm, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_PWM)
    return np.array([dxl_present_position, dxl_present_velocity, dxl_present_current, dxl_present_pwm])

# Normalizing state variables
def normalize(raw_state):
    norm_position = (raw_state[0] - DXL_MINIMUM_POSITION_VALUE)/(DXL_MAXIMUM_POSITION_VALUE - DXL_MINIMUM_POSITION_VALUE)
    norm_velocity = (raw_state[1])/(4294967295) # Maximum 4 byte unsigned int
    norm_current = (raw_state[2])/(65535) # Maximum 2 byte unsigned int
    norm_pwm = (raw_state[3])/(4294967295) # Maximum 4 byte unsigned int
    return np.array([norm_position, norm_velocity, norm_current, norm_pwm])

# 4D binning
def fourd_binning(values, num_bins = 20): # num_bins = 20 is the default value
    fourd_array = np.zeros((num_bins, num_bins, num_bins, num_bins))
    bin_indices = [min(int(value * num_bins), num_bins - 1) for value in values]
    fourd_array[bin_indices[0], bin_indices[1], bin_indices[2], bin_indices[3]] = 1
    feature_vector = fourd_array.flatten()
    return feature_vector

def compute_true_return(gamma_list, cumulant_list):
    true_return = 0
    for k, cumulant in enumerate(cumulant_list): # For each k, multiply cumulant by gamma^k
        gamma_product = 1
        for i in range(k): # Calculate γᵏ by multiplying k gammas
            gamma_product *= gamma_list[i]
        true_return += gamma_product * cumulant
    return true_return

# ===========================================================================================================
time_step = 0
window_size = 10 # Smaller because gamma is fixed
gamma_history = [] # Empty array to store gamma values for calculating true return
cumulant_history = [] # Empty array to store cumulant values for calculating true return

# Prepare plot for predicted, present, and goal position
p = DynamicPlot(window_x = 50, title = 'Learned Prediction vs Signal', xlabel = 'Time Steps', ylabel = 'Position')
p.add_line(label="Predictions")
p.add_line(label="Cumulant")
p.add_line(label="True Return")

# Prepare variables for goal position mapping
index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]
goal_position = dxl_goal_position[index]

while time_step <= 1000:
    # Sends a goal position to motor and checks for errors
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, goal_position)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Continuously reads present position and checks for errors
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    
    # Switch the goal position from minimum to maximum continuously
    if abs(dxl_present_position - goal_position) < 100:
        index += 1
        index %= 2
        goal_position = dxl_goal_position[index]

    # ===========================================================================================================
    # Observes current state from motor
    raw_state = get_motor_info() # Raw motor info in 2 and 4 byte ints
    norm_state = normalize(raw_state) # Normalizing them between 0 and 1 in an array with 4 elements
    motor_state = fourd_binning(norm_state, num_bins = 20) # Flatten into a 1D array with 20**4 elements

    # Calculate and store the cumulant
    norm_goal_position = (goal_position - DXL_MINIMUM_POSITION_VALUE)/(DXL_MAXIMUM_POSITION_VALUE - DXL_MINIMUM_POSITION_VALUE)

    # Gamma and cumulant values
    gamma = 0.8
    gamma_history.append(gamma)
    cumulant = norm_goal_position
    cumulant_history.append(cumulant)

    # Calculate true return
    if len(cumulant_history) > window_size:
        true_return = compute_true_return(gamma_history[-window_size:], cumulant_history[-window_size:])
    else:
        true_return = 0
    norm_true_return = true_return*(1-gamma)

    # Updates the GVF with cumulant and state, gets normalized prediction
    agent.update(cumulant = cumulant, S_prime = motor_state, gamma = gamma)
    norm_pred_pos = agent.predict*(1-gamma)
    
    # ===========================================================================================================
    # Logs the prediction, cumulant, and true return
    print(f"Time Step: {time_step}, Prediction:{norm_pred_pos:.2f}, Cumulant:{cumulant:.2f}, True Return:{norm_true_return:.2f}")
    log_file.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] Prediction:{norm_pred_pos:.2f}, Cumulant:{cumulant:.2f}, True Return:{norm_true_return:.2f}\n")

    # Update the plot and saves image at time steps 100, 500, and 1000
    time.sleep(0.1) # Accounts for delay in plotting update
    p.update(time_step, [norm_pred_pos, cumulant, norm_true_return])
    if time_step == 100 or time_step == 500 or time_step == 1000:
        p.save(f"const_gamma_step_{time_step}.png")

    time_step += 1