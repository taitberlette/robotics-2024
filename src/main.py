# import the vex stuff 🤖
from vex import *

# import math stuff for triangles 📐
from math import sqrt, sin, cos, acos, atan2, degrees, radians

# main computer 🧠
brain = Brain()

# radio controller (automatically set up by the brain) 🎮
controller = Controller()

# wheels (i think the gear ratio is wrong for all of the motors 💀)
left_drive = Motor(Ports.PORT12, GearSetting.RATIO_18_1, False)
right_drive = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)

# arm & clam (again the ratios are wrong i think 🤡) 
claw_motor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
arm_base_motor = Motor(Ports.PORT9, GearSetting.RATIO_18_1, False)
arm_joint_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)

# other motors (guess what? the ratios are def wrong 😢)
tray_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
lift_motor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)

# the speed the segments of the arm can move
ARM_SPEED = 60

# the gear ratio for the arms for trig (84 teeth big gear / 12 teeth little gear)
ARM_GEAR_RATIO = 7

# approx length of segments in inches (8 = three long, 12.5 = five long; should the math check out? probably 💀)
a = 8
b = 12.5

# move the arm to a new x & y coordinate, given the initial angles to stay in the same triangle shape when pulling the arm back
# diagram attached in repo under /.images
def move_arm(x, y, start_alpha, start_beta):
	# get the hypotenuse
  r = sqrt(pow(x, 2) + pow(y, 2))

	# solve all angles according to diagram
  theta = degrees(atan2(abs(y), abs(x)))
  gamma = degrees(acos((pow(b, 2) - pow(a, 2) - pow(r, 2)) / (-2 * a * r)))
  zeta = degrees(acos((pow(r, 2) - pow(a, 2) - pow(b, 2)) / (-2 * a * b)))

  alpha, beta = 0, 0

	# make sure to solve correctly so our arms move as expected
  if start_alpha > 0: 
    if y > 0:
      alpha = gamma + theta
    else:
      alpha = gamma - theta
  else: 
    if y > 0:
      alpha = gamma - theta
    else:
      alpha = gamma + theta

  if start_beta < 0: 
    if y < 0:
      beta = zeta - (180 - alpha)
    else:
      beta = 180 - alpha - zeta
  else: 
    if y < 0:
      beta = 180 - alpha - zeta
    else:
      beta = zeta - (180 - alpha)

	# idk what this anymore, shouldn't b be a negative sometimes too??? 😭😭😭
  if start_alpha < 0:
     alpha *= -1

  return alpha, beta

# yay we don't have to use trig we can use components 💪💪💪 so much easier 😎
def calculate_arm(alpha, beta):
  # find components of the first (smaller) segment
  alpha_abs = abs(alpha)
  alpha_x = cos(radians(alpha_abs)) * a
  alpha_y = sin(radians(alpha_abs)) * a * (1 if alpha > 0 else -1)
  
	# find components of the second (longer) segment
  beta_abs = abs(beta)
  beta_x = cos(radians(beta_abs)) * b
  beta_y = sin(radians(beta_abs)) * b * (1 if beta > 0 else -1)

	# add them together and return
  x = alpha_x + beta_x
  y = alpha_y + beta_y

  return x, y

# reset all the angles of the motor to zero (do this when the arm is straight out, parallel to the ground, otherwise funky things happen)
def recalibrate():
  arm_base_motor.reset_position()
  arm_joint_motor.reset_position()

# try to move the arm back in a straight line as far as it goes
# SUPER HACKY BE WARNED ⚠️
def pull_back():
  # get the initial angles
  start_alpha = arm_base_motor.position() / ARM_GEAR_RATIO
  start_beta = arm_joint_motor.position() / ARM_GEAR_RATIO
  
  x, y = 0, 0

	# calculate the current x & y position of the arm
  try:
    x, y = calculate_arm(start_alpha, start_beta)
  except Exception:
    print("failed to calculate position 🤨")
    return

	# save the new angles as the current ones in case we can't move back anymore
  alpha = start_alpha
  beta = start_beta

	# basically just keep moving it back little by little until an error happens 💀
  for i in range(1000):
      x -= 0.1
      try:
        alpha, beta = move_arm(x, y, start_alpha, start_beta)
        # this might be wrong i just guessed with a paper protractor i printed of of the internet 🤡
        if abs(alpha) > 70 or abs(beta) > 75:
            raise RuntimeError("that's physically impossible 😭😭 our robot will die")
      except Exception:
         break

	# find the angle of how far each segment needs to rotate 📐
  dist_base = abs(alpha + start_alpha)
  dist_joint = abs(beta + start_beta)
  max_dist = max(dist_base, dist_joint)

	# make sure it will take the same amount of time for them to move (for optimal levelness 🧮)
  speed_base = dist_base / max_dist
  speed_joint = dist_joint / max_dist 

	# make the motors spin 😎
  arm_base_motor.spin_to_position(alpha * ARM_GEAR_RATIO, DEGREES, ARM_SPEED * speed_base, RPM, False)
  arm_joint_motor.spin_to_position(beta * ARM_GEAR_RATIO, DEGREES, ARM_SPEED * speed_joint, RPM, False)  

# move an segment of the arm up or down (see event description bellow!)
def control_arm(direction, motor):
  motor.spin(FORWARD, direction * ARM_SPEED)

# main driving/controlling thread
def drive_task():
    # speed of each motor
    drive_left = 0
    drive_right = 0

    # loop forever until the end of time ⌛
    while True:
				# get the value of the two joysticks
        drive_left = controller.axis3.position()
        drive_right = controller.axis2.position()

        # in case the joystick is a little wonky and doesn't work right
        dead_band = 15
        if abs(drive_left) < dead_band:
            drive_left = 0
        if abs(drive_right) < dead_band:
            drive_right = 0

				# actually move the wheels
        # divide by two so it isn't crazy fast
        left_drive.spin(FORWARD, drive_left / 2, PERCENT)
        right_drive.spin(FORWARD, drive_right / 2, PERCENT)
            
        # a little delay just for fun 😇
        sleep(10)

# start running the controller code 🤖
drive = Thread(drive_task)

# setup the claw motor
# make it less torque to not obliterate the dowel 💣
claw_motor.set_max_torque(25, PERCENT)
claw_motor.set_stopping(HOLD)

# setup the arm motors
arm_base_motor.set_stopping(HOLD)
arm_joint_motor.set_stopping(HOLD)

# when the down button is pressed, try to pull the arm/dowel back
controller.buttonDown.pressed(pull_back)

# when the b button is pressed, recalibrate the arm (straight out so the arm is parallel to the ground, needed to make the triangle stuff work)
# DO BEFORE YOU TRY TO PULL THE ARM BACK!!
controller.buttonB.pressed(recalibrate)

# move the different segments (i was too lazy to make separate functions for each) 😴
# left buttons, first (shorter) segment
controller.buttonL1.pressed(control_arm, (1, arm_base_motor))
controller.buttonL1.released(control_arm, (0, arm_base_motor))
controller.buttonL2.pressed(control_arm, (-1, arm_base_motor))
controller.buttonL2.released(control_arm, (0, arm_base_motor))
# right buttons, second (longer) segment
controller.buttonR1.pressed(control_arm, (1, arm_joint_motor))
controller.buttonR1.released(control_arm, (0, arm_joint_motor))
controller.buttonR2.pressed(control_arm, (-1, arm_joint_motor))
controller.buttonR2.released(control_arm, (0, arm_joint_motor))
