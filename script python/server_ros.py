#! usr/bin/env python
from geometry_msgs.msg import Twist
import rospy
import time
import asyncio
import websockets
import os
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import threading

skipRemainingCommands = False
goFlag = False
finishedCommands = []
exit_event = threading.Event()

def checkObstacles(msg):
	frontScanners = msg.ranges[350:359]
	for scannerValue in frontScanners:
		if scannerValue < 0.5:
			# print('Entro qui per fermarmi')
			global skipRemainingCommands
			global goFlag
			if not skipRemainingCommands:
				skipRemainingCommands = True
				goFlag = False
			velocity_publisher.publish(Twist())
			return


PI = 3.1415926535897
velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('robot_master')
sub = rospy.Subscriber('/scan', LaserScan, checkObstacles)

def rotate(angle, clockwise, speed=45):
	global goFlag
	vel_msg = Twist()
	goCopy = goFlag
	if goFlag:
		goFlag = False
		velocity_publisher.publish(Twist())
	time.sleep(1)
	angular_speed = speed*2*PI/360
	relative_angle = angle*2*PI/360
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	if clockwise:
		vel_msg.angular.z = -abs(angular_speed)
	else:
		vel_msg.angular.z = abs(angular_speed)
	t0 = rospy.Time.now().to_sec()
	current_angle = 0
	while(current_angle < relative_angle):
		velocity_publisher.publish(vel_msg)
		t1 = rospy.Time.now().to_sec()
		current_angle = angular_speed*(t1-t0)
	vel_msg.angular.z = 0
	velocity_publisher.publish(vel_msg)
	if goCopy:
		goStraight()
	


def goStraight():
	global goFlag
	goFlag = True
	goStraightMsg = Twist()
	rate = rospy.Rate(10)
	secondsPassed = 0
	x = threading.Thread(target=goUntilStop)
	x.start()

def goUntilStop():
	global goFlag
	global skipRemainingCommands
	global finishedCommands
	goStraightMsg = Twist()
	while goFlag and not skipRemainingCommands:
		goStraightMsg.linear.x = 0.22
		velocity_publisher.publish(goStraightMsg)
	if skipRemainingCommands:
		rotate(180, True)

async def listenToCommands(websocket, path):
	commands = await websocket.recv()
	global finishedCommands
	global skipRemainingCommands
	skipRemainingCommands = False
	print(commands)
	for command in commands.split():
		if not skipRemainingCommands:
			# print('Eseguo comando')
			executeCommand(command)
			# finishedCommands.append(command)
	messageToSend = "Comandi eseguiti correttamente"
	if skipRemainingCommands:
		messageToSend = "Mi sono fermato a causa di un ostacolo"
		angleToReset = 180
	await websocket.send(messageToSend)


def executeCommand(command):
	if command == 'stop':
		global goFlag
		goFlag = False
		velocity_publisher.publish(Twist())
	if command == 'avanti':
		print('Eseguo avanti')
		goStraight()
	if command == 'destra':
		rotate(90, True)
	if command == 'sinistra':
		rotate(90, False)
	time.sleep(1)

start_server = websockets.serve(listenToCommands, "localhost", 8090)
try:
	asyncio.get_event_loop().run_until_complete(start_server)
	asyncio.get_event_loop().run_forever()
except KeyboardInterrupt:
		print('Interrupted')
		try:
			sys.exit(0)
		except SystemExit:
			os._exit(0)
