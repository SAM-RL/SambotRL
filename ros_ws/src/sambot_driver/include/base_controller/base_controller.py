from Adafruit_MotorHAT import Adafruit_MotorHAT
from motor.motor import Motor
from encoder.encoder import Encoder
import traitlets
import rospy
from sambot_msgs.msg import EncodersStamped
from sambot_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from std_msgs.msg import Float32

class SamBotBaseController(object):

    encoder_resolution = 576
    i2c_bus = 1

    publishRate = 15

    # Motor Configurations
    left_motor_channel = 1
    left_motor_alpha = 1
    right_motor_channel = 2
    right_motor_alpha = 1

    # Encoder Configurations
    right_encoder_pins = [38, 40]
    left_encoder_pins = [35, 37]

    # Publish Topics
    topic_encoder = "encoder_ticks"
    topic_joint_state = "measured_joint_states"

    # Subscribed Topic
    topic_motor_cmd = "wheel_cmd_velocities"
    topic_motor_left_cmd = "motor_left"
    topic_motor_right_cmd = "motor_right"
    topic_reset_encoder = "reset_encoder"

    def __init__(self):
        self.lastUpdateTime = rospy.Time.now()        
        self.motor_driver = Adafruit_MotorHAT(i2c_bus=self.i2c_bus)
        self.left_motor = Motor(self.motor_driver, channel=self.left_motor_channel, alpha=self.left_motor_alpha)
        self.right_motor = Motor(self.motor_driver, channel=self.right_motor_channel, alpha=self.right_motor_alpha)
        #self.encoder_left = Encoder(self.encoder_resolution, self.left_encoder_pins)
        #self.encoder_right = Encoder(self.encoder_resolution, self.right_encoder_pins)

        self.left_motor_cmd = 0
        self.right_motor_cmd = 0

        #self.encoder_tick_pub = rospy.Publisher(self.topic_encoder, EncodersStamped, queue_size=10)
        #self.measured_joint_state_pub = rospy.Publisher(self.topic_joint_state, JointState, queue_size=10)
        
        #self.motor_cmd_sub = rospy.Subscriber(self.topic_motor_cmd, WheelsCmdStamped, self.handle_motor_vel_cmd)
        self.motor_left_cmd_sub = rospy.Subscriber(self.topic_motor_left_cmd, Float32, self.handle_motor_left_cmd)
        self.motor_right_cmd_sub = rospy.Subscriber(self.topic_motor_right_cmd, Float32, self.handle_motor_right_cmd)

        #self.reset_encoder_sub = rospy.Subscriber(self.topic_reset_encoder, Empty, self.handle_reset_encoder)


    def handle_motor_vel_cmd(self, msg):
        self.left_motor_cmd = msg.wheels_cmd.angular_velocities.joint[0]
        self.right_motor_cmd = msg.wheels_cmd.angular_velocities.joint[1]
        
    def handle_motor_left_cmd(self, msg):
        self.left_motor_cmd = msg.data
    def handle_motor_right_cmd(self, msg):
        self.right_motor_cmd = msg.data
    

    def read(self):
        # Get the current joint states
        #left__angular_position, left__angular_velocity = self.encoder_left.get_joint_state()
        #right__angular_position, right__angular_velocity = self.encoder_right.get_joint_state()
        
        #msg_measured_joint_states = JointState()
        #msg_measured_joint_states.position = [left__angular_position, right__angular_position]
        #msg_measured_joint_states.velocity = [left__angular_velocity, right__angular_velocity]

        #self.measured_joint_state_pub.publish(msg_measured_joint_states)

        # Get tick counts
        #ticks_left = self.encoder_left.get_tick_count()
        #ticks_right = self.encoder_right.get_tick_count()

        #msg_encoders = EncodersStamped()
        #msg_encoders.encoders.ticks = [ticks_left, ticks_right]

        #self.encoder_tick_pub.publish(msg_encoders)
        pass
    
    
    def write(self):
        self.left_motor.value = self.left_motor_cmd
        self.right_motor.value = self.right_motor_cmd
        


    def eStop(self):
        self.left_motor.value = 0
        self.right_motor.value = 0

    
    #def handle_reset_encoder(self):
        #self.encoder_left.reset()
        #self.encoder_right.reset()
