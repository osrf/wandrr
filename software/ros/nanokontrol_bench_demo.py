#!/usr/bin/env python
#import roslib; roslib.load_manifest('wandrr')
import rospy

import pygame
import pygame.midi

import sys

from std_msgs.msg import Float32, Empty

control_axes = [{
  # mode 1, sliders
   2:  0,  3:  1,  4:  2,  5:  3,  6:  4,  8:  5,  9:  6, 12:  7, 13:  8,
  # mode 1, knobs
  14:  9, 15: 10, 16: 11, 17: 12, 18: 13, 19: 14, 20: 15, 21: 16, 22: 17,
  },{
  # mode 2, sliders
  42:  0, 43:  1, 50:  2, 51:  3, 52:  4, 53:  5, 54:  6, 55:  7, 56:  8,
  # mode 2, knobs
  57:  9, 58: 10, 59: 11, 60: 12, 61: 13, 62: 14, 63: 15, 65: 16, 66: 17,
  },{
  # mode 3, sliders
  85:  0, 86:  1, 87:  2, 88:  3, 89:  4, 90:  5, 91:  6, 92:  7, 93:  8,
  # mode 3, knobs
  94:  9, 95: 10, 96: 11, 97: 12, 102: 13, 103: 14, 104: 15, 105: 16, 106: 17,
  },{
  # mode 4, sliders
  7: 0, 263: 1, 519: 2, 775: 3, 1031: 4, 1287: 5, 1543: 6, 1799: 7, 2055: 8,
  # mode 4, knobs
  10: 9, 266: 10, 522: 11, 778: 12, 1034: 13, 1290: 14, 1546: 15, 1802: 16,
  2058: 17,
  }]

control_buttons = [[
  # mode 1
  # up, down
  23, 33, 24, 34, 25, 35, 26, 36, 27, 37, 28, 38, 29, 39, 30, 40, 31, 41,
  # rew, play, ff, repeat, stop, rec
  47, 45, 48, 49, 46, 44
],[
  # mode 2
  # up, down
  67, 76, 68, 77, 69, 78, 70, 79, 71, 80, 72, 81, 73, 82, 74, 83, 75, 84,
  # rew, play, ff, repeat, stop, rec
  47, 45, 48, 49, 46, 44
],[
  # mode 3
  # up, down
  107, 116, 108, 117, 109, 118, 110, 119, 111, 120, 112, 121, 113, 122, 114, 123, 115, 124,
  # rew, play, ff, repeat, stop, rec
  47, 45, 48, 49, 46, 44
],[
  # mode 4
  # up, down
  16, 17, 272, 273, 528, 529, 784, 785, 1040, 1041, 1296, 1297, 1552, 1553, 1808, 1809, 2064, 2065,
  # rew, play, ff, repeat, stop, rec
  47, 45, 48, 49, 46, 44
]]

def main():
  args = rospy.myargv(argv=sys.argv)
  pygame.midi.init()
  devices = pygame.midi.get_count()
  if devices < 1:
    print "No MIDI devices detected"
    exit(1)
  print "Found %d MIDI devices" % devices

  #if len(args) == 1:
  #  print "usage: nanokontrol_bus_voltage.py JOINT_PREFIX [MIDI_DEVICE]"
  #  exit(1)
  #prefix = args[1]

  if len(args) > 2:
    input_dev = int(args[2])
  else:
    print "using device 3..."
    input_dev = 3
    '''
    print "no input device supplied. will try to use default device."
    input_dev = pygame.midi.get_default_input_id()
    if input_dev == -1:
       print "No default MIDI input device"
       exit(-1)
    '''
  print "Using input device %d" % input_dev

  controller = pygame.midi.Input(input_dev)

  rospy.init_node("nanokontrol_target")
  #max_voltage  = rospy.get_param('~max_voltage', 0.2)
  #max_current  = rospy.get_param('~max_current', 0.5)
  #max_position = rospy.get_param('~max_position', 2.0)
  #print "max voltage = %f" % max_voltage
  #print "max current = %f" % max_current
  #print "max position = %f" % max_position
  #global_gain_scale = rospy.get_param('~global_gain_scale', 1.0)

  pub = rospy.Publisher("target_angle", Float32)
  pub_kp = rospy.Publisher("kp", Float32)

  while not rospy.is_shutdown():
    # count the number of events that are coalesced together
    c = 0
    while controller.poll():
      c += 1
      data = controller.read(1)
      # loop through events received
      for event in data:
        control = event[0]
        timestamp = event[1]
        print control
        if (control[0] & 0xF0) == 176:
          control_id = control[1] | ((control[0] & 0x0F) << 8)
          print "control id: %d" % control_id
          angle_control = 16
          kp_control = 0
          if control_id == angle_control:
            control_val = 2 * (float(control[2]) / 127.0 - 0.5) # [-1, 1]
            angle = control_val * 3
            pub.publish(Float32(angle))
          elif control_id == kp_control:
            control_val = (float(control[2]) / 127.0) # [0, 1]
            kp = -control_val * 80
            print "kp = %.3f" % kp
            pub_kp.publish(Float32(kp))
    rospy.sleep(0.001) # 1000Hz maximum input

if __name__ == '__main__':
   try:
      main()
   except rospy.ROSInterruptException: pass
