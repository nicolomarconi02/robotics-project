#! /usr/bin/env python

"""!
Executes the robot. Before doing so, it modifies its code in order
for it to create a topic when the homing procedure is completed.
"""

def insertAfter(haystack, needle, newText):
  """!
  Inserts 'newText' into 'haystack' right after 'needle'.
  """
  
  i = haystack.find(needle)
  return haystack[:i + len(needle)] + newText + haystack[i + len(needle):]

with open('../locosim/robot_control/base_controllers/ur5_generic.py', 'r', encoding='utf8') as ur5_file:
        ur5_code = ur5_file.read()

ur5_code = insertAfter(ur5_code, 'print(colored("HOMING PROCEDURE ACCOMPLISHED", \'red\'))\n', '                ur5_ready = ros.Publisher("/ur5/ready", WrenchStamped, queue_size=1)\n')

exec(ur5_code) 