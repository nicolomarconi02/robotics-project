with open('../locosim/robot_control/base_controllers/ur5_generic.py', 'r', encoding='utf8') as ur5_file:
        ur5_code = ur5_file.read()

ur5_code.replace('self.world_name = None', 'self.world_name = \'robotics_project.world\'')

exec(ur5_code)