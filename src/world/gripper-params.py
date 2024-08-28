import sys

def main(path):
    print('SET GRIPPER PARAMS: STARTED')
    print('Path to gripper file: ', path[0])
    with open(path[0], 'r', encoding='utf8') as gripper_file:
        gripper_code = gripper_file.readlines()
    
    gripper_code[24] = "	    <limit effort=\"50\" velocity=\"10.0\" lower=\"-1.6\" upper=\"1.6\" />\n"
    with open(path[0], 'w', encoding='utf-8') as gripper_file: 
        gripper_file.writelines(gripper_code) 
    print('SET GRIPPER PARAMS: ENDED')

if __name__ == '__main__':
    args = sys.argv[1:]
    if len(args) == 1:
        main(args)
    else:
        print('Usage: gripper-params.py <path/to/file>')