'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
import threading
from xmlrpc.server import SimpleXMLRPCServer
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    def __init__(self):
        super(ServerAgent, self).__init__()
        self.server = SimpleXMLRPCServer(('localhost', 8000), allow_none = True)
        self.server.register_function(self.get_angle)
        self.server.register_function(self.set_angle)
        self.server.register_function(self.get_posture)
        self.server.register_function(self.execute_keyframes)
        self.server.register_function(self.get_transform)
        self.server.register_function(self.set_transform)

        self.thread = threading.Thread(target=self.server_forever(), daemon=True)
        self.thread.start()
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        self.target_joints[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        return self.self_interpolation(keyframes, self.perception)

    def get_transform(self, name):
        '''get transform with given name
        '''
        return self.transforms[name]

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        self.set_transform(effector_name, transform)

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

