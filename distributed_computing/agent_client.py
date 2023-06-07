'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import threading
import weakref
from xmlrpc.server import XMLRPCDocGenerator

import numpy as np

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        thread = threading.Thread(target=self.proxy.execute_keyframes, args=[keyframes])
        thread.start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        thread = threading.Thread(target=self.proxy.set_transfrom, args=[effector_name, transform])
        thread.start()


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        self.proxy = XMLRPCDocGenerator.client.ServerProxy('http://localhost:9999')
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.proxy.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        self.proxy.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        return self.proxy.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        self.proxy.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        return np.asarray(self.s.get_transform(name))

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        return self.proxy.set_transform(effector_name, transform.tolist())

if __name__ == '__main__':
    agent = ClientAgent()
    print(agent.get_angle("RKneeRoll"))
    print(agent.set_angle("HeadPitch", 0.1))


