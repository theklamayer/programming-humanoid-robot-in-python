'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.spawn_time = self.perception.time

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)
    

    def angle_interpolation(self, keyframes, perception):
        if self.startTime < 0:
            self.startTime = perception.time

        passed_time = perception.time - self.startTime

        names, times, keys = keyframes

        for joint in range(0, len(names)):
            for key in range(0, len(keys[joint]) - 1):
                t = (passed_time - times[joint][key]) / (times[joint][key + 1] - times[joint][key])
                if passed_time < times[joint][0]:
                    p0 = 0
                    p3 = keys[joint][0][0]
                    p1 = 0
                    p2 = keys[joint][0][1][2]
                    self.target_joints[joint] = ((1 - t) ** 3) * p0 + 3 * ((1 - t) ** 2) * t * p1 + 3 * (1 - t) * (
                                t ** 2) * p2 + (t ** 3) * p3
                elif times[joint][key] <= passed_time < times[joint][key + 1]:
                    p0 = keys[joint][key][0]
                    p3 = keys[joint][key + 1][0]
                    p1 = keys[joint][key][1][2] + p0
                    p2 = keys[joint][key + 1][1][2] + p3
                    self.target_joints[joint] = ((1 - t) ** 3) * p0 + 3 * ((1 - t) ** 2) * t * p1 + 3 * (1 - t) * (
                                t ** 2) * p2 + (t ** 3) * p3

        return self.target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
