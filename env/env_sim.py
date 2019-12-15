import vrep.vrep as vrep
import vrep.vrepConst as const_v
import time
import sys
import numpy as np
import cv2
import math
import os
from gym import spaces

class rozum_sim:

    def __init__(self):
        self.DoF = 6
        self.action_bound = [-5, 5]
        self.action_dim = self.DoF
        self.angles_bound = [-180,180]

        self.action_space=spaces.Box(low=-5,high=5,shape=[self.action_dim])
#         self.observation_space=spaces.Box(0, 1, [256, 256, 3])
#         self.observation_space=spaces.Box(0,1,[256,256,1])#+spaces.Box(0,1,[4],dtype=np.float64)
#         self.observation_space=spaces.Box(low=0,high=1,shape=[self.action_dim+4],dtype=np.float64)
        self.observation_space=spaces.Box(low=0,high=100,shape=[self.action_dim+4+4],dtype=np.float64)
        self.metadata=''
        
        # os.chdir("/Vrep_server")
        # os.system("git pull")
        # os.chdir("/")
        #
        self.vrep_root = "/V-REP_PRO_EDU_V3_5_0_Linux/"
        self.scene_file = "/Vrep_server/env/rozum_model.ttt"
        #
        os.chdir(self.vrep_root)
        # os.system("xvfb-run --auto-servernum --server-num=1 -s \"-screen 0 640x480x24\" 
        os.system("./vrep.sh -h -s " + self.scene_file + " &")
        os.chdir("/")

        vrep.simxFinish(-1)
        time.sleep(1)

        # get the ID of the running simulation
        self.ID = vrep.simxStart('172.17.0.1', 19999, True, False, 5000, 5)
        # check the connection
        if self.ID != -1:
            print("Connected")
        else:
            sys.exit("Error")
        # get handles
        # for camera
        self.cam_handle = self.get_handle('Vision_sensor')
        (code, res, im) = vrep.simxGetVisionSensorImage(self.ID, self.cam_handle, 0, const_v.simx_opmode_streaming)

        self.render_handle = self.get_handle('render')
        (code, res, im) = vrep.simxGetVisionSensorImage(self.ID, self.render_handle, 0, const_v.simx_opmode_streaming)

        # joints
        self.joint_handles = []
        for i in range(self.DoF):
            tmp = self.get_handle("joint%d" % (i))
            self.joint_handles.append(tmp)
            code, angle = vrep.simxGetJointPosition(self.ID, tmp, const_v.simx_opmode_streaming)

        # gripper tip
        self.tip_handle = self.get_handle("Tip")
        (code, pose) = vrep.simxGetObjectPosition(self.ID, self.tip_handle, -1, const_v.simx_opmode_streaming)
        # cube
        self.cube_handle = self.get_handle("Cube")
        (code, pose) = vrep.simxGetObjectPosition(self.ID, self.cube_handle, -1, const_v.simx_opmode_streaming)
        # get the goal handle
        self.goal_handle = self.get_handle("Goal")
        (code, pose) = vrep.simxGetObjectPosition(self.ID, self.goal_handle, -1, const_v.simx_opmode_streaming)

        # angles' array
        self.angles = self.get_angles()

        # gripper handles (used in closing and opening gripper)
        self.gripper_motor = self.get_handle('RG2_openCloseJoint')
        # task part
        self.task_part = 0
        
        self.goal_l = (80, 0, 0)
        self.goal_u = (120, 255, 255)
        self.cube_l = (55, 50, 50)
        self.cube_u = (80, 255, 255)
        self.er_kernel = np.ones((2, 2), np.uint8)
        self.di_kernel = np.ones((2, 2), np.uint8)
        self.task_part = 0
        self.part_1_center = np.array([120.0, 178.0])/256
        self.part_2_center = np.array([128.0, 155.0])/256
        self.part_1_area = 0.25
        self.part_2_area = 0.75
        self.target=np.array([120.0/256, 178.0/256,0.25,0.0])

        self.init_angles = self.get_angles()

        self.init_pose_cube = self.get_position(self.cube_handle)
        # print(self.init_pose_cube)
        self.init_goal_pose = self.get_position(self.goal_handle)
        # print(self.init_goal_pose)
        self.open_gripper()
        self.t=0
        self.reset()
        self.tip_position = self.get_position(self.tip_handle)


    def get_handle(self, name):
        (check, handle) = vrep.simxGetObjectHandle(self.ID, name, const_v.simx_opmode_blocking)
        if check != 0:
            print("Couldn't find %s" % name)
        return handle

    def get_position(self, handle):
        (code, pose) = vrep.simxGetObjectPosition(self.ID, handle, -1, const_v.simx_opmode_buffer)
        # print(code)
        return np.array(pose)

    def close_gripper(self):
        code=vrep.simxSetJointForce(self.ID, self.gripper_motor, 20, const_v.simx_opmode_blocking)
        # print(code)
        code=vrep.simxSetJointTargetVelocity(self.ID, self.gripper_motor, -0.05, const_v.simx_opmode_blocking)
        # print(code)
        # time.sleep(0.1)

    def open_gripper(self):
        code=vrep.simxSetJointForce(self.ID, self.gripper_motor, 20, const_v.simx_opmode_blocking)
        # print(code)
        code=vrep.simxSetJointTargetVelocity(self.ID, self.gripper_motor, 0.05, const_v.simx_opmode_blocking)
        # print(code)
        #time.sleep(0.1)

    def get_image(self, cam_handle):
        (code, res, im) = vrep.simxGetVisionSensorImage(self.ID, cam_handle, 0, const_v.simx_opmode_buffer)
        # print(code)
        img = np.array(im, dtype=np.uint8)
        img.resize([res[0], res[1], 3])
        img=cv2.flip(img,0)
        return img

    def move_joint(self, num, value):
        # in radian
        code=vrep.simxSetJointTargetPosition(self.ID, self.joint_handles[num], value*math.pi/180, const_v.simx_opmode_blocking)
        # print(code)
        time.sleep(0.3)

    def get_angles(self):
        angles = []
        for i in range(self.DoF):
            code, angle = vrep.simxGetJointPosition(self.ID, self.joint_handles[i], const_v.simx_opmode_buffer)
            angles.append(angle*180/math.pi)
        return angles

    def sample_action(self):
        return np.random.uniform(*self.action_bound, size=self.action_dim)

    def step(self, action):
        self.t+=1
        action = np.clip(action, *self.action_bound)
        self.angles = self.get_angles()
        for i in range(self.DoF):
            self.angles[i] += action[i]
        self.angles=np.clip(self.angles,*self.angles_bound)
        for i in range(self.DoF):
            self.move_joint(i, self.angles[i])
        img = self.get_image(self.cam_handle)
        obs,reward, done,binary = self.get_reward(img)
        angles=self.get_angles()
        s=np.concatenate((angles,obs,self.target),axis=None)
        return s, reward, done, {}

    def reset(self):
        self.t=0
        self.task_part=0
        self.target=np.array([120.0/256, 178.0/256,0.25,0.0])
        self.angles = self.init_angles
        for i in range(self.DoF):
            self.move_joint(i, self.angles[i])
        self.open_gripper()
        vrep.simxSetObjectPosition(self.ID, self.cube_handle, -1, self.init_pose_cube, const_v.simx_opmode_oneshot_wait)
        vrep.simxSetObjectPosition(self.ID, self.goal_handle, -1, self.init_goal_pose, const_v.simx_opmode_oneshot_wait)
        img = self.get_image(self.cam_handle)
        center, area, rotation,binary=self.image_processeing(img, self.goal_l, self.goal_u, [1, 1])
        obs=np.array([center[0],center[1], area, rotation])
        angles=self.get_angles()
        s=np.concatenate((angles,obs,self.target),axis=None)
#         print(s)
        return s

    def image_processeing(self,img,lower,upper,num_iter):
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
#         h=hsv.copy()
#         h[:,:,1]=0
#         h[:,:,2]=0
        binary = cv2.inRange(hsv, lower, upper)
        binary = cv2.erode(binary, self.er_kernel, iterations=num_iter[0])
        binary = cv2.dilate(binary, self.di_kernel, iterations=num_iter[1])
        # cv2.imshow("1",binary)
        # cv2.waitKey(1)
        cnt, _ = cv2.findContours(binary, 1, 1)
        cnt = sorted(cnt, key=cv2.contourArea, reverse=True)
        center=np.array([0.0,0.0])
        area_percentage=0
        rotation=0
        if len(cnt) > 0:
            rect = cv2.minAreaRect(cnt[0])
            angle = rect[2]
            if angle < -45:
                angle += 90
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            center = np.average(box, axis=0)/256
            area = cv2.contourArea(cnt[0])
            area_percentage=area/(256*256)
            rotation = abs(angle)/90
        # print(center)
        binary=binary[...,np.newaxis]
        return center,area_percentage,rotation,binary

#     def get_reward(self, img):
#         reward = -0.1
#         done = False
#         if self.task_part == 0:
#             center, area, rotation,binary = self.image_processeing(img, self.goal_l, self.goal_u, [1, 1])
#             obs=np.array([center[0],center[1], area, rotation])
#             distance = np.linalg.norm(center - self.part_1_center, axis=-1)
#             area_difference = abs(area - self.part_1_area)
#             # print(distance, area_difference, rotation)
#             if distance < 0.01 and area > self.part_1_area and rotation < 1:
#                 self.task_part=1
#                 self.target=np.array([128.0/256, 155.0/256,0.25,0.0])
#                 self.det_goal=self.get_angles()
#                 self.angles = self.init_angles.copy()
#                 for i in range(self.DoF):
#                     self.move_joint(i, self.angles[i])
#                 reward += 5
#                 return obs,reward, done,binary
#         else:
#             center, area, rotation,binary = self.image_processeing(img, self.cube_l, self.cube_u, [1, 1])
#             obs=np.array([center[0],center[1], area, rotation])
#             distance = np.linalg.norm(center - self.part_2_center, axis=-1)
#             area_difference = abs(area - self.part_2_area)
#             # print(distance,area_difference,rotation)
#             if distance < 0.01 and area > self.part_2_area and rotation < 1:
#                 reward += 5
#                 done = True
#                 self.close_gripper()
#                 self.angles = self.init_angles.copy()
#                 for i in range(self.DoF):
#                     self.move_joint(i, self.angles[i])
#                 self.angles = self.det_goal.copy()
#                 for i in range(self.DoF):
#                     self.move_joint(i, self.angles[i])
#                 self.open_gripper()
#                 return obs,reward, done,binary
#         if obs[2]<0.01:
# #             reward-=5
#             done=True
#             return obs,reward, done,binary
# #         reward -= (0.0025 * distance + 0.015 * area_difference + 0.015 * rotation)
# #         reward+= np.exp(-(0.0025 * distance + 1.5 * area_difference + 0.015 * rotation))
#         reward+=0.05*(1/(1+math.pow(distance,1.2)))+0.03*(1/(1+math.pow(area_difference,1.2)))+0.02*(1/(1+math.pow(rotation,1.2)))
#         return obs,reward, done,binary

    def get_reward(self, img):
        done=False
        time_discount=self.t/200
        if self.task_part == 0:
            center, area, rotation,binary = self.image_processeing(img, self.goal_l, self.goal_u, [1, 1])
            obs=np.array([center[0],center[1], area, rotation])
            distance = np.linalg.norm(center - self.part_1_center, axis=-1)
            area_difference = abs(area - self.part_1_area)
        else:
            center, area, rotation,binary = self.image_processeing(img, self.cube_l, self.cube_u, [1, 1])
            obs=np.array([center[0],center[1], area, rotation])
            distance = np.linalg.norm(center - self.part_2_center, axis=-1)
            area_difference = abs(area - self.part_2_area)
        if self.t>=200:
            done=True
            reward=-10
            return obs,reward,done,binary
        if area_difference>0.25:
            done=True
            reward=-100
            return obs,reward,done,binary
        if distance<0.01:
            if area_difference<0.01:
                if rotation<0.1:
                    reaching_reward=100
                    reward=reaching_reward*time_discount
                    if self.task_part==0:
                        self.task_part=1
                        self.target=np.array([128.0/256, 155.0/256,0.25,0.0])
                        self.det_goal=self.get_angles()
                        self.angles = self.init_angles.copy()
                        for i in range(self.DoF):
                            self.move_joint(i, self.angles[i])
                    else:
                        done = True
                        self.close_gripper()
                        self.angles = self.init_angles.copy()
                        for i in range(self.DoF):
                            self.move_joint(i, self.angles[i])
                        self.angles = self.det_goal.copy()
                        for i in range(self.DoF):
                            self.move_joint(i, self.angles[i])
                        self.open_gripper()
                else:
                    reward=-10*abs(rotation)
            else:
                reward=-100*area_difference
        else:
            distance_reward=1-math.pow(distance,0.4)
            reward=distance_reward*time_discount
        if self.task_part==1:
            reward+=100
        return obs,reward,done,binary
        

    def render(self):
        im=self.get_image(self.render_handle)
        return im

# env=rozum_sim()
# while True:
#     a=env.sample_action()
#     _,r,_,_=env.step(a)
#     print(r)