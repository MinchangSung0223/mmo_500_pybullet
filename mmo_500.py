import pybullet as p
import pybullet_data
import time
import math
pi = math.pi

base_wheel_direction_id = [1,3,5,7]
base_wheel_id = [2,4,6,8]
arm_joint_id = [13,14,15,16,17,18]


rp=[pi/2,1.25,1.95,pi/2,0,0]  # reset position for arm


def twistToVelocity(twist):
	wheel_direction = [0,0,0,0]
	wheel_velocity = [0,0,0,0]
	w = twist[0];
	vx = twist[1];
	vy = twist[2];
	if abs(w)<0.001:
		theta = math.atan2(vy,vx);
		wheel_direction = [theta,theta,theta,theta];
		velo = math.sqrt(vx*vx+vy*vy);
		wheel_velocity = [velo,-velo,velo,-velo];					
	else:
		wheel_direction = [-pi/4,pi/4,pi/4,-pi/4]	
		wheel_velocity = [w,w ,w,w];

	return	wheel_direction,wheel_velocity


if __name__ == "__main__":
	clid = p.connect(p.SHARED_MEMORY)
	if (clid < 0):
		p.connect(p.GUI)
		#p.connect(p.SHARED_MEMORY_GUI)
	p.setGravity(0, 0, -10)
	p.setAdditionalSearchPath(pybullet_data.getDataPath())
	plane=p.loadURDF("plane.urdf", [0, 0, 0.0])
	mmo_500 = p.loadURDF("mmo_500.urdf", [0, 0, 0.00],p.getQuaternionFromEuler([0, 0, 0]))
	numJoints = p.getNumJoints(mmo_500) 
	lateralFrictionId = p.addUserDebugParameter("lateral friction", 0, 10, 1)
	vx_id = p.addUserDebugParameter("vx", -10, 10, 0)
	vy_id = p.addUserDebugParameter("vy", -10, 10, 0)
	w_id = p.addUserDebugParameter("w", -10, 10, 0)

	while(1):
		lateralFriction = p.readUserDebugParameter(lateralFrictionId)
		vx = p.readUserDebugParameter(vx_id)
		vy = p.readUserDebugParameter(vy_id)
		w = p.readUserDebugParameter(w_id)
		twist = [w,vx,vy]

		p.changeDynamics(plane, -1, lateralFriction=lateralFriction)
		wheel_direction,wheel_velocity=twistToVelocity(twist)

		for idx,arm_id in enumerate(arm_joint_id, start=0):
			p.resetJointState(mmo_500,  arm_id, rp[idx])
		for idx,wheel_direction_id in enumerate(base_wheel_direction_id, start=0):
			p.resetJointState(mmo_500,  wheel_direction_id, wheel_direction[idx])

		for idx,wheel_id in enumerate(base_wheel_id, start=0):
			p.setJointMotorControl2(mmo_500,
				    wheel_id,
				    p.VELOCITY_CONTROL,
				    targetVelocity=wheel_velocity[idx],
				    force=1000)

		time.sleep(0.01);
		p.stepSimulation()
