clear, clc, close all

robot = robotics.RigidBodyTree('DataFormat','row','MaxNumBodies',5);

L1= 0.2;
L2 = 0.3;
L3 = 0.3;
L4 = 0.1;
Lg = 0.05;
L5 = Lg;
%Base

body = robotics.RigidBody('base2');
joint = robotics.Joint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.PositionLimits = [-pi/2 pi/2];
joint.JointAxis = [0 0 1];
joint.HomePosition = 0;
body.Joint = joint;
addBody(robot, body, 'base');
%link 1
body = robotics.RigidBody('column');
joint = robotics.Joint('joint2','revolute');
setFixedTransform(joint, trvec2tform([0,0,-L1]));
joint.PositionLimits = [-pi/2 pi/2];
joint.JointAxis = [1 0 0];
joint.HomePosition = -pi/4;
body.Joint = joint;
addBody(robot, body, 'base2');

body = robotics.RigidBody('shoulder');
joint = robotics.Joint('joint3','revolute');
setFixedTransform(joint, trvec2tform([0,L2,0]));
joint.JointAxis = [1 0 0];
joint.PositionLimits = [-pi/2 pi/2];
joint.HomePosition = pi/2;
body.Joint = joint;
addBody(robot, body, 'column');

body = robotics.RigidBody('forearm');
joint = robotics.Joint('pitch','revolute');
setFixedTransform(joint, trvec2tform([0,L3,0]));
joint.JointAxis = [1 0 0];
joint.PositionLimits = [-pi/2 pi/2];
joint.HomePosition = -pi/4;
body.Joint = joint;
addBody(robot, body, 'shoulder');

body = robotics.RigidBody('wrist');
joint = robotics.Joint('roll','revolute');
setFixedTransform(joint, trvec2tform([0,L4,0]));
joint.JointAxis = [0 1 0];
joint.PositionLimits = [-pi/2 pi/2];
body.Joint = joint;
addBody(robot, body, 'forearm');

body = robotics.RigidBody('hbody');
joint = robotics.Joint('yar','revolute');
setFixedTransform(joint, trvec2tform([0,0,0]));
joint.JointAxis = [0 0 1];
joint.PositionLimits = [-pi/2 pi/2];
body.Joint = joint;
addBody(robot, body, 'wrist');

body = robotics.RigidBody('Gripper');
joint = robotics.Joint('fixed','fixed');
setFixedTransform(joint, trvec2tform([0, Lg, 0]));
body.Joint = joint;
addBody(robot, body, 'hbody');

jointAnglesH = [0,-0.785398163397448,1.57079632679490,-0.785398163397448,0,0];



showdetails(robot)
%getTransform(robot,config,'tool','base')
config = homeConfiguration(robot);
show(robot)

%%

jacobian = geometricJacobian(robot,config,'Gripper')

