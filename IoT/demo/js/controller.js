'use strict';

function WebRosController(ros, ros2dmap) {
  this.ros = ros;

  this.currentVelocity = {
    layout: {
          dim: [
            {
              label: 'height',
              size: 2,
              stride: 2 * 3 * 3,
            },
            {
              label: 'weight',
              size: 3,
              stride: 3 * 3,
            },
            {
              label: 'channel',
              size: 3,
              stride: 3,
            },
          ],
          data_offset: 0,
        },
		data: [
          0.0,
          0.0,
          0.0,
          0.0,
        ],
  };
  this.zeroVelocity = {
    layout: {
          dim: [
            {
              label: 'height',
              size: 2,
              stride: 2 * 3 * 3,
            },
            {
              label: 'weight',
              size: 3,
              stride: 3 * 3,
            },
            {
              label: 'channel',
              size: 3,
              stride: 3,
            },
          ],
          data_offset: 0,
        },
		data: [
          0.0,
          0.0,
          0.0,
          0.0,
        ],
  };
  this.defaultVelocity = {
    layout: {
          dim: [
            {
              label: 'height',
              size: 2,
              stride: 2 * 3 * 3,
            },
            {
              label: 'weight',
              size: 3,
              stride: 3 * 3,
            },
            {
              label: 'channel',
              size: 3,
              stride: 3,
            },
          ],
          data_offset: 0,
        },
		data: [
          500,
          500,
          500,
          500,
        ],
  };
  this.CamWebMsg =
  {
    data: 1
  };  
  this.pubTopicTimer = null;
  this.pubTopicInterval = 30;
  this.pubVelTopicTimer = 30;
  this.pubVelTopicInterval = 30;
  this.pubCamWebTopicTimer = null;
  this.pubCamWebTopicInterval = 30;
  this.isRobotMoving = false;
  this.webControl = true;
  this.velocityTopic = new ROSLIB.Topic({
    ros: this.ros,
    name: '/soll_speed',
    messageType: 'std_msgs/msg/Float32MultiArray'
  });
  this.CamWebTopic = new ROSLIB.Topic({
    ros: this.ros,
    name: '/CamWeb',
    messageType: 'std_msgs/Int8'
  });

  this.poseX = 0;
  this.poseY = 0;
  this.radius = 0;
  this.rotation = 0;
  this.startTime = new Date();
  this.endTime = new Date();

  this.ros2dmap = ros2dmap;
  this.logger = new Logger('log');
}

WebRosController.prototype.sendVelTopic = function(vel) {
  console.log('send velocity topic');

  this.shutdownTimer();
  this.pubVelTopicTimer = setInterval(() => {

    this.velocityTopic.publish(vel);
    this.updateMap(vel);
    this.logger.showTerminalLog(vel);

    this.startTime = new Date();
  }, this.pubVelTopicInterval);
};

WebRosController.prototype.sendCamWebTopic = function(camweb) {
  console.log('send /CamWeb topic');

  this.shutdownTimer();
  this.pubCamWebTopicTimer = setInterval(() => {

    this.CamWebTopic.publish(camweb);
    this.logger.showTerminalLog(camweb);
  }, this.pubCamWebTopicInterval);
};

WebRosController.prototype.updateMap = function(vel) {
  this.endTime = new Date();

  let dt = (this.endTime - this.startTime) / 1000;

  let vx = vel.data[0];
  let az = vel.data[1];

  if (vx || az) {
    console.log('dt: ', dt);
    console.log('vx, az:', vx, ',', az);
  }
  let deltaX = (vx * Math.cos(this.radius)) * dt;
  let deltaY = (vx * Math.sin(this.radius)) * dt;
  let deltaRadius = az * dt;

  console.log(deltaX, deltaY, deltaRadius);

  if (deltaX || deltaY) {
    console.log('deltaX, deltaY:', deltaX, deltaY);
    this.poseX += deltaX;
    this.poseY += deltaY;
  }

  if (vx > 0) {
    this.rotation = this.radius * 180 / Math.PI;
  }

  if (az) {
    this.radius -= deltaRadius;
    this.rotation = this.radius * 180 / Math.PI;
  }


  if (deltaRadius > 0) {
    console.log('radius: ', this.radius);
  }

  if (vx || az) {
    console.log(this.poseX, ',', this.poseY, ',', this.rotation);
  }

  this.ros2dmap.update({
    y: this.poseX, x: this.poseY
  }, this.rotation - 90);
};

WebRosController.prototype.shutdownTimer = function() {
  if (this.pubVelTopicTimer) {
    clearInterval(this.pubVelTopicTimer);
    this.pubVelTopicTimer = null;
  }
};

WebRosController.prototype.shutdownCamWebTimer = function() {
  if (this.pubCamWebTopicTimer) {
    clearInterval(this.pubCamWebTopicTimer);
    this.pubCamWebTopicTimer = null;
  }
};

WebRosController.prototype.moveForward = function() {
  console.log('web ros controller: move forward');
  console.log(this.currentVelocity);
  console.log(this.defaultVelocity);

  this.startTime = new Date();
if (this.currentVelocity.data[0] > 0) {
	/*this.currentVelocity.linear.x = -this.currentVelocity.linear.x;*/
    this.sendVelTopic(this.currentVelocity);
  } else {
    this.sendVelTopic(this.defaultVelocity);
  }
  this.isRobotMoving = true;
};

WebRosController.prototype.turnLeft = function() {
  console.log('web ros controller: turn left');

  let turnLeftMsg = {
    layout: {
          dim: [
            {
              label: 'height',
              size: 2,
              stride: 2 * 3 * 3,
            },
            {
              label: 'weight',
              size: 3,
              stride: 3 * 3,
            },
            {
              label: 'channel',
              size: 3,
              stride: 3,
            },
          ],
          data_offset: 0,
        },
		data: [
          500,
          900,
          500,
          900,
        ],
  };

  this.shutdownTimer();
  this.startTime = new Date();

  this.sendVelTopic(turnLeftMsg);
};

WebRosController.prototype.turnRight = function() {
  console.log('web ros controller: turn left');

  let turnRightMsg = {
    layout: {
          dim: [
            {
              label: 'height',
              size: 2,
              stride: 2 * 3 * 3,
            },
            {
              label: 'weight',
              size: 3,
              stride: 3 * 3,
            },
            {
              label: 'channel',
              size: 3,
              stride: 3,
            },
          ],
          data_offset: 0,
        },
		data: [
          900,
          500,
          900,
          500,
        ],
  };
  this.shutdownTimer();
  this.startTime = new Date();

  this.sendVelTopic(turnRightMsg);
};

WebRosController.prototype.moveBack = function() {
  console.log('web ros controller: move back');

  console.log(this.currentVelocity);
  console.log(this.defaultVelocity);

  this.startTime = new Date();
  if (this.currentVelocity.data[0]) {
    if (this.currentVelocity.data[0] > 0) {
      this.currentVelocity.data[0] = -this.currentVelocity.data[0];
    }
    this.sendVelTopic(this.currentVelocity);
  } else {
    let backVel = {
      layout: {
          dim: [
            {
              label: 'height',
              size: 2,
              stride: 2 * 3 * 3,
            },
            {
              label: 'weight',
              size: 3,
              stride: 3 * 3,
            },
            {
              label: 'channel',
              size: 3,
              stride: 3,
            },
          ],
          data_offset: 0,
        },
		data: [
          0.0,
         -500,
         -500,
         -500,
        ],
    };
    backVel.data[0] = -this.defaultVelocity.data[0];
    this.sendVelTopic(backVel);
  }
  this.isRobotMoving = true;
};

WebRosController.prototype.start = function() {
  console.log('web ros controller: start');

  if (this.currentVelocity.data[0]) {
    this.sendVelTopic(this.currentVelocity);
  } else {
    this.currentVelocity.data[0] = this.defaultVelocity.data[0];
    this.sendVelTopic(this.defaultVelocity);
  }

  this.isRobotMoving = true;
};

WebRosController.prototype.stop = function() {
  console.log('web ros controller: stop');

  this.sendVelTopic(this.zeroVelocity);
  this.isRobotMoving = false;
};

WebRosController.prototype.CamOrWeb = function(vel) {
  console.log('send topic: CamWeb');

  
	if (this.CamWebMsg.data == 1){
		this.CamWebMsg.data = 0;
		this.sendCamWebTopic(this.CamWebMsg);
                this.webControl = false;
	} else{
		this.CamWebMsg.data = 1;
		this.sendCamWebTopic(this.CamWebMsg);
                this.webControl = true;
	}
    
};
