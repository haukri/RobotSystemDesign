#!/usr/bin/env node

/************************************************************************
 Copyright (c) 2017, Rethink Robotics
 Copyright (c) 2017, Ian McMahon

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
************************************************************************/

'use strict';

const rosnodejs = require('rosnodejs');
const request = require('request');

var mirStatusPub;

async function mirRobot() {


  // Register node with ROS master
  rosnodejs.initNode('/mir_robot_node')
    .then((rosNode) => {

      const callMIRSub = rosNode.subscribe('/call_mir', 'std_msgs/String', (msg) => {
        postMission(() => {
          console.log("MIR arrived!");
          mirStatusPub.publish({data: ''});
        });
      });

      const releaseMIRSub = rosNode.subscribe('/release_mir', 'std_msgs/String', (msg) => {
        setRegister(() => {
          console.log("MIR released!");
        })
      });

      mirStatusPub = rosNode.advertise('/mir_arrived', 'std_msgs/String');
    });
}

function postMission(callback) {
  request.post({
    headers: {
      'Content-Type': 'application/json',
      'Authorization': 'Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='
    },
    url: 'http://10.10.19.42:8080/v2.0.0/mission_queue',
    body: '{"mission_id": "e9d454d1-11c6-11ea-97b2-94c691159b76"}'
  }, function(error, response, body){
    if(response.statusCode == 201) {
      let missionInfo = JSON.parse(body);
      var interval = setInterval(function() {
        getRegister((value) => {
          if (value > 0) {
            clearInterval(interval);
            callback();
          }
          else {
            console.log("Waiting for MIR to arrive");
          }
        })
      }, 1000);
    }
  });
}

function getRegister(callback) {
  request.get({
    headers: {
      'Content-Type': 'application/json',
      'Authorization': 'Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='
    },
    url: 'http://10.10.19.42:8080/v2.0.0/registers/10'
  }, function(error, response, body){
    if(response.statusCode == 200) {
      let registerStatus = JSON.parse(body);
      callback(registerStatus.value);
    }
  });
}

function setRegister(callback) {
  request.post({
    headers: {
      'Content-Type': 'application/json',
      'Authorization': 'Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='
    },
    url: 'http://10.10.19.42:8080/v2.0.0/registers/10',
    body: '{"value": 0}'
  }, function(error, response, body){
    if(response.statusCode == 200) {
      callback();
    }
  });
}

if (require.main === module) {
  // Invoke Main Function
  mirRobot();
}
