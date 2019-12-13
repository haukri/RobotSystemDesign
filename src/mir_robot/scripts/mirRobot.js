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
var simulation = false;

var commonHeader = {
  'Content-Type': 'application/json',
  'Authorization': 'Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='
}

async function mirRobot() {

  // Register node with ROS master
  rosnodejs.initNode('/mir_robot_node')
    .then((rosNode) => {

      const callMIRSub = rosNode.subscribe('/call_mir', 'std_msgs/String', (msg) => {
        setRegister(13, 0);
        postMission(() => {
          console.log("MIR arrived!");
          mirStatusPub.publish({data: ''});
        });
      });

      const releaseMIRSub = rosNode.subscribe('/release_mir', 'std_msgs/String', async (msg) => {
        setRegister(12, 0);
      });

      mirStatusPub = rosNode.advertise('/mir_arrived', 'std_msgs/String');

      const mirService = rosNode.advertiseService('/mir_should_charge', 'std_srvs/SetBool', (req, res) => {
        return new Promise(async function(resolve, reject) {
          var batteryStatus = await checkMirBattery();
          res.success = batteryStatus;
          res.message = "success";
          resolve(true);
        });
      });

      const mirMissionService = rosNode.advertiseService('/mir_mission_started', 'std_srvs/SetBool', (req, res) => {
        return new Promise(async function(resolve, reject) {
          res.success = await getRegister(13);;
          res.message = "success";
          resolve(true);
        });
      });

      const mirDoneService = rosNode.advertiseService('/mir_charging_done', 'std_srvs/SetBool', (req, res) => {
        return new Promise(async function(resolve, reject) {
          var doneCharging = await mirDoneCharging();
          res.success = doneCharging;
          res.message = "success";
          resolve(true);
        });
      });
    });
}

async function mirDoneCharging() {
  var doneRecharging = await getRegister(90);
  return !doneRecharging;
}

async function checkMirBattery() {
  var group9 = await getRegister(1);
  var group11 = await getRegister(21);
  var group12 = await getRegister(31);
  var shouldCharge = await getRegister(89);

  if(shouldCharge) {
    await setRegister(11, 1);
    await setRegister(90, 1);
    if(group9 && group11 && group12) {
      await postNewMission("94acb9cb-1a6d-11ea-845d-94c691159b76");
    }
    return true
  }
  else {
    return false;
  }
}

async function postNewMission(missionID) {
  return new Promise((resolve, reject) => {
    request.post({
      headers: commonHeader,
      url: 'http://10.10.19.42:8080/v2.0.0/mission_queue',
      body: '{"mission_id": "' + missionID + '"}'
    }, function(error, response, body){
      resolve(true);
    });
  })
}

function postMission(callback) {
  request.post({
    headers: commonHeader,
    url: 'http://10.10.19.42:8080/v2.0.0/mission_queue',
    body: '{"mission_id": "e9d454d1-11c6-11ea-97b2-94c691159b76"}'
  }, function(error, response, body){
    if(!error) {
      if(response.statusCode == 201) {
        let missionInfo = JSON.parse(body);
        var interval = setInterval(async function() {
          var value = await getRegister(12);
          if (value > 0) {
            clearInterval(interval);
            callback();
          }
          else {
            console.log("Waiting for MIR to arrive");
          }
        }, 1000);
      }
    } 
    else if(simulation) {
      callback();
    }
  });
}

async function getRegister(registerNumber) {
  return new Promise((resolve, reject) => {
    request.get({
      headers: commonHeader,
      url: 'http://10.10.19.42:8080/v2.0.0/registers/' + registerNumber
    }, function(error, response, body){
      if(!error) {
        if(response.statusCode == 200) {
          let registerStatus = JSON.parse(body);
          resolve(registerStatus.value);
        }
      }
      else if(simulation) {
        resolve(1);
      }
    });
  })
}

async function setRegister(registerNumber, value) {
  return new Promise((resolve, reject) => {
    request.post({
      headers: commonHeader,
      url: 'http://10.10.19.42:8080/v2.0.0/registers/' + registerNumber,
      body: '{"value": ' + value + '}'
    }, function(error, response, body){
      if(!error) {
        if(response.statusCode == 200) {
          resolve();
        }
      }
      else if(simulation) {
        resolve();
      }
    });
  })
}

if (require.main === module) {
  // Invoke Main Function
  mirRobot();
}
