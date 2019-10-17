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
const express = require('express')
var app = express();
var http = require('http').createServer(app);
var io = require('socket.io')(http);

const GetStats = rosnodejs.require('packml_msgs').srv.GetStats;
const Transition = rosnodejs.require('packml_msgs').srv.Transition;

var packMLStatus = {}
var packMLTransitionService;

function userInterface() {

  // Register node with ROS master
  rosnodejs.initNode('/user_interface_node')
    .then((rosNode) => {
        io.on('connection', function (socket) {
        console.log('a user connected');
        if(packMLStatus) {
          io.sockets.emit('packml_status', packMLStatus);
        }
        socket.on('packml_command', message => {
            const request = new Transition.Request();
            if (message.command == "start")
            {
                request.command = 2;
            }
            else if (message.command == "hold")
            {
                request.command = 4;
            }
            else if (message.command == "stop")
            {
                request.command = 3;
            }
            else if (message.command == "abort")
            {
                request.command = 5;
            }
            packMLTransitionService.call(request).then((resp) => {
            console.log('packML Transition state successfull')
            })
        });

        });
      const subStatus = rosNode.subscribe('/packml_node/packml/status', 'packml_msgs/Status', (msg) => {
        packMLStatus = msg;
        io.sockets.emit('packml_status', packMLStatus);
      });

      let serviceClient = rosNode.serviceClient('/packml_node/packml/get_stats', 'packml_msgs/GetStats', { persist: true });
      rosNode.waitForService(serviceClient.getService(), 2000)
        .then((available) => {
          if (available) {
            const request = new GetStats.Request();
            setInterval(() => {
              serviceClient.call(request).then((resp) => {
                io.sockets.emit('packml_stats', resp);
              })
            }, 1000);
          }
        });
      packMLTransitionService = rosNode.serviceClient('/packml_node/packml/transition', 'packml_msgs/Transition', { persist: true });
      rosNode.waitForService(packMLTransitionService.getService(), 2000)
        .then((available) => {
          if (available) {
            // TODO
          }
        });
    });
}

if (require.main === module) {

  app.use(express.static(__dirname + '/public'));



    http.listen(3000, function () {
    console.log('listening on *:3000');
  });

  // Invoke Main Function
  userInterface();
}
