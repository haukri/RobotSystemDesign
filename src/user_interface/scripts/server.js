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

var feederPub, setCycleTimePub;

var ordersPerHour = 40;

var packMLStatus = {}
var orderStatus;
var mainControlStatus;
var feederEmpty = false;
var packMLTransitionService;

function userInterface() {

  // Register node with ROS master
  rosnodejs.initNode('/user_interface_node')
    .then((rosNode) => {
      io.on('connection', function (socket) {
        console.log('a user connected');
        if (packMLStatus) {
          io.sockets.emit('packml_status', packMLStatus);
        }
        if (orderStatus) {
          io.sockets.emit('order_status', orderStatus);
        }
        if (mainControlStatus) {
          io.sockets.emit('main_control_status', mainControlStatus);
        }
        io.sockets.emit('feeder_empty', feederEmpty);
        io.sockets.emit('orders_per_hour', ordersPerHour);

        socket.on('packml_command', message => {
          const request = new Transition.Request();
          if (message.command == "start") {
            if (packMLStatus.state.val == 2) {
              request.command = 6;
              console.log('test of reset request')
            }
            else {
              request.command = 2;
              console.log('test else statement')
            }
          }
          else if (message.command == "hold") {
            if (packMLStatus.state.val == 11) {
              request.command = 102;
            }
            else {
              request.command = 4;
            }
          }
          else if (message.command == "stop") {
            if (packMLStatus.state.val == 9) {
              request.command = 1;
            }
            else {
              request.command = 3;
            }
          }
          else if (message.command == "abort") {
            request.command = 5;
          }
          packMLTransitionService.call(request).then((resp) => {
            console.log('packML Transition state successfull')
          })
        });

        socket.on('feeder_full', message => {
          feederEmpty = false;
          feederPub.publish({ data: "" });
        })

        socket.on('set_orders_per_hour', message => {
          setCycleTimePub.publish({ data: message.ordersPerHour });
        })

      });
      const subStatus = rosNode.subscribe('/packml_node/packml/status', 'packml_msgs/Status', (msg) => {
        packMLStatus = msg;
        io.sockets.emit('packml_status', packMLStatus);
      });

      const orderStatusSub = rosNode.subscribe('/order_status', 'order_msgs/OrderStatus', (msg) => {
        orderStatus = msg;
        io.sockets.emit('order_status', orderStatus);
      });

      const mainControlStatusSub = rosNode.subscribe('/main_control_status', 'std_msgs/String', (msg) => {
        mainControlStatus = msg;
        io.sockets.emit('main_control_status', mainControlStatus);
      });

      const feederEmptySub = rosNode.subscribe('/feeder_empty', 'std_msgs/String', (msg) => {
        feederEmpty = true;
        io.sockets.emit('feeder_empty', feederEmpty);
      });

      feederPub = rosNode.advertise('/feeder_full', 'std_msgs/String');

      setCycleTimePub = rosNode.advertise('/set_cycle_time_seconds', 'std_msgs/Float64');

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
