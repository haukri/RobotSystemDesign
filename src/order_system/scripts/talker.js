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
/**
 * This example demonstrates simple sending of messages over the ROS system.
 */

// Require rosnodejs itself
const rosnodejs = require('rosnodejs');
const request = require('request');
// Requires the std_msgs message package
const std_msgs = rosnodejs.require('std_msgs').msg;

function talker() {
  // Register node with ROS master
  rosnodejs.initNode('/talker_node')
    .then((rosNode) => {
      // Create ROS publisher on the 'chatter' topic with String message
      let pub = rosNode.advertise('/chatter', std_msgs.String);
      let count = 0;
      const msg = new std_msgs.String();
      // Define a function to execute every 100ms
      /*setInterval(() => {
        // Construct the message
        msg.data = 'hello world ' + count;
        // Publish over ROS
        pub.publish(msg);
        // Log through stdout and /rosout
        rosnodejs.log.info('I said: [' + msg.data + ']');
        ++count;
      }, 10000);*/

      const service = rosNode.advertiseService('/new_order', 'order_msgs/NewOrder', (req, res) => {

        const options = {
          url: 'https://www.reddit.com/r/funny.json',
          method: 'GET',
          headers: {
            'Accept': 'application/json',
            'Accept-Charset': 'utf-8',
            'User-Agent': 'my-reddit-client'
          }
        };
        
        request(options, function(err, res, body) {
          let json = JSON.parse(body);
          console.log(json);
        });

        res.order_number = Math.round(Math.random()*1000);
        return true;
      });

    });
}

if (require.main === module) {
  // Invoke Main Talker Function
  talker();
}
