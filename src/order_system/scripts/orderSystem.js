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
const storage = require('node-persist');

var lastPackmlState = 0;

var orderStatusPub;
var lastLogEvent = "";

var ipAddress = "10.10.19.20";
//var ipAddress = "127.0.0.1:5000";

var selectedOrder;
var smallestOrderAmount = 1000;

async function orderSystem() {

  await storage.init();

  // Register node with ROS master
  rosnodejs.initNode('/order_system_node')
    .then((rosNode) => {

      const service = rosNode.advertiseService('/new_order', 'order_msgs/NewOrder', (req, res) => {
        return new Promise(async function(resolve, reject) {
          getNewOrder((newOrder) => {
            orderStatusPub.publish({ order_number: newOrder.id, yellow_amount: newOrder.yellow, red_amount: newOrder.red, blue_amount: newOrder.blue });
            console.log('Got new order from server');
            console.log(newOrder);
            res.order_number = newOrder.id;
            res.yellow_amount = newOrder.yellow;
            res.red_amount = newOrder.red;
            res.blue_amount = newOrder.blue;
            postLog(10, 'Order_Start', 'order ' + newOrder.id);
            resolve(true);
          });
        });
      });

      const completeService = rosNode.advertiseService('/complete_order', 'order_msgs/CompleteOrder', (req, res) => {
        return new Promise(function(resolve, reject) {
          completeOrder(req.order_number, (orderCompleted) => {
            if(orderCompleted) {
              console.log('Order completed');
              res.success = true;
              postLog(10, 'Order_Done', 'order ' + req.order_number);
              resolve(true);
            }
            else {
              console.log('Order could not be completed');
              res.success = false;
              resolve(true);
            }
          })
        });
      });

      orderStatusPub = rosNode.advertise('/order_status', 'order_msgs/OrderStatus');

      rosNode.subscribe('/packml_node/packml/status', 'packml_msgs/Status', (msg) => {
        if(packMLStates[msg.state.val] && msg.state.val != lastPackmlState) {
          postLog(10, packMLStates[msg.state.val], '');
        }
        lastPackmlState = msg.state.val;
      });

    });
}

async function completeOrder(orderID, callback) {
  var currentOrder = await storage.getItem(String(orderID));
  if(currentOrder) {
    var options = {
      url: 'http://' + ipAddress + '/orders/' + String(orderID) + '/' + currentOrder.ticket,
      method: 'DELETE',
      //headers: {
      //  'Accept': 'application/json',
      //  'Accept-Charset': 'utf-8'
      //}
    };
    callback(true);
    request(options, async function(err, res, body) {
      if(res.statusCode === 204) {
        callback(true);
      }
      else {
        setTimeout(async () => {
          if(currentOrder.maxDeleteAttempts > 0) {
            currentOrder.maxDeleteAttempts -= 1;
            await storage.setItem(String(currentOrder.id), currentOrder);
            console.log('ERROR: Could not delete order, trying again, attempts left: ' + currentOrder.maxDeleteAttempts);
            completeOrder(orderID, callback);
          }
          else {
            console.log('ERROR: Could not delete order and no attempts left');
            callback(false);
          }
        }, 500);
      }
    });
  }
  else {
    console.log('ERROR: The ticket for this order can not be found');
    callback(false);
  }

}

async function sleep(duration) {
  return new Promise(resolve => setTimeout(resolve, duration));
} 

function getNewOrder(callback) {
  var options = {
    url: 'http://' + ipAddress +'/orders',
    method: 'GET',
    //headers: {
    //  'Accept': 'application/json',
    //  'Accept-Charset': 'utf-8'
    //}
  };
  
  request(options, async function(err, res, body) {
    let json = JSON.parse(body);     
    if(json.orders && json.orders.length > 0) {
      var readyOrders = json.orders.filter((order) => {
        return order.status === 'ready';
      })
      while(readyOrders.length > 0) {
        readyOrders.forEach(orderInfo => {
          if((orderInfo.blue + orderInfo.yellow + orderInfo.red) < smallestOrderAmount) {
            smallestOrderAmount = orderInfo.blue + orderInfo.yellow + orderInfo.red;
            selectedOrder = orderInfo;
          }
        });
        // var selectedOrder = readyOrders[Math.floor(Math.random()*readyOrders.length)];
        if(await requestOrder(selectedOrder)) {
          callback(selectedOrder);
          break;
        }
        else {
          readyOrders = readyOrders.filter((order) => {
            return order.id != selectedOrder.id;
          })
          smallestOrderAmount = 1000;
          console.log(readyOrders.length);
          console.log("Try to get a new order");
          await sleep(200);
        }
      }
    }
    else {
      console.log('ERROR: No orders');
      setTimeout(getNewOrder(callback), 500);
    }
  });
}

async function requestOrder(order)  {
  return new Promise((resolve, reject) => {
    let options = {
      url: 'http://' + ipAddress + '/orders/' + String(order.id),
      method: 'PUT',
    };
    request(options, async function(err, res, body) {
      let json = JSON.parse(body);
      if(json.ticket) {
        var storedOrder = {};
        storedOrder.id = order.id;
        storedOrder.blue = order.blue;
        storedOrder.red = order.red;
        storedOrder.yellow = order.yellow;
        storedOrder.ticket = json.ticket;
        storedOrder.maxDeleteAttempts = 10;
        await storage.setItem(String(order.id), storedOrder);
        resolve(true);
      }
      else {
        console.log('ERROR: The requested order is already taken');
        resolve(false);
      }
    })
  })
}

function postLog(cell_id, event, comment) {
  if(event != lastLogEvent) {
    request.post({
      url: 'http://' + ipAddress + '/log',
      json: true,
      body: {
        cell_id: cell_id,
        event: event,
        comment: comment
      }
    }, function(error, response, body){
      if(response && response.statusCode == 201) {
        console.log('Event logged');
      }
    });
    lastLogEvent = event;
  }

}

var packMLStates = {};
packMLStates[2] = 'PML_Stopped'
packMLStates[4] = 'PML_Idle'
packMLStates[5] = 'PML_Suspended'
packMLStates[6] = 'PML_Execute'
packMLStates[9] = 'PML_Aborted'
packMLStates[11] = 'PML_Held'
packMLStates[106] = 'PML_Complete'

if (require.main === module) {
  // Invoke Main Function
  orderSystem();
}

