'use strict'

const _ = require('underscore')
const request = require('request')

// const simulationData = {srcround: resourceName}

// Basic version of an event queue that dispatches events
// by doing PUT to a configured route.
// It will GET the latest status from remote server, extend it
// with the current event, and finally PUT that updated status
// to the remote route.
let sendEventInterval = 500 
if (process.env.NODE_ENV === 'test') {
  sendEventInterval = 1
}


const eventsQueue = []
let initialized = false

let remoteRoute
let token


function init(route, userToken) {
  if (!route) {
    return
  }
  remoteRoute = route
  token = userToken
  console.log("EVENTS INITIALIZED")
  initialized = true
}

function emit(event) {
  console.log("Event EMITTED", JSON.stringify(event))
  console.log("Event EMITTED intialized?", initialized)
  if (!initialized) {
    return
  }
  eventsQueue.push(event)
  sendEvents()
}


let timeoutId
let running
function sendEvents() {
  if (timeoutId || running) {
    // already running or about to run. Return
    return
  }
  running = true
  sendNextEvent((err, remaining) => {
    console.log("YA SALI DEL SENDNEXTEVENT")
    running = false
    if (err || remaining) {
      console.log("ABOUT TO CREATE TIMEOUT", sendEventInterval)
      timeoutId = setTimeout(() => {
        timeoutId = undefined
        sendEvents()
      }, sendEventInterval)
    }
  })
}

function sendNextEvent(cb) {
  console.log("PEPPEEEEEEEE", eventsQueue.length)
  if (!eventsQueue.length) {
    cb(null, 0)
    return
  }
  const ev = _.first(eventsQueue)
  // GET current server status
  console.log("ABOUT TO SEND GET", remoteRoute)
  request({
    url: remoteRoute,
    json: true,
    method: 'GET',
    headers: { 'authorization': token }
  }, function (err, resp, body) {
    if (err) {
     console.log("ERRORRRRRRRRRRRRRRRR", err) 
      cb(err)
      return
    }
    const data = JSON.parse(body);
    console.log("PATOOOOOO", body)
    _.extend(data, ev)
    // PUT updated data
    request({
      url: remoteRoute,
      json: true,
      body: data,
      method: 'PUT',
      headers: { 'authorization': token }
    }, function (putErr, putRes) {
      if (putErr) {
        cb(putErr)
        return
      }
      // Success. Remove event from queue
      console.log("PATOOOOOO PUTTTTT")
      eventsQueue.shift()
      cb(null, eventsQueue.length)
    })
  }
  )
}

exports.init = init
exports.emit = emit