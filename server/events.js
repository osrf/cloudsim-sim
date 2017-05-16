'use strict'

const _ = require('underscore')
const request = require('request')

// Basic version of an event queue that dispatches events
// by doing PUT to a configured route.
// It will GET the latest status from remote server, extend it
// with the current event, and finally PUT that updated status
// to the remote route.

// when false, log output is suppressed
exports.showLog = false

// log to console
function log(str, o) {
  if (exports.showLog) {
    if (o) {
      console.log('events> ' + str, o)
    } else {
      console.log('events> ', str)
    }
  }
}

let sendEventInterval = 500 
if (process.env.NODE_ENV === 'test') {
  sendEventInterval = 1
}


const eventsQueue = []
let initialized = false

let remoteRoute
let token


function init(route, userToken) {
  if (!route || route == 'undefined') {
    return
  }
  remoteRoute = route
  token = userToken
  log("Received a valid remoteRoute. Events module initialized", remoteRoute)
  initialized = true
}

function emit(event) {
  log("Emit event", JSON.stringify(event))
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
    running = false
    if (err || remaining) {
      log("About to create new timeout with delay:", sendEventInterval)
      timeoutId = setTimeout(() => {
        timeoutId = undefined
        sendEvents()
      }, sendEventInterval)
    }
  })
}

function sendNextEvent(cb) {
  log("Events queue. Current size", eventsQueue.length)
  if (!eventsQueue.length) {
    cb(null, 0)
    return
  }
  const ev = _.first(eventsQueue)
  request({
    url: remoteRoute,
    json: true,
    body: ev,
    method: 'PUT',
    headers: { 'authorization': token }
  }, function (err) {
    if (err) {
      cb(err)
      return
    }
    // Success. Remove event from queue
    log("Event successfully sent")
    eventsQueue.shift()
    cb(null, eventsQueue.length)
  })
}

exports.init = init
exports.emit = emit