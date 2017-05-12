'use strict'

const _ = require('underscore')
const request = require('request')

// const simulationData = {srcround: resourceName}

// Basic version of an event queue that dispatches events
// by doing PUT to a configured route.
// It will GET the latest status from remote server, extend it
// with the current event, and finally PUT that updated status
// to the remote route.

// when false, log output is suppressed
exports.showLog = true

// log to console
// @s string to log
function log(s) {
  if (exports.showLog) {
    console.log('events> ', s)
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
  if (!route) {
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
  // GET current server status
  log("About to GET", remoteRoute)
  request({
    url: remoteRoute,
    json: true,
    method: 'GET',
    headers: { 'authorization': token }
  }, function (err, resp, body) {
    if (err) {
      log("Error trying to GET: " + remoteRoute, err)
      cb(err)
      return
    }
    const data = body.result.data;
    log("Got body", body)
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
      log("Event successfully sent")
      eventsQueue.shift()
      cb(null, eventsQueue.length)
    })
  }
  )
}

exports.init = init
exports.emit = emit