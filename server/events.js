'use strict'

const _ = require('underscore')
const request = require('request')
const csgrant = require('cloudsim-grant')

// Basic version of an event queue that dispatches events
// by doing PUT to a configured route.

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
let parentProperty

// route: remote server that will accept our events
// userToken: needed authorization header
// parentProp: (optional). If we want to wrap events in the
// form: { <parentProp>: <event object> }. If parentProp is not
// set, then the event will be sent as is.
function init(route, userToken, parentProp) {
  if (!route || route == 'undefined') {
    return
  }
  remoteRoute = route
  token = userToken
  log("Received a valid remoteRoute. Events module initialized", remoteRoute)
  initialized = true
  parentProperty = parentProp
}

function emit(event) {
  log("Emit event", JSON.stringify(event))
  if (!initialized) {
    return
  }
  if (parentProperty) {
    const wrapper = {}
    wrapper[parentProperty] = event
    event = wrapper
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

// set the event url for the server.
// NOTE: We request same permission as for simulations resource.
function setRoutes(app) {
  log('EVENTS setRoutes')
  // list all simulations for the user
  app.get('/events',
    csgrant.authenticate,
    csgrant.ownsResource('simulations', false),
    function(req, res) {
      const r = {
        success: true,
        operation: "get pending events",
        result: eventsQueue,
        requester: req.user
      }
      res.jsonp(r)
    }
  )
  // post a new event to be sent to portal
  app.post('/events',
    csgrant.authenticate,
    csgrant.ownsResource('simulations', false),
    function(req, res) {
      emit(req.body)
      const r = {
        success: true,
        operation: "posted event",
        requester: req.user
      }
      res.jsonp(r)
    }
  )
}

exports.init = init
exports.emit = emit
exports.setRoutes = setRoutes
