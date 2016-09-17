'use strict'

const express = require('express')
const csgrant = require('cloudsim-grant')
const state_machine = require('./state_machine.js')

const router = express.Router()

const proc = state_machine.createMachine()

// This is the data used in the simulation scheduler state machine
// proc contains the process handle to the current simulation,
// (gzserver process or roslaunch process), or null when no simulation
// is running.
// output is the process stdout
// counter simply increments
// state is the state of the scheduler
proc.schedulerData = {
    interval: null,
    counter: 0,
    proc: null,
    output: '',
    state: 'ready',
    task: ''
  }

proc.boot()

proc.on("transition", function (data) {
  console.log('action:', '"' + data.action + '"',
              'state:', '"' + this.state + '"',
              'prior:', '"' + this.priorState + '"')

  // action: "nothing.boot" state: "ready" prior: "nothing"
  if (data.action === "nothing.boot") {

  }
  // action: "ready.start" state: "prerun" prior: "ready"
  else if (data.action  === "ready.start") {
    // setup (latency)
  }
  // action: "prerun.run" state: "running" prior: "prerun"
  else if (data.action === "prerun.run") {
    // start the simulator
  }
  // action: "running.stop" state: "postrun" prior: "running"
  else if (data.action === "running.stop") {
    // if necessary, stop the simulator
  }
  // action: "postrun.done" state: "ready" prior: "postrun"
  else if (data.action === "postrun.done") {
    // send the logs to the server
  }
  // action: "postrun.done" state: "ready" prior: "postrun"
  else if(data.action === "postrun.done") {
    // nothing to do (phone home?)
  }
  // oops ... this is not a state we expected
  else {
    throw 'state "' + this.state + '" is not recognized'
  }
})

// This function is called periodically. If no simulation is running,
// it looks for the next available one
function schedulerUpdate() {

  // Is a process running?
  if (!proc.schedulerData.proc) {
    const sims = getSimulationQ()
    let nextTask = null
    for(var i=0; i< sims.lenght; i++) {
      const currentTask = sims[i]
      if (currentTask.stat == 'waiting') {
        nextTask = currentTask
        break
      }
    }
    // have we found a new task?
    if(nextTask && nextTask !== proc.schedulerData.task){
      if (nexTask.auto == true) {
        proc.schedulerData.task = nextTask
        console.log(proc.schedulerData.counter,
          proc.schedulerData.task,
          'task ready', JSON.stringify(proc.schedulerData.task, null, 2)
        )
      }
    }
  }
  // increment counter for fun
  proc.schedulerData.counter += 1
}

// starts the periodic scheduler update that launches simulations
function startSimulationsScheduler(simulationsSchedulerInterval) {
  console.log('Starting simulations scheduler. Interval:', simulationsSchedulerInterval)
  try {
    schedulerUpdate()
  }
  catch(e) {
    const d = new Date()
    console.log(d,'simulation scheduler update error:', e)
  }

  proc.schedulerData.interval = setInterval(schedulerUpdate, simulationsSchedulerInterval)
}

function stopSimulationScheduler() {
  console.log('stopping simulation scheduler')
  if (proc.schedulerData.interval) {
    clearInterval(proc.schedulerData.interval)
  }
}


function setRoutes(app) {

  console.log('SIMULATIONS setRoutes')

  // list all resources
  app.get('/simulations',
    csgrant.authenticate,
    csgrant.ownsResource('simulation_list', true),
    csgrant.userResources,
    csgrant.allResources)

  app.get('/simulations/:simId',
    csgrant.authenticate,
    csgrant.ownsResource(':simId', true),
    csgrant.resource)

  // create a new simulation
  app.post('/simulations',
           csgrant.authenticate,
           csgrant.ownsResource('simulation_list', false),
           function(req, res) {

    console.log('create sim:')
    console.log('  body:' +  JSON.stringify(req.body))
    console.log('  query:' + JSON.stringify(req.query))

    const data = req.query
    const resourceData = { cmd: data.cmd,
                           auto: data.auto,
                           stat:'WAITING'
                          }
    const error = function(msg) {
      return {operation: 'createSimulation',
              success: false,
              error: msg}
    }
    const op = 'createSimulation'
    const user = req.user
    const r = {success: false}
    csgrant.getNextResourceId('sim', (err, resourceName) => {
      if(err) {
        res.jsonp(error(err))
        return
      }
      // step 4. add simulation
      csgrant.createResource(user, resourceName, resourceData,
                            (err, data) => {
        if(err) {
          res.jsonp(error(err))
          return
        }
        // step 5. success!
        const r = { success: true,
                    operation: op,
                    result: data,
                    id: resourceName}
        res.jsonp(r)
      })
    })
  })

  // Update a simulation
  app.put('/simulations/:simId',
          csgrant.authenticate,
          csgrant.ownsResource(':simId', true),
          function(req, res) {

    const resourceName = req.simId
    const newData = req.query
    console.log(' Update simulation: ' + resourceName)
    console.log(' new data: ' + JSON.stringify(newData))
    const user = req.user

    const r = {success: false}
    if (!newData.cmd) {
       return res.jsonp({success: false, error: 'invalid new simulation: missing cmd'})
    }
    if (!newData.auto) {
       return res.jsonp({success: false, error: 'invalid new simulation: missing auto'})
    }

    csgrant.readResource(user, resourceName, function(err, oldData) {
      if(err)
        return res.jsonp({success: false, error: 'error trying to read existing data: ' + err})
      const futureData = oldData.data
      // merge with existing fields of the newData... thus keeping old fields intact
      for (var attrname in newData) { futureData[attrname] = newData[attrname] }
      csgrant.updateResource(user, resourceName, futureData, (err, data) => {
        if(err) {
          return res.jsonp({success: false, error: err})
        }
        r.success = true
        r.result = data
        // success
        res.jsonp(r)
      })

    })

  })

  // Delete a simulation
  app.delete('/simulations/:simId',
             csgrant.authenticate,
             csgrant.ownsResource(':simId', false),
             function(req, res) {
    console.log('delete simulation ' + req.simId)
    const resourceName = req.simId
    const r = {success: false}
    const user = req.user  // from previous middleware
    const resource = req.simId // from app.param (see below)
    csgrant.deleteResource(user, resource, (err, data) => {
      if(err) {
        return res.jsonp({success: false, error: err})
      }
      r.success = true
      r.result = data
      // success
      res.jsonp(r)
    })
  })

  // simId
  app.param('simId', function( req, res, next, id) {
    req.simId = id
    next()
  })

  // This is the route to start a simulation (when no sim is running)
  app.get('/start',
          csgrant.authenticate,
          csgrant.ownsResource('start_stop_process', false),
          function(req, res) {
    console.log('START Next cmd!!!')
  })

  // This is the route to stop a simulation (before starting a new one)
  app.get('/stop',
          csgrant.authenticate,
          csgrant.ownsResource('start_stop_processe', false),
          function(req, res) {
    console.log('STOP executing cmd!!!')
  })
}


// this function looks into all resources (for all users), and
// returns only the simulations
function getSimulationQ() {
  const db = csgrant.copyInternalDatabase()
  const keys = Object.keys(db)
  keys.sort()
  const simQ =[]
  for (let resource in keys) {
    // is it a simulation task?
    if (resource.indexOf('sim') == 0) {
      simQ.push(db[resource])
    }
  }
  // sort according to position in the queue
  // todo
  return simQ
}

// list of exported functions in this module
exports.startSimulationsScheduler = startSimulationsScheduler
exports.stopSimulationScheduler = stopSimulationScheduler
exports.setRoutes = setRoutes
