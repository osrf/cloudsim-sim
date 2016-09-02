'use strict'

const express = require('express')
const router = express.Router()
const csgrant = require('cloudsim-grant')


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


// This is the data used in the simulation scheduler state machine
// proc contains the process handle to the current simulation,
// (gzserver process or roslaunch process), or null when no simulation
// is running.
// output is the process stdout
// counter simply increments
// state is the state of the scheduler
const schedulerData = {
    interval: null,
    counter: 0,
    proc: null,
    output: '',
    state: 'ready',
    task: ''
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

// This function is called periodically. If no simulation is running,
// it looks for the next available one
function schedulerUpdate() {

  // Is a process running?
  if (!schedulerData.proc) {
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
    if(nextTask && nextTask !== schedulerData.task){
      if (nexTask.auto == true) {
        schedulerData.task = nextTask
        console.log(schedulerData.counter,
          schedulerData.task,
          'task ready', JSON.stringify(schedulerData.task, null, 2)
        )
      }
    }
  }
  // increment counter for fun
  schedulerData.counter += 1
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

  schedulerData.interval = setInterval(schedulerUpdate, simulationsSchedulerInterval)
}

function stopSimulationScheduler() {
  console.log('stopping simulation scheduler')
  if (schedulerData.interval) {
    clearInterval(schedulerData.interval)
  }
}

// list of exported functions in this module
exports.startSimulationsScheduler = startSimulationsScheduler
exports.stopSimulationScheduler = stopSimulationScheduler
exports.setRoutes = setRoutes
