'use strict'

const express = require('express')
const csgrant = require('cloudsim-grant')
const state_machine = require('./state_machine.js')
const ansi_to_html = require('ansi-to-html')

const ansi2html = new ansi_to_html()
const spawn = require('child_process').spawn
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
    simId: null,
    simData: null
  }

// machina.js state machine don't have constructor, so
// there's a boot state for that
proc.bootStateMachine = function() {
  console.log("Make sure no simulation is in 'RUNNING' state")
  console.log(" Make sure no simulator process is running")
  console.log(' Looking good', '\n\n')
}


proc.getReadyTorunSimulator = function() {
  console.log('before sim run')
}

proc.startTheSimulator =  function() {
  // has a sim been selected?
  if (!this.schedulerData.sim) {
    console.warn('False start')
    return
  }
  // extract simulation information into its components..
  // the simulation id (simulation-33)
  const simId = this.schedulerData.sim.id
  // the simulation data (cmd, auto ...)
  const simData = this.schedulerData.sim.sim.data
  const permissions = this.schedulerData.sim.sim.permissions
  // find a user with read/write permission
  let userName = null
  for (let user in permissions) {
    if (!permissions[user].readOnly) {
      userName = user
      break
    }
  }
  if (!userName) {
    throw 'simulation "' + simId + '" has invalid permissions'
  }
  // isolate the cmd
  const cmdLine = simData.cmd
  // split the program name from its arguments
  const items = cmdLine.split(' ')
  const procName = items[0]
  const args = items.slice(1)
  console.log('spwaning: ' + procName + ' ' + args)
  this.schedulerData.proc = spawn(procName, args, {stdio:'pipe'})

  // set the state to "running"
  simData.stat = 'RUNNING'
  csgrant.updateResource(userName, simId, simData, function(){
    console.log('sim "' + simId  + '" running')
  })

  var colorize = function (buf) {
    const txt = buf.toString()
    // replace new lines with html line breaks
    const html = txt.split('\n').join('<br>')
    // convert the console color codes to html
    //   ex: "[0m[1;31m:[0m[1;31m96[0m[1;31m] [0m[1;31m"
    const ansi = ansi2html.toHtml(html)
    return ansi
  }

  this.schedulerData.proc.stdout.on('data', (data)=> {
    const newData = colorize(data)
  })

  this.schedulerData.proc.stderr.on('data', (data)=> {
    const newData = colorize(data)
  })

  this.schedulerData.proc.on('close', (code)=>{
    console.log('simulation process has terminated')
    // turn the state machine into stopping action
    this.stop()
  })
}

proc.stopTheSimulator = function() {
  console.log("proc.stopTheSimulator")
  // if necessary, stop the simulator
  if(!this.schedulerData.proc.exitCode) {
    this.schedulerData.proc.kill()
  }
}

proc.sendLogs = function() {
  console.log('Todo: send logs after execution',
    'state', proc.state,
    'priorState', proc.priorState)
}

proc.cleanUpAfterRun = function() {
  console.log('clean up after run')
  this.schedulerData.proc = null
  this.schedulerData.simId = null
  this.schedulerData.simData = null
}

// This is state machine state changes callback.
// This is where simulators are started and stopped
proc.on("transition", function (data) {
  // action: "nothing.boot" state: "ready" prior: "nothing"
  if (data.action === "nothing.boot") {
    this.bootStateMachine()
    console.log('state machine ready')
  }
  // action: "ready.start" state: "prerun" prior: "ready"
  else if (data.action  === "ready.start") {
    // setup (latency)
    this.getReadyTorunSimulator()
    console.log('Pre simulation run setup')
  }
  // action: "prerun.run" state: "running" prior: "prerun"
  else if (data.action === "prerun.run") {
    // start the simulator
    this.startTheSimulator()
    console.log('simulator started')
  }

  // action: "running.stop" state: "postrun" prior: "running"
  else if (data.action === "running.stop") {
    this.stopTheSimulator()
    console.log('Simulation stopped')
  }
  // action: "postrun.done" state: "ready" prior: "postrun"
  else if (data.action === "postrun.done") {
    // send the logs to the server
    this.sendLogs()
    console.log('logs sent')
  }
  // action: "postrun.done" state: "ready" prior: "postrun"
  else if(data.action === "postrun.done") {
    // clean up
    this.cleanUpAfterRun()
  }
  // oops ... this is not a state we expected
  else {
    throw 'state "' + this.state + '" is not recognized'
  }
})

// This function is called periodically. If no simulation is running,
// it looks for the next available one
function schedulerUpdate() {
  // Is it time to start a simulation?
  if(proc.state === "ready") {
    const split = splitSimulations()
    if (split.ready) {
      proc.schedulerData.sim = split.ready
      proc.start()
    }
  }
  // increment counter for fun
  proc.schedulerData.counter += 1
  // console.log(proc.schedulerData.counter)
}

// starts the periodic scheduler update that launches simulations
function startSimulationsScheduler(simulationsSchedulerInterval) {
  console.log('Starting simulations scheduler. Interval:',
              simulationsSchedulerInterval)
  try {
    schedulerUpdate()
  }
  catch(e) {
    const d = new Date()
    console.log(d,'simulation scheduler update error:', e)
  }

  proc.schedulerData.interval = setInterval(schedulerUpdate,
                                  simulationsSchedulerInterval)
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
    csgrant.ownsResource('simulations', true),
    csgrant.userResources,

    function(req, res, next) {
      // we're going to filter out the non
      // simulation before the next middleware.
      req.allResources = req.userResources
      req.userResources = req.allResources.filter( (obj)=>{
        if(obj.name.indexOf('simulation-') == 0)
          return true
        return false
      })
      next()
    },

    csgrant.allResources)

  app.get('/simulations/:simId',
    csgrant.authenticate,
    csgrant.ownsResource(':simId', true),
    csgrant.resource)

  // create a new simulation
  app.post('/simulations',
           csgrant.authenticate,
           csgrant.ownsResource('simulations', false),
           function(req, res) {

    const data = req.body
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
    csgrant.getNextResourceId('simulation', (err, resourceName) => {
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
    const newData = req.body
    console.log(' Update simulation: ' + resourceName)
    console.log(' new data: ' + JSON.stringify(newData))
    const user = req.user
    const r = {success: false}

    if (!newData.cmd) {
       return res.jsonp({success: false,
                         error: 'invalid new simulation: missing cmd'})
    }

    if (!newData.auto) {
       return res.jsonp({success: false,
                         error: 'invalid new simulation: missing auto'})
    }

    csgrant.readResource(user, resourceName, function(err, oldData) {
      if(err)
        return res.jsonp({success: false,
                          error: 'error trying to read existing data: ' + err})
      const futureData = oldData.data
      // merge with existing fields of the newData...
      // thus keeping old fields intact
      for (var attrname in newData) {futureData[attrname] = newData[attrname]}
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
  // This is the route to stop a simulation (before starting a new one)
  app.get('/stopsimulation',
          csgrant.authenticate,
          csgrant.ownsResource('simulations', false),
          function(req, res) {
    console.log('STOP received', proc.state)
    proc.stop()
    console.log('STOP done', proc.state, '(', proc.priorState, ')')
    res.jsonp({state: proc.state,
               prior: proc.priorState
              })
  })
}

// this function looks into all resources (for all users), and
// returns the simulations
function splitSimulations() {
  const db = csgrant.copyInternalDatabase()
  const keys = Object.keys(db)
  keys.sort()

  const sims =[]
  for (let i in keys) {
    const resource = keys[i]
    // is it a simulation task?
    if (resource.indexOf('simulation-') == 0) {
      sims.push({id: resource, sim: db[resource]})
    }
  }
  const split = {finished : [],
                 running: null,
                 ready: null,
                 waiting: []}

  // sort according to position in the queue
  //   todo
  // get next waiting sim and return it if it is auto
  // find next available task (hasn't run already)
  // skip the terminated
  for (let i in sims) {
    const state = sims[i].sim.data.stat
    if(state === "FINISHED"){
      split.finished.push(sims[i])
      continue
    }
    if (state === "RUNNING") {
      // state indicates a running sim. But this may not be
      split.running = sims[i]
      continue
    }
    if(state === "WAITING") {
      if (!split.ready) {
        if (sims[i].sim.data.auto) {
          split.ready = sims[i]
          continue
        }
      }
      split.waiting.push(sims[i])
    }
  }
  return split
}

// start the state machine
proc.boot()

// list of exported functions in this module
exports.startSimulationsScheduler = startSimulationsScheduler
exports.stopSimulationScheduler = stopSimulationScheduler
exports.setRoutes = setRoutes
