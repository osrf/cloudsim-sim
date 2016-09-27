'use strict'

const express = require('express')
const csgrant = require('cloudsim-grant')
const state_machine = require('./state_machine.js')
const ansi_to_html = require('ansi-to-html')
const ansi2html = new ansi_to_html()
const spawn = require('child_process').spawn

// when false, log output is suppressed
exports.showLog = false

// log to console
// @s string to log
function log(s) {
  if (exports.showLog) {
    console.log('fsm> ', s)
  }
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

// create a state machine, and add data and methods to it
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
    simId: null,
    sim: null,
    userName: null
  }

// helper function to get the userName from the resource
function getSimUser(simulation) {
  const permissions = simulation.sim.permissions
  // find a user with read/write permission
  for (let user in permissions) {
    if (!permissions[user].readOnly) {
      return user
      break
    }
  }
  const simId = simulation.id
  throw 'simulation "' + simId + '" has invalid permissions'
}

// machina.js state machine don't have constructor, so
// there's a boot state for that
proc.bootStateMachine = function() {
  log("Make sure no simulation is in 'RUNNING' state")
  const split = splitSimulations()
  if (split.running) {
    const simId = split.running.id
    const simData = split.running.sim.data
    const userName = getSimUser(split.running)
    log('RUNNNNNING', userName, simId)
    // set the state to "FINISHED"
    simData.stat = 'FINISHED'
    csgrant.updateResource(userName, simId, simData, function(){
      log('sim "' + simId  + '" is FINISHED')
    })
  }
  // killall gz-server? roslaunch?
  log('state machine booted')
}

// Called before the simulation starts. This
// is for things like setting up latency, call home
proc.getReadyToRunSimulator = function() {
  log('before sim run')
}

// Called to start the simulator
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
  const userName = getSimUser(this.schedulerData.sim)
  // keep it for later
  this.schedulerData.userName = userName
  // isolate the cmd
  const cmdLine = simData.cmd
  // split the program name from its arguments
  const items = cmdLine.split(' ')
  const procName = items[0]
  const args = items.slice(1)
  log('spwaning: ' + procName + ' ' + args)
  this.schedulerData.proc = spawn(procName, args, {stdio:'pipe'})

  // set the state to "running"
  simData.stat = 'RUNNING'
  csgrant.updateResource(userName, simId, simData, function(){
    log('sim "' + simId  + '" running')
  })
  // transform ascii terminal output into colored html
  var colorize = function (buf) {
    const txt = buf.toString()
    // replace new lines with html line breaks
    const html = txt.split('\n').join('<br>')
    // convert the color codes to html
    //   ex: "[0m[1;31m:[0m[1;31m96[0m[1;31m] [0m[1;31m"
    const ansi = ansi2html.toHtml(html)
    return ansi
  }
  // when new text is sent to std out
  this.schedulerData.proc.stdout.on('data', (data)=> {
    const newData = colorize(data)
    // add text to the output
    simData.output += newData
    csgrant.updateResource(userName, simId, simData, function(){
      log('sim "' + simId  + 'data:' + newData )
     })

  })
  // when new text is sent to std err
  this.schedulerData.proc.stderr.on('data', (data)=> {
     const newData = colorize(data)
     simData.output += newData
     csgrant.updateResource(userName, simId, simData, function(){
      log('sim "' + simId  + 'data:' + newData )
     })
  })
  //
  this.schedulerData.proc.on('close', (code)=>{
    log('simulation process has terminated')
    // turn the state machine into stopping action
    this.stop()
  })
}

// Simulation process is commanded to stop, or the process ended
// for another reason
proc.stopTheSimulator = function() {
  log("proc.stopTheSimulator")
  // if necessary, stop the simulator
  if(!this.schedulerData.proc.exitCode) {
    this.schedulerData.proc.kill()
  }
  const simData = this.schedulerData.sim.sim.data
  // set the state to "running"
  simData.stat = 'FINISHED'
  const userName = this.schedulerData.userName
  csgrant.updateResource(userName, simId, simData, function(){
    log('sim "' + simId  + '" finished')
  })
}

// After the simulation, send the logs
proc.sendLogs = function() {
  log('Todo: send logs after execution',
    'state', proc.state,
    'priorState', proc.priorState)

  log('clean up after run')
  this.schedulerData.proc = null
  this.schedulerData.simId = null
  this.schedulerData.sim = null
}

// starts the periodic scheduler update that launches simulations
// this should only be called once
function startSimulationsScheduler(simulationsSchedulerInterval) {
  log('Starting simulations scheduler. Interval:',
              simulationsSchedulerInterval)

  // start the state machine
  proc.boot()
  try {
    schedulerUpdate()
  }
  catch(e) {
    const d = new Date()
    log(d,'simulation scheduler update error:', e)
  }

  proc.schedulerData.interval = setInterval(schedulerUpdate,
                                  simulationsSchedulerInterval)
}

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
  // increment counter
  // proc.schedulerData.counter += 1
  // log(proc.state, proc.schedulerData.counter)
}

// clean up... this should only be called once (end of process)
function stopSimulationScheduler() {
  log('stopping simulation scheduler')
  if (proc.schedulerData.interval) {
    clearInterval(proc.schedulerData.interval)
  }
}

// set the simulations urls for the server
function setRoutes(app) {
  log('SIMULATIONS setRoutes')
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
}

// list of exported functions in this module
exports.startSimulationsScheduler = startSimulationsScheduler
exports.stopSimulationScheduler = stopSimulationScheduler
exports.setRoutes = setRoutes
