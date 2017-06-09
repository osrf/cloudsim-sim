'use strict'

const csgrant = require('cloudsim-grant')
const state_machine = require('./state_machine')
const ansi_to_html = require('ansi-to-html')
const ansi2html = new ansi_to_html()
const spawn = require('child_process').spawn
const events = require('./events')

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

  // sort according to position in the queue
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
    csgrant.updateResource(userName, simId, simData, (err) => {
      if (err) {
        log("error updating resource simId: " + simId)
      } else {
        log('sim "' + simId  + '" is FINISHED')
      }
    })
  }
  events.emit({ state_machine: 'READY' })
  // killall gz-server? roslaunch?
  log('state machine booted')
}

// Called before the simulation starts. This
// is for things like setting up latency, call home
proc.getReadyToRunSimulator = function() {
  log('before sim run')
}

// transform ascii terminal output into colored html
function colorize(buf) {
  const txt = buf.toString()
  // replace new lines with html line breaks
  if (txt.length == 0)
    return ""
  const html = txt.split('\n').join('<br>')
  // convert the color codes to html
  //   ex: "[0m[1;31m:[0m[1;31m96[0m[1;31m] [0m[1;31m"
  const ansi = ansi2html.toHtml(html)
  // hot fix!!! hack todo superbad
  const x = ansi.replace('undefinedGazebo', 'Gazebo')
  return x
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
  this.schedulerData.proc = this.spawnProcess(simData.cmd)

  // set the state to "running"
  simData.stat = 'RUNNING'
  events.emit({ state_machine: 'RUNNING' })
  csgrant.updateResource(userName, simId, simData, (err) => {
    if (err) {
      log("error updating resource simId: " + simId)
    } else {
      log('sim "' + simId  + '" running')
    }
  })

  // when new text is sent to std out
  this.schedulerData.proc.stdout.on('data', (data)=> {
    const newData = colorize(data)
    // add text to the output
    // simData.output += newData
    csgrant.updateResource(userName, simId, simData, (err) => {
      if (err) {
        log("error updating resource simId: " + simId)
      } else {
        log('sim "' + simId  + 'data:' + newData )
      }
    })
  })
  // when new text is sent to std err
  this.schedulerData.proc.stderr.on('data', (data)=> {
    const newData = colorize(data)
    // simData.output += newData
    csgrant.updateResource(userName, simId, simData, (err) => {
      if (err) {
        log("error updating resource simId: " + simId)
      } else {
        log('sim "' + simId  + 'data:' + newData )
      }
    })
  })
  this.schedulerData.proc.on('close', ()=>{
    log('simulation process has terminated')
    // turn the state machine into stopping action
    this.stop()
  })
}

// spawns a process from the given cmd arg. Returns the process handle
proc.spawnProcess = function(command) {
  // isolate the cmd
  const cmdLine = command
  // split the program name from its arguments
  const items = cmdLine.split(' ')
  const procName = items[0]
  const args = items.slice(1)
  log('spawning: ' + procName + ' ' + args)
  return spawn(procName, args, {stdio:'pipe'})
}

// Simulation process is commanded to stop, or the process ended
// for another reason
// @param done A callback to let caller know the stop has been done
proc.stopTheSimulator = function(done) {
  log("proc.stopTheSimulator")
  // has a sim been selected?
  if (!this.schedulerData.sim) {
    console.warn('No simulation is running')
    return done()
  }

  events.emit({ state_machine: 'FINISHING' })
  const simData = this.schedulerData.sim.sim.data
  const simId = this.schedulerData.sim.id
  const userName = this.schedulerData.userName
  const markSimAsFinished = () => {
    log('marking simulation as finished')
    simData.stat = 'FINISHED'
    events.emit({ state_machine: 'FINISHED' })
    csgrant.updateResource(userName, simId, simData, (err) => {
      if (err) {
        log("error updating resource simId: " + simId)
      } else {
        log('sim "' + simId  + '" finished')
      }
    })
    log('About to invoke done() callback')
    done()
  }

  // do we have a stopCmd? If not then let's just kill the running process  
  if (!simData.stopCmd) {
    log('No stopCmd. Just marking the simulation as finished')
    if (!this.schedulerData.proc.exitCode) {
      this.schedulerData.proc.kill()
    }
    markSimAsFinished()
    return
  }

  // we do have a stopCmd. Let's use it
  log('Found stopCmd. Spawn it')
  this.schedulerData.stopProc = this.spawnProcess(simData.stopCmd)
  // when new text is sent to std out
  this.schedulerData.stopProc.stdout.on('data', (data)=> {
    const newData = colorize(data)
    // add text to the output
    // simData.stopCmdOutput += newData
    csgrant.updateResource(userName, simId, simData, (err) => {
      if (err) {
        log("error updating resource simId: " + simId)
      } else {
        log('sim "' + simId  + 'stop data:' + newData )
      }
    })
  })
  // when new text is sent to std err
  this.schedulerData.stopProc.stderr.on('data', (data)=> {
    const newData = colorize(data)
    // simData.stopCmdOutput += newData
    csgrant.updateResource(userName, simId, simData, (err) => {
      if (err) {
        log("error updating resource simId: " + simId)
      } else {
        log('sim "' + simId  + 'stop data:' + newData )
      }
    })
  })
  // in the case of stop cmd we don't case about stdio, so we just
  // listen to exit event
  this.schedulerData.stopProc.on('exit', markSimAsFinished)
}

// After the simulation, send the logs
// @param done A callback to let caller know the sendLogs has been done
proc.sendLogs = function(done) {
  log('send logs after execution',
    'state', proc.state,
    'priorState', proc.priorState
  )

  const cleanUp = () => {
    log('clean up after run')
    this.schedulerData.proc = null
    this.schedulerData.stopProc = null
    this.schedulerData.logProc = null
    this.schedulerData.simId = null
    this.schedulerData.sim = null
    events.emit({ state_machine: 'READY' })
    done()
  }

  // Get custom log command, if any
  const simData = this.schedulerData.sim.sim.data
  const simId = this.schedulerData.sim.id
  const userName = this.schedulerData.userName
  if (simData.logCmd) {
    log('Found logCmd. Spawn it')
    events.emit({ state_machine: 'SENDING LOGS' })
    this.schedulerData.logProc = this.spawnProcess(simData.logCmd)
    this.schedulerData.logProc.on('close', cleanUp)
    // when new text is sent to std out
    this.schedulerData.logProc.stdout.on('data', (data)=> {
      const newData = colorize(data)
      // add text to the output
      // simData.logCmdOutput += newData
      csgrant.updateResource(userName, simId, simData, (err) => {
        if (err) {
          log("error updating resource simId: " + simId)
        } else {
          log('sim "' + simId  + 'log data:' + newData )
        }
      })
    })
    // when new text is sent to std err
    this.schedulerData.logProc.stderr.on('data', (data)=> {
      const newData = colorize(data)
      // simData.logCmdOutput += newData
      csgrant.updateResource(userName, simId, simData, (err) => {
        if (err) {
          log("error updating resource simId: " + simId)
        } else {
          log('sim "' + simId  + 'log data:' + newData )
        }
      })
    })
  } else {
    // we don't need to log. Just do the clean up
    log('No logCmd. Just cleaning up after run')
    cleanUp()
    return
  }
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
  // list all simulations for the user
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

      const data = req.body
      const resourceData = { cmd: data.cmd,
        stopCmd: data.stopCmd,
        logCmd: data.logCmd,
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
      csgrant.getNextResourceId('simulation', (err, resourceName) => {
        if(err) {
          res.jsonp(error(err))
          return
        }
        csgrant.createResource(user, resourceName, resourceData,
          (err, data) => {
            if(err) {
              res.jsonp(error(err))
              return
            }
            const r = { success: true,
              operation: op,
              result: data,
              id: resourceName,
              requester: req.user
            }
            res.jsonp(r)
          })
      })
    })

  // Update a simulation
  app.put('/simulations/:simId',
    csgrant.authenticate,
    csgrant.ownsResource(':simId', false),
    function(req, res) {

      const resourceName = req.simId
      const newData = req.body
      console.log(' Update simulation: ' + resourceName)
      console.log(' new data: ' + JSON.stringify(newData))
      const user = req.user

      const r = {success: false}
      if (!newData.cmd) {
        return res.status(400).jsonp({success: false,
          error: 'Can\'t update simulation: missing [cmd]'})
      }
      if (newData.auto === undefined) {
        return res.status(400).jsonp({success: false,
          error: 'Can\'t update simulation: missing auto'})
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
          r.id = resourceName
          r.requester = req.user
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
      const r = {success: false}
      const user = req.user  // from previous middleware
      const resourceId = req.simId // from app.param (see below)

      csgrant.deleteResource(user, resourceId, (err, data) => {
        if(err) {
          return res.jsonp({success: false, error: err})
        }
        r.success = true
        r.result = data
        r.id = resourceId
        // success
        res.jsonp(r)
      })
    })

  // This is the route to stop a simulation (before starting a new one)
  app.post('/stopsimulation',
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
