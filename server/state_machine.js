'use strict'

const machina = require('machina')

// little fancy logger
exports.showLog = false
const log = exports.showLog? console.log: ()=>{}

// The state machine configuration. It defines the states and transitions.
// It is used to create a new FSM. The new FSM is expected to implement the
// following functions:
//   bootStateMachine: called once, this is similar to a constructor
//   getReadyToRunSimulator: called before a sim should run
//   this.startTheSimulator: called to start the sim
//   stopTheSimulator: called to stop the sim
//   sendLogs: called after the sim (to send logs)
//   cleanUpAfterRun: called before being ready for the next sim
let config = {
  // this call to set tell the FSM to boot
  boot: function() {
    this.handle("boot")
  },
  // called to start a simulation
  start: function() {
    this.handle("start")
    this.handle("run")
  },
  // called to set
  stop: function() {
    this.handle("stop")
    // "done" will be handled as a callback from the stop state
  },
  initialState: "nothing",
  states: {
    "nothing": {
      "boot": function () {
        this.transition("ready")
      },
    },
    "ready": {
      "start": function () {
        this.transition("prerun")
      },
    },
    "prerun": {
      "run": function () {
        this.transition("running")
      }
    },
    "running": {
      "stop": function () {
        this.transition("postrun")
      },
    },
    "postrun": {
      "cleanup": function() {
        // send the logs to the server
        this.sendLogs(() => {
          log('logs sent')
          this.handle("done")
        })
      },
      "done": function () {
        this.transition("ready")
      },
    },
  },
}

// A function that creates a process state machine, and
// splits the transition event into the different stages
// of simulation scheduler execution
exports.createMachine = function () {
  const proc = new machina.Fsm(config)
  // This is state machine state changes callback.
  // This is where simulators are started and stopped
  proc.on("transition", function (data) {
    // action: "nothing.boot" state: "ready" prior: "nothing"
    if (data.action === "nothing.boot") {
      this.bootStateMachine()
      log('state machine ready')
    }
    // action: "ready.start" state: "prerun" prior: "ready"
    else if (data.action  === "ready.start") {
      // setup (latency)
      this.getReadyToRunSimulator()
      log('Pre simulation run setup')
    }
    // action: "prerun.run" state: "running" prior: "prerun"
    else if (data.action === "prerun.run") {
      // start the simulator
      this.startTheSimulator()
      log('simulator started')
    }

    // action: "running.stop" state: "postrun" prior: "running"
    else if (data.action === "running.stop") {
      this.stopTheSimulator(() => {
        log('Simulation stopped')
        this.handle("cleanup")
      })
    }
    // action: "postrun.done" state: "ready" prior: "postrun"
    else if (data.action === "postrun.done") {
      log('about to execute postrun.done')
    }
    // oops ... this is not a state we expected
    else {
      const err = 'state "' + this.state + '" is not recognized'
      log('Error: ' + err)
      throw err
    }
  })
  // machine is hooked up and ready to go
  return proc
}


